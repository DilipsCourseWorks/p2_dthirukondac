/**
 * CS 424/524 - Intelligent Mobile Robotics | Assignment 2 - Part 2
 * Vision Follower Node  (depth-primary strategy for 20 % bonus)
 *
 * Strategy
 * ─────────
 * PRIMARY  (depth-only)  – Always active.
 *   • Reads the 32FC1/16UC1 depth image from the RGBD camera.
 *   • Finds the closest non-NaN pixel within the centre horizontal band.
 *   • Steers toward it and maintains ~1 m (3 ft) standoff distance.
 *   • Works even when the RGB stream is unavailable or the lights are off.
 *
 * SECONDARY (colour confirmation) – Active only when RGB is available.
 *   • Finds the "most red" pixel in the RGB frame.
 *   • Its column is used to bias the steering angle if it is close to the
 *     depth-derived centre — confirming we are tracking the red ball rather
 *     than any random nearby object.
 *   • If RGB goes dark / is covered, the node falls back gracefully to the
 *     depth-only path without interruption.
 *
 * Topics (adjust to your robot's actual topic names in the launch file):
 *   Subscribed:
 *     /camera/depth/image_raw          sensor_msgs/Image   (depth)
 *     /camera/rgb/image_raw            sensor_msgs/Image   (colour, optional)
 *   Published:
 *     /cmd_vel                         geometry_msgs/Twist
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <limits>
#include <mutex>

// ── Tunable parameters ───────────────────────────────────────────────────────

static constexpr double TARGET_DISTANCE_M  = 1.0;   // 1 m ≈ 3 ft standoff
static constexpr double MAX_LINEAR_SPEED   = 0.3;   // m/s
static constexpr double MAX_ANGULAR_SPEED  = 0.6;   // rad/s
static constexpr double DISTANCE_TOLERANCE = 0.10;  // ±10 cm dead-band

// Horizontal band (fraction of frame height) used for closest-point search
static constexpr double BAND_TOP_FRAC    = 0.30;
static constexpr double BAND_BOTTOM_FRAC = 0.70;

// Maximum depth value to consider as valid (metres); reject walls far away
static constexpr double MAX_VALID_DEPTH_M = 5.0;

// How far (in columns) the RGB "red centroid" may be from the depth centroid
// and still be counted as a confirmation hit
static constexpr int RGB_DEPTH_COLUMN_TOLERANCE = 60;  // pixels

// ── Node state ───────────────────────────────────────────────────────────────

struct FollowerState {
    // Depth-derived values (updated in depth callback)
    double  closest_dist_m{std::numeric_limits<double>::quiet_NaN()};
    double  closest_col_frac{0.5};   // normalised [0,1]; 0.5 = straight ahead

    // RGB-derived values (updated in colour callback, optional)
    bool    rgb_active{false};
    double  red_col_frac{0.5};

    std::mutex mtx;
};

static FollowerState g_state;

// ── Helpers ──────────────────────────────────────────────────────────────────

/**
 * Return the column fraction [0,1] and depth [m] of the closest non-NaN,
 * non-zero pixel within the horizontal band of the depth image.
 */
static bool findClosestInBand(const cv::Mat& depth_f32,
                               double& out_col_frac,
                               double& out_dist_m)
{
    const int rows = depth_f32.rows;
    const int cols = depth_f32.cols;
    const int row_top    = static_cast<int>(rows * BAND_TOP_FRAC);
    const int row_bottom = static_cast<int>(rows * BAND_BOTTOM_FRAC);

    double min_d = std::numeric_limits<double>::max();
    int    min_c = cols / 2;
    bool   found = false;

    for (int r = row_top; r < row_bottom; ++r) {
        const float* row_ptr = depth_f32.ptr<float>(r);
        for (int c = 0; c < cols; ++c) {
            float d = row_ptr[c];
            if (std::isnan(d) || d <= 0.0f || d > MAX_VALID_DEPTH_M)
                continue;
            if (d < min_d) {
                min_d = d;
                min_c = c;
                found = true;
            }
        }
    }

    if (found) {
        out_col_frac = static_cast<double>(min_c) / static_cast<double>(cols - 1);
        out_dist_m   = min_d;
    }
    return found;
}

/**
 * Find the centroid column of the "most red" region in an BGR image.
 * Returns false when no sufficiently red pixels are found.
 */
static bool findRedCentroid(const cv::Mat& bgr, double& out_col_frac)
{
    // Convert to HSV for robust red detection under varying lighting
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    // Red wraps around 0/180 in HSV
    cv::Mat mask1, mask2, mask;
    cv::inRange(hsv, cv::Scalar(  0, 100,  60), cv::Scalar( 10, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(160, 100,  60), cv::Scalar(180, 255, 255), mask2);
    cv::bitwise_or(mask1, mask2, mask);

    // Require a minimum blob size to suppress noise
    cv::Moments m = cv::moments(mask, true);
    if (m.m00 < 200.0)   // fewer than 200 red pixels → ignore
        return false;

    out_col_frac = (m.m10 / m.m00) / static_cast<double>(bgr.cols - 1);
    return true;
}

// ── ROS callbacks ─────────────────────────────────────────────────────────────

void depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        // Accept both 32FC1 (metres) and 16UC1 (millimetres) depth encodings
        if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        } else if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            // Convert mm → m
            cv::Mat f32;
            cv_ptr->image.convertTo(f32, CV_32FC1, 1.0 / 1000.0);
            std::lock_guard<std::mutex> lock(g_state.mtx);
            double col_frac, dist_m;
            if (findClosestInBand(f32, col_frac, dist_m)) {
                g_state.closest_col_frac = col_frac;
                g_state.closest_dist_m   = dist_m;
            }
            return;
        } else {
            ROS_WARN_THROTTLE(5.0, "vision_follower: Unsupported depth encoding: %s",
                              msg->encoding.c_str());
            return;
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR_THROTTLE(5.0, "vision_follower: cv_bridge depth exception: %s", e.what());
        return;
    }

    std::lock_guard<std::mutex> lock(g_state.mtx);
    double col_frac, dist_m;
    if (findClosestInBand(cv_ptr->image, col_frac, dist_m)) {
        g_state.closest_col_frac = col_frac;
        g_state.closest_dist_m   = dist_m;
    }
}

void rgbCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_WARN_THROTTLE(5.0, "vision_follower: cv_bridge RGB exception: %s", e.what());
        std::lock_guard<std::mutex> lock(g_state.mtx);
        g_state.rgb_active = false;
        return;
    }

    double col_frac;
    bool found = findRedCentroid(cv_ptr->image, col_frac);

    std::lock_guard<std::mutex> lock(g_state.mtx);
    g_state.rgb_active  = found;
    if (found)
        g_state.red_col_frac = col_frac;
}

// ── Main ─────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vision_follower");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // Allow topic remapping via private parameters (or use launch file remaps)
    std::string depth_topic, rgb_topic, cmd_vel_topic;
    nh_private.param<std::string>("depth_topic",   depth_topic,   "/camera/depth/image_raw");
    nh_private.param<std::string>("rgb_topic",     rgb_topic,     "/camera/rgb/image_raw");
    nh_private.param<std::string>("cmd_vel_topic", cmd_vel_topic, "/cmd_vel");

    image_transport::ImageTransport it(nh);

    image_transport::Subscriber depth_sub =
        it.subscribe(depth_topic, 1, depthCallback);

    image_transport::Subscriber rgb_sub =
        it.subscribe(rgb_topic, 1, rgbCallback);

    ros::Publisher cmd_pub =
        nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 10);

    ros::Rate loop_rate(20);  // 20 Hz control loop

    ROS_INFO("vision_follower: Started. depth='%s'  rgb='%s'  cmd_vel='%s'",
             depth_topic.c_str(), rgb_topic.c_str(), cmd_vel_topic.c_str());
    ROS_INFO("vision_follower: Depth-primary tracking active. "
             "RGB stream used as optional confirmation.");

    while (ros::ok()) {
        ros::spinOnce();

        geometry_msgs::Twist cmd;

        double dist_m, col_frac;
        bool   rgb_active, red_col_frac_valid;
        double red_col_frac;

        {
            std::lock_guard<std::mutex> lock(g_state.mtx);
            dist_m              = g_state.closest_dist_m;
            col_frac            = g_state.closest_col_frac;
            rgb_active          = g_state.rgb_active;
            red_col_frac        = g_state.red_col_frac;
            red_col_frac_valid  = rgb_active;
        }

        if (std::isnan(dist_m)) {
            // No valid depth reading yet — stop and wait
            ROS_WARN_THROTTLE(2.0, "vision_follower: No depth data. Stopping.");
            cmd_pub.publish(cmd);
            loop_rate.sleep();
            continue;
        }

        // ── Steering angle ────────────────────────────────────────────────
        // col_frac in [0,1]; 0.5 = centre.  Convert to signed error [-0.5, 0.5].
        double steering_error = col_frac - 0.5;  // positive → object is right

        // If RGB is active and the red centroid is within tolerance of the
        // depth centroid, bias steering toward the red centroid (stronger
        // confirmation that we found the ball).
        if (red_col_frac_valid) {
            int depth_col_px = static_cast<int>(col_frac       * 640);
            int red_col_px   = static_cast<int>(red_col_frac   * 640);
            if (std::abs(depth_col_px - red_col_px) < RGB_DEPTH_COLUMN_TOLERANCE) {
                // Weighted blend: 60 % depth, 40 % colour
                double blended_frac = 0.6 * col_frac + 0.4 * red_col_frac;
                steering_error = blended_frac - 0.5;
            }
        }

        // Angular velocity: proportional controller, negate because
        // positive error (object on right) requires left turn (negative z).
        double Kp_angular = 1.2;
        cmd.angular.z = std::max(-MAX_ANGULAR_SPEED,
                        std::min( MAX_ANGULAR_SPEED,
                                 -Kp_angular * steering_error));

        // ── Linear velocity ───────────────────────────────────────────────
        double dist_error = dist_m - TARGET_DISTANCE_M;  // positive → too far

        if (std::abs(dist_error) < DISTANCE_TOLERANCE) {
            cmd.linear.x = 0.0;  // within dead-band → hold position
        } else {
            double Kp_linear = 0.4;
            cmd.linear.x = std::max(-MAX_LINEAR_SPEED,
                           std::min( MAX_LINEAR_SPEED,
                                    Kp_linear * dist_error));
        }

        cmd_pub.publish(cmd);
        loop_rate.sleep();
    }

    // Stop the robot on shutdown
    geometry_msgs::Twist stop;
    cmd_pub.publish(stop);
    return 0;
}
