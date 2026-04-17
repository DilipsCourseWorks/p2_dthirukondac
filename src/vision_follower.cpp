/**
 * CS 424/524 - Intelligent Mobile Robotics | Assignment 2 - Part 2
 * Vision Follower Node  (depth-first approach, going for the 20% bonus)
 *
 * How it works:
 *
 * DEPTH (main approach, always running):
 *   - Grabs the depth image from the RGBD camera (handles both 32FC1 and 16UC1).
 *   - Scans the middle horizontal strip for the closest valid pixel (ignores NaNs and far walls).
 *   - Turns toward that pixel and tries to stay about 1 m (roughly 3 ft) away.
 *   - This keeps working even if the RGB camera is blocked or the room goes dark.
 *
 * RGB (backup confirmation, only used when the color stream is up):
 *   - Looks for the reddest region in the frame.
 *   - If that red blob lines up reasonably well with what depth found, we blend
 *     both signals together — gives us more confidence we're chasing the red ball
 *     and not some random thing that happens to be nearby.
 *   - If the RGB feed dies or gets covered, the node just keeps going on depth alone.
 *
 * Topics (you may need to remap these in the launch file to match your robot):
 *   Subscribed:
 *     /camera/depth/image_raw          sensor_msgs/Image   (depth)
 *     /camera/rgb/image_raw            sensor_msgs/Image   (color, optional)
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

// --- Parameters you might want to tweak ---

static constexpr double TARGET_DISTANCE_M  = 1.0;   // how far we want to stay from the target (~3 ft)
static constexpr double MAX_LINEAR_SPEED   = 0.3;   // cap forward/backward speed (m/s)
static constexpr double MAX_ANGULAR_SPEED  = 0.6;   // cap turning speed (rad/s)
static constexpr double DISTANCE_TOLERANCE = 0.10;  // within 10 cm is close enough, don't bother moving

// only scan the middle 40% of the frame height — avoids floor and ceiling noise
static constexpr double BAND_TOP_FRAC    = 0.30;
static constexpr double BAND_BOTTOM_FRAC = 0.70;

// anything beyond 5 m is probably a wall or background, skip it
static constexpr double MAX_VALID_DEPTH_M = 5.0;

// if the red blob (from RGB) and the closest depth point are more than 60 px apart
// horizontally, we probably aren't looking at the same object — ignore the RGB reading
static constexpr int RGB_DEPTH_COLUMN_TOLERANCE = 60;  // pixels

// --- Shared state between the depth/RGB callbacks and the control loop ---

struct FollowerState {
    // set by the depth callback each frame
    double  closest_dist_m{std::numeric_limits<double>::quiet_NaN()};
    double  closest_col_frac{0.5};   // 0 = far left, 1 = far right, 0.5 = center

    // set by the RGB callback — only valid when rgb_active is true
    bool    rgb_active{false};
    double  red_col_frac{0.5};

    std::mutex mtx;
};

static FollowerState g_state;

// --- Helper functions ---

/**
 * Scans the middle horizontal band of the depth image and finds the closest
 * valid pixel. Returns its column (as a 0-1 fraction) and distance in metres.
 * Returns false if nothing valid was found in that region.
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
 * Finds the horizontal center of the reddest region in a BGR image.
 * Returns false if there isn't enough red to be confident (too noisy).
 */
static bool findRedCentroid(const cv::Mat& bgr, double& out_col_frac)
{
    // HSV makes red detection much more reliable across different lighting conditions
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    // Red sits at both ends of the hue wheel in HSV (near 0 and near 180), so we need two masks
    cv::Mat mask1, mask2, mask;
    cv::inRange(hsv, cv::Scalar(  0, 100,  60), cv::Scalar( 10, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(160, 100,  60), cv::Scalar(180, 255, 255), mask2);
    cv::bitwise_or(mask1, mask2, mask);

    // Don't trust the reading if the red region is tiny — likely just noise
    cv::Moments m = cv::moments(mask, true);
    if (m.m00 < 200.0)   // need at least 200 red pixels to count it
        return false;

    out_col_frac = (m.m10 / m.m00) / static_cast<double>(bgr.cols - 1);
    return true;
}

// --- ROS topic callbacks ---

void depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        // Some cameras give depth in metres (32FC1), others in millimetres (16UC1) — handle both
        if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        } else if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            // camera reports mm, we need metres
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

// --- Entry point ---

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vision_follower");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // topic names can be overridden via private params or launch file remaps
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

    ros::Rate loop_rate(20);  // run the control loop at 20 Hz

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
            // haven't received a valid depth frame yet, just sit still
            ROS_WARN_THROTTLE(2.0, "vision_follower: No depth data. Stopping.");
            cmd_pub.publish(cmd);
            loop_rate.sleep();
            continue;
        }

        // --- Figure out how much to turn ---
        // col_frac is 0-1 across the frame width; subtracting 0.5 gives us a
        // signed error where positive means the target is to the right.
        double steering_error = col_frac - 0.5;  // positive → object is right of center

        // If the RGB stream is up and the red blob lines up with the depth
        // reading, blend both together. Depth gets 60%, color gets 40%.
        // This only kicks in when they're actually pointing at the same thing.
        if (red_col_frac_valid) {
            int depth_col_px = static_cast<int>(col_frac       * 640);
            int red_col_px   = static_cast<int>(red_col_frac   * 640);
            if (std::abs(depth_col_px - red_col_px) < RGB_DEPTH_COLUMN_TOLERANCE) {
                // blend depth and color signals
                double blended_frac = 0.6 * col_frac + 0.4 * red_col_frac;
                steering_error = blended_frac - 0.5;
            }
        }

        // P controller for turning. We negate it because if the target is on
        // the right (positive error), we need to turn left (negative z in ROS).
        double Kp_angular = 1.2;
        cmd.angular.z = std::max(-MAX_ANGULAR_SPEED,
                        std::min( MAX_ANGULAR_SPEED,
                                 -Kp_angular * steering_error));

        // --- Figure out how fast to drive ---
        double dist_error = dist_m - TARGET_DISTANCE_M;  // positive means we're too far away

        if (std::abs(dist_error) < DISTANCE_TOLERANCE) {
            cmd.linear.x = 0.0;  // close enough, no need to inch forward or back
        } else {
            double Kp_linear = 0.4;
            cmd.linear.x = std::max(-MAX_LINEAR_SPEED,
                           std::min( MAX_LINEAR_SPEED,
                                    Kp_linear * dist_error));
        }

        cmd_pub.publish(cmd);
        loop_rate.sleep();
    }

    // make sure the robot isn't still moving when we exit
    geometry_msgs::Twist stop;
    cmd_pub.publish(stop);
    return 0;
}
