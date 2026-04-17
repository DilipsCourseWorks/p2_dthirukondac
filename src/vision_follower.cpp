// CS 424/524 - Assignment 2, Part 2
// Ball follower node - follows a red ball using RGB and depth cameras
// Keeps a distance of 1 meter (about 3 feet) from the ball
// Bonus: uses depth as primary signal, RGB is optional confirmation

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <limits>
#include <mutex>

// distance we want to keep from the ball
static const double TARGET_DIST = 1.0;
static const double MAX_LINEAR  = 0.3;
static const double MAX_ANGULAR = 0.6;
static const double DIST_TOL    = 0.10;
static const double MAX_DEPTH   = 5.0;

// only look at the middle band of the depth image
static const double BAND_TOP    = 0.30;
static const double BAND_BOTTOM = 0.70;

// shared state between callbacks
struct State {
    double depth_dist = std::numeric_limits<double>::quiet_NaN();
    double depth_col  = 0.5;
    bool   rgb_found  = false;
    double rgb_col    = 0.5;
    std::mutex mtx;
};

static State g_state;

// find the closest point in the middle band of the depth image
bool getClosestPoint(const cv::Mat& depth, double& col_out, double& dist_out)
{
    int rows = depth.rows;
    int cols = depth.cols;
    int r_top = (int)(rows * BAND_TOP);
    int r_bot = (int)(rows * BAND_BOTTOM);

    double min_d = std::numeric_limits<double>::max();
    int min_c = cols / 2;
    bool found = false;

    for (int r = r_top; r < r_bot; r++) {
        const float* row = depth.ptr<float>(r);
        for (int c = 0; c < cols; c++) {
            float d = row[c];
            if (std::isnan(d) || d <= 0 || d > MAX_DEPTH)
                continue;
            if (d < min_d) {
                min_d = d;
                min_c = c;
                found = true;
            }
        }
    }

    if (found) {
        col_out  = (double)min_c / (double)(cols - 1);
        dist_out = min_d;
    }
    return found;
}

// find the "most red" region in the image and return its column position
bool getRedColumn(const cv::Mat& bgr, double& col_out)
{
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    // red hue wraps around in HSV so we need two ranges
    cv::Mat mask1, mask2, mask;
    cv::inRange(hsv, cv::Scalar(0, 100, 60),   cv::Scalar(10, 255, 255),  mask1);
    cv::inRange(hsv, cv::Scalar(160, 100, 60), cv::Scalar(180, 255, 255), mask2);
    cv::bitwise_or(mask1, mask2, mask);

    cv::Moments m = cv::moments(mask, true);
    if (m.m00 < 200.0)
        return false;

    col_out = (m.m10 / m.m00) / (double)(bgr.cols - 1);
    return true;
}

void depthCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr ptr;
    cv::Mat depth_f32;

    try {
        if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            depth_f32 = ptr->image;
        } else if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            ptr->image.convertTo(depth_f32, CV_32FC1, 1.0 / 1000.0);
        } else {
            ROS_WARN_THROTTLE(5.0, "Unknown depth encoding: %s", msg->encoding.c_str());
            return;
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR_THROTTLE(5.0, "cv_bridge error: %s", e.what());
        return;
    }

    double col, dist;
    if (getClosestPoint(depth_f32, col, dist)) {
        std::lock_guard<std::mutex> lock(g_state.mtx);
        g_state.depth_col  = col;
        g_state.depth_dist = dist;
    }
}

void rgbCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr ptr;
    try {
        ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        std::lock_guard<std::mutex> lock(g_state.mtx);
        g_state.rgb_found = false;
        return;
    }

    double col;
    bool found = getRedColumn(ptr->image, col);

    std::lock_guard<std::mutex> lock(g_state.mtx);
    g_state.rgb_found = found;
    if (found)
        g_state.rgb_col = col;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vision_follower");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    std::string depth_topic, rgb_topic, cmd_topic;
    nh_priv.param<std::string>("depth_topic",   depth_topic, "/camera/depth/image_raw");
    nh_priv.param<std::string>("rgb_topic",     rgb_topic,   "/camera/rgb/image_raw");
    nh_priv.param<std::string>("cmd_vel_topic", cmd_topic,   "/cmd_vel");

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber depth_sub = it.subscribe(depth_topic, 1, depthCb);
    image_transport::Subscriber rgb_sub   = it.subscribe(rgb_topic,   1, rgbCb);

    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>(cmd_topic, 10);

    ros::Rate rate(20);

    ROS_INFO("vision_follower started. depth=%s rgb=%s cmd=%s",
             depth_topic.c_str(), rgb_topic.c_str(), cmd_topic.c_str());

    while (ros::ok()) {
        ros::spinOnce();

        double dist, depth_col, rgb_col;
        bool rgb_found;

        {
            std::lock_guard<std::mutex> lock(g_state.mtx);
            dist       = g_state.depth_dist;
            depth_col  = g_state.depth_col;
            rgb_found  = g_state.rgb_found;
            rgb_col    = g_state.rgb_col;
        }

        geometry_msgs::Twist cmd;

        if (std::isnan(dist)) {
            ROS_WARN_THROTTLE(2.0, "No depth data, stopping.");
            cmd_pub.publish(cmd);
            rate.sleep();
            continue;
        }

        // figure out which direction to turn
        double steer = depth_col - 0.5;

        // if RGB is active and agrees with depth, blend them together
        if (rgb_found) {
            int d_px = (int)(depth_col * 640);
            int r_px = (int)(rgb_col   * 640);
            if (abs(d_px - r_px) < 60) {
                double blended = 0.6 * depth_col + 0.4 * rgb_col;
                steer = blended - 0.5;
            }
        }

        // angular control
        cmd.angular.z = std::max(-MAX_ANGULAR, std::min(MAX_ANGULAR, -1.2 * steer));

        // linear control - move toward or away from ball to keep 1m distance
        double err = dist - TARGET_DIST;
        if (fabs(err) < DIST_TOL) {
            cmd.linear.x = 0.0;
        } else {
            cmd.linear.x = std::max(-MAX_LINEAR, std::min(MAX_LINEAR, 0.4 * err));
        }

        cmd_pub.publish(cmd);
        rate.sleep();
    }

    // stop robot before exiting
    geometry_msgs::Twist stop;
    cmd_pub.publish(stop);
    return 0;
}
