#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <algorithm>
#include <cmath>


class BlobDetectorNode {
public:
    BlobDetectorNode() {
        image_sub_ = nh_.subscribe("/usb_cam/image_raw", 1, &BlobDetectorNode::imageCallback, this);
        image_pub_ = nh_.advertise<sensor_msgs::Image>("/processed_image", 1);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Publisher image_pub_;
    ros::Publisher cmd_vel_pub_;


    float prev_linear_x_ = 0.0;
    float prev_angular_z_ = 0.0;
    float max_linear_speed_ = 0.2;
    float max_angular_speed_ = 1;
    float smoothing_factor_ = 0.02;


    template <typename T>
    T clamp(T value, T min_val, T max_val) {
        return (value < min_val) ? min_val : (value > max_val ? max_val : value);
    }

    void smoothControl(float& current_value, float target_value, float max_change) {
        float change = target_value - current_value;
        if (std::abs(change) > max_change) {
            change = (change > 0 ? max_change : -max_change);
        }
        current_value += change;
    }






    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        // Convert ROS image message to OpenCV image
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat cv_image = cv_ptr->image;
        cv::Mat hsv_image, mask_light, mask_dark, combined_mask;

        // Convert BGR to HSV
        cv::cvtColor(cv_image, hsv_image, cv::COLOR_BGR2HSV);
        

        // Equalize histogram on the V channel to improve visibility in shadows
        std::vector<cv::Mat> hsv_channels;
        cv::split(hsv_image, hsv_channels);
        cv::equalizeHist(hsv_channels[2], hsv_channels[2]);  // Equalize V channel
        cv::merge(hsv_channels, hsv_image);

        // // Define range for bright and dark blue colors in HSV
        // cv::Scalar lower_light(85, 50, 150);
        // cv::Scalar upper_light(110, 255, 255);

        // cv::Scalar lower_dark(85, 30, 50);
        // cv::Scalar upper_dark(110, 255, 150);


        cv::Scalar lower_light(140, 50, 100);  // Rose clair
        cv::Scalar upper_light(170, 255, 255);

        cv::Scalar lower_dark(140, 80, 50);   // Rose foncé
        cv::Scalar upper_dark(170, 255, 150);




        // Threshold the HSV image to get light and dark blue colors
        cv::inRange(hsv_image, lower_light, upper_light, mask_light);
        cv::inRange(hsv_image, lower_dark, upper_dark, mask_dark);

        // Combine masks
        cv::bitwise_or(mask_light, mask_dark, combined_mask);

        // Apply Gaussian Blur to reduce noise
        cv::GaussianBlur(combined_mask, combined_mask, cv::Size(9, 9), 2);

        // Set up blob detector parameters
        cv::SimpleBlobDetector::Params params;
        params.filterByColor = false;
        params.filterByArea = true;
        params.minArea = 50;  // Lowered to detect smaller blobs
        params.maxArea = 5000;
        params.minThreshold = 10;  // Increased sensitivity
        params.maxThreshold = 255;

        

        // Create a blob detector with the parameters
        cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

        // Detect blobs
        std::vector<cv::KeyPoint> keypoints;
        detector->detect(combined_mask, keypoints);

        geometry_msgs::Twist cmd_vel_msg;
        if (!keypoints.empty()) {
            // Select the largest blob
            auto largest_blob = std::max_element(keypoints.begin(), keypoints.end(), [](const cv::KeyPoint& a, const cv::KeyPoint& b) {
                return a.size < b.size;
            });

            float blob_x = largest_blob->pt.x;
            float blob_size = largest_blob->size;
            float img_center_x = cv_image.cols / 2.0;

            // Control logic
            float linear_speed = 0.0;
            float angular_speed = 0.0;

            if (blob_size < 3000) {  // If the blob is small, move forward
                linear_speed = max_linear_speed_;
            }

            if (std::abs(blob_x - img_center_x) > 20) {  // If the blob is not centered
                angular_speed = -0.01 * (blob_x - img_center_x);  // Adjust rotation to center the blob  0.002
                angular_speed = clamp(angular_speed, -max_angular_speed_, max_angular_speed_);
            }

            ROS_INFO("Linear Speed: %f, Angular Speed: %f", linear_speed, angular_speed);
            // Smooth control for linear and angular speeds
            smoothControl(prev_linear_x_, linear_speed, smoothing_factor_);
            smoothControl(prev_angular_z_, angular_speed, smoothing_factor_);

            cmd_vel_msg.linear.x = -prev_linear_x_;
            cmd_vel_msg.angular.z = prev_angular_z_;

            // Draw detected blob as a circle
            cv::circle(cv_image, largest_blob->pt, static_cast<int>(largest_blob->size), cv::Scalar(0, 255, 0), 2);
        }

        // Publish velocity commands
        cmd_vel_pub_.publish(cmd_vel_msg);

        // Display the original and mask images for debugging
        cv::imshow("Original Image", cv_image);
        cv::imshow("Mask", combined_mask);
        cv::waitKey(1);

        // Convert OpenCV image back to ROS image message
        sensor_msgs::ImagePtr processed_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image).toImageMsg();

        // Publish the processed image
        image_pub_.publish(processed_image_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "blob_detector_node");
    BlobDetectorNode node;
    ros::spin();
    return 0;
}
