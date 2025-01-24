#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

class BlobDetectorNode {
public:
    BlobDetectorNode() {
        image_sub_ = nh_.subscribe("/usb_cam/image_raw", 1, &BlobDetectorNode::imageCallback, this);
        image_pub_ = nh_.advertise<sensor_msgs::Image>("/processed_image", 1);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Publisher image_pub_;

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

        // Define range for bright and dark blue colors in HSV
        cv::Scalar lower_light(85, 50, 150);
        cv::Scalar upper_light(110, 255, 255);

        cv::Scalar lower_dark(85, 30, 50);
        cv::Scalar upper_dark(110, 255, 150);

        // Threshold the HSV image to get light and dark blue colors
        cv::inRange(hsv_image, lower_light, upper_light, mask_light);
        cv::inRange(hsv_image, lower_dark, upper_dark, mask_dark);

        // Combine masks
        cv::bitwise_or(mask_light, mask_dark, combined_mask);

        // Apply Gaussian Blur to reduce noise
        cv::GaussianBlur(combined_mask, combined_mask, cv::Size(5, 5), 0);

        // Set up blob detector parameters
        cv::SimpleBlobDetector::Params params;
        params.filterByColor = false;
        params.filterByArea = true;
        params.minArea = 50;  // Lowered to detect smaller blobs
        params.maxArea = 10000;
        params.minThreshold = 10;  // Increased sensitivity
        params.maxThreshold = 200;

        // Create a blob detector with the parameters
        cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

        // Detect blobs
        std::vector<cv::KeyPoint> keypoints;
        detector->detect(combined_mask, keypoints);

        // Draw detected blobs as circles
        for (size_t i = 0; i < keypoints.size(); i++) {
            cv::circle(cv_image, keypoints[i].pt, static_cast<int>(keypoints[i].size), cv::Scalar(0, 255, 0), 2);
        }

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



































































// #include <ros/ros.h>
// #include <sensor_msgs/Image.h>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>
// #include <opencv2/features2d.hpp>

// class BlobDetectorNode {
// public:
//     BlobDetectorNode() {
//         image_sub_ = nh_.subscribe("/usb_cam/image_raw", 1, &BlobDetectorNode::imageCallback, this);
//         image_pub_ = nh_.advertise<sensor_msgs::Image>("/processed_image", 1);
//     }

// private:
//     ros::NodeHandle nh_;
//     ros::Subscriber image_sub_;
//     ros::Publisher image_pub_;

//     void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
//         // Convert ROS image message to OpenCV image
//         cv_bridge::CvImagePtr cv_ptr;
//         try {
//             cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//         } catch (cv_bridge::Exception& e) {
//             ROS_ERROR("cv_bridge exception: %s", e.what());
//             return;
//         }

//         cv::Mat cv_image = cv_ptr->image;
//         cv::Mat hsv_image, mask;

//         // Convert BGR to HSV
//         cv::cvtColor(cv_image, hsv_image, cv::COLOR_BGR2HSV);

//         // Equalize histogram on the V channel
//         std::vector<cv::Mat> hsv_channels;
//         cv::split(hsv_image, hsv_channels);
//         cv::equalizeHist(hsv_channels[2], hsv_channels[2]);  // Améliorer le canal V (luminosité)
//         cv::merge(hsv_channels, hsv_image);

//         // Apply Gaussian Blur to reduce noise
//         cv::GaussianBlur(hsv_image, hsv_image, cv::Size(5, 5), 0);

//         cv::Mat gamma_corrected;
//         cv_image.convertTo(gamma_corrected, -1, 1, 50);  // Ajustez l'éclairage

//         // Define range of pink color in HSV
//         // cv::Scalar lower_pink(150, 80, 150);
//         // cv::Scalar upper_pink(180, 255, 255);


//         // cv::Scalar lower_cyan(85, 50, 150);
//         // cv::Scalar upper_cyan(110, 255, 255);

//         // Threshold the HSV image to get only cyan colors
//         // cv::inRange(hsv_image, lower_cyan, upper_cyan, mask);


//         cv::Mat mask_light, mask_dark, combined_mask;
//         cv::Scalar lower_light(85, 50, 150);
//         cv::Scalar upper_light(110, 255, 255);

//         cv::Scalar lower_dark(85, 30, 50);
//         cv::Scalar upper_dark(110, 255, 150);

//         cv::inRange(hsv_image, lower_light, upper_light, mask_light);
//         cv::inRange(hsv_image, lower_dark, upper_dark, mask_dark);
//         cv::bitwise_or(mask_light, mask_dark, combined_mask);



//         // Set up blob detector parameters
//         cv::SimpleBlobDetector::Params params;
//         params.filterByColor = false;
//         params.filterByArea = true;
//         params.minArea = 50;
//         params.maxArea = 3000;
//         params.minThreshold = 10;  // Augmentez la sensibilité
//         params.maxThreshold = 200;

//         // Create a blob detector with the parameters
//         cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

//         // Detect blobs
//         std::vector<cv::KeyPoint> keypoints;
//         detector->detect(mask, keypoints);

//         // Draw detected blobs as circles
//         for (size_t i = 0; i < keypoints.size(); i++) {
//             cv::circle(cv_image, keypoints[i].pt, static_cast<int>(keypoints[i].size), cv::Scalar(0, 255, 0), 2);
//         }

//         // Display the original and mask images for debugging
//         cv::imshow("Original Image", cv_image);
//         cv::imshow("Mask", mask);
//         cv::waitKey(1);

//         // Convert OpenCV image back to ROS image message
//         sensor_msgs::ImagePtr processed_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image).toImageMsg();

//         // Publish the processed image
//         image_pub_.publish(processed_image_msg);
//     }
// };

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "blob_detector_node");
//     BlobDetectorNode node;
//     ros::spin();
//     return 0;
// }



























// #include <ros/ros.h>
// #include <sensor_msgs/Image.h>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>
// #include <opencv2/features2d.hpp>

// class BlobDetectorNode {
// public:
//     BlobDetectorNode() {
//         image_sub_ = nh_.subscribe("/usb_cam/image_raw", 1, &BlobDetectorNode::imageCallback, this);
//         image_pub_ = nh_.advertise<sensor_msgs::Image>("/processed_image", 1);
//     }

// private:
//     ros::NodeHandle nh_;
//     ros::Subscriber image_sub_;
//     ros::Publisher image_pub_;

//     void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
//         // Convert ROS image message to OpenCV image
//         cv_bridge::CvImagePtr cv_ptr;
//         try {
//             cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//         } catch (cv_bridge::Exception& e) {
//             ROS_ERROR("cv_bridge exception: %s", e.what());
//             return;
//         }

//         cv::Mat cv_image = cv_ptr->image;
//         cv::Mat hsv_image, mask;

//         // Convert BGR to HSV
//         cv::cvtColor(cv_image, hsv_image, cv::COLOR_BGR2HSV);

//         // Define range of purple color in HSV
//         // cv::Scalar lower_purple(120, 50, 50);
//         // cv::Scalar upper_purple(160, 255, 255);

//         cv::Scalar lower_pink(160, 100, 100);
//         cv::Scalar upper_pink(180, 255, 255);

//         // Threshold the HSV image to get only purple colors
//         cv::inRange(hsv_image, lower_pink, upper_pink, mask);

//         // Set up blob detector parameters
//         cv::SimpleBlobDetector::Params params;
//         params.filterByColor = false;
//         params.filterByArea = true;
//         params.minArea = 100;
//         params.maxArea = 3000;

//         // Create a blob detector with the parameters
//         cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

//         // Detect blobs
//         std::vector<cv::KeyPoint> keypoints;
//         detector->detect(mask, keypoints);

//         // Draw detected blobs as circles
//         for (size_t i = 0; i < keypoints.size(); i++) {
//             cv::circle(cv_image, keypoints[i].pt, static_cast<int>(keypoints[i].size), cv::Scalar(0, 255, 0), 2);
//         }

//         // Convert OpenCV image back to ROS image message
//         sensor_msgs::ImagePtr processed_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image).toImageMsg();

//         // Publish the processed image
//         image_pub_.publish(processed_image_msg);
//     }
// };

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "blob_detector_node");
//     BlobDetectorNode node;
//     ROS_INFO("Blob detector node started");
//     ros::spin();
//     return 0;
// }
