#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

class T265Undistorsion {
    cv::Mat map_x, map_y;

    ros::Subscriber img_sub;
    image_transport::Publisher img_rect_pub;
    sensor_msgs::CameraInfo camera_info_out;
    ros::Publisher camera_info_out_pub;

public:
    T265Undistorsion(ros::NodeHandle nh, image_transport::ImageTransport it, const std::string &param_file_path) {
        cv::Mat K, K1, D;
        cv::Vec3d T;
        cv::Vec2i size_input, size_output;
        cv::Mat identity = cv::Mat::eye(3, 3, cv::DataType<double>::type);

        cv::FileStorage param_file = cv::FileStorage(param_file_path, cv::FileStorage::READ);
        param_file["K1"] >> K;
        param_file["D1"] >> D;
        param_file["T"] >> T;
        param_file["input"] >> size_input;
        param_file["output"] >> size_output;

        cv::Size in_img_size(size_input[0], size_input[1]);
        cv::Size out_img_size(size_output[0], size_output[1]);

        K1 = cv::getOptimalNewCameraMatrix(K, D, in_img_size, 1., out_img_size);
        cv::fisheye::initUndistortRectifyMap(K, D, identity, K1,
                                             out_img_size, CV_32FC1, map_x, map_y); // or CV_16SC2

        // Copy the parameters for rectified images to the camera_info messages
        camera_info_out.width = size_output[0];
        camera_info_out.height = size_output[1];
        camera_info_out.D = std::vector<double>(5, 0);

        for (int i = 0; i < 9; i++) {
            camera_info_out.K[i] = K.at<double>(i);
        }
        for (int i = 0; i < 12; i++) {
            camera_info_out.P[i] = K.at<double>(i);
        }

        img_sub = nh.subscribe("/camera/fisheye1/image_raw", 1, &T265Undistorsion::img_callback, this);
        img_sub.getTopic();

        img_rect_pub = it.advertise("/camera/fisheye1/rect/image", 1);
        camera_info_out_pub = nh.advertise<sensor_msgs::CameraInfo>("/camera/fisheye1/rect/camera_info", 1);

        ROS_INFO("Initialization complete. Publishing rectified images and camera_info when raw images arrive...");
    }

    void img_callback(const sensor_msgs::ImageConstPtr &msg) {
        cv::Mat src = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Mat dst;

        // undistort
        remap(src, dst, map_x, map_y, cv::INTER_LINEAR, CV_HAL_BORDER_CONSTANT);

        sensor_msgs::ImagePtr rect_img = cv_bridge::CvImage(msg->header, "bgr8", dst).toImageMsg();
        img_rect_pub.publish(rect_img);

        std_msgs::Header header = msg->header;
        camera_info_out.header = header;
        camera_info_out_pub.publish(camera_info_out);
    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "t265_single_undistort");

    ros::NodeHandle nh("~");
    auto it = image_transport::ImageTransport(nh);

    std::string param_file_path;
    if (nh.getParam("param_file_path", param_file_path)) {
        ROS_INFO("Using parameter file: %s", param_file_path.c_str());
    } else {
        ROS_ERROR("Failed to get param file path. Please check and try again.");
        ros::shutdown();
        return 1;
    }

    auto t265_runner = T265Undistorsion(nh, it, param_file_path);
    ros::spin();
}
