#pragma once

#include <rclcpp/rclcpp.hpp>
#include "cam_port_manager/Camera.hpp"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <condition_variable>
#include <mutex>

namespace cam_port_manager
{
    class CaptureNode : public rclcpp::Node
    {
    public:
        CaptureNode();
        ~CaptureNode();

        void InitCameras();

    private:
        void _load_params();
        void _load_cameras();
        void _camera_info_setup(const Camera &cam);
        void _get_image_matrix();
        void _export_to_ros();

        void _run();

        inline std::string _save_path() { return this->get_parameter("save_path").as_string(); }
        inline std::vector<int64_t> _cam_ids()
        {
            std::vector<int64_t> id_vec;
            this->get_parameter("cam_ids", id_vec);
            return id_vec;
        }
        inline std::vector<std::string> _cam_aliases() { return this->get_parameter("cam_aliases").as_string_array(); }
        inline int _master_cam_id() { return this->get_parameter("master_cam_id").as_int(); }
        inline int _image_height() { return this->get_parameter("image_height").as_int(); }
        inline int _image_width() { return this->get_parameter("image_width").as_int(); }
        inline std::string _distortion_model() { return this->get_parameter("distortion_model").as_string(); }
        inline int _binning() { return this->get_parameter("binning").as_int(); }
        inline std::vector<double_t> _distortion_coeffs(std::string alias)
        {
            std::vector<double_t> coeffs;
            this->get_parameter("distortion_coeffs." + alias, coeffs);
            return coeffs;
        }
        inline std::vector<double_t> _intrinsic_coeffs(std::string alias)
        {
            std::vector<double_t> coeffs;
            this->get_parameter("intrinsic_coeffs." + alias, coeffs);
            return coeffs;
        }
        inline std::vector<double_t> _rectification_coeffs(std::string alias)
        {
            std::vector<double_t> coeffs;
            this->get_parameter("rectification_coeffs." + alias, coeffs);
            return coeffs;
        }
        inline std::vector<double_t> _projection_coeffs(std::string alias)
        {
            std::vector<double_t> coeffs;
            this->get_parameter("projection_coeffs." + alias, coeffs);
            return coeffs;
        }
        inline int _exposure_time() { return this->get_parameter("exposure_time").as_int(); }
        inline bool _is_max_rate() { return this->get_parameter("is_max_rate").as_bool(); }

        Spinnaker::SystemPtr _pSystem;
        rclcpp::Node::SharedPtr node_handle;
        std::vector<Camera> _camList;
        std::vector<cv::Mat> _cam_frames;
        std::vector<std::string> _timestamps;
        std::vector<sensor_msgs::msg::Image::SharedPtr> _img_msgs;
        std::vector<sensor_msgs::msg::CameraInfo::SharedPtr> _cam_info_msgs;
        image_transport::ImageTransport it;
        std::vector<image_transport::Publisher> _publishers_camera_image;

        std::thread _runner;
        std::mutex _wait_lock;
        std::condition_variable _wait_start;
    };
} // namespace cam_port_manager
