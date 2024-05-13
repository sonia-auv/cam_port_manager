#pragma once

#include <rclcpp/rclcpp.hpp>
#include "cam_port_manager/Camera.hpp"

namespace cam_port_manager
{
    class CaptureNode : public rclcpp::Node
    {
    public:
        CaptureNode();
        ~CaptureNode();

    private:
        void _load_params();

        inline std::string _save_path() { return this->get_parameter("save_path").as_string(); }
        inline std::vector<int64_t> _cam_ids()
        {
            std::vector<int64_t> id_vec;
            this->get_parameter("cam_ids", id_vec);
            return id_vec;
        }
        inline std::vector<std::string> _cam_aliases() { return this->get_parameter("cam_aliases").as_string_array(); }
        inline int _default_cam_id() {return this->get_parameter("default_cam_id").as_int(); }

        Spinnaker::SystemPtr _pSystem;
        Spinnaker::CameraList _camList;
    };
} // namespace cam_port_manager
