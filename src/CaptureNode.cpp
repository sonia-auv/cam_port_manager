#include "cam_port_manager/CaptureNode.hpp"
#include <fstream>
#include <rcpputils/asserts.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/filesystem.hpp>
#include <cv_bridge/cv_bridge.h>
#include "image_transport/image_transport.hpp"

namespace cam_port_manager
{
    CaptureNode::CaptureNode()
        : rclcpp::Node("cam_port_manager"), _camList(), _publishers_camera_image(),node_handle(std::shared_ptr<CaptureNode>(this, [](auto *) {})), it(node_handle)
    {
        int mem;
        std::ifstream usb_mem("/sys/module/usbcore/parameters/usbfs_memory_mb");
        if (usb_mem)
        {
            usb_mem >> mem;
            if (mem >= 1000)
            {
                RCLCPP_INFO(this->get_logger(), "[ OK ] USB memory: %d MB", mem);
            }
            else
            {
                RCLCPP_FATAL(this->get_logger(), "USB memory on system too low (%d MB)! Must be atleast 1000 MB. Run: \nsudo sh -c \"echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb\"\n Terminating..", mem);
                this->~CaptureNode();
                return;
            }
        }
        else
        {
            RCLCPP_FATAL(this->get_logger(), "Could not check USB memory on system! Terminating...");
            this->~CaptureNode();
            return;
        }

        _load_params();

        RCLCPP_INFO(this->get_logger(), "Creating System instance...");
        _pSystem = Spinnaker::System::GetInstance();
        _load_cameras();

        _runner = std::thread(std::bind(&CaptureNode::_run, this));
    }

    CaptureNode::~CaptureNode()
    {
        if (_pSystem != nullptr)
        {
            _pSystem->ReleaseInstance();
        }
        for (Camera cam : _camList)
        {
            cam.~Camera();
        }
    }

    void CaptureNode::InitCameras()
    {
        for (Camera cam : _camList)
        {
            RCLCPP_INFO(this->get_logger(), "Initializing Camera %s", cam.GetAlias().c_str());
            cam.Init();
            cam.BeginAquisition();
            cam.EndAquisition();
            cam.Deinit();
            cam.Init();
            try
            {
                cam.SetEnumValue("ExposureMode", "Timed");
                RCLCPP_INFO(this->get_logger(), "Param 'ExposureMode' = %s", cam.GetEnumValue("ExposureMode").c_str());
                if (_exposure_time() > 0)
                {
                    cam.SetEnumValue("ExposureAuto", "Off");
                    RCLCPP_INFO(this->get_logger(), "Param 'ExposureAuto' = %s", cam.GetEnumValue("ExposureAuto").c_str());
                    cam.SetFloatValue("ExposureTime", _exposure_time());
                    RCLCPP_INFO(this->get_logger(), "Param 'ExposureTime' = %s", cam.GetEnumValue("ExposureTime").c_str());
                }
                else
                {
                    cam.SetEnumValue("ExposureAuto", "Continuous");
                    RCLCPP_INFO(this->get_logger(), "Param 'ExposureAuto' = %s", cam.GetEnumValue("ExposureAuto").c_str());
                }
                cam.SetEnumValue("PixelFormat", "BayerRG8");
                RCLCPP_INFO(this->get_logger(), "Param 'PixelFormat' = %s", cam.GetEnumValue("PixelFormat").c_str());

                cam.SetEnumValue("AcquisitionMode", "Continuous");
                RCLCPP_INFO(this->get_logger(), "Param 'AcquisitionMode' = %s", cam.GetEnumValue("AcquisitionMode").c_str());

                if (cam.IsMaster())
                {
                    cam.SetEnumValue("LineSelector", "Line2");
                    RCLCPP_INFO(this->get_logger(), "Param 'LineSelector' = %s", cam.GetEnumValue("LineSelector").c_str());
                    cam.SetEnumValue("LineMode", "Output");
                    RCLCPP_INFO(this->get_logger(), "Param 'LineMode' = %s", cam.GetEnumValue("LineMode").c_str());

                    if (_is_max_rate())
                    {
                        cam.SetBoolValue("AcquisitionFrameRateEnable", false);
                        RCLCPP_INFO(this->get_logger(), "Param 'AcquisitionFrameRateEnable' = %s", cam.GetBoolValue("AcquisitionFrameRateEnable") ? "true" : "false");
                    }
                    else
                    {
                        cam.SetEnumValue("TriggerMode", "On");
                        RCLCPP_INFO(this->get_logger(), "Param 'TriggerMode' = %s", cam.GetEnumValue("TriggerMode").c_str());
                        cam.SetEnumValue("TriggerSource", "Software");
                        RCLCPP_INFO(this->get_logger(), "Param 'TriggerSource' = %s", cam.GetEnumValue("TriggerSource").c_str());
                    }
                }
                else
                {
                    cam.SetEnumValue("LineSelector", "Line3");
                    RCLCPP_INFO(this->get_logger(), "Param 'LineSelector' = %s", cam.GetEnumValue("LineSelector").c_str());
                    cam.SetEnumValue("LineMode", "Output");
                    RCLCPP_INFO(this->get_logger(), "Param 'LineMode' = %s", cam.GetEnumValue("LineMode").c_str());
                    cam.SetEnumValue("TriggerSource", "Software");
                    RCLCPP_INFO(this->get_logger(), "Param 'TriggerSource' = %s", cam.GetEnumValue("TriggerSource").c_str());
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "SOMETHING WENT WRONG WHILE SETTING THE PARAMETERS!!!\n\t%s", e.what());
            }
        }
        _wait_start.notify_all();
    }

    void CaptureNode::_load_params()
    {
        { // save_path
            this->declare_parameter("save_path", rclcpp::PARAMETER_STRING);
            std::string save_path;
            try
            {
                save_path = _save_path();
            }
            catch (const rclcpp::exceptions::ParameterUninitializedException &e)
            {
                boost::filesystem::path canonicalPath = boost::filesystem::canonical(".", boost::filesystem::current_path());
                RCLCPP_WARN(this->get_logger(), "%s, setting to default", e.what());
                this->set_parameter(rclcpp::Parameter("save_path", (canonicalPath.string().back() != '/') ? canonicalPath.string() + '/' : canonicalPath.string()));
                save_path = _save_path();
            }
            RCLCPP_INFO(this->get_logger(), "Param Loaded: 'save_path' %s", save_path.c_str());
        }

        { // cam_ids
            this->declare_parameter("cam_ids", rclcpp::PARAMETER_INTEGER_ARRAY);
            std::vector<int64_t> ids = _cam_ids();
            RCLCPP_INFO(this->get_logger(), "Param Loaded: 'cam_ids'");
            for (uint8_t i = 0; i < ids.size(); i++)
            {
                RCLCPP_INFO(this->get_logger(), "\tcam_id %d : %ld", i, ids.at(i));
            }
        }

        { // cam_aliases
            auto ids = _cam_ids();
            this->declare_parameter("cam_aliases", rclcpp::PARAMETER_STRING_ARRAY);
            std::vector<std::string> aliases;
            try
            {
                aliases = _cam_aliases();
            }
            catch (const rclcpp::exceptions::ParameterUninitializedException &e)
            {
                RCLCPP_WARN(this->get_logger(), "Param not found: cam_aliases, using ids as alias");
                std::vector<std::string> ids_as_aliases;
                for (uint8_t i = 0; i < ids.size(); i++)
                {
                    ids_as_aliases.push_back(std::to_string(ids.at(i)).c_str());
                }
                this->set_parameter(rclcpp::Parameter("cam_aliases", ids_as_aliases));
                aliases = _cam_aliases();
            }

            RCLCPP_INFO(this->get_logger(), "Param Loaded: 'cam_aliases'");
            for (uint8_t i = 0; i < aliases.size(); i++)
            {
                RCLCPP_INFO(this->get_logger(), "\tcam_alias %d : %s", i, aliases.at(i).c_str());
            }
        }

        { // master_cam_id
            this->declare_parameter("master_cam_id", rclcpp::PARAMETER_INTEGER);
            RCLCPP_INFO(this->get_logger(), "Param Loaded: 'master_cam_id' %d", _master_cam_id());
        }

        { // image_height
            this->declare_parameter("image_height", rclcpp::PARAMETER_INTEGER);
            RCLCPP_INFO(this->get_logger(), "Param Loaded: 'image_height' %d", _image_height());
        }

        { // image_width
            this->declare_parameter("image_width", rclcpp::PARAMETER_INTEGER);
            RCLCPP_INFO(this->get_logger(), "Param Loaded: 'image_width' %d", _image_width());
        }

        { // distortion model
            this->declare_parameter("distortion_model", rclcpp::PARAMETER_STRING);
            RCLCPP_INFO(this->get_logger(), "Param Loaded: 'distortion_model' %s", _distortion_model().c_str());
        }

        { // binning
            this->declare_parameter("binning", rclcpp::PARAMETER_INTEGER);
            RCLCPP_INFO(this->get_logger(), "Param Loaded: 'binning' %d", _binning());
        }

        { // distortion_coeffs
            RCLCPP_INFO(this->get_logger(), "Param Loaded: 'distortion_config'");
            for (std::string alias : _cam_aliases())
            {
                this->declare_parameter("distortion_coeffs." + alias, rclcpp::PARAMETER_DOUBLE_ARRAY);
                std::vector<double> coeffs = _distortion_coeffs(alias);

                std::string s = "[ ";
                for (size_t j = 0; j < coeffs.size(); j++)
                {
                    s.append(std::to_string(coeffs.at(j)) + " ");
                }
                s.append("]");
                RCLCPP_INFO(this->get_logger(), "\t- %s : %s", alias.c_str(), s.c_str());
            }
        }

        { // intrinsic_coeffs
            RCLCPP_INFO(this->get_logger(), "Param Loaded: 'instrinsic_config'");
            for (std::string alias : _cam_aliases())
            {
                this->declare_parameter("instrinsic_config." + alias, rclcpp::PARAMETER_DOUBLE_ARRAY);
                std::vector<double> coeffs = _distortion_coeffs(alias);

                std::string s = "[ ";
                for (size_t j = 0; j < coeffs.size(); j++)
                {
                    s.append(std::to_string(coeffs.at(j)) + " ");
                }
                s.append("]");
                RCLCPP_INFO(this->get_logger(), "\t- %s : %s", alias.c_str(), s.c_str());
            }
        }

        { // rectification_coeffs
            RCLCPP_INFO(this->get_logger(), "Param Loaded: 'rectification_coeffs'");
            for (std::string alias : _cam_aliases())
            {
                this->declare_parameter("rectification_coeffs." + alias, rclcpp::PARAMETER_DOUBLE_ARRAY);
                std::vector<double> coeffs = _rectification_coeffs(alias);

                std::string s = "[ ";
                for (size_t j = 0; j < coeffs.size(); j++)
                {
                    s.append(std::to_string(coeffs.at(j)) + " ");
                }
                s.append("]");
                RCLCPP_INFO(this->get_logger(), "\t- %s : %s", alias.c_str(), s.c_str());
            }
        }

        { // projection_coeffs
            RCLCPP_INFO(this->get_logger(), "Param Loaded: 'projection_coeffs'");
            for (std::string alias : _cam_aliases())
            {
                this->declare_parameter("projection_coeffs." + alias, rclcpp::PARAMETER_DOUBLE_ARRAY);
                std::vector<double> coeffs = _projection_coeffs(alias);

                std::string s = "[ ";
                for (size_t j = 0; j < coeffs.size(); j++)
                {
                    s.append(std::to_string(coeffs.at(j)) + " ");
                }
                s.append("]");
                RCLCPP_INFO(this->get_logger(), "\t- %s : %s", alias.c_str(), s.c_str());
            }
        }

        { // exposure_time
            this->declare_parameter("exposure_time", rclcpp::PARAMETER_INTEGER);
            RCLCPP_INFO(this->get_logger(), "Param Loaded: 'exposure_time' %d", _exposure_time());
        }

        { // is_max_rate
            this->declare_parameter("is_max_rate", rclcpp::PARAMETER_BOOL);
            RCLCPP_INFO(this->get_logger(), "Param Loaded: 'is_max_rate' %d", _is_max_rate());
        }
    }

    void CaptureNode::_load_cameras()
    {
        RCLCPP_INFO(this->get_logger(), "Retrieving List of Cameras...");
        Spinnaker::CameraList system_cameras = _pSystem->GetCameras();

        if (system_cameras.GetSize() == 0)
        {
            RCLCPP_FATAL(this->get_logger(), "No Cameras Found... Shutting down");
            this->~CaptureNode();
        }
        RCLCPP_INFO(this->get_logger(), "Number of Cameras Found: %d", system_cameras.GetSize());

        std::vector<int64_t> my_ids = _cam_ids();
        bool default_detected = false;

        //image_transport::ImageTransport it((rclcpp::Node::SharedPtr)this);
        //image_transport::ImageTransport it(this->shared_from_this());


        for (size_t i = 0; i < my_ids.size(); i++)
        {
            bool cam_found = false;
            for (size_t j = 0; j < system_cameras.GetSize(); j++)
            {
                Camera cam(system_cameras.GetByIndex(j));
                if (std::to_string(my_ids.at(i)) == cam.GetID())
                {
                    cam.SetAlias(_cam_aliases().at(i));
                    _camList.push_back(cam);
                    if (cam.GetID() == std::to_string(_master_cam_id()))
                    {
                        default_detected = true;
                        cam.SetMaster();
                    }

                    RCLCPP_INFO(this->get_logger(), "\t- %s", cam.GetID().c_str());

                    cam_found = true;

                    _publishers_camera_image.push_back(it.advertise("/camera_array/" + cam.GetAlias() + "/image_raw", 10,"compressed"));

                    cv::Mat img;
                    _cam_frames.push_back(img);

                    _img_msgs.push_back(sensor_msgs::msg::Image::SharedPtr());
                    _camera_info_setup(cam);
                    break;
                }
            }
            if (!cam_found)
            {
                RCLCPP_WARN(this->get_logger(), "\t- %ld NOT FOUND", my_ids.at(i));
            }
        }
        if (!default_detected)
        {
            RCLCPP_ERROR(this->get_logger(), "Default Camera not detected!!");
        }
    }

    void CaptureNode::_camera_info_setup(const Camera &cam)
    {
        sensor_msgs::msg::CameraInfo::SharedPtr ci_msg(new sensor_msgs::msg::CameraInfo());
        ci_msg->header.frame_id = "cam_" + cam.GetAlias() + "_optical_frame";
        ci_msg->height = _image_height();
        ci_msg->width = _image_width();
        ci_msg->distortion_model = _distortion_model();
        ci_msg->binning_x = _binning();
        ci_msg->binning_y = _binning();

        if (!_distortion_coeffs(cam.GetAlias()).empty() && !_intrinsic_coeffs(cam.GetAlias()).empty())
        {
            ci_msg->d = _distortion_coeffs(cam.GetAlias());
            for (size_t i = 0; i < _distortion_coeffs(cam.GetAlias()).size(); i++)
            {
                ci_msg->k[i] = _distortion_coeffs(cam.GetAlias()).at(i);
            }

            if (!_rectification_coeffs(cam.GetAlias()).empty())
            {
                ci_msg->r = {
                    _rectification_coeffs(cam.GetAlias())[0],
                    _rectification_coeffs(cam.GetAlias())[1],
                    _rectification_coeffs(cam.GetAlias())[2],
                    _rectification_coeffs(cam.GetAlias())[3],
                    _rectification_coeffs(cam.GetAlias())[4],
                    _rectification_coeffs(cam.GetAlias())[5],
                    _rectification_coeffs(cam.GetAlias())[6],
                    _rectification_coeffs(cam.GetAlias())[7],
                    _rectification_coeffs(cam.GetAlias())[8]};
            }
            if (!_projection_coeffs(cam.GetAlias()).empty())
            {
                ci_msg->p = {
                    _projection_coeffs(cam.GetAlias())[0],
                    _projection_coeffs(cam.GetAlias())[1],
                    _projection_coeffs(cam.GetAlias())[2],
                    _projection_coeffs(cam.GetAlias())[3],
                    _projection_coeffs(cam.GetAlias())[4],
                    _projection_coeffs(cam.GetAlias())[5],
                    _projection_coeffs(cam.GetAlias())[6],
                    _projection_coeffs(cam.GetAlias())[7],
                    _projection_coeffs(cam.GetAlias())[8],
                    _projection_coeffs(cam.GetAlias())[9],
                    _projection_coeffs(cam.GetAlias())[10],
                    _projection_coeffs(cam.GetAlias())[11],
                };
            }
            else if (_cam_ids().size() == 1)
            {
                ci_msg->p = {
                    _projection_coeffs(cam.GetAlias())[0],
                    _projection_coeffs(cam.GetAlias())[1],
                    _projection_coeffs(cam.GetAlias())[2],
                    0,
                    _projection_coeffs(cam.GetAlias())[3],
                    _projection_coeffs(cam.GetAlias())[4],
                    _projection_coeffs(cam.GetAlias())[5],
                    0,
                    _projection_coeffs(cam.GetAlias())[6],
                    _projection_coeffs(cam.GetAlias())[7],
                    _projection_coeffs(cam.GetAlias())[8],
                    0};
            }
        }

        _cam_info_msgs.push_back(ci_msg);
    }

    void CaptureNode::_get_image_matrix()
    {
        for (size_t i = 0; i < _camList.size(); i++)
        {
            _cam_frames[i] = _camList.at(i).GetNextFrame();
        }
    }

    void CaptureNode::_export_to_ros()
    {
        std_msgs::msg::Header imgHeader;
        imgHeader.stamp = this->get_clock().get()->now();

        for (size_t i = 0; i < _camList.size(); i++)
        {
            imgHeader.frame_id = "cam_" + _cam_aliases().at(i) + "_optical_frame";
            //sensor_msgs::msg::Image img;
            //cv_bridge::CvImage(imgHeader, "bgr8", _cam_frames[i]).toImageMsg(img);
            //_publishers_camera_image[i].publish(img);
            auto img = cv_bridge::CvImage(imgHeader, "bgr8", _cam_frames[i]).toImageMsg();
            _publishers_camera_image[i].publish(*img);
        }
    }

    void CaptureNode::_run()
    {
        std::unique_lock<std::mutex> lk(_wait_lock);
        _wait_start.wait(lk);
        RCLCPP_INFO(this->get_logger(), "Starting Aquisition");
        for (Camera cam : _camList)
        {
            cam.BeginAquisition();
        }
        rclcpp::Rate r(20);
        while (rclcpp::ok())
        {
            _get_image_matrix();
            _export_to_ros();
            r.sleep();
        }
        RCLCPP_INFO(this->get_logger(), "Stop Aquisition");
        for (Camera cam : _camList)
        {
            cam.EndAquisition();
        }
    }
}
