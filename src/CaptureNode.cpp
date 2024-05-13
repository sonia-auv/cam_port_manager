#include "cam_port_manager/CaptureNode.hpp"
#include <fstream>
#include <image_transport/image_transport.hpp>
#include <rcpputils/asserts.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/filesystem.hpp>
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

namespace cam_port_manager
{
    CaptureNode::CaptureNode()
        : rclcpp::Node("cam_port_manager")
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
        
        // _load_params();

        RCLCPP_INFO(this->get_logger(), "Creating System instance...");
        _pSystem = System::GetInstance();
        RCLCPP_INFO(this->get_logger(), "Retrieving List of Cameras...");
        _camList = _pSystem->GetCameras();

        RCLCPP_INFO(this->get_logger(), "Number of Cameras Found: %d", _camList.GetSize());
        if (_camList.GetSize() == 0)
        {
            RCLCPP_FATAL(this->get_logger(), "No Cameras Found... Shutting down");
            _pSystem->ReleaseInstance();
            this->~CaptureNode();
        }
        for (size_t i = 0; i < _camList.GetSize(); i++)
        {
            Camera cam(_camList.GetByDeviceID(std::to_string(_cam_ids().at(i))));
        }
    }

    CaptureNode::~CaptureNode()
    {
    }

    void CaptureNode::_load_params()
    {
        this->declare_parameter("save_path", rclcpp::PARAMETER_STRING);
        std::string save_path;
        try
        {
            save_path =_save_path();
        }
        catch (const rclcpp::exceptions::ParameterUninitializedException &e)
        {
            boost::filesystem::path canonicalPath = boost::filesystem::canonical(".", boost::filesystem::current_path());
            RCLCPP_WARN(this->get_logger(), "%s, setting to default", e.what());
            this->set_parameter(rclcpp::Parameter("save_path", (canonicalPath.string().back() != '/') ? canonicalPath.string() + '/' : canonicalPath.string()));
            save_path =_save_path();
        }
        RCLCPP_INFO(this->get_logger(), "Param Loaded: 'save_path' %s", save_path.c_str());

        this->declare_parameter("cam_ids", rclcpp::PARAMETER_INTEGER_ARRAY);
        std::vector<int64_t> ids = _cam_ids();
        RCLCPP_INFO(this->get_logger(), "Param Loaded: 'cam_ids'");
        for (uint8_t i = 0; i < ids.size(); i++)
        {
            RCLCPP_INFO(this->get_logger(), "\tcam_id %d : %ld", i, ids.at(i));
        }

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

        this->declare_parameter("default_cam_id", rclcpp::PARAMETER_INTEGER);
        RCLCPP_INFO(this->get_logger(), "Param Loaded: 'default_cam_id' %d", _default_cam_id());

    }
}
