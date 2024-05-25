#pragma once
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <opencv2/opencv.hpp>

namespace cam_port_manager
{
    class Camera
    {
    public:
        /**
         * @brief Construct a new Camera object
         *
         * @param pCam
         */
        Camera(Spinnaker::CameraPtr pCam);
        ~Camera();

        /**
         * @brief Initialize the Camera.
         * The Camera needs to be initialized before it can be used.
         */
        void Init();

        /**
         * @brief Deinitize the Camera
         * It is important to deinitialize the cameras when shutting down
         * as they will keep runing otherwise.
         */
        void Deinit();

        void BeginAquisition();

        void EndAquisition();

        cv::Mat GetNextFrame();

        std::string GetEnumValue(std::string name);
        bool SetEnumValue(std::string name, std::string value);

        bool ExecuteCommand(std::string name);

        float GetFloatValue(std::string name);
        bool SetFloatValue(std::string name, float value);

        int GetIntValue(std::string name);
        bool SetIntValue(std::string name, int value);

        bool GetBoolValue(std::string name);
        bool SetBoolValue(std::string name, bool value);

        std::string GetStringValue(std::string name);

        std::string GetID();

        std::string GetAlias() const { return _alias; }
        void SetAlias(std::string alias) { _alias = alias; }

        bool IsMaster() { return _isMaster; };
        void SetMaster() { _isMaster = true; };

    private:
        bool _isMaster = false;
        Spinnaker::ImagePtr _get_next_image();

        Spinnaker::CameraPtr _pCam;
        std::string _alias;
    };
}