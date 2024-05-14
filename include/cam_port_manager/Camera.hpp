#pragma once
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

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
        Camera(Spinnaker::CameraPtr pCam, int index);
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

        std::string GetID();

        std::string GetAlias() const { return _alias; }
        void SetAlias(std::string alias) { _alias = alias; }

        int GetIndex() const { return _index; }

    private:
        int _index;
        Spinnaker::CameraPtr _pCam;
        std::string _alias;
    };
}