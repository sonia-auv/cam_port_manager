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

    private:
        Spinnaker::CameraPtr _pCam;
    };
}