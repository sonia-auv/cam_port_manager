#include "cam_port_manager/Camera.hpp"
#include <boost/log/trivial.hpp>

namespace cam_port_manager
{
    Camera::Camera(Spinnaker::CameraPtr pCam)
    {
        _pCam = pCam;
        if (_pCam->IsInitialized())
        {
            BOOST_LOG_TRIVIAL(warning) << "Camera already initialized. Deinitializing...";
            _pCam->EndAcquisition();
            _pCam->DeInit();
        }
    }

    Camera::~Camera()
    {
        _pCam = NULL;
    }

    void Camera::Init()
    {
        _pCam->Init();
    }

    void Camera::Deinit()
    {
        _pCam->DeInit();
    }

    void Camera::BeginAquisition()
    {
        BOOST_LOG_TRIVIAL(info) << "Begining Aquisition...";
        _pCam->BeginAcquisition();
    }

    void Camera::EndAquisition()
    {
        if (_pCam->GetNumImagesInUse() > 0)
        {
            BOOST_LOG_TRIVIAL(warning) << "Some images still currently in use! Use image->Release() before deinitializing.";
        }
        BOOST_LOG_TRIVIAL(info) << "End Aquisition...";
        _pCam->EndAcquisition();
    }
}
