#include "cam_port_manager/Camera.hpp"
#include <boost/log/trivial.hpp>

namespace cam_port_manager
{
    Camera::Camera(Spinnaker::CameraPtr pCam, int index) : _index(index)
    {
        _pCam = pCam;
        if (_pCam->IsInitialized())
        {
            BOOST_LOG_TRIVIAL(warning) << "Camera already initialized. Deinitializing...";
            _pCam->EndAcquisition();
            _pCam->DeInit();
        }
        _alias = GetID();
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

    std::string Camera::GetID()
    {
        std::string camera_id = std::string(_pCam->GetUniqueID());
        std::string serial_nb = "SRL_";

        size_t position = camera_id.find(serial_nb) + 4;

        serial_nb = camera_id.substr(position, 8);

        return std::to_string(stoi(serial_nb, nullptr, 16));
    }
}
