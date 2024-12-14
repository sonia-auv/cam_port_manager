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

    cv::Mat Camera::GetNextFrame()
    {
        Spinnaker::ImagePtr pImg = _get_next_image();
        Spinnaker::ImagePtr pConvertedImage;

        // if (COLOR_)
        pConvertedImage = pImg->Convert(Spinnaker::PixelFormat_BGR8); //, NEAREST_NEIGHBOR);
        // else
        //     convertedImage = pImage->Convert(PixelFormat_Mono8); //, NEAREST_NEIGHBOR);
        unsigned int XPadding = pConvertedImage->GetXPadding();
        unsigned int YPadding = pConvertedImage->GetYPadding();
        unsigned int rowsize = pConvertedImage->GetWidth();
        unsigned int colsize = pConvertedImage->GetHeight();

        cv::Mat img;
        cv::Mat img2;
        // if (COLOR_)
        img = cv::Mat(colsize + YPadding, rowsize + XPadding, CV_8UC3, pConvertedImage->GetData(), pConvertedImage->GetStride());
        // else
        //     img = Mat(colsize + YPadding, rowsize + XPadding, CV_8UC1, pConvertedImage->GetData(), pConvertedImage->GetStride());

        cv::resize(img, img2, cv::Size(600, 400), cv::INTER_LINEAR);
        return img2.clone();
    }

    std::string Camera::GetEnumValue(std::string name)
    {
        Spinnaker::GenApi::CEnumerationPtr pEnum = _pCam->GetNodeMap().GetNode(name.c_str());
        if (pEnum == nullptr)
        {
            throw std::invalid_argument(("%s Is not an Enum Parameter!", name.c_str()));
        }
        return std::string(pEnum->GetEntry(pEnum->GetIntValue())->GetDisplayName());
    }

    bool Camera::SetEnumValue(std::string name, std::string value)
    {
        Spinnaker::GenApi::CEnumerationPtr pEnum = _pCam->GetNodeMap().GetNode(name.c_str());
        if (pEnum == nullptr)
        {
            BOOST_LOG_TRIVIAL(warning) << "FAILED TO SET Param '" << name << "'. Is not a parameter!";
            return false;
        }
        Spinnaker::GenApi::CEnumEntryPtr pEntry = pEnum->GetEntryByName(value.c_str());
        if (pEntry == nullptr)
        {
            BOOST_LOG_TRIVIAL(warning) << "FAILED TO SET Param '" << name << "'. Is not an entry!";
            return false;
        }
        pEnum->SetIntValue(pEntry->GetValue());
        return true;
    }

    bool Camera::ExecuteCommand(std::string name)
    {
        Spinnaker::GenApi::CCommandPtr pCmd = _pCam->GetNodeMap().GetNode(name.c_str());
        if (pCmd == nullptr)
        {
            BOOST_LOG_TRIVIAL(warning) << "FAILED TO EXECUTE Param '" << name << "'. Is not a Command!";
            return false;
        }
        pCmd->Execute();
        return true;
    }

    float Camera::GetFloatValue(std::string name)
    {
        Spinnaker::GenApi::CFloatPtr pFloat = _pCam->GetNodeMap().GetNode(name.c_str());
        if (pFloat == nullptr)
        {
            throw std::invalid_argument(("%s Is not a Float Parameter!", name.c_str()));
        }
        return pFloat->GetValue();
    }

    bool Camera::SetFloatValue(std::string name, float value)
    {
        Spinnaker::GenApi::CFloatPtr pFloat = _pCam->GetNodeMap().GetNode(name.c_str());
        if (pFloat == nullptr)
        {
            BOOST_LOG_TRIVIAL(warning) << "FAILED TO SET Param '" << name << "'. Is not a parameter!";
            return false;
        }
        pFloat->SetValue(value);
        return true;
    }

    int Camera::GetIntValue(std::string name)
    {
        Spinnaker::GenApi::CIntegerPtr pInt = _pCam->GetNodeMap().GetNode(name.c_str());
        if (pInt == nullptr)
        {
            throw std::invalid_argument(("%s Is not an Integer Parameter!", name.c_str()));
        }
        return pInt->GetValue();
    }

    bool Camera::SetIntValue(std::string name, int value)
    {
        Spinnaker::GenApi::CIntegerPtr pInt = _pCam->GetNodeMap().GetNode(name.c_str());
        if (pInt == nullptr)
        {
            BOOST_LOG_TRIVIAL(warning) << "FAILED TO SET Param '" << name << "'. Is not a parameter!";
            return false;
        }
        pInt->SetValue(value);
        return true;
    }

    bool Camera::GetBoolValue(std::string name)
    {
        Spinnaker::GenApi::CBooleanPtr pBool = _pCam->GetNodeMap().GetNode(name.c_str());
        if (pBool == nullptr)
        {
            throw std::invalid_argument(("%s Is not a Boolean Parameter!", name.c_str()));
        }
        return pBool->GetValue();
    }

    bool Camera::SetBoolValue(std::string name, bool value)
    {
        Spinnaker::GenApi::CBooleanPtr pBool = _pCam->GetNodeMap().GetNode(name.c_str());
        if (pBool == nullptr)
        {
            BOOST_LOG_TRIVIAL(warning) << "FAILED TO SET Param '" << name << "'. Is not a parameter!";
            return false;
        }
        pBool->SetValue(value);
    }

    std::string Camera::GetStringValue(std::string name)
    {
        Spinnaker::GenApi::CStringPtr pString = _pCam->GetNodeMap().GetNode(name.c_str());
        if (pString == nullptr)
        {
            throw std::invalid_argument(("%s Is not a String Parameter!", name.c_str()));
        }
        return std::string(pString->GetValue());
    }

    std::string Camera::GetID()
    {
        std::string camera_id = std::string(_pCam->GetUniqueID());
        std::string serial_nb = "SRL_";

        size_t position = camera_id.find(serial_nb) + 4;

        serial_nb = camera_id.substr(position, 8);

        return std::to_string(stoi(serial_nb, nullptr, 16));
    }

    Spinnaker::ImagePtr Camera::_get_next_image()
    {
        Spinnaker::ImagePtr pImg = _pCam->GetNextImage();

        if (pImg->IsIncomplete())
        {
            BOOST_LOG_TRIVIAL(warning) << "Image incomplete with image status " << pImg->GetImageStatus() << "!";
        }

        return pImg;
    }
}
