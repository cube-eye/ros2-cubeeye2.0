#include <sstream>
#include <thread>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/empty.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.h>

// service
#include "cubeeye_camera/srv/last_state.hpp"
#include "cubeeye_camera/srv/last_error.hpp"
#include "cubeeye_camera/srv/scan.hpp"
#include "cubeeye_camera/srv/connect.hpp"
#include "cubeeye_camera/srv/run.hpp"
#include "cubeeye_camera/srv/stop.hpp"
#include "cubeeye_camera/srv/disconnect.hpp"

#include "ProjectDefines.h"
#include "CameraModule.h"
#include "CubeEyeCameraNode.h"

CubeEyeCameraNode::CubeEyeCameraNode() : Node("cubeeye_camera_node"), 
    mLogger(rclcpp::get_logger("camera_node")), mCamera(std::make_shared<CameraModule>())
{
}

void CubeEyeCameraNode::init()
{
    // init services
    mLastStateService = create_service<cubeeye_camera::srv::LastState>("~/get_last_state", 
                            std::bind(&CubeEyeCameraNode::getLastStateServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
    mLastErrorService = create_service<cubeeye_camera::srv::LastError>("~/get_last_error", 
                            std::bind(&CubeEyeCameraNode::getLastErrorServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
    mScanService = create_service<cubeeye_camera::srv::Scan>("~/scan", 
                            std::bind(&CubeEyeCameraNode::getScanServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
    mConnectService = create_service<cubeeye_camera::srv::Connect>("~/connect", 
                            std::bind(&CubeEyeCameraNode::getConnectServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
    mRunService = create_service<cubeeye_camera::srv::Run>("~/run", 
                            std::bind(&CubeEyeCameraNode::getRunServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
    mStopService = create_service<cubeeye_camera::srv::Stop>("~/stop", 
                            std::bind(&CubeEyeCameraNode::getStopServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
    mDisconnectService = create_service<cubeeye_camera::srv::Disconnect>("~/disconnect", 
                            std::bind(&CubeEyeCameraNode::getDisconnectServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void CubeEyeCameraNode::getLastStateServiceCallback(
    const std::shared_ptr<cubeeye_camera::srv::LastState::Request> request,
    std::shared_ptr<cubeeye_camera::srv::LastState::Response>      response)
{
    UNUSED(request);
    response->state = mCamera->getLastState();
} 

void CubeEyeCameraNode::getLastErrorServiceCallback(
    const std::shared_ptr<cubeeye_camera::srv::LastError::Request> request,
    std::shared_ptr<cubeeye_camera::srv::LastError::Response>      response)
{
    UNUSED(request);
    response->error = mCamera->getLastError();
} 

void CubeEyeCameraNode::getScanServiceCallback(const std::shared_ptr<cubeeye_camera::srv::Scan::Request> request,
                                std::shared_ptr<cubeeye_camera::srv::Scan::Response> response)
{
    UNUSED(request);
    RCLCPP_INFO(mLogger, "scan cameras...");
    response->connections = mCamera->scan();
}

void CubeEyeCameraNode::getConnectServiceCallback(const std::shared_ptr<cubeeye_camera::srv::Connect::Request> request,
                                    std::shared_ptr<cubeeye_camera::srv::Connect::Response> response)
{
    RCLCPP_INFO(mLogger, "connect camera(index: %d)", request->index);
    response->result = mCamera->connect(request->index);
    if (response->result != meere::sensor::result::success) {
        RCLCPP_ERROR(mLogger, "camera connection failed.");
        return;
    }

    mCamera->connectTo(this);
}

void CubeEyeCameraNode::getRunServiceCallback(const std::shared_ptr<cubeeye_camera::srv::Run::Request> request,
                                std::shared_ptr<cubeeye_camera::srv::Run::Response> response)
{
    RCLCPP_INFO(mLogger, "run camera(type: %d)", request->type);
    response->result = mCamera->run(request->type);
}

void CubeEyeCameraNode::getStopServiceCallback(const std::shared_ptr<cubeeye_camera::srv::Stop::Request> request,
                                std::shared_ptr<cubeeye_camera::srv::Stop::Response> response)
{
    UNUSED(request);
    RCLCPP_INFO(mLogger, "stop camera");    
    response->result = mCamera->stop();
}

void CubeEyeCameraNode::getDisconnectServiceCallback(const std::shared_ptr<cubeeye_camera::srv::Disconnect::Request> request,
                                        std::shared_ptr<cubeeye_camera::srv::Disconnect::Response> response)
{
    UNUSED(request);
    RCLCPP_INFO(mLogger, "disconnect camera");
    response->result = mCamera->disconnect();
    if (response->result != meere::sensor::result::success) {
        RCLCPP_ERROR(mLogger, "camera disconnection failed.");
        return;
    }

    mCamera->disconnectFrom(this);
}

void CubeEyeCameraNode::autorun(std::string serialNumber, uint32_t frameType) {
    RCLCPP_INFO(mLogger, "autorun (serial number: %s, frametype %d)", serialNumber.c_str(), frameType);

    std::vector<std::string> list = mCamera->scan();

    if (!list.empty()) {
        RCLCPP_INFO(mLogger, "connect camera(serialnumber: %s)", serialNumber.c_str());
        meere::sensor::result _rt = mCamera->connect(serialNumber);
        if (_rt != meere::sensor::result::success) {
            RCLCPP_ERROR(mLogger, "camera connection failed.");
            return;
        }

        mCamera->connectTo(this);

        RCLCPP_INFO(mLogger, "run camera(type: %d)", frameType);
        _rt = mCamera->run(frameType);
        if (_rt != meere::sensor::result::success) {
            RCLCPP_ERROR(mLogger, "mCamera->run() failed");
        }

        RCLCPP_INFO(mLogger, "camera is running.");
    }
    else {
        RCLCPP_ERROR(mLogger, "no source found.");
    }
}

bool CubeEyeCameraNode::shutdown()
{
    RCLCPP_INFO(mLogger, "node shutdown");
    if (mCamera != nullptr) {
        mCamera->shutdown();
    }
    return true;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CubeEyeCameraNode>();
    node->init();

    RCLCPP_INFO(node->get_logger(), "cubeeye camera node started");

    // declare launch parameters
    node->declare_parameter("autorun_onoff", false);
    node->declare_parameter("autorun_serialnumber", "");
    node->declare_parameter("autorun_frametype", 6);

    bool _autorun = node->get_parameter("autorun_onoff").as_bool();
    RCLCPP_INFO(node->get_logger(), "autorun_onoff %d", _autorun);

    if (_autorun) {
        std::string _serialNumber = node->get_parameter("autorun_serialnumber").as_string();
        RCLCPP_INFO(node->get_logger(), "autorun_serialnumber %s", _serialNumber.c_str());

        int _frameType = node->get_parameter("autorun_frametype").as_int();
        RCLCPP_INFO(node->get_logger(), "autorun_frametype %d", _frameType);

        if (!_serialNumber.empty()) {
            node->autorun(_serialNumber, _frameType);
        }
    }

    rclcpp::spin(node);
    rclcpp::shutdown();

    node->shutdown();
    RCLCPP_INFO(node->get_logger(), "cubeeye camera node stopped");
    return 0;
}

