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

#include "ProjectDefines.h"
#include "CameraModule.h"

class ReceivedIntensityPCLFrameSink : public meere::sensor::sink
 , public meere::sensor::prepared_listener
{
public:
    ReceivedIntensityPCLFrameSink(CameraModule* node) : mNode(node) {}
    virtual ~ReceivedIntensityPCLFrameSink() = default;

    virtual std::string name() const {
        return std::string("ReceivedIntensityPCLFrameSink");
    }

    virtual void onCubeEyeCameraState(const meere::sensor::ptr_source source, meere::sensor::CameraState state) {
        RCLCPP_DEBUG(mNode->mLogger, "%s:%d source(%s) state = %d\n", __FUNCTION__, __LINE__, source->uri().c_str(), static_cast<int>(state));
        mNode->setLastState((int32_t)state);
    }

    virtual void onCubeEyeCameraError(const meere::sensor::ptr_source source, meere::sensor::CameraError error) {
        RCLCPP_DEBUG(mNode->mLogger, "%s:%d source(%s) error = %d\n", __FUNCTION__, __LINE__, source->uri().c_str(), static_cast<int>(error));
        mNode->setLastError((int32_t)error);
    }

    virtual void onCubeEyeFrameList(const meere::sensor::ptr_source source , const meere::sensor::sptr_frame_list& frames) {
        UNUSED(source);
        mNode->publishFrames(frames);
    }

public:
    virtual void onCubeEyeCameraPrepared(const meere::sensor::ptr_camera camera) {
        RCLCPP_INFO(mNode->mLogger, "%s:%d source(%s)\n", __FUNCTION__, __LINE__, camera->source()->uri().c_str());
    }

public:
    CameraModule* mNode;
};

CameraModule::CameraModule() : mLogger(rclcpp::get_logger("camera_module")), 
        mSink(std::make_shared<ReceivedIntensityPCLFrameSink>(this)), 
        mLastState(0), mLastError(0) {

}

std::vector<std::string> CameraModule::scan() {
    mSourceList = meere::sensor::search_camera_source();
    if (mSourceList == nullptr || mSourceList->size() == 0) {
        RCLCPP_ERROR(mLogger, "no searched device!");
        return std::vector<std::string>();
    }

    std::vector<std::string> connections;

    int i = 0;
    for (auto it : (*mSourceList)) {
        RCLCPP_INFO(mLogger, "%d) source name : %s, serialNumber : %s, uri : %s",
            i++, it->name().c_str(), it->serialNumber().c_str(), it->uri().c_str());

        connections.push_back(it->uri());
    }

    return connections;
}

meere::sensor::result CameraModule::connect(int32_t index) {
    if (mSourceList == nullptr || mSourceList->size() == 0) {
        RCLCPP_ERROR(mLogger, "source list is null");
        return meere::sensor::result::fail;
    }

    if (index >= static_cast<int32_t>(mSourceList->size())) {
        RCLCPP_ERROR(mLogger, "wrong source index");
        return meere::sensor::result::fail;
    }

    meere::sensor::add_prepared_listener(mSink.get());

    // create camera
    mCamera = meere::sensor::create_camera(mSourceList->at(index));
    if (mCamera == nullptr) {
        RCLCPP_ERROR(mLogger, "camera creation failed");
        return meere::sensor::result::fail;
    }

    mCamera->addSink(mSink.get());

    meere::sensor::result _rt;
    _rt = mCamera->prepare();
    assert(meere::sensor::result::success == _rt);
    if (meere::sensor::result::success != _rt) {
        RCLCPP_ERROR(mLogger, "mCamera->prepare() failed");

        meere::sensor::destroy_camera(mCamera);
        return meere::sensor::result::fail;
    }
    RCLCPP_INFO(mLogger, "camera is connected");

    return meere::sensor::result::success;
}

meere::sensor::result CameraModule::connect(std::string serialNumber) {
    if (mSourceList == nullptr || mSourceList->size() == 0) {
        RCLCPP_ERROR(mLogger, "source list is null");
        return meere::sensor::result::fail;
    }

	for (const auto& it : (*mSourceList)) {
		if (it->serialNumber() == serialNumber) {
			mCamera = meere::sensor::create_camera(it);
			break;
		}
	}

	if (!mCamera) {
        RCLCPP_ERROR(mLogger, "camera creation failed");
        return meere::sensor::result::fail;
	}

    meere::sensor::add_prepared_listener(mSink.get());

    mCamera->addSink(mSink.get());

    meere::sensor::result _rt;
    _rt = mCamera->prepare();
    assert(meere::sensor::result::success == _rt);
    if (meere::sensor::result::success != _rt) {
        RCLCPP_ERROR(mLogger, "mCamera->prepare() failed");

        meere::sensor::destroy_camera(mCamera);
        return meere::sensor::result::fail;
    }
    RCLCPP_INFO(mLogger, "camera is connected");

    return meere::sensor::result::success;
}

meere::sensor::result CameraModule::run(int32_t type) {
    if (mCamera == nullptr) {
        RCLCPP_ERROR(mLogger, "camera is not created");
        return meere::sensor::result::fail;
    }

    meere::sensor::result _rt = mCamera->run(type);
    if (_rt != meere::sensor::result::success) {
        RCLCPP_ERROR(mLogger, "mCamera->run() failed");
        return _rt;
    }

    return _rt;
}

meere::sensor::result CameraModule::stop() {
    if (mCamera == nullptr) {
        RCLCPP_ERROR(mLogger, "camera is not created");
        return meere::sensor::result::fail;
    }

    meere::sensor::result _rt = mCamera->stop();
    if (_rt != meere::sensor::result::success) {
        RCLCPP_ERROR(mLogger, "mCamera->stop() failed");
        return _rt;
    }

    return _rt;
}

meere::sensor::result CameraModule::disconnect() {
    if (mCamera == nullptr) {
        RCLCPP_ERROR(mLogger, "camera is not created");
        return meere::sensor::result::fail;
    }

    mCamera->release();

    meere::sensor::remove_prepared_listener(mSink.get());
    meere::sensor::result _rt = meere::sensor::destroy_camera(mCamera);
    mCamera.reset();

    return _rt;
}

void CameraModule::shutdown() {
    if (nullptr != mCamera) {
        mCamera->stop();
        mCamera->release();
        meere::sensor::destroy_camera(mCamera);
    }
}

void CameraModule::connectTo(rclcpp::Node* node) {
    // initialize model
    RCLCPP_INFO(mLogger, "make model parameter (%s)", mCamera->source()->name().c_str());
    mModelParams = ModelParameter::create(mCamera);
    if (mModelParams != nullptr) {
        mModelParams->addTo(node);
    }

    // create publisher
    createPublishers(node);
}

void CameraModule::disconnectFrom(rclcpp::Node* node) {
    if (mModelParams != nullptr) {
        mModelParams->removeFrom(node);
        mModelParams.reset();
    }
}

void CameraModule::publishFrames(const meere::sensor::sptr_frame_list& frames)
{
    for (auto it : (*frames)) {
#if 0
        RCLCPP_DEBUG(mLogger, "frame : %d, "
                "frameWidth = %d "
                "frameHeight = %d "
                "frameDataType = %d "
                "timestamp = %lu \n",
                it->frameType(),
                it->frameWidth(),
                it->frameHeight(),
                it->frameDataType(),
                it->timestamp());
#endif
        // intensity-PointCloud frame
        if (it->frameType() == meere::sensor::FrameType::Depth
            || it->frameType() == meere::sensor::FrameType::Amplitude
            || it->frameType() == meere::sensor::FrameType::RegisteredDepth) {
            if (it->frameDataType() == meere::sensor::DataType::U16) {
                auto _sptr_basic_frame = meere::sensor::frame_cast_basic16u(it);
                auto _sptr_frame_data = _sptr_basic_frame->frameData();	// data array

                sensor_msgs::msg::Image::SharedPtr _msg = createImageMessage(it->frameType(),
                                                                _sptr_basic_frame->frameWidth(), _sptr_basic_frame->frameHeight());
                uint16_t* _data = reinterpret_cast<uint16_t*>(&_msg->data[0]);
                for (int y = 0 ; y < _sptr_basic_frame->frameHeight(); y++) {
                    for (int x = 0 ; x < _sptr_basic_frame->frameWidth(); x++) {
                        int _pos = y * _sptr_basic_frame->frameWidth() + x;
                        _data[_pos] = (*_sptr_frame_data)[_pos];
                    }
                }

                if (it->frameType() == meere::sensor::FrameType::Depth
                    || it->frameType() == meere::sensor::FrameType::RegisteredDepth) {
                    mDepthImagePublisher->publish(*_msg);
                }
                else if (it->frameType() == meere::sensor::FrameType::Amplitude) {
                    mAmplitudeImagePublisher->publish(*_msg);
                }
            }
        }
        else if (it->frameType() == meere::sensor::FrameType::RGB
            || it->frameType() == meere::sensor::FrameType::RegisteredRGB) {

            auto _sptr_frame = meere::sensor::frame_cast_basic8u(it);
			auto _ptr_frame_data = _sptr_frame->frameData();
            
            sensor_msgs::msg::Image::SharedPtr _msg = createImageMessage(it->frameType(),
                                                            _sptr_frame->frameWidth(), _sptr_frame->frameHeight());
            uint8_t* _data = reinterpret_cast<uint8_t*>(&_msg->data[0]);
            for (int y = 0 ; y < _sptr_frame->frameHeight(); y++) {
                for (int x = 0 ; x < _sptr_frame->frameWidth(); x++) {
                    int _pos = y * (_sptr_frame->frameWidth() * 3) + x * 3;
                    _data[_pos] = (*_ptr_frame_data)[_pos];
                    _data[_pos + 1] = (*_ptr_frame_data)[_pos + 1];
                    _data[_pos + 2] = (*_ptr_frame_data)[_pos + 2];
                }
            }

            mRGBImagePublisher->publish(*_msg);
        }
        else if (it->frameType() == meere::sensor::FrameType::PointCloud
            || it->frameType() == meere::sensor::FrameType::RegisteredPointCloud) {
            if (it->frameDataType() == meere::sensor::DataType::F32) {
                auto _sptr_pointcloud_frame = meere::sensor::frame_cast_pcl32f(it);
                auto _sptr_frame_dataX = _sptr_pointcloud_frame->frameDataX();
                auto _sptr_frame_dataY = _sptr_pointcloud_frame->frameDataY();
                auto _sptr_frame_dataZ = _sptr_pointcloud_frame->frameDataZ();

                sensor_msgs::msg::PointCloud2::SharedPtr _pcl_msg = createPointCloudMessage(meere::sensor::FrameType::PointCloud,
                                                                _sptr_pointcloud_frame->frameWidth(), _sptr_pointcloud_frame->frameHeight());

                float* _pcl_data = reinterpret_cast<float*>(&_pcl_msg->data[0]);
                for (int y = 0 ; y < _sptr_pointcloud_frame->frameHeight(); y++) {
                    for (int x = 0 ; x < _sptr_pointcloud_frame->frameWidth(); x++) {
                        int _pos = y * _sptr_pointcloud_frame->frameWidth() + x;
                        int _pcl_pos = _pos * 3;

                        // points
                        _pcl_data[_pcl_pos] = -(*_sptr_frame_dataY)[_pos];
                        _pcl_data[_pcl_pos + 1] = -(*_sptr_frame_dataX)[_pos];
                        _pcl_data[_pcl_pos + 2] = (*_sptr_frame_dataZ)[_pos];
                    }
                }
                mPointCloudPublisher->publish(*_pcl_msg);
            }
        }
        else if (it->frameType() == meere::sensor::FrameType::IntensityPointCloud) {
            if (it->frameDataType() == meere::sensor::DataType::F32) {
                auto _sptr_intensity_pointcloud_frame = meere::sensor::frame_cast_ipcl32f(it);
                auto _sptr_frame_dataX = _sptr_intensity_pointcloud_frame->frameDataX();
                auto _sptr_frame_dataY = _sptr_intensity_pointcloud_frame->frameDataY();
                auto _sptr_frame_dataZ = _sptr_intensity_pointcloud_frame->frameDataZ();
                // auto _sptr_frame_dataI = _sptr_intensity_pointcloud_frame->frameDataI();

                sensor_msgs::msg::PointCloud2::SharedPtr _pcl_msg = createPointCloudMessage(meere::sensor::FrameType::PointCloud,
                                                                _sptr_intensity_pointcloud_frame->frameWidth(), _sptr_intensity_pointcloud_frame->frameHeight());

                float* _pcl_data = reinterpret_cast<float*>(&_pcl_msg->data[0]);
                for (int y = 0 ; y < _sptr_intensity_pointcloud_frame->frameHeight(); y++) {
                    for (int x = 0 ; x < _sptr_intensity_pointcloud_frame->frameWidth(); x++) {
                        int _pos = y * _sptr_intensity_pointcloud_frame->frameWidth() + x;
                        int _pcl_pos = _pos * 3;

                        // points
                        _pcl_data[_pcl_pos] = -(*_sptr_frame_dataY)[_pos];
                        _pcl_data[_pcl_pos + 1] = -(*_sptr_frame_dataX)[_pos];
                        _pcl_data[_pcl_pos + 2] = (*_sptr_frame_dataZ)[_pos];
                    }
                }
                mPointCloudPublisher->publish(*_pcl_msg);
            }
        }
    }    
}

void CameraModule::createPublishers(rclcpp::Node* node)
{
    auto _qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

    mDepthImagePublisher = node->create_publisher<sensor_msgs::msg::Image>("~/depth", _qos);
    mAmplitudeImagePublisher = node->create_publisher<sensor_msgs::msg::Image>("~/amplitude", _qos);
    mRGBImagePublisher = node->create_publisher<sensor_msgs::msg::Image>("~/rgb", _qos);
    mPointCloudPublisher = node->create_publisher<sensor_msgs::msg::PointCloud2>("~/points", _qos);
}

sensor_msgs::msg::Image::SharedPtr CameraModule::createImageMessage(meere::sensor::FrameType type, int32_t width, int32_t height) 
{
    sensor_msgs::msg::Image::SharedPtr _imageMsg;

    if (type == meere::sensor::FrameType::Depth
        || type == meere::sensor::FrameType::Amplitude
        || type == meere::sensor::FrameType::RegisteredDepth) {
        _imageMsg = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_16UC1).toImageMsg();

        switch (type) {
        case meere::sensor::FrameType::Depth:
        case meere::sensor::FrameType::RegisteredDepth:
            _imageMsg->header.frame_id = "depth";
            break;
        case meere::sensor::FrameType::Amplitude:
            _imageMsg->header.frame_id = "amplitude";
            break;
        default:
            _imageMsg->header.frame_id = "depth";
            break;
        }

        _imageMsg->width = width;
        _imageMsg->height = height;
        _imageMsg->is_bigendian = false;
        _imageMsg->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        _imageMsg->step = (uint32_t)(sizeof(uint16_t) * width);
        _imageMsg->data.resize(sizeof(uint16_t) * width * height);
    }
    else if (type == meere::sensor::FrameType::RGB
        || type == meere::sensor::FrameType::RegisteredRGB) {
        _imageMsg = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8).toImageMsg();

        _imageMsg->header.frame_id = "rgb";
        _imageMsg->width = width;
        _imageMsg->height = height;
        _imageMsg->is_bigendian = false;
        _imageMsg->encoding = sensor_msgs::image_encodings::BGR8;
        _imageMsg->step = (uint32_t)(sizeof(uint8_t) * 3 * width);
        _imageMsg->data.resize(sizeof(uint8_t) * 3 * width * height);
    }

    return _imageMsg;
}

sensor_msgs::msg::PointCloud2::SharedPtr CameraModule::createPointCloudMessage(meere::sensor::FrameType type, int32_t width, int32_t height) 
{
    sensor_msgs::msg::PointCloud2::SharedPtr _pclMsg;

    _pclMsg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    _pclMsg->header.frame_id = "pcl";
    _pclMsg->width = width;
    _pclMsg->height = height;
    _pclMsg->is_bigendian = false;
    _pclMsg->is_dense = false;

    _pclMsg->point_step = (uint32_t)(3 * sizeof(float));
    _pclMsg->row_step = (uint32_t)(_pclMsg->point_step * width);
    _pclMsg->fields.resize(3);
    _pclMsg->fields[0].name = "z";
    _pclMsg->fields[0].offset = 0;
    _pclMsg->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    _pclMsg->fields[0].count = 1;

    _pclMsg->fields[1].name = "y";
    _pclMsg->fields[1].offset = _pclMsg->fields[0].offset + (uint32_t)sizeof(float);
    _pclMsg->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    _pclMsg->fields[1].count = 1;

    _pclMsg->fields[2].name = "x";
    _pclMsg->fields[2].offset = _pclMsg->fields[1].offset + (uint32_t)sizeof(float);
    _pclMsg->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    _pclMsg->fields[2].count = 1;
    _pclMsg->data.resize(_pclMsg->point_step * _pclMsg->width * _pclMsg->height);

    return _pclMsg;
}
