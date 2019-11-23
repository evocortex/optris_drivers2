#include "OptrisImager.h"


#include <chrono>

namespace optris_drivers2
{

OptrisImager::OptrisImager(evo::IRDevice* dev, evo::IRDeviceParams params) : Node("optris_imager")
{

  RCLCPP_INFO(get_logger(), "Serial: %d", params.serial);

  _imager.init(&params, dev->getFrequency(), dev->getWidth(), dev->getHeight(), dev->controlledViaHID());
  _imager.setClient(this);

  _bufferRaw = new unsigned char[dev->getRawBufferSize()];

  auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(
        // The history policy determines how messages are saved until taken by
        // the reader.
        // KEEP_ALL saves all messages until they are taken.
        // KEEP_LAST enforces a limit on the number of messages that are saved,
        // specified by the "depth" parameter.
        RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        // The next parameter represents how many messages to store in history when the
        // history policy is KEEP_LAST.
        1
    ));

  // The reliability policy can be reliable, meaning that the underlying transport layer will try
  // ensure that every message gets received in order, or best effort, meaning that the transport
  // makes no guarantees about the order or reliability of delivery.
  // Options are: SYSTEM_DEFAULT, RELIABLE, BEST_EFFORT and UNKNOWN
  rmw_qos_reliability_policy_t reliability_policy = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
  qos.reliability(reliability_policy);

  _thermal_pub = this->create_publisher<sensor_msgs::msg::Image>("thermal_image", qos);
  _thermal_image.header.frame_id = "thermal_image";
  _thermal_image.height          = _imager.getHeight();
  _thermal_image.width           = _imager.getWidth();
  _thermal_image.encoding        = "mono16";
  _thermal_image.step            = _thermal_image.width * 2;
  _thermal_image.data.resize(_thermal_image.height * _thermal_image.step);

  if(_imager.hasBispectralTechnology())
  {
    _visible_pub = this->create_publisher<sensor_msgs::msg::Image>("visible_image", qos);
    _visible_image.header.frame_id = "visible_image";
    _visible_image.height          = _imager.getVisibleHeight();
    _visible_image.width           = _imager.getVisibleWidth();
    _visible_image.encoding        = "yuv422";
    _visible_image.step            = _visible_image.width * 2;
    _visible_image.data.resize(_visible_image.height * _visible_image.step);
  }

  // advertise the camera internal timer
  _timer_pub = this->create_publisher<sensor_msgs::msg::TimeReference>("optris_timer", qos);

  // advertise the internal temperature measurements
  _temp_pub = this->create_publisher<optris_drivers2::msg::Temperature> ("internal_temperature", qos);

  // advertise the flag state
  _flag_pub = this->create_publisher<optris_drivers2::msg::Flag> ("flag_state", qos);

  // provide AutoFlag service
  _sAuto  = this->create_service<optris_drivers2::srv::AutoFlag> ("auto_flag", std::bind(&OptrisImager::onAutoFlag, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // provide service to force a flag cycle manually
  _sForce = this->create_service<std_srvs::srv::Empty> ("force_flag", std::bind(&OptrisImager::onForceFlag, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // provide service to change temperature range
  _sTemp  = this->create_service<optris_drivers2::srv::TemperatureRange> ("temperature_range", std::bind(&OptrisImager::onSetTemperatureRange, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  _img_cnt = 0;

  _dev = dev;

  _dev->startStreaming();

  // create_wall_timer changes the behaviour of spin. Services will stop working
  _run = true;
  _th = new std::thread(&OptrisImager::timer_callback, this);

  return;
}

OptrisImager::~OptrisImager()
{
  _run = false;
  _th->join();
  _dev->stopStreaming();

  delete [] _bufferRaw;
}

void OptrisImager::timer_callback()
{
  auto durInSec = std::chrono::duration<double>(1.0/_imager.getMaxFramerate());
  RCLCPP_INFO(get_logger(), "Sampling rate: %lf sec", 1.0/_imager.getMaxFramerate());
  while(_run)
  {
    int retval = _dev->getFrame(_bufferRaw);
    if(retval==evo::IRIMAGER_SUCCESS)
    {
      _imager.process(_bufferRaw);
    }
    if(retval==evo::IRIMAGER_DISCONNECTED)
    {
      rclcpp::shutdown();
    }
    std::this_thread::sleep_for(durInSec);
  }
}

void OptrisImager::onThermalFrame(unsigned short* image, unsigned int w, unsigned int h, evo::IRFrameMetadata meta, void* arg)
{
  (void) arg;

  RCLCPP_INFO(get_logger(), "onThermalFrame");
  memcpy(&_thermal_image.data[0], image, w * h * sizeof(*image));

  _thermal_image.header.frame_id = "";
  _thermal_image.header.stamp = rclcpp::Node::now();
  _thermal_pub->publish(_thermal_image);

  _device_timer.header.frame_id=_thermal_image.header.frame_id;
  _device_timer.header.stamp = _thermal_image.header.stamp;
  //TODO: Check validity of timestamp
  _device_timer.time_ref.sec = (int32_t) (meta.timestamp / 10000000);
  _device_timer.time_ref.nanosec = (int32_t) (meta.timestamp % 10000000);
  _timer_pub->publish(_device_timer);

  _internal_temperature.header.stamp     = _thermal_image.header.stamp;
  _internal_temperature.header.frame_id  = _thermal_image.header.frame_id;
  _internal_temperature.temperature_flag = _imager.getTempFlag();
  _internal_temperature.temperature_box  = _imager.getTempBox();
  _internal_temperature.temperature_chip = _imager.getTempChip(); 
  _temp_pub->publish(_internal_temperature);
  
}

void OptrisImager::onVisibleFrame(unsigned char* image, unsigned int w, unsigned int h, evo::IRFrameMetadata meta, void* arg)
{
  (void) arg;
  (void) meta;

  if(_visible_pub->get_subscription_count()==0) return;

  memcpy(&_visible_image.data[0], image, 2 * w * h * sizeof(*image));

  _visible_image.header.frame_id  = _img_cnt;
  _visible_image.header.stamp = rclcpp::Node::now();
  _visible_pub->publish(_visible_image);
}

void OptrisImager::onFlagStateChange(evo::EnumFlagState flagstate, void* arg)
{
  (void) arg;

  optris_drivers2::msg::Flag flag;
  flag.flag_state      = flagstate;
  flag.header.frame_id = _thermal_image.header.frame_id;
  flag.header.stamp    = _thermal_image.header.stamp;
  _flag_pub->publish(flag);
}

void OptrisImager::onProcessExit(void* arg)
{
  (void) arg;
}

void OptrisImager::onAutoFlag(const std::shared_ptr<rmw_request_id_t> request_header,
                              const std::shared_ptr<optris_drivers2::srv::AutoFlag::Request> req,
                              const std::shared_ptr<optris_drivers2::srv::AutoFlag::Response> res)
{
  (void) request_header;
  RCLCPP_INFO(get_logger(), "Calling service on_auto_flag");
  _imager.setAutoFlag(req->auto_flag);
  res->is_auto_flag_active = _imager.getAutoFlag();
}

void OptrisImager::onForceFlag(const std::shared_ptr<rmw_request_id_t> request_header,
                               const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                               const std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
  (void) request_header;
  (void) req;
  (void) res;
  _imager.forceFlagEvent();
}

void OptrisImager::onSetTemperatureRange(const std::shared_ptr<rmw_request_id_t> request_header,
                                         const std::shared_ptr<optris_drivers2::srv::TemperatureRange::Request> req,
                                         const std::shared_ptr<optris_drivers2::srv::TemperatureRange::Response> res)
{
  (void) request_header;

  bool validParam = _imager.setTempRange(req->temperature_range_min, req->temperature_range_max);

  if(validParam)
  {
    _imager.forceFlagEvent(1000.f);
  }

  res->success = validParam;
}

}
