#ifndef _OPTRISCOLORCONVERT_H_
#define _OPTRISCOLORCONVERT_H_

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <thread>
#include <image_transport/image_transport.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/msg/camera_info.hpp>

#include "libirimager/ImageBuilder.h"
#include <optris_drivers2/srv/palette.hpp>

namespace optris_drivers2
{

/**
 * @class OptrisColorconvert
 * @brief False color conversion class
 * @author Stefan May (Evocortex GmbH)
 */
class OptrisColorconvert : public rclcpp::Node
{
public:

  /**
   * Constructor
   */
  OptrisColorconvert();

  /**
   * Destructor
   */
  virtual ~OptrisColorconvert();

  void onThermalDataReceive(const sensor_msgs::msg::Image::ConstSharedPtr & image);

  void onVisibleDataReceive(const sensor_msgs::msg::Image::ConstSharedPtr & image);

  /**
   * ROS service callback for switching false color palettes
   */
  void onPalette(const std::shared_ptr<rmw_request_id_t> request_header,
                 const std::shared_ptr<optris_drivers2::srv::Palette::Request> req,
                 const std::shared_ptr<optris_drivers2::srv::Palette::Response> res);


private:

  unsigned char*                                            _bufferThermal;
  unsigned char*                                            _bufferVisible;
  image_transport::CameraPublisher                          _pubThermal;
  image_transport::CameraPublisher                          _pubVisible;
  image_transport::Subscriber                               _subThermal;
  image_transport::Subscriber                               _subVisible;
  unsigned int                                              _frame;

  evo::ImageBuilder                                         _iBuilder;
  camera_info_manager::CameraInfoManager                    _camera_info_manager;
  rclcpp::Service<optris_drivers2::srv::Palette>::SharedPtr _sPalette;
  
};

} //namespace

#endif // _OPTRISCOLORCONVERT_H_
