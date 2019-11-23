/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019
 *  Evocortex GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Nuremberg Institute of Technology
 *     Georg Simon Ohm nor the authors names may be used to endorse
 *     or promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Stefan May
 *********************************************************************/

#include "rclcpp/rclcpp.hpp"
#include <image_transport/image_transport.h>

#include "libirimager/ImageBuilder.h"

#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/msg/camera_info.hpp>

#include <optris_drivers2/srv/palette.hpp>

unsigned char*                    _bufferThermal = NULL;
unsigned char*                    _bufferVisible = NULL;
image_transport::CameraPublisher* _pubThermal;
image_transport::CameraPublisher* _pubVisible;
unsigned int                      _frame = 0;

evo::ImageBuilder                 _iBuilder;
std::shared_ptr<rclcpp::Node>     _node;
camera_info_manager::CameraInfoManager* _camera_info_manager = NULL;
rclcpp::Service<optris_drivers2::srv::Palette>::SharedPtr _sPalette = NULL;

void onThermalDataReceive(const sensor_msgs::msg::Image::ConstSharedPtr & image) 
{
  RCLCPP_INFO(_node->get_logger(), "Received new image");

   // check for any subscribers to save computation time
  //if(_pubThermal->getNumSubscribers() == 0)
  //   return;

  unsigned short* data = (unsigned short*)&image->data[0];
  _iBuilder.setData(image->width, image->height, data);

  if(_bufferThermal==NULL)
    _bufferThermal = new unsigned char[image->width * image->height * 3];

  _iBuilder.convertTemperatureToPaletteImage(_bufferThermal, true);

  sensor_msgs::msg::Image img;
  img.header.frame_id = "thermal_image_view";
  img.height 	        = image->height;
  img.width 	        = image->width;
  img.encoding        = "rgb8";
  img.step            = image->width*3;
  img.header.stamp    = _node->now();

  // copy the image buffer
  img.data.resize(img.height*img.step);
  memcpy(&img.data[0], &_bufferThermal[0], img.height * img.step * sizeof(*_bufferThermal));
  
  sensor_msgs::msg::CameraInfo camera_info = _camera_info_manager->getCameraInfo();
  camera_info.header = img.header;
  _pubThermal->publish(img, camera_info);
}

void onVisibleDataReceive(const sensor_msgs::msg::Image::ConstSharedPtr & image) 
{
  // check for any subscribers to save computation time
  //if(_pubVisible->getNumSubscribers() == 0)
  //   return;

  if(_bufferVisible==NULL)
    _bufferVisible = new unsigned char[image->width * image->height * 3];

  const unsigned char* data = &image->data[0];
  _iBuilder.yuv422torgb24(data, _bufferVisible, image->width, image->height);

  sensor_msgs::msg::Image img;
  img.header.frame_id = "visible_image_view";
  img.height          = image->height;
  img.width           = image->width;
  img.encoding        = "rgb8";
  img.step            = image->width*3;
  img.data.resize(img.height*img.step);

  img.header.stamp    = _node->now();

  for(unsigned int i=0; i<image->width*image->height*3; i++) {
    img.data[i] = _bufferVisible[i];
  }

  sensor_msgs::msg::CameraInfo camera_info = _camera_info_manager->getCameraInfo();
  camera_info.header = img.header;
  _pubVisible->publish(img, camera_info);
}

bool onPalette(const std::shared_ptr<rmw_request_id_t> request_header,
               const std::shared_ptr<optris_drivers2::srv::Palette::Request> req,
               const std::shared_ptr<optris_drivers2::srv::Palette::Response> res)
{
  (void) request_header;

  res->success = false;

  if(req->palette > 0 && req->palette < 12)
  {
    _iBuilder.setPalette((evo::EnumOptrisColoringPalette)req->palette);
    res->success = true;
  }

  if(req->palette_scaling >=1 && req->palette_scaling <= 4)
  {
    _iBuilder.setPaletteScalingMethod((evo::EnumOptrisPaletteScalingMethod) req->palette_scaling);
    res->success = true;
  }

  if(_iBuilder.getPaletteScalingMethod() == evo::eManual &&  req->temperature_min < req->temperature_max)
  {
    _iBuilder.setManualTemperatureRange(req->temperature_min, req->temperature_max);
    res->success = true;
  }

  return true;
}

int main (int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("optris_colorconvert_node");
  _node = node;

  int palette = 6;
  //n_.getParam("palette", palette);

  evo::EnumOptrisPaletteScalingMethod scalingMethod = evo::eMinMax;
  /*int sm;
  n_.getParam("paletteScaling", sm);
  if(sm>=1 && sm <=4) scalingMethod = (evo::EnumOptrisPaletteScalingMethod) sm;*/

  _iBuilder.setPaletteScalingMethod(scalingMethod);
  _iBuilder.setPalette((evo::EnumOptrisColoringPalette)palette);

  double tMin     = 20.;
  double tMax     = 40.;

  /*n_.getParam("temperatureMin", tMin);
  n_.getParam("temperatureMax", tMax);*/

  _iBuilder.setManualTemperatureRange((float)tMin, (float)tMax);

  image_transport::ImageTransport it(node);
  auto subThermal = it.subscribe("thermal_image", 1, onThermalDataReceive);
  auto subVisible = it.subscribe("visible_image", 1, onVisibleDataReceive);
  
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  auto pubt = image_transport::create_camera_publisher(node.get(), "thermal_image_view", qos);
  auto pubv = image_transport::create_camera_publisher(node.get(), "visible_image_view", qos);

  _pubThermal = &pubt;
  _pubVisible = &pubv;

  //_sPalette = node->create_service<optris_drivers2::srv::Palette>("palette", onPalette);

  // initialize CameraInfoManager, providing set_camera_info service for geometric calibration
  // see http://wiki.ros.org/camera_info_manager
  camera_info_manager::CameraInfoManager cinfo_manager(node.get());
  _camera_info_manager = &cinfo_manager;

  std::string camera_name;
  std::string camera_info_url;
  node->get_parameter("camera_name", camera_name);
  node->get_parameter("camera_info_url", camera_info_url);

  if (!_camera_info_manager->setCameraName(camera_name))
  {
    // GUID is 16 hex digits, which should be valid.
    // If not, use it for log messages anyway.
    RCLCPP_WARN(node->get_logger(), "[%s] name not valid for camera_info_manger", camera_name);
  }

  if (_camera_info_manager->validateURL(camera_info_url))
  {
    if ( !_camera_info_manager->loadCameraInfo(camera_info_url) )
    {
      RCLCPP_WARN(node->get_logger(), "camera_info_url does not contain calibration data." );
    } 
    else if ( !_camera_info_manager->isCalibrated() )
    {
      RCLCPP_WARN(node->get_logger(), "Camera is not calibrated. Using default values." );
    } 
  } 
  else
  {
    RCLCPP_ERROR_ONCE(node->get_logger(), "Calibration URL syntax is not supported by CameraInfoManager." );
  }

  // set to png compression
  /*std::string key;
  if(ros::param::search("thermal_image/compressed/format", key))
  {
     ros::param::set(key, "png");
  }
  if(ros::param::search("thermal_image/compressed/png_level", key))
  {
     ros::param::set(key, 9);
  }*/

  rclcpp::spin(node);
  rclcpp::shutdown();

  if(_bufferThermal)	delete [] _bufferThermal;
  if(_bufferVisible)  delete [] _bufferVisible;
}
