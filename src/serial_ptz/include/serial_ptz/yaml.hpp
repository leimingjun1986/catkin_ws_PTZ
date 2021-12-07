#ifndef  YAML_HPP
#define  YAML_HPP

#include "serial_ptz/serial_ptz.hpp"
#include  <yaml-cpp/yaml.h>
#include  <string>
#include  <iostream>
#include  <ros/ros.h>

using namespace std;
namespace ptz_control_ns
{
//extern SerialParam  serparam;
//通过配置文件名路径导入参数
bool  yaml_import_serial_param(string  configFilePath_,SerialParam&  param){
    try
    {
        YAML::Node  configNode_ = YAML::LoadFile(configFilePath_);   //  YAML::Node
        if(configNode_.IsNull())
          {
              ROS_ERROR("ptz_node load YAML File error!!!!");
              return false ;
          } 
        param.serial_dev = configNode_["serial_param"]["dev_name"].as<string>();
        param.baudrate   = configNode_["serial_param"]["bound_rate"].as<int>();
        param.databit    = configNode_["serial_param"]["databit"].as<int>();
        param.parity     = configNode_["serial_param"]["parity"].as<int>();
        param.timeout    = configNode_["serial_param"]["timeout"].as<int>();
        ROS_INFO("yaml serial param: %s  %d  ",param.serial_dev.c_str(),param.baudrate);
        return true;    
    }
    catch(std::exception& e){
        ROS_ERROR_STREAM("ptz_node load YAML File ERROR."<<e.what());
        return false;       
      }
        
  }

 bool  yaml_import_ptz_param(string&  configFilePath_,PTZParam&  param){
    try
    {
        YAML::Node  configNode_ = YAML::LoadFile(configFilePath_);   //  YAML::Node
        if(configNode_.IsNull())
          {
              ROS_ERROR("ptz_node load YAML File error!!!!");
              return false ;
          }   
        param.zoom_limit=configNode_["PTZ_param"]["zoom_limit"].as<int>();
        param.pan_max_limit=configNode_["PTZ_param"]["pan_max_limit"].as<int>();
        param.pan_min_limit=configNode_["PTZ_param"]["pan_min_limit"].as<int>();
        param.angle_unit=configNode_["PTZ_param"]["angle_unit"].as<int>();
        param.tilt_down_limit=configNode_["PTZ_param"]["tilt_down_limit"].as<int>();
        param.tilt_up_limit=configNode_["PTZ_param"]["tilt_up_limit"].as<int>();
        ROS_INFO("PTZ param: %d  %d %d %d %d %d",param.zoom_limit,param.angle_unit,param.pan_max_limit,param.pan_min_limit,param.tilt_down_limit,param.tilt_up_limit);
        return true;    
    }
    catch(std::exception& e){
        ROS_ERROR_STREAM("ptz_node load YAML File ERROR."<<e.what());
        return false;       
      }
        
  }

}

#endif