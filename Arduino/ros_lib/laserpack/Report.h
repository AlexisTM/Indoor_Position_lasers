#ifndef _ROS_laserpack_Report_h
#define _ROS_laserpack_Report_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "laserpack/RPYPose.h"
#include "laserpack/Distance.h"
#include "geometry_msgs/Vector3.h"
#include "laserpack/Battery.h"

namespace laserpack
{

  class Report : public ros::Msg
  {
    public:
      std_msgs::Header header;
      laserpack::RPYPose mocap;
      laserpack::RPYPose local;
      laserpack::RPYPose vision;
      laserpack::RPYPose setpoint;
      laserpack::Distance lasers_raw;
      laserpack::RPYPose lasers_pose;
      laserpack::RPYPose laser_filtered;
      geometry_msgs::Vector3 linear_acceleration;
      geometry_msgs::Vector3 linear_speed;
      geometry_msgs::Vector3 angular_speed;
      laserpack::Battery battery;
      bool connected;
      bool armed;
      bool guided;
      const char* mode;

    Report():
      header(),
      mocap(),
      local(),
      vision(),
      setpoint(),
      lasers_raw(),
      lasers_pose(),
      laser_filtered(),
      linear_acceleration(),
      linear_speed(),
      angular_speed(),
      battery(),
      connected(0),
      armed(0),
      guided(0),
      mode("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->mocap.serialize(outbuffer + offset);
      offset += this->local.serialize(outbuffer + offset);
      offset += this->vision.serialize(outbuffer + offset);
      offset += this->setpoint.serialize(outbuffer + offset);
      offset += this->lasers_raw.serialize(outbuffer + offset);
      offset += this->lasers_pose.serialize(outbuffer + offset);
      offset += this->laser_filtered.serialize(outbuffer + offset);
      offset += this->linear_acceleration.serialize(outbuffer + offset);
      offset += this->linear_speed.serialize(outbuffer + offset);
      offset += this->angular_speed.serialize(outbuffer + offset);
      offset += this->battery.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_connected;
      u_connected.real = this->connected;
      *(outbuffer + offset + 0) = (u_connected.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->connected);
      union {
        bool real;
        uint8_t base;
      } u_armed;
      u_armed.real = this->armed;
      *(outbuffer + offset + 0) = (u_armed.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->armed);
      union {
        bool real;
        uint8_t base;
      } u_guided;
      u_guided.real = this->guided;
      *(outbuffer + offset + 0) = (u_guided.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->guided);
      uint32_t length_mode = strlen(this->mode);
      memcpy(outbuffer + offset, &length_mode, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->mode, length_mode);
      offset += length_mode;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->mocap.deserialize(inbuffer + offset);
      offset += this->local.deserialize(inbuffer + offset);
      offset += this->vision.deserialize(inbuffer + offset);
      offset += this->setpoint.deserialize(inbuffer + offset);
      offset += this->lasers_raw.deserialize(inbuffer + offset);
      offset += this->lasers_pose.deserialize(inbuffer + offset);
      offset += this->laser_filtered.deserialize(inbuffer + offset);
      offset += this->linear_acceleration.deserialize(inbuffer + offset);
      offset += this->linear_speed.deserialize(inbuffer + offset);
      offset += this->angular_speed.deserialize(inbuffer + offset);
      offset += this->battery.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_connected;
      u_connected.base = 0;
      u_connected.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->connected = u_connected.real;
      offset += sizeof(this->connected);
      union {
        bool real;
        uint8_t base;
      } u_armed;
      u_armed.base = 0;
      u_armed.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->armed = u_armed.real;
      offset += sizeof(this->armed);
      union {
        bool real;
        uint8_t base;
      } u_guided;
      u_guided.base = 0;
      u_guided.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->guided = u_guided.real;
      offset += sizeof(this->guided);
      uint32_t length_mode;
      memcpy(&length_mode, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_mode; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_mode-1]=0;
      this->mode = (char *)(inbuffer + offset-1);
      offset += length_mode;
     return offset;
    }

    const char * getType(){ return "laserpack/Report"; };
    const char * getMD5(){ return "17c64fbd685fe18dda1c70e1ebc32f4f"; };

  };

}
#endif