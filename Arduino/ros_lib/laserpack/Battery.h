#ifndef _ROS_laserpack_Battery_h
#define _ROS_laserpack_Battery_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace laserpack
{

  class Battery : public ros::Msg
  {
    public:
      float voltage;
      float current;
      float remaining;

    Battery():
      voltage(0),
      current(0),
      remaining(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_voltage;
      u_voltage.real = this->voltage;
      *(outbuffer + offset + 0) = (u_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->voltage);
      union {
        float real;
        uint32_t base;
      } u_current;
      u_current.real = this->current;
      *(outbuffer + offset + 0) = (u_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current);
      union {
        float real;
        uint32_t base;
      } u_remaining;
      u_remaining.real = this->remaining;
      *(outbuffer + offset + 0) = (u_remaining.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_remaining.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_remaining.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_remaining.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->remaining);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_voltage;
      u_voltage.base = 0;
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->voltage = u_voltage.real;
      offset += sizeof(this->voltage);
      union {
        float real;
        uint32_t base;
      } u_current;
      u_current.base = 0;
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current = u_current.real;
      offset += sizeof(this->current);
      union {
        float real;
        uint32_t base;
      } u_remaining;
      u_remaining.base = 0;
      u_remaining.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_remaining.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_remaining.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_remaining.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->remaining = u_remaining.real;
      offset += sizeof(this->remaining);
     return offset;
    }

    const char * getType(){ return "laserpack/Battery"; };
    const char * getMD5(){ return "df7f08c1443b38b4ac0bbc90dbc93e28"; };

  };

}
#endif