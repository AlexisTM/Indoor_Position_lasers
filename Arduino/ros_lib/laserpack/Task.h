#ifndef _ROS_laserpack_Task_h
#define _ROS_laserpack_Task_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"

namespace laserpack
{

  class Task : public ros::Msg
  {
    public:
      std_msgs::String name;
      uint8_t mission_type;
      geometry_msgs::Point position;
      float yaw;
      int32_t data;
      enum { TYPE_INIT_UAV =  9      };
      enum { TYPE_ARM =  13          };
      enum { TYPE_DISARM =  11       };
      enum { TYPE_LOITER =  121      };
      enum { TYPE_TAKEOFF =  122     };
      enum { TYPE_LAND =  123        };
      enum { TYPE_TARGET =  124      };
      enum { TYPE_GRAB =  193        };
      enum { TYPE_TEST =  254        };

    Task():
      name(),
      mission_type(0),
      position(),
      yaw(0),
      data(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->name.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->mission_type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mission_type);
      offset += this->position.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->yaw);
      union {
        int32_t real;
        uint32_t base;
      } u_data;
      u_data.real = this->data;
      *(outbuffer + offset + 0) = (u_data.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_data.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_data.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_data.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->name.deserialize(inbuffer + offset);
      this->mission_type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->mission_type);
      offset += this->position.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->yaw));
      union {
        int32_t real;
        uint32_t base;
      } u_data;
      u_data.base = 0;
      u_data.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_data.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_data.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_data.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->data = u_data.real;
      offset += sizeof(this->data);
     return offset;
    }

    const char * getType(){ return "laserpack/Task"; };
    const char * getMD5(){ return "6e4b90b9f74b50bb878ffb45f872807c"; };

  };

}
#endif