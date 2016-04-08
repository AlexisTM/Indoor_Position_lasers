#ifndef _ROS_laserpack_Distance_h
#define _ROS_laserpack_Distance_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace laserpack
{

  class Distance : public ros::Msg
  {
    public:
      uint8_t lasers_length;
      int16_t st_lasers;
      int16_t * lasers;
      uint8_t status_length;
      uint8_t st_status;
      uint8_t * status;
      enum { NEED_RESET =  48 };
      enum { RESET_PENDING =  80 };
      enum { NEED_CONFIGURE =  144 };
      enum { ACQUISITION_READY =  32 };
      enum { ACQUISITION_PENDING =  64 };
      enum { ACQUISITION_DONE =  128 };
      enum { SHUTING_DOWN =  240 };

    Distance():
      lasers_length(0), lasers(NULL),
      status_length(0), status(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = lasers_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < lasers_length; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_lasersi;
      u_lasersi.real = this->lasers[i];
      *(outbuffer + offset + 0) = (u_lasersi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_lasersi.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->lasers[i]);
      }
      *(outbuffer + offset++) = status_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < status_length; i++){
      *(outbuffer + offset + 0) = (this->status[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t lasers_lengthT = *(inbuffer + offset++);
      if(lasers_lengthT > lasers_length)
        this->lasers = (int16_t*)realloc(this->lasers, lasers_lengthT * sizeof(int16_t));
      offset += 3;
      lasers_length = lasers_lengthT;
      for( uint8_t i = 0; i < lasers_length; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_st_lasers;
      u_st_lasers.base = 0;
      u_st_lasers.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_lasers.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_lasers = u_st_lasers.real;
      offset += sizeof(this->st_lasers);
        memcpy( &(this->lasers[i]), &(this->st_lasers), sizeof(int16_t));
      }
      uint8_t status_lengthT = *(inbuffer + offset++);
      if(status_lengthT > status_length)
        this->status = (uint8_t*)realloc(this->status, status_lengthT * sizeof(uint8_t));
      offset += 3;
      status_length = status_lengthT;
      for( uint8_t i = 0; i < status_length; i++){
      this->st_status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_status);
        memcpy( &(this->status[i]), &(this->st_status), sizeof(uint8_t));
      }
     return offset;
    }

    const char * getType(){ return "laserpack/Distance"; };
    const char * getMD5(){ return "4504081b9bb38e5c96a8ced2906b2556"; };

  };

}
#endif