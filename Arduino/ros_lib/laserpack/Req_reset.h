#ifndef _ROS_laserpack_Req_reset_h
#define _ROS_laserpack_Req_reset_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace laserpack
{

  class Req_reset : public ros::Msg
  {
    public:
      uint8_t rst_length;
      uint8_t st_rst;
      uint8_t * rst;

    Req_reset():
      rst_length(0), rst(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = rst_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < rst_length; i++){
      *(outbuffer + offset + 0) = (this->rst[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->rst[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t rst_lengthT = *(inbuffer + offset++);
      if(rst_lengthT > rst_length)
        this->rst = (uint8_t*)realloc(this->rst, rst_lengthT * sizeof(uint8_t));
      offset += 3;
      rst_length = rst_lengthT;
      for( uint8_t i = 0; i < rst_length; i++){
      this->st_rst =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_rst);
        memcpy( &(this->rst[i]), &(this->st_rst), sizeof(uint8_t));
      }
     return offset;
    }

    const char * getType(){ return "laserpack/Req_reset"; };
    const char * getMD5(){ return "20b38dac9ca495dbc4f26bb98dcf372c"; };

  };

}
#endif