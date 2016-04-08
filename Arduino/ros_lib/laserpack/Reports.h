#ifndef _ROS_laserpack_Reports_h
#define _ROS_laserpack_Reports_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "laserpack/Report.h"

namespace laserpack
{

  class Reports : public ros::Msg
  {
    public:
      uint8_t reports_length;
      laserpack::Report st_reports;
      laserpack::Report * reports;

    Reports():
      reports_length(0), reports(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = reports_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < reports_length; i++){
      offset += this->reports[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t reports_lengthT = *(inbuffer + offset++);
      if(reports_lengthT > reports_length)
        this->reports = (laserpack::Report*)realloc(this->reports, reports_lengthT * sizeof(laserpack::Report));
      offset += 3;
      reports_length = reports_lengthT;
      for( uint8_t i = 0; i < reports_length; i++){
      offset += this->st_reports.deserialize(inbuffer + offset);
        memcpy( &(this->reports[i]), &(this->st_reports), sizeof(laserpack::Report));
      }
     return offset;
    }

    const char * getType(){ return "laserpack/Reports"; };
    const char * getMD5(){ return "21416c4d245b7610aac13eaacf102efb"; };

  };

}
#endif