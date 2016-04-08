#ifndef _ROS_laserpack_RPYPose_h
#define _ROS_laserpack_RPYPose_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Point.h"
#include "laserpack/RPY.h"

namespace laserpack
{

  class RPYPose : public ros::Msg
  {
    public:
      geometry_msgs::Point position;
      laserpack::RPY orientation;

    RPYPose():
      position(),
      orientation()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->position.serialize(outbuffer + offset);
      offset += this->orientation.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->position.deserialize(inbuffer + offset);
      offset += this->orientation.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "laserpack/RPYPose"; };
    const char * getMD5(){ return "f07c76c97106e1489e710afe90fb0211"; };

  };

}
#endif