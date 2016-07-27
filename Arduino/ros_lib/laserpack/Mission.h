#ifndef _ROS_laserpack_Mission_h
#define _ROS_laserpack_Mission_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "laserpack/Task.h"

namespace laserpack
{

  class Mission : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t tasks_length;
      laserpack::Task st_tasks;
      laserpack::Task * tasks;

    Mission():
      header(),
      tasks_length(0), tasks(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = tasks_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < tasks_length; i++){
      offset += this->tasks[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t tasks_lengthT = *(inbuffer + offset++);
      if(tasks_lengthT > tasks_length)
        this->tasks = (laserpack::Task*)realloc(this->tasks, tasks_lengthT * sizeof(laserpack::Task));
      offset += 3;
      tasks_length = tasks_lengthT;
      for( uint8_t i = 0; i < tasks_length; i++){
      offset += this->st_tasks.deserialize(inbuffer + offset);
        memcpy( &(this->tasks[i]), &(this->st_tasks), sizeof(laserpack::Task));
      }
     return offset;
    }

    const char * getType(){ return "laserpack/Mission"; };
    const char * getMD5(){ return "be8768b09e1941380e444293544733e2"; };

  };

}
#endif