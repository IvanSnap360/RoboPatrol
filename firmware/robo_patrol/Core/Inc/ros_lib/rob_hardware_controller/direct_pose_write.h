#ifndef _ROS_SERVICE_direct_pose_write_h
#define _ROS_SERVICE_direct_pose_write_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "rob_hardware_controller/joint_params_message.h"

namespace rob_hardware_controller
{

static const char DIRECT_POSE_WRITE[] = "rob_hardware_controller/direct_pose_write";

  class direct_pose_writeRequest : public ros::Msg
  {
    public:
      typedef uint32_t _joint_id_type;
      _joint_id_type joint_id;
      typedef float _pose_type;
      _pose_type pose;
      typedef uint8_t _units_type;
      _units_type units;

    direct_pose_writeRequest():
      joint_id(0),
      pose(0),
      units(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->joint_id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_id);
      union {
        float real;
        uint32_t base;
      } u_pose;
      u_pose.real = this->pose;
      *(outbuffer + offset + 0) = (u_pose.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pose.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pose.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pose.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pose);
      *(outbuffer + offset + 0) = (this->units >> (8 * 0)) & 0xFF;
      offset += sizeof(this->units);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->joint_id =  ((uint32_t) (*(inbuffer + offset)));
      this->joint_id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->joint_id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->joint_id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->joint_id);
      union {
        float real;
        uint32_t base;
      } u_pose;
      u_pose.base = 0;
      u_pose.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pose.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pose.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pose.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pose = u_pose.real;
      offset += sizeof(this->pose);
      this->units =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->units);
     return offset;
    }

    virtual const char * getType() override { return DIRECT_POSE_WRITE; };
    virtual const char * getMD5() override { return "213937b18ed2705b8896b59f269eb684"; };

  };

  class direct_pose_writeResponse : public ros::Msg
  {
    public:
      typedef rob_hardware_controller::joint_params_message _joint_param_message_type;
      _joint_param_message_type joint_param_message;

    direct_pose_writeResponse():
      joint_param_message()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->joint_param_message.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->joint_param_message.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return DIRECT_POSE_WRITE; };
    virtual const char * getMD5() override { return "b6bc5468bff7197aff9f546fd9cf508b"; };

  };

  class direct_pose_write {
    public:
    typedef direct_pose_writeRequest Request;
    typedef direct_pose_writeResponse Response;
  };

}
#endif
