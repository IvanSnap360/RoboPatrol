#ifndef _ROS_SERVICE_gripper_h
#define _ROS_SERVICE_gripper_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rob_hardware_controller
{

static const char GRIPPER[] = "rob_hardware_controller/gripper";

  class gripperRequest : public ros::Msg
  {
    public:
      typedef int32_t _joint_gripper_type;
      _joint_gripper_type joint_gripper;

    gripperRequest():
      joint_gripper(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_joint_gripper;
      u_joint_gripper.real = this->joint_gripper;
      *(outbuffer + offset + 0) = (u_joint_gripper.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_gripper.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_gripper.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_gripper.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_gripper);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_joint_gripper;
      u_joint_gripper.base = 0;
      u_joint_gripper.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint_gripper.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint_gripper.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint_gripper.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint_gripper = u_joint_gripper.real;
      offset += sizeof(this->joint_gripper);
     return offset;
    }

    virtual const char * getType() override { return GRIPPER; };
    virtual const char * getMD5() override { return "6230891a712d074da973bd5dcda5cb89"; };

  };

  class gripperResponse : public ros::Msg
  {
    public:
      typedef bool _result_type;
      _result_type result;

    gripperResponse():
      result(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.real = this->result;
      *(outbuffer + offset + 0) = (u_result.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->result);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.base = 0;
      u_result.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->result = u_result.real;
      offset += sizeof(this->result);
     return offset;
    }

    virtual const char * getType() override { return GRIPPER; };
    virtual const char * getMD5() override { return "eb13ac1f1354ccecb7941ee8fa2192e8"; };

  };

  class gripper {
    public:
    typedef gripperRequest Request;
    typedef gripperResponse Response;
  };

}
#endif
