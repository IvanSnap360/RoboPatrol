#ifndef _ROS_SERVICE_update_params_h
#define _ROS_SERVICE_update_params_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "rob_hardware_controller/joint_params_message.h"

namespace rob_hardware_controller
{

static const char UPDATE_PARAMS[] = "rob_hardware_controller/update_params";

  class update_paramsRequest : public ros::Msg
  {
    public:
      typedef rob_hardware_controller::joint_params_message _joint_param_message_type;
      _joint_param_message_type joint_param_message;

    update_paramsRequest():
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

    virtual const char * getType() override { return UPDATE_PARAMS; };
    virtual const char * getMD5() override { return "b6bc5468bff7197aff9f546fd9cf508b"; };

  };

  class update_paramsResponse : public ros::Msg
  {
    public:
      typedef bool _result_type;
      _result_type result;

    update_paramsResponse():
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

    virtual const char * getType() override { return UPDATE_PARAMS; };
    virtual const char * getMD5() override { return "eb13ac1f1354ccecb7941ee8fa2192e8"; };

  };

  class update_params {
    public:
    typedef update_paramsRequest Request;
    typedef update_paramsResponse Response;
  };

}
#endif
