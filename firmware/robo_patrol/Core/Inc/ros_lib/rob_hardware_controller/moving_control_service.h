#ifndef _ROS_SERVICE_moving_control_service_h
#define _ROS_SERVICE_moving_control_service_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rob_hardware_controller
{

static const char MOVING_CONTROL_SERVICE[] = "rob_hardware_controller/moving_control_service";

  class moving_control_serviceRequest : public ros::Msg
  {
    public:
      typedef float _joint_1_type;
      _joint_1_type joint_1;
      typedef float _joint_2_type;
      _joint_2_type joint_2;
      typedef float _joint_3_type;
      _joint_3_type joint_3;
      typedef float _joint_4_type;
      _joint_4_type joint_4;

    moving_control_serviceRequest():
      joint_1(0),
      joint_2(0),
      joint_3(0),
      joint_4(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_joint_1;
      u_joint_1.real = this->joint_1;
      *(outbuffer + offset + 0) = (u_joint_1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_1);
      union {
        float real;
        uint32_t base;
      } u_joint_2;
      u_joint_2.real = this->joint_2;
      *(outbuffer + offset + 0) = (u_joint_2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_2);
      union {
        float real;
        uint32_t base;
      } u_joint_3;
      u_joint_3.real = this->joint_3;
      *(outbuffer + offset + 0) = (u_joint_3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_3);
      union {
        float real;
        uint32_t base;
      } u_joint_4;
      u_joint_4.real = this->joint_4;
      *(outbuffer + offset + 0) = (u_joint_4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_4.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_4.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_4.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_4);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_joint_1;
      u_joint_1.base = 0;
      u_joint_1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint_1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint_1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint_1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint_1 = u_joint_1.real;
      offset += sizeof(this->joint_1);
      union {
        float real;
        uint32_t base;
      } u_joint_2;
      u_joint_2.base = 0;
      u_joint_2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint_2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint_2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint_2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint_2 = u_joint_2.real;
      offset += sizeof(this->joint_2);
      union {
        float real;
        uint32_t base;
      } u_joint_3;
      u_joint_3.base = 0;
      u_joint_3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint_3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint_3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint_3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint_3 = u_joint_3.real;
      offset += sizeof(this->joint_3);
      union {
        float real;
        uint32_t base;
      } u_joint_4;
      u_joint_4.base = 0;
      u_joint_4.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint_4.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint_4.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint_4.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint_4 = u_joint_4.real;
      offset += sizeof(this->joint_4);
     return offset;
    }

    virtual const char * getType() override { return MOVING_CONTROL_SERVICE; };
    virtual const char * getMD5() override { return "ab9c47c63b70f349cc6b6b06b71d7bcf"; };

  };

  class moving_control_serviceResponse : public ros::Msg
  {
    public:
      typedef float _joint_1_err_type;
      _joint_1_err_type joint_1_err;
      typedef float _joint_2_err_type;
      _joint_2_err_type joint_2_err;
      typedef float _joint_3_err_type;
      _joint_3_err_type joint_3_err;
      typedef float _joint_4_err_type;
      _joint_4_err_type joint_4_err;

    moving_control_serviceResponse():
      joint_1_err(0),
      joint_2_err(0),
      joint_3_err(0),
      joint_4_err(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_joint_1_err;
      u_joint_1_err.real = this->joint_1_err;
      *(outbuffer + offset + 0) = (u_joint_1_err.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_1_err.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_1_err.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_1_err.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_1_err);
      union {
        float real;
        uint32_t base;
      } u_joint_2_err;
      u_joint_2_err.real = this->joint_2_err;
      *(outbuffer + offset + 0) = (u_joint_2_err.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_2_err.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_2_err.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_2_err.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_2_err);
      union {
        float real;
        uint32_t base;
      } u_joint_3_err;
      u_joint_3_err.real = this->joint_3_err;
      *(outbuffer + offset + 0) = (u_joint_3_err.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_3_err.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_3_err.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_3_err.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_3_err);
      union {
        float real;
        uint32_t base;
      } u_joint_4_err;
      u_joint_4_err.real = this->joint_4_err;
      *(outbuffer + offset + 0) = (u_joint_4_err.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_4_err.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_4_err.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_4_err.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_4_err);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_joint_1_err;
      u_joint_1_err.base = 0;
      u_joint_1_err.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint_1_err.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint_1_err.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint_1_err.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint_1_err = u_joint_1_err.real;
      offset += sizeof(this->joint_1_err);
      union {
        float real;
        uint32_t base;
      } u_joint_2_err;
      u_joint_2_err.base = 0;
      u_joint_2_err.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint_2_err.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint_2_err.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint_2_err.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint_2_err = u_joint_2_err.real;
      offset += sizeof(this->joint_2_err);
      union {
        float real;
        uint32_t base;
      } u_joint_3_err;
      u_joint_3_err.base = 0;
      u_joint_3_err.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint_3_err.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint_3_err.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint_3_err.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint_3_err = u_joint_3_err.real;
      offset += sizeof(this->joint_3_err);
      union {
        float real;
        uint32_t base;
      } u_joint_4_err;
      u_joint_4_err.base = 0;
      u_joint_4_err.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint_4_err.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint_4_err.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint_4_err.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint_4_err = u_joint_4_err.real;
      offset += sizeof(this->joint_4_err);
     return offset;
    }

    virtual const char * getType() override { return MOVING_CONTROL_SERVICE; };
    virtual const char * getMD5() override { return "c71f61d5e7eeb4be0857123f04404dac"; };

  };

  class moving_control_service {
    public:
    typedef moving_control_serviceRequest Request;
    typedef moving_control_serviceResponse Response;
  };

}
#endif
