#ifndef _ROS_SERVICE_leds_control_h
#define _ROS_SERVICE_leds_control_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace leds_pkg
{

static const char LEDS_CONTROL[] = "leds_pkg/leds_control";

  class leds_controlRequest : public ros::Msg
  {
    public:
      typedef int8_t _led_num_type;
      _led_num_type led_num;
      typedef int8_t _state_type;
      _state_type state;
      enum { ERROR = -1 };
      enum { NONE = 0 };
      enum { HOME = 1 };
      enum { WAITING = 100 };

    leds_controlRequest():
      led_num(0),
      state(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_led_num;
      u_led_num.real = this->led_num;
      *(outbuffer + offset + 0) = (u_led_num.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->led_num);
      union {
        int8_t real;
        uint8_t base;
      } u_state;
      u_state.real = this->state;
      *(outbuffer + offset + 0) = (u_state.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_led_num;
      u_led_num.base = 0;
      u_led_num.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->led_num = u_led_num.real;
      offset += sizeof(this->led_num);
      union {
        int8_t real;
        uint8_t base;
      } u_state;
      u_state.base = 0;
      u_state.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->state = u_state.real;
      offset += sizeof(this->state);
     return offset;
    }

    virtual const char * getType() override { return LEDS_CONTROL; };
    virtual const char * getMD5() override { return "f9fb13ac1ed47659ccbc8d33d560f75f"; };

  };

  class leds_controlResponse : public ros::Msg
  {
    public:
      typedef bool _res_type;
      _res_type res;

    leds_controlResponse():
      res(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_res;
      u_res.real = this->res;
      *(outbuffer + offset + 0) = (u_res.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->res);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_res;
      u_res.base = 0;
      u_res.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->res = u_res.real;
      offset += sizeof(this->res);
     return offset;
    }

    virtual const char * getType() override { return LEDS_CONTROL; };
    virtual const char * getMD5() override { return "e27848a10f8e7e4030443887dfea101b"; };

  };

  class leds_control {
    public:
    typedef leds_controlRequest Request;
    typedef leds_controlResponse Response;
  };

}
#endif
