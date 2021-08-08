#ifndef _ROS_SERVICE_SetLEDEffect_h
#define _ROS_SERVICE_SetLEDEffect_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace leds_pkg
{

static const char SETLEDEFFECT[] = "leds_pkg/SetLEDEffect";

  class SetLEDEffectRequest : public ros::Msg
  {
    public:
      typedef const char* _effect_type;
      _effect_type effect;
      typedef uint8_t _r_type;
      _r_type r;
      typedef uint8_t _g_type;
      _g_type g;
      typedef uint8_t _b_type;
      _b_type b;

    SetLEDEffectRequest():
      effect(""),
      r(0),
      g(0),
      b(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_effect = strlen(this->effect);
      varToArr(outbuffer + offset, length_effect);
      offset += 4;
      memcpy(outbuffer + offset, this->effect, length_effect);
      offset += length_effect;
      *(outbuffer + offset + 0) = (this->r >> (8 * 0)) & 0xFF;
      offset += sizeof(this->r);
      *(outbuffer + offset + 0) = (this->g >> (8 * 0)) & 0xFF;
      offset += sizeof(this->g);
      *(outbuffer + offset + 0) = (this->b >> (8 * 0)) & 0xFF;
      offset += sizeof(this->b);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_effect;
      arrToVar(length_effect, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_effect; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_effect-1]=0;
      this->effect = (char *)(inbuffer + offset-1);
      offset += length_effect;
      this->r =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->r);
      this->g =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->g);
      this->b =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->b);
     return offset;
    }

    virtual const char * getType() override { return SETLEDEFFECT; };
    virtual const char * getMD5() override { return "f671ae8175b9ebc5acde7321ebe360c1"; };

  };

  class SetLEDEffectResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef const char* _message_type;
      _message_type message;

    SetLEDEffectResponse():
      success(0),
      message("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      uint32_t length_message = strlen(this->message);
      varToArr(outbuffer + offset, length_message);
      offset += 4;
      memcpy(outbuffer + offset, this->message, length_message);
      offset += length_message;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      uint32_t length_message;
      arrToVar(length_message, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_message; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_message-1]=0;
      this->message = (char *)(inbuffer + offset-1);
      offset += length_message;
     return offset;
    }

    virtual const char * getType() override { return SETLEDEFFECT; };
    virtual const char * getMD5() override { return "937c9679a518e3a18d831e57125ea522"; };

  };

  class SetLEDEffect {
    public:
    typedef SetLEDEffectRequest Request;
    typedef SetLEDEffectResponse Response;
  };

}
#endif
