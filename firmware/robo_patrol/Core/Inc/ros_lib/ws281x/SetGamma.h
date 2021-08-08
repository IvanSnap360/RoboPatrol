#ifndef _ROS_SERVICE_SetGamma_h
#define _ROS_SERVICE_SetGamma_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ws281x
{

static const char SETGAMMA[] = "ws281x/SetGamma";

  class SetGammaRequest : public ros::Msg
  {
    public:
      uint8_t gamma[256];

    SetGammaRequest():
      gamma()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 256; i++){
      *(outbuffer + offset + 0) = (this->gamma[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gamma[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 256; i++){
      this->gamma[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->gamma[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return SETGAMMA; };
    virtual const char * getMD5() override { return "a08443963f514e7b9d053771973ae1a0"; };

  };

  class SetGammaResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetGammaResponse():
      success(0)
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
     return offset;
    }

    virtual const char * getType() override { return SETGAMMA; };
    virtual const char * getMD5() override { return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetGamma {
    public:
    typedef SetGammaRequest Request;
    typedef SetGammaResponse Response;
  };

}
#endif
