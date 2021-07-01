#ifndef _ROS_SERVICE_doThis_h
#define _ROS_SERVICE_doThis_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace scara_pkg
{

static const char DOTHIS[] = "scara_pkg/doThis";

  class doThisRequest : public ros::Msg
  {
    public:
      typedef const char* _action_type;
      _action_type action;

    doThisRequest():
      action("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_action = strlen(this->action);
      varToArr(outbuffer + offset, length_action);
      offset += 4;
      memcpy(outbuffer + offset, this->action, length_action);
      offset += length_action;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_action;
      arrToVar(length_action, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_action; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_action-1]=0;
      this->action = (char *)(inbuffer + offset-1);
      offset += length_action;
     return offset;
    }

    virtual const char * getType() override { return DOTHIS; };
    virtual const char * getMD5() override { return "7757aad79fa343e61bc69ed7f1b7666d"; };

  };

  class doThisResponse : public ros::Msg
  {
    public:
      typedef bool _res_type;
      _res_type res;

    doThisResponse():
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

    virtual const char * getType() override { return DOTHIS; };
    virtual const char * getMD5() override { return "e27848a10f8e7e4030443887dfea101b"; };

  };

  class doThis {
    public:
    typedef doThisRequest Request;
    typedef doThisResponse Response;
  };

}
#endif
