#ifndef _ROS_SERVICE_scan_h
#define _ROS_SERVICE_scan_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace cv_pkg
{

static const char SCAN[] = "cv_pkg/scan";

  class scanRequest : public ros::Msg
  {
    public:
      typedef const char* _place_type;
      _place_type place;

    scanRequest():
      place("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_place = strlen(this->place);
      varToArr(outbuffer + offset, length_place);
      offset += 4;
      memcpy(outbuffer + offset, this->place, length_place);
      offset += length_place;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_place;
      arrToVar(length_place, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_place; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_place-1]=0;
      this->place = (char *)(inbuffer + offset-1);
      offset += length_place;
     return offset;
    }

    virtual const char * getType() override { return SCAN; };
    virtual const char * getMD5() override { return "1b9cfee4aec177f2fc73f9506b6022dd"; };

  };

  class scanResponse : public ros::Msg
  {
    public:
      typedef bool _result_type;
      _result_type result;

    scanResponse():
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

    virtual const char * getType() override { return SCAN; };
    virtual const char * getMD5() override { return "eb13ac1f1354ccecb7941ee8fa2192e8"; };

  };

  class scan {
    public:
    typedef scanRequest Request;
    typedef scanResponse Response;
  };

}
#endif
