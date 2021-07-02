#ifndef _ROS_field_color_HSV_h
#define _ROS_field_color_HSV_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace field
{

  class color_HSV : public ros::Msg
  {
    public:
      typedef std_msgs::Header _Header_type;
      _Header_type Header;
      typedef uint8_t _H_type;
      _H_type H;
      typedef uint8_t _S_type;
      _S_type S;
      typedef uint8_t _V_type;
      _V_type V;

    color_HSV():
      Header(),
      H(0),
      S(0),
      V(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->Header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->H >> (8 * 0)) & 0xFF;
      offset += sizeof(this->H);
      *(outbuffer + offset + 0) = (this->S >> (8 * 0)) & 0xFF;
      offset += sizeof(this->S);
      *(outbuffer + offset + 0) = (this->V >> (8 * 0)) & 0xFF;
      offset += sizeof(this->V);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->Header.deserialize(inbuffer + offset);
      this->H =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->H);
      this->S =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->S);
      this->V =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->V);
     return offset;
    }

    virtual const char * getType() override { return "field/color_HSV"; };
    virtual const char * getMD5() override { return "1fb4a4991eece98901b767fac65d981d"; };

  };

}
#endif
