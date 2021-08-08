#ifndef _ROS_field_color_BGR_h
#define _ROS_field_color_BGR_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace field
{

  class color_BGR : public ros::Msg
  {
    public:
      typedef std_msgs::Header _Header_type;
      _Header_type Header;
      typedef uint8_t _B_type;
      _B_type B;
      typedef uint8_t _G_type;
      _G_type G;
      typedef uint8_t _R_type;
      _R_type R;

    color_BGR():
      Header(),
      B(0),
      G(0),
      R(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->Header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->B >> (8 * 0)) & 0xFF;
      offset += sizeof(this->B);
      *(outbuffer + offset + 0) = (this->G >> (8 * 0)) & 0xFF;
      offset += sizeof(this->G);
      *(outbuffer + offset + 0) = (this->R >> (8 * 0)) & 0xFF;
      offset += sizeof(this->R);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->Header.deserialize(inbuffer + offset);
      this->B =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->B);
      this->G =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->G);
      this->R =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->R);
     return offset;
    }

    virtual const char * getType() override { return "field/color_BGR"; };
    virtual const char * getMD5() override { return "01fa9cac99bc0ac030d55cfb8570da82"; };

  };

}
#endif
