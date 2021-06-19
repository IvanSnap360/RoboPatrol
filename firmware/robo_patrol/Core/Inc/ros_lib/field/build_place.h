#ifndef _ROS_field_build_place_h
#define _ROS_field_build_place_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "field/cube.h"

namespace field
{

  class build_place : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      field::cube top_layer[3];
      field::cube middle_layer[3];
      field::cube bottom_layer[3];

    build_place():
      header(),
      top_layer(),
      middle_layer(),
      bottom_layer()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      for( uint32_t i = 0; i < 3; i++){
      offset += this->top_layer[i].serialize(outbuffer + offset);
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += this->middle_layer[i].serialize(outbuffer + offset);
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += this->bottom_layer[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      for( uint32_t i = 0; i < 3; i++){
      offset += this->top_layer[i].deserialize(inbuffer + offset);
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += this->middle_layer[i].deserialize(inbuffer + offset);
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += this->bottom_layer[i].deserialize(inbuffer + offset);
      }
     return offset;
    }

    virtual const char * getType() override { return "field/build_place"; };
    virtual const char * getMD5() override { return "31cd0ba0281998d27b57837d1ff5d0fc"; };

  };

}
#endif
