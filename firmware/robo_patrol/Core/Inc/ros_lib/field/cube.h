#ifndef _ROS_field_cube_h
#define _ROS_field_cube_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "field/color_BGR.h"
#include "field/color_HSV.h"

namespace field
{

  class cube : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _name_type;
      _name_type name;
      typedef field::color_BGR _virtual_color_type;
      _virtual_color_type virtual_color;
      typedef field::color_HSV _max_hsv_type;
      _max_hsv_type max_hsv;
      typedef field::color_HSV _min_hsv_type;
      _min_hsv_type min_hsv;
      typedef const char* _size_type;
      _size_type size;

    cube():
      header(),
      name(""),
      virtual_color(),
      max_hsv(),
      min_hsv(),
      size("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      offset += this->virtual_color.serialize(outbuffer + offset);
      offset += this->max_hsv.serialize(outbuffer + offset);
      offset += this->min_hsv.serialize(outbuffer + offset);
      uint32_t length_size = strlen(this->size);
      varToArr(outbuffer + offset, length_size);
      offset += 4;
      memcpy(outbuffer + offset, this->size, length_size);
      offset += length_size;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      offset += this->virtual_color.deserialize(inbuffer + offset);
      offset += this->max_hsv.deserialize(inbuffer + offset);
      offset += this->min_hsv.deserialize(inbuffer + offset);
      uint32_t length_size;
      arrToVar(length_size, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_size; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_size-1]=0;
      this->size = (char *)(inbuffer + offset-1);
      offset += length_size;
     return offset;
    }

    virtual const char * getType() override { return "field/cube"; };
    virtual const char * getMD5() override { return "267037906fc753f5adf3fabb86490042"; };

  };

}
#endif
