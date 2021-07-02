#ifndef _ROS_ros2can_ros2can_message_h
#define _ROS_ros2can_ros2can_message_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace ros2can
{

  class ros2can_message : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint64_t _id_type;
      _id_type id;
      typedef int32_t _data_1_type;
      _data_1_type data_1;
      typedef int32_t _data_2_type;
      _data_2_type data_2;
      typedef int32_t _data_3_type;
      _data_3_type data_3;
      typedef int32_t _data_4_type;
      _data_4_type data_4;
      typedef int32_t _data_5_type;
      _data_5_type data_5;
      typedef int32_t _data_6_type;
      _data_6_type data_6;
      typedef int32_t _data_7_type;
      _data_7_type data_7;
      typedef int32_t _data_8_type;
      _data_8_type data_8;

    ros2can_message():
      header(),
      id(0),
      data_1(0),
      data_2(0),
      data_3(0),
      data_4(0),
      data_5(0),
      data_6(0),
      data_7(0),
      data_8(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->id >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->id >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->id >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->id >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->id >> (8 * 7)) & 0xFF;
      offset += sizeof(this->id);
      union {
        int32_t real;
        uint32_t base;
      } u_data_1;
      u_data_1.real = this->data_1;
      *(outbuffer + offset + 0) = (u_data_1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_data_1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_data_1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_data_1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_1);
      union {
        int32_t real;
        uint32_t base;
      } u_data_2;
      u_data_2.real = this->data_2;
      *(outbuffer + offset + 0) = (u_data_2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_data_2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_data_2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_data_2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_2);
      union {
        int32_t real;
        uint32_t base;
      } u_data_3;
      u_data_3.real = this->data_3;
      *(outbuffer + offset + 0) = (u_data_3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_data_3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_data_3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_data_3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_3);
      union {
        int32_t real;
        uint32_t base;
      } u_data_4;
      u_data_4.real = this->data_4;
      *(outbuffer + offset + 0) = (u_data_4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_data_4.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_data_4.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_data_4.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_4);
      union {
        int32_t real;
        uint32_t base;
      } u_data_5;
      u_data_5.real = this->data_5;
      *(outbuffer + offset + 0) = (u_data_5.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_data_5.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_data_5.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_data_5.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_5);
      union {
        int32_t real;
        uint32_t base;
      } u_data_6;
      u_data_6.real = this->data_6;
      *(outbuffer + offset + 0) = (u_data_6.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_data_6.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_data_6.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_data_6.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_6);
      union {
        int32_t real;
        uint32_t base;
      } u_data_7;
      u_data_7.real = this->data_7;
      *(outbuffer + offset + 0) = (u_data_7.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_data_7.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_data_7.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_data_7.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_7);
      union {
        int32_t real;
        uint32_t base;
      } u_data_8;
      u_data_8.real = this->data_8;
      *(outbuffer + offset + 0) = (u_data_8.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_data_8.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_data_8.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_data_8.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_8);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->id =  ((uint64_t) (*(inbuffer + offset)));
      this->id |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->id |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->id |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->id |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->id |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->id |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->id |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->id);
      union {
        int32_t real;
        uint32_t base;
      } u_data_1;
      u_data_1.base = 0;
      u_data_1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_data_1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_data_1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_data_1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->data_1 = u_data_1.real;
      offset += sizeof(this->data_1);
      union {
        int32_t real;
        uint32_t base;
      } u_data_2;
      u_data_2.base = 0;
      u_data_2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_data_2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_data_2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_data_2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->data_2 = u_data_2.real;
      offset += sizeof(this->data_2);
      union {
        int32_t real;
        uint32_t base;
      } u_data_3;
      u_data_3.base = 0;
      u_data_3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_data_3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_data_3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_data_3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->data_3 = u_data_3.real;
      offset += sizeof(this->data_3);
      union {
        int32_t real;
        uint32_t base;
      } u_data_4;
      u_data_4.base = 0;
      u_data_4.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_data_4.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_data_4.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_data_4.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->data_4 = u_data_4.real;
      offset += sizeof(this->data_4);
      union {
        int32_t real;
        uint32_t base;
      } u_data_5;
      u_data_5.base = 0;
      u_data_5.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_data_5.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_data_5.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_data_5.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->data_5 = u_data_5.real;
      offset += sizeof(this->data_5);
      union {
        int32_t real;
        uint32_t base;
      } u_data_6;
      u_data_6.base = 0;
      u_data_6.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_data_6.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_data_6.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_data_6.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->data_6 = u_data_6.real;
      offset += sizeof(this->data_6);
      union {
        int32_t real;
        uint32_t base;
      } u_data_7;
      u_data_7.base = 0;
      u_data_7.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_data_7.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_data_7.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_data_7.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->data_7 = u_data_7.real;
      offset += sizeof(this->data_7);
      union {
        int32_t real;
        uint32_t base;
      } u_data_8;
      u_data_8.base = 0;
      u_data_8.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_data_8.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_data_8.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_data_8.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->data_8 = u_data_8.real;
      offset += sizeof(this->data_8);
     return offset;
    }

    virtual const char * getType() override { return "ros2can/ros2can_message"; };
    virtual const char * getMD5() override { return "4527e8822642c2a2889f6ba1a41ba103"; };

  };

}
#endif
