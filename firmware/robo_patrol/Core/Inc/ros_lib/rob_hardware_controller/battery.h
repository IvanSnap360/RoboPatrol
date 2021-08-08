#ifndef _ROS_rob_hardware_controller_battery_h
#define _ROS_rob_hardware_controller_battery_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace rob_hardware_controller
{

  class battery : public ros::Msg
  {
    public:
      typedef std_msgs::Header _Header_type;
      _Header_type Header;
      typedef float _cell_1_type;
      _cell_1_type cell_1;
      typedef float _cell_2_type;
      _cell_2_type cell_2;
      typedef float _cell_3_type;
      _cell_3_type cell_3;
      typedef float _all_type;
      _all_type all;

    battery():
      Header(),
      cell_1(0),
      cell_2(0),
      cell_3(0),
      all(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->Header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_cell_1;
      u_cell_1.real = this->cell_1;
      *(outbuffer + offset + 0) = (u_cell_1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cell_1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cell_1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cell_1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cell_1);
      union {
        float real;
        uint32_t base;
      } u_cell_2;
      u_cell_2.real = this->cell_2;
      *(outbuffer + offset + 0) = (u_cell_2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cell_2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cell_2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cell_2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cell_2);
      union {
        float real;
        uint32_t base;
      } u_cell_3;
      u_cell_3.real = this->cell_3;
      *(outbuffer + offset + 0) = (u_cell_3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cell_3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cell_3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cell_3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cell_3);
      union {
        float real;
        uint32_t base;
      } u_all;
      u_all.real = this->all;
      *(outbuffer + offset + 0) = (u_all.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_all.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_all.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_all.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->all);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->Header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_cell_1;
      u_cell_1.base = 0;
      u_cell_1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cell_1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cell_1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cell_1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cell_1 = u_cell_1.real;
      offset += sizeof(this->cell_1);
      union {
        float real;
        uint32_t base;
      } u_cell_2;
      u_cell_2.base = 0;
      u_cell_2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cell_2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cell_2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cell_2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cell_2 = u_cell_2.real;
      offset += sizeof(this->cell_2);
      union {
        float real;
        uint32_t base;
      } u_cell_3;
      u_cell_3.base = 0;
      u_cell_3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cell_3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cell_3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cell_3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cell_3 = u_cell_3.real;
      offset += sizeof(this->cell_3);
      union {
        float real;
        uint32_t base;
      } u_all;
      u_all.base = 0;
      u_all.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_all.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_all.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_all.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->all = u_all.real;
      offset += sizeof(this->all);
     return offset;
    }

    virtual const char * getType() override { return "rob_hardware_controller/battery"; };
    virtual const char * getMD5() override { return "44c9b19dda2c4a77571d240a355e0fc8"; };

  };

}
#endif
