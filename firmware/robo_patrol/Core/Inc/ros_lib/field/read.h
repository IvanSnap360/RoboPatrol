#ifndef _ROS_SERVICE_read_h
#define _ROS_SERVICE_read_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "field/cube.h"

namespace field
{

static const char READ[] = "field/read";

  class readRequest : public ros::Msg
  {
    public:
      typedef uint32_t _i_type;
      _i_type i;
      typedef uint32_t _j_type;
      _j_type j;

    readRequest():
      i(0),
      j(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->i >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->i >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->i >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->i >> (8 * 3)) & 0xFF;
      offset += sizeof(this->i);
      *(outbuffer + offset + 0) = (this->j >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->j >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->j >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->j >> (8 * 3)) & 0xFF;
      offset += sizeof(this->j);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->i =  ((uint32_t) (*(inbuffer + offset)));
      this->i |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->i |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->i |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->i);
      this->j =  ((uint32_t) (*(inbuffer + offset)));
      this->j |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->j |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->j |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->j);
     return offset;
    }

    virtual const char * getType() override { return READ; };
    virtual const char * getMD5() override { return "f81216c9139e19bcca4b26d87f431485"; };

  };

  class readResponse : public ros::Msg
  {
    public:
      typedef field::cube _cube_type;
      _cube_type cube;

    readResponse():
      cube()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->cube.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->cube.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return READ; };
    virtual const char * getMD5() override { return "7816bceb7e0f419263eeb693214c4f85"; };

  };

  class read {
    public:
    typedef readRequest Request;
    typedef readResponse Response;
  };

}
#endif
