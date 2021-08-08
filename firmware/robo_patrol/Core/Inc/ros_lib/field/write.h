#ifndef _ROS_SERVICE_write_h
#define _ROS_SERVICE_write_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "field/cube.h"

namespace field
{

static const char WRITE[] = "field/write";

  class writeRequest : public ros::Msg
  {
    public:
      typedef uint32_t _i_type;
      _i_type i;
      typedef uint32_t _j_type;
      _j_type j;
      typedef field::cube _cube_type;
      _cube_type cube;

    writeRequest():
      i(0),
      j(0),
      cube()
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
      offset += this->cube.serialize(outbuffer + offset);
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
      offset += this->cube.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return WRITE; };
    virtual const char * getMD5() override { return "811c666c974601bf84bde75bc02e4a06"; };

  };

  class writeResponse : public ros::Msg
  {
    public:
      typedef bool _res_type;
      _res_type res;

    writeResponse():
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

    virtual const char * getType() override { return WRITE; };
    virtual const char * getMD5() override { return "e27848a10f8e7e4030443887dfea101b"; };

  };

  class write {
    public:
    typedef writeRequest Request;
    typedef writeResponse Response;
  };

}
#endif
