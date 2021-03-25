#ifndef _ROS_field_robot_Blobs_h
#define _ROS_field_robot_Blobs_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "field_robot/Pixel.h"

namespace field_robot
{

  class Blobs : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t pixel_length;
      typedef field_robot::Pixel _pixel_type;
      _pixel_type st_pixel;
      _pixel_type * pixel;

    Blobs():
      header(),
      pixel_length(0), st_pixel(), pixel(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->pixel_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pixel_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pixel_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pixel_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pixel_length);
      for( uint32_t i = 0; i < pixel_length; i++){
      offset += this->pixel[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t pixel_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      pixel_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      pixel_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      pixel_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->pixel_length);
      if(pixel_lengthT > pixel_length)
        this->pixel = (field_robot::Pixel*)realloc(this->pixel, pixel_lengthT * sizeof(field_robot::Pixel));
      pixel_length = pixel_lengthT;
      for( uint32_t i = 0; i < pixel_length; i++){
      offset += this->st_pixel.deserialize(inbuffer + offset);
        memcpy( &(this->pixel[i]), &(this->st_pixel), sizeof(field_robot::Pixel));
      }
     return offset;
    }

    virtual const char * getType() override { return "field_robot/Blobs"; };
    virtual const char * getMD5() override { return "9ece87f048d0c83d54667fc0805de044"; };

  };

}
#endif
