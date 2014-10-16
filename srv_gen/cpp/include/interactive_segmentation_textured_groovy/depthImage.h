/* Auto-generated by genmsg_cpp for file /home/hao/groovy_ws/myws/bosch-ros-pkg-code/stacks/bosch_interactive_segmentation/interactive_segmentation_textured_groovy/srv/depthImage.srv */
#ifndef INTERACTIVE_SEGMENTATION_TEXTURED_GROOVY_SERVICE_DEPTHIMAGE_H
#define INTERACTIVE_SEGMENTATION_TEXTURED_GROOVY_SERVICE_DEPTHIMAGE_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "ros/service_traits.h"



#include "sensor_msgs/Image.h"

namespace interactive_segmentation_textured_groovy
{
template <class ContainerAllocator>
struct depthImageRequest_ {
  typedef depthImageRequest_<ContainerAllocator> Type;

  depthImageRequest_()
  {
  }

  depthImageRequest_(const ContainerAllocator& _alloc)
  {
  }


  typedef boost::shared_ptr< ::interactive_segmentation_textured_groovy::depthImageRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::interactive_segmentation_textured_groovy::depthImageRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct depthImageRequest
typedef  ::interactive_segmentation_textured_groovy::depthImageRequest_<std::allocator<void> > depthImageRequest;

typedef boost::shared_ptr< ::interactive_segmentation_textured_groovy::depthImageRequest> depthImageRequestPtr;
typedef boost::shared_ptr< ::interactive_segmentation_textured_groovy::depthImageRequest const> depthImageRequestConstPtr;



template <class ContainerAllocator>
struct depthImageResponse_ {
  typedef depthImageResponse_<ContainerAllocator> Type;

  depthImageResponse_()
  : depth_image()
  {
  }

  depthImageResponse_(const ContainerAllocator& _alloc)
  : depth_image(_alloc)
  {
  }

  typedef  ::sensor_msgs::Image_<ContainerAllocator>  _depth_image_type;
   ::sensor_msgs::Image_<ContainerAllocator>  depth_image;


  typedef boost::shared_ptr< ::interactive_segmentation_textured_groovy::depthImageResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::interactive_segmentation_textured_groovy::depthImageResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct depthImageResponse
typedef  ::interactive_segmentation_textured_groovy::depthImageResponse_<std::allocator<void> > depthImageResponse;

typedef boost::shared_ptr< ::interactive_segmentation_textured_groovy::depthImageResponse> depthImageResponsePtr;
typedef boost::shared_ptr< ::interactive_segmentation_textured_groovy::depthImageResponse const> depthImageResponseConstPtr;


struct depthImage
{

typedef depthImageRequest Request;
typedef depthImageResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct depthImage
} // namespace interactive_segmentation_textured_groovy

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::interactive_segmentation_textured_groovy::depthImageRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::interactive_segmentation_textured_groovy::depthImageRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::interactive_segmentation_textured_groovy::depthImageRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::interactive_segmentation_textured_groovy::depthImageRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::interactive_segmentation_textured_groovy::depthImageRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "interactive_segmentation_textured_groovy/depthImageRequest";
  }

  static const char* value(const  ::interactive_segmentation_textured_groovy::depthImageRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::interactive_segmentation_textured_groovy::depthImageRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
";
  }

  static const char* value(const  ::interactive_segmentation_textured_groovy::depthImageRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::interactive_segmentation_textured_groovy::depthImageRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::interactive_segmentation_textured_groovy::depthImageResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::interactive_segmentation_textured_groovy::depthImageResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::interactive_segmentation_textured_groovy::depthImageResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "b2c36e91badbc08644feff566a2abdfe";
  }

  static const char* value(const  ::interactive_segmentation_textured_groovy::depthImageResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xb2c36e91badbc086ULL;
  static const uint64_t static_value2 = 0x44feff566a2abdfeULL;
};

template<class ContainerAllocator>
struct DataType< ::interactive_segmentation_textured_groovy::depthImageResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "interactive_segmentation_textured_groovy/depthImageResponse";
  }

  static const char* value(const  ::interactive_segmentation_textured_groovy::depthImageResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::interactive_segmentation_textured_groovy::depthImageResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "sensor_msgs/Image depth_image\n\
\n\
================================================================================\n\
MSG: sensor_msgs/Image\n\
# This message contains an uncompressed image\n\
# (0, 0) is at top-left corner of image\n\
#\n\
\n\
Header header        # Header timestamp should be acquisition time of image\n\
                     # Header frame_id should be optical frame of camera\n\
                     # origin of frame should be optical center of cameara\n\
                     # +x should point to the right in the image\n\
                     # +y should point down in the image\n\
                     # +z should point into to plane of the image\n\
                     # If the frame_id here and the frame_id of the CameraInfo\n\
                     # message associated with the image conflict\n\
                     # the behavior is undefined\n\
\n\
uint32 height         # image height, that is, number of rows\n\
uint32 width          # image width, that is, number of columns\n\
\n\
# The legal values for encoding are in file src/image_encodings.cpp\n\
# If you want to standardize a new string format, join\n\
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.\n\
\n\
string encoding       # Encoding of pixels -- channel meaning, ordering, size\n\
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h\n\
\n\
uint8 is_bigendian    # is this data bigendian?\n\
uint32 step           # Full row length in bytes\n\
uint8[] data          # actual matrix data, size is (step * rows)\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::interactive_segmentation_textured_groovy::depthImageResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::interactive_segmentation_textured_groovy::depthImageRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct depthImageRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::interactive_segmentation_textured_groovy::depthImageResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.depth_image);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct depthImageResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<interactive_segmentation_textured_groovy::depthImage> {
  static const char* value() 
  {
    return "b2c36e91badbc08644feff566a2abdfe";
  }

  static const char* value(const interactive_segmentation_textured_groovy::depthImage&) { return value(); } 
};

template<>
struct DataType<interactive_segmentation_textured_groovy::depthImage> {
  static const char* value() 
  {
    return "interactive_segmentation_textured_groovy/depthImage";
  }

  static const char* value(const interactive_segmentation_textured_groovy::depthImage&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<interactive_segmentation_textured_groovy::depthImageRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "b2c36e91badbc08644feff566a2abdfe";
  }

  static const char* value(const interactive_segmentation_textured_groovy::depthImageRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<interactive_segmentation_textured_groovy::depthImageRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "interactive_segmentation_textured_groovy/depthImage";
  }

  static const char* value(const interactive_segmentation_textured_groovy::depthImageRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<interactive_segmentation_textured_groovy::depthImageResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "b2c36e91badbc08644feff566a2abdfe";
  }

  static const char* value(const interactive_segmentation_textured_groovy::depthImageResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<interactive_segmentation_textured_groovy::depthImageResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "interactive_segmentation_textured_groovy/depthImage";
  }

  static const char* value(const interactive_segmentation_textured_groovy::depthImageResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // INTERACTIVE_SEGMENTATION_TEXTURED_GROOVY_SERVICE_DEPTHIMAGE_H
