#ifndef SHM_MSGS__OPENCV_CONVERSIONS_HPP_
#define SHM_MSGS__OPENCV_CONVERSIONS_HPP_

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/header.hpp>

#include <shm_msgs/msg/image.hpp>
// #include <shm_msgs/msg/compressed_image.hpp>
#include <shm_msgs/image_encodings.hpp>
#include <shm_msgs/array_helper.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>

#include <memory>
#include <ostream>
#include <stdexcept>
#include <string>

namespace shm_msgs
{

class Exception : public std::runtime_error
{
public:
  explicit Exception(const std::string & description)
  : std::runtime_error(description) {}
};

class CvImage;

typedef std::shared_ptr<CvImage> CvImagePtr;
typedef std::shared_ptr<CvImage const> CvImageConstPtr;

// From: http://docs.opencv.org/modules/highgui/doc/reading_and_writing_images_and_video.html#Mat
// imread(const string& filename, int flags)
typedef enum
{
  BMP, DIB,
  JPG, JPEG, JPE,
  JP2,
  PNG,
  PBM, PGM, PPM,
  SR, RAS,
  TIFF, TIF,
} Format;

/**
 * \brief Image message class that is interoperable with sensor_msgs/Image but uses a
 * more convenient cv::Mat representation for the image data.
 */
class CvImage
{
public:
  std_msgs::msg::Header header;  // !< ROS header
  std::string encoding;    // !< Image encoding ("mono8", "bgr8", etc.)
  cv::Mat image;           // !< Image data for use with OpenCV

  /**
   * \brief Empty constructor.
   */
  CvImage() {}

  /**
   * \brief Constructor.
   */
  CvImage(
    const std_msgs::msg::Header & header, const std::string & encoding,
    const cv::Mat & image = cv::Mat())
  : header(header), encoding(encoding), image(image)
  {
  }

  /**
   * \brief Convert this message to a ROS shm_msgs::msg::Image8k message.
   *
   * The returned shm_msgs::msg::Image8k message contains a copy of the image data.
   */
  // zs: extend here
  shm_msgs::msg::Image8k::SharedPtr toImageMsg8k() const;
  shm_msgs::msg::Image512k::SharedPtr toImageMsg512k() const;
  shm_msgs::msg::Image1m::SharedPtr toImageMsg1m() const;
  shm_msgs::msg::Image2m::SharedPtr toImageMsg2m() const;
  shm_msgs::msg::Image4m::SharedPtr toImageMsg4m() const;
  shm_msgs::msg::Image8m::SharedPtr toImageMsg8m() const;

  /**
   * \brief Copy the message data to a ROS shm_msgs::msg::Image8k message.
   *
   * This overload is intended mainly for aggregate messages such as stereo_msgs::DisparityImage,
   * which contains a shm_msgs::msg::Image8k as a data member.
   */
  // zs: extend here
  void toImageMsg(shm_msgs::msg::Image8k & ros_image) const;
  void toImageMsg(shm_msgs::msg::Image512k & ros_image) const;
  void toImageMsg(shm_msgs::msg::Image1m & ros_image) const;
  void toImageMsg(shm_msgs::msg::Image2m & ros_image) const;
  void toImageMsg(shm_msgs::msg::Image4m & ros_image) const;
  void toImageMsg(shm_msgs::msg::Image8m & ros_image) const;


  typedef std::shared_ptr<CvImage> Ptr;
  typedef std::shared_ptr<CvImage const> ConstPtr;

protected:
  std::shared_ptr<void const> tracked_object_;  // for sharing ownership

  /// @cond DOXYGEN_IGNORE
  friend
  CvImageConstPtr toCvShare(
    const shm_msgs::msg::Image8k & source,
    const std::shared_ptr<void const> & tracked_object,
    const std::string & encoding);
  /// @endcond
  /// @cond DOXYGEN_IGNORE
  friend
  CvImageConstPtr toCvShare(
    const shm_msgs::msg::Image512k & source,
    const std::shared_ptr<void const> & tracked_object,
    const std::string & encoding);
  /// @endcond
  /// @cond DOXYGEN_IGNORE
  friend
  CvImageConstPtr toCvShare(
    const shm_msgs::msg::Image1m & source,
    const std::shared_ptr<void const> & tracked_object,
    const std::string & encoding);
  /// @endcond
  /// @cond DOXYGEN_IGNORE
  friend
  CvImageConstPtr toCvShare(
    const shm_msgs::msg::Image2m & source,
    const std::shared_ptr<void const> & tracked_object,
    const std::string & encoding);
  /// @endcond
  /// @cond DOXYGEN_IGNORE
  friend
  CvImageConstPtr toCvShare(
    const shm_msgs::msg::Image4m & source,
    const std::shared_ptr<void const> & tracked_object,
    const std::string & encoding);
  /// @endcond
  /// @cond DOXYGEN_IGNORE
  friend
  CvImageConstPtr toCvShare(
    const shm_msgs::msg::Image8m & source,
    const std::shared_ptr<void const> & tracked_object,
    const std::string & encoding);
  /// @endcond
};

/**
 * \brief Convert a shm_msgs::msg::Image8k message to an OpenCV-compatible CvImage, copying the
 * image data.
 *
 * \param source   A shared_ptr to a shm_msgs::msg::Image8k message
 * \param encoding The desired encoding of the image data, one of the following strings:
 *    - \c "mono8"
 *    - \c "bgr8"
 *    - \c "bgra8"
 *    - \c "rgb8"
 *    - \c "rgba8"
 *    - \c "mono16"
 *
 * If \a encoding is the empty string (the default), the returned CvImage has the same encoding
 * as \a source.
 */
// zs: extend here
CvImagePtr toCvCopy(
  const shm_msgs::msg::Image8k::ConstSharedPtr & source,
  const std::string & encoding = std::string());
CvImagePtr toCvCopy(
  const shm_msgs::msg::Image512k::ConstSharedPtr & source,
  const std::string & encoding = std::string());
CvImagePtr toCvCopy(
  const shm_msgs::msg::Image1m::ConstSharedPtr & source,
  const std::string & encoding = std::string());
CvImagePtr toCvCopy(
  const shm_msgs::msg::Image2m::ConstSharedPtr & source,
  const std::string & encoding = std::string());
CvImagePtr toCvCopy(
  const shm_msgs::msg::Image4m::ConstSharedPtr & source,
  const std::string & encoding = std::string());
CvImagePtr toCvCopy(
  const shm_msgs::msg::Image8m::ConstSharedPtr & source,
  const std::string & encoding = std::string());

/**
 * \brief Convert a shm_msgs::msg::Image8k message to an OpenCV-compatible CvImage, copying the
 * image data.
 *
 * \param source   A shm_msgs::msg::Image8k message
 * \param encoding The desired encoding of the image data, one of the following strings:
 *    - \c "mono8"
 *    - \c "bgr8"
 *    - \c "bgra8"
 *    - \c "rgb8"
 *    - \c "rgba8"
 *    - \c "mono16"
 *
 * If \a encoding is the empty string (the default), the returned CvImage has the same encoding
 * as \a source.
 * If the source is 8bit and the encoding 16 or vice-versa, a scaling is applied (65535/255 and
 * 255/65535 respectively). Otherwise, no scaling is applied and the rules from the convertTo OpenCV
 * function are applied (capping): http://docs.opencv.org/modules/core/doc/basic_structures.html#mat-convertto
 */
// zs: extend here
CvImagePtr toCvCopy(
  const shm_msgs::msg::Image8k & source,
  const std::string & encoding = std::string());
CvImagePtr toCvCopy(
  const shm_msgs::msg::Image512k & source,
  const std::string & encoding = std::string());
CvImagePtr toCvCopy(
  const shm_msgs::msg::Image1m & source,
  const std::string & encoding = std::string());
CvImagePtr toCvCopy(
  const shm_msgs::msg::Image2m & source,
  const std::string & encoding = std::string());
CvImagePtr toCvCopy(
  const shm_msgs::msg::Image4m & source,
  const std::string & encoding = std::string());
CvImagePtr toCvCopy(
  const shm_msgs::msg::Image8m & source,
  const std::string & encoding = std::string());

/**
 * \brief Convert an immutable shm_msgs::msg::Image8k message to an OpenCV-compatible CvImage, sharing
 * the image data if possible.
 *
 * If the source encoding and desired encoding are the same, the returned CvImage will share
 * the image data with \a source without copying it. The returned CvImage cannot be modified, as that
 * could modify the \a source data.
 *
 * \param source   A shared_ptr to a shm_msgs::msg::Image8k message
 * \param encoding The desired encoding of the image data, one of the following strings:
 *    - \c "mono8"
 *    - \c "bgr8"
 *    - \c "bgra8"
 *    - \c "rgb8"
 *    - \c "rgba8"
 *    - \c "mono16"
 *
 * If \a encoding is the empty string (the default), the returned CvImage has the same encoding
 * as \a source.
 */
// zs: extend here
CvImageConstPtr toCvShare(
  const shm_msgs::msg::Image8k::ConstSharedPtr & source,
  const std::string & encoding = std::string());
CvImageConstPtr toCvShare(
  const shm_msgs::msg::Image512k::ConstSharedPtr & source,
  const std::string & encoding = std::string());
CvImageConstPtr toCvShare(
  const shm_msgs::msg::Image1m::ConstSharedPtr & source,
  const std::string & encoding = std::string());
CvImageConstPtr toCvShare(
  const shm_msgs::msg::Image2m::ConstSharedPtr & source,
  const std::string & encoding = std::string());
CvImageConstPtr toCvShare(
  const shm_msgs::msg::Image4m::ConstSharedPtr & source,
  const std::string & encoding = std::string());
CvImageConstPtr toCvShare(
  const shm_msgs::msg::Image8m::ConstSharedPtr & source,
  const std::string & encoding = std::string());

/**
 * \brief Convert an immutable sensor_msgs::msg::Image message to an OpenCV-compatible CvImage, sharing
 * the image data if possible.
 *
 * If the source encoding and desired encoding are the same, the returned CvImage will share
 * the image data with \a source without copying it. The returned CvImage cannot be modified, as that
 * could modify the \a source data.
 *
 * This overload is useful when you have a shared_ptr to a message that contains a
 * sensor_msgs::msg::Image, and wish to share ownership with the containing message.
 *
 * \param source         The sensor_msgs::msg::Image message
 * \param tracked_object A shared_ptr to an object owning the sensor_msgs::msg::Image
 * \param encoding       The desired encoding of the image data, one of the following strings:
 *    - \c "mono8"
 *    - \c "bgr8"
 *    - \c "bgra8"
 *    - \c "rgb8"
 *    - \c "rgba8"
 *    - \c "mono16"
 *
 * If \a encoding is the empty string (the default), the returned CvImage has the same encoding
 * as \a source.
 */
// zs: extend here
CvImageConstPtr toCvShare(
  const shm_msgs::msg::Image8k & source,
  const std::shared_ptr<void const> & tracked_object,
  const std::string & encoding = std::string());
CvImageConstPtr toCvShare(
  const shm_msgs::msg::Image512k & source,
  const std::shared_ptr<void const> & tracked_object,
  const std::string & encoding = std::string());
CvImageConstPtr toCvShare(
  const shm_msgs::msg::Image1m & source,
  const std::shared_ptr<void const> & tracked_object,
  const std::string & encoding = std::string());
CvImageConstPtr toCvShare(
  const shm_msgs::msg::Image2m & source,
  const std::shared_ptr<void const> & tracked_object,
  const std::string & encoding = std::string());
CvImageConstPtr toCvShare(
  const shm_msgs::msg::Image4m & source,
  const std::shared_ptr<void const> & tracked_object,
  const std::string & encoding = std::string());
CvImageConstPtr toCvShare(
  const shm_msgs::msg::Image8m & source,
  const std::shared_ptr<void const> & tracked_object,
  const std::string & encoding = std::string());

/**
 * \brief Convert a CvImage to another encoding using the same rules as toCvCopy
 */
CvImagePtr cvtColor(
  const CvImageConstPtr & source,
  const std::string & encoding);

/**
 * \brief Get the OpenCV type enum corresponding to the encoding.
 *
 * For example, "bgr8" -> CV_8UC3, "32FC1" -> CV_32FC1, and "32FC10" -> CV_32FC10.
 */
int getCvType(const std::string & encoding);

}  // namespace shm_msgs

#endif  // SHM_MSGS__OPENCV_CONVERSIONS_HPP_
