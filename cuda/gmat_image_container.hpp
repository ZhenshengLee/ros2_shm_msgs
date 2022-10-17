#ifndef SHM_MSGS__GPU_MAT_SENSOR_MSGS_IMAGE_TYPE_ADAPTER_HPP_
#define SHM_MSGS__GPU_MAT_SENSOR_MSGS_IMAGE_TYPE_ADAPTER_HPP_

#include <cstddef>
#include <memory>
#include <variant>

#include "opencv2/core/mat.hpp"
#include <opencv2/cudaimgproc.hpp>

#include "rclcpp/type_adapter.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace shm_msgs
{

class ROSGpuMatContainer
{

public:
  using SensorMsgsImageStorageType = std::variant<
    std::nullptr_t,
    std::unique_ptr<sensor_msgs::msg::Image>,
    std::shared_ptr<sensor_msgs::msg::Image>
  >;

  ROSGpuMatContainer()
  {
    cv_cuda_stream_ = std::make_shared<cv::cuda::Stream>();
    cv_cuda_event_ = std::make_shared<cv::cuda::Event>();
  }

  // deep copy from other
  explicit ROSGpuMatContainer(const ROSGpuMatContainer & other)
  : header_(other.header_)
  {
    // Make a new stream and have it wait on the previous one.
    cv_cuda_stream_ = std::make_shared<cv::cuda::Stream>();
    cv_cuda_event_ = std::make_shared<cv::cuda::Event>();
    cv_cuda_event_->record(*other.cv_cuda_stream_);
    cv_cuda_stream_->waitEvent(*cv_cuda_event_);

    other.frame_.copyTo(frame_, *cv_cuda_stream_);
    if (std::holds_alternative<std::shared_ptr<sensor_msgs::msg::Image>>(other.storage_)) {
      storage_ = std::get<std::shared_ptr<sensor_msgs::msg::Image>>(other.storage_);
    } else if (std::holds_alternative<std::unique_ptr<sensor_msgs::msg::Image>>(other.storage_)) {
      storage_ = std::make_unique<sensor_msgs::msg::Image>(
        *std::get<std::unique_ptr<sensor_msgs::msg::Image>>(other.storage_));
    }
  }

  // shallow copy only
  ROSGpuMatContainer & operator=(const ROSGpuMatContainer & other)
  {
    if (this != &other) {
      header_ = other.header_;
      frame_ = other.frame_;
      cv_cuda_stream_ = other.cv_cuda_stream_;
      cv_cuda_event_ = other.cv_cuda_event_;
      if (std::holds_alternative<std::shared_ptr<sensor_msgs::msg::Image>>(other.storage_)) {
        storage_ = std::get<std::shared_ptr<sensor_msgs::msg::Image>>(other.storage_);
      } else if (std::holds_alternative<std::unique_ptr<sensor_msgs::msg::Image>>(other.storage_)) {
        storage_ = std::make_unique<sensor_msgs::msg::Image>(
          *std::get<std::unique_ptr<sensor_msgs::msg::Image>>(other.storage_));
      } else if (std::holds_alternative<std::nullptr_t>(other.storage_)) {
        storage_ = nullptr;
      }
    }
    return *this;
  }

  /// Store an owning pointer to a sensor_msg::msg::Image, and create a cv::cuda::GpuMat that references it.
  explicit ROSGpuMatContainer(std::unique_ptr<sensor_msgs::msg::Image> unique_sensor_msgs_image);

  /// copy the given cv::Mat into this class.
  ROSGpuMatContainer(
    const cv::Mat & mat_frame,
    const std_msgs::msg::Header & header,
    std::shared_ptr<cv::cuda::Stream> cv_cuda_stream);

  /// Copy the sensor_msgs::msg::Image into this contain and create a cv::cuda::GpuMat that references it.
  explicit ROSGpuMatContainer(const sensor_msgs::msg::Image & sensor_msgs_image);

  /// Return true if this class owns the data the cv_gpu_mat references.
  /**
   * Note that this does not check if the cv::cuda::GpuMat owns its own data, only if
   * this class owns a sensor_msgs::msg::Image that the cv::cuda::GpuMat references.
   */
  bool
  is_owning() const;

  /// Const access the cv::cuda::GpuMat in this class.
  const cv::cuda::GpuMat &
  cv_gpu_mat() const;

  /// Get a shallow copy of the cv::cuda::GpuMat that is in this class.
  /**
   * Note that if you want to let this container go out of scope you should
   * make a deep copy with cv::cuda::GpuMat::clone() beforehand.
   */
  cv::cuda::GpuMat
  cv_gpu_mat();

  /// Const access the ROS Header.
  const std_msgs::msg::Header &
  header() const;

  /// Access the ROS Header.
  std_msgs::msg::Header &
  header();

  /// Get shared const pointer to the sensor_msgs::msg::Image if available, otherwise nullptr.
  // std::shared_ptr<const sensor_msgs::msg::Image>
  // get_sensor_msgs_msg_image_pointer() const;

  /// Get copy as a unique pointer to the sensor_msgs::msg::Image.
  std::unique_ptr<sensor_msgs::msg::Image>
  get_sensor_msgs_msg_image_pointer_copy() const;

  /// Get a copy of the image as a sensor_msgs::msg::Image.
  sensor_msgs::msg::Image
  get_sensor_msgs_msg_image_copy() const;

  /// Get a copy of the image as a sensor_msgs::msg::Image.
  void
  get_sensor_msgs_msg_image_copy(sensor_msgs::msg::Image & sensor_msgs_image) const;

private:
  std_msgs::msg::Header header_;
  cv::cuda::GpuMat frame_;
  std::shared_ptr<cv::cuda::Stream> cv_cuda_stream_;
  std::shared_ptr<cv::cuda::Event> cv_cuda_event_;
  SensorMsgsImageStorageType storage_;
};

}  // namespace shm_msgs

template<>
struct rclcpp::TypeAdapter<shm_msgs::ROSGpuMatContainer, sensor_msgs::msg::Image>
{
  using is_specialized = std::true_type;
  using custom_type = shm_msgs::ROSGpuMatContainer;
  using ros_message_type = sensor_msgs::msg::Image;

  static
  void
  convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination)
  {
    source.get_sensor_msgs_msg_image_copy(destination);
  }

  static
  void
  convert_to_custom(
    const ros_message_type & source,
    custom_type & destination)
  {
    destination = shm_msgs::ROSGpuMatContainer(source);
  }
};

#endif  // SHM_MSGS__GPU_MAT_SENSOR_MSGS_IMAGE_TYPE_ADAPTER_HPP_