#ifndef SHM_MSGS__CUDA_IMAGE_CONTAINER_HPP_
#define SHM_MSGS__CUDA_IMAGE_CONTAINER_HPP_

#include <memory>
#include <string>

#include "rclcpp/type_adapter.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

#include "cuda.h"         // NOLINT
#include "cuda_runtime.h" // NOLINT

namespace shm_msgs
{
class CUDAStreamWrapper final
{
  public:
    CUDAStreamWrapper();

    ~CUDAStreamWrapper();

    cudaStream_t &stream()
    {
        return main_stream_;
    }

  private:
    cudaStream_t main_stream_{};
};

class CUDAMemoryWrapper final
{
  public:
    explicit CUDAMemoryWrapper(size_t size_in_bytes);

    void copy_to_device(uint8_t *host_mem, size_t bytes_to_copy, const cudaStream_t &stream);

    void copy_from_device(uint8_t *host_mem, size_t bytes_to_copy, const cudaStream_t &stream);

    uint8_t *device_memory();

    ~CUDAMemoryWrapper();

  private:
    size_t bytes_allocated_{0};

    uint8_t *cuda_mem_{nullptr};
};

class CUDAEventWrapper final
{
  public:
    CUDAEventWrapper();

    void record(std::shared_ptr<CUDAStreamWrapper> cuda_stream);

    cudaEvent_t &event()
    {
        return event_;
    }

    ~CUDAEventWrapper();

  private:
    cudaEvent_t event_;
};

class CudaImageContainer final
{
  public:
    CudaImageContainer();

    /// Store an owning pointer to a sensor_msg::msg::Image, and create CUDA memory that references it
    explicit CudaImageContainer(std::unique_ptr<sensor_msgs::msg::Image> unique_sensor_msgs_image);

    /// Copy the sensor_msgs::msg::Image into this contain and create CUDA memory that references it.
    explicit CudaImageContainer(const sensor_msgs::msg::Image &sensor_msgs_image);

    explicit CudaImageContainer(const CudaImageContainer &other);

    CudaImageContainer(std_msgs::msg::Header header, uint32_t height, uint32_t width, std::string encoding,
                       uint32_t step,
                       std::shared_ptr<CUDAStreamWrapper> cuda_stream = std::make_shared<CUDAStreamWrapper>());

    CudaImageContainer &operator=(const CudaImageContainer &other);

    ~CudaImageContainer();

    /// Const access the ROS Header.
    const std_msgs::msg::Header &header() const;

    /// Access the ROS Header.
    std_msgs::msg::Header &header();

    void get_sensor_msgs_image(sensor_msgs::msg::Image &destination) const;

    uint8_t *cuda_mem();

    size_t size_in_bytes() const;

    std::shared_ptr<CUDAStreamWrapper> cuda_stream() const
    {
        return cuda_stream_;
    }

    uint32_t height() const
    {
        return height_;
    }

    uint32_t width() const
    {
        return width_;
    }

    const std::string &encoding() const
    {
        return encoding_;
    }

    uint32_t step() const
    {
        return step_;
    }

  private:
    std_msgs::msg::Header header_;

    std::shared_ptr<CUDAStreamWrapper> cuda_stream_;

    std::shared_ptr<CUDAMemoryWrapper> cuda_mem_;

    std::shared_ptr<CUDAEventWrapper> cuda_event_;

    uint32_t height_{0};
    uint32_t width_{0};
    std::string encoding_;
    uint32_t step_{0};
};

} // namespace shm_msgs

template <> struct rclcpp::TypeAdapter<shm_msgs::CudaImageContainer, sensor_msgs::msg::Image>
{
    using is_specialized = std::true_type;
    using custom_type = shm_msgs::CudaImageContainer;
    using ros_message_type = sensor_msgs::msg::Image;

    static void convert_to_ros_message(const custom_type &source, ros_message_type &destination)
    {
        source.get_sensor_msgs_image(destination);
    }

    static void convert_to_custom(const ros_message_type &source, custom_type &destination)
    {
        destination = shm_msgs::CudaImageContainer(source);
    }
};

#endif // SHM_MSGS__CUDA_IMAGE_CONTAINER_HPP_
