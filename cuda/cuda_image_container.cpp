#include <memory>
#include <stdexcept>
#include <string>

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

#include "cuda_image_container.hpp"

#include "cuda.h"         // NOLINT
#include "cuda_runtime.h" // NOLINT
#include <nvToolsExt.h>   // NOLINT

namespace shm_msgs
{
namespace
{
template <typename T> struct NotNull
{
    NotNull(const T *pointer_in, const char *msg) : pointer(pointer_in)
    {
        if (pointer == nullptr)
        {
            throw std::invalid_argument(msg);
        }
    }

    const T *pointer;
};

} // namespace

CUDAStreamWrapper::CUDAStreamWrapper()
{
    cudaStreamCreate(&main_stream_);
}

CUDAStreamWrapper::~CUDAStreamWrapper()
{
    cudaStreamDestroy(main_stream_);
}

CUDAMemoryWrapper::CUDAMemoryWrapper(size_t bytes_to_allocate) : bytes_allocated_(bytes_to_allocate)
{
    // non-jetson using cudaMalloc rather than cudaMallocManaged
#ifndef __aarch64__
    if (cudaMalloc(&cuda_mem_, bytes_to_allocate) != cudaSuccess)
#else
    if (cudaMallocManaged(&cuda_mem_, bytes_to_allocate) != cudaSuccess)
#endif
    {
        throw std::runtime_error("Failed to allocate device memory");
    }
}

void CUDAMemoryWrapper::copy_to_device(uint8_t *host_mem, size_t bytes_to_copy, const cudaStream_t &stream)
{
    nvtxRangePushA("CudaImageContainer:CopyToDevice");
    if (bytes_to_copy > bytes_allocated_)
    {
        throw std::invalid_argument("Tried to copy too many bytes to device");
    }
    if (cudaMemcpyAsync(cuda_mem_, host_mem, bytes_to_copy, cudaMemcpyHostToDevice, stream) != cudaSuccess)
    {
        throw std::runtime_error("Failed to copy memory to the GPU");
    }
    cudaStreamSynchronize(stream);
    nvtxRangePop();
}

void CUDAMemoryWrapper::copy_from_device(uint8_t *host_mem, size_t bytes_to_copy, const cudaStream_t &stream)
{
    nvtxRangePushA("CudaImageContainer:CopyFromDevice");
    if (bytes_to_copy > bytes_allocated_)
    {
        throw std::invalid_argument("Tried to copy too many bytes from device");
    }
    if (cudaMemcpyAsync(host_mem, cuda_mem_, bytes_to_copy, cudaMemcpyDeviceToHost, stream) != cudaSuccess)
    {
        throw std::runtime_error("Failed to copy memory from the GPU");
    }
    cudaStreamSynchronize(stream);
    nvtxRangePop();
}

uint8_t *CUDAMemoryWrapper::device_memory()
{
    return cuda_mem_;
}

CUDAMemoryWrapper::~CUDAMemoryWrapper()
{
    cudaFree(cuda_mem_);
}

CUDAEventWrapper::CUDAEventWrapper()
{
    cudaEventCreate(&event_);
}

void CUDAEventWrapper::record(std::shared_ptr<CUDAStreamWrapper> cuda_stream)
{
    cudaEventRecord(event_, cuda_stream->stream());
}

CUDAEventWrapper::~CUDAEventWrapper()
{
    cudaEventDestroy(event_);
}

CudaImageContainer::CudaImageContainer()
{
    cuda_stream_ = std::make_shared<CUDAStreamWrapper>();
    cuda_event_ = std::make_shared<CUDAEventWrapper>();
}

CudaImageContainer::CudaImageContainer(std_msgs::msg::Header header, uint32_t height, uint32_t width,
                                       std::string encoding, uint32_t step,
                                       std::shared_ptr<CUDAStreamWrapper> cuda_stream)
    : header_(header), cuda_stream_(cuda_stream), height_(height), width_(width), encoding_(encoding), step_(step)
{
    nvtxRangePushA("CudaImageContainer:Create");
    cuda_mem_ = std::make_shared<CUDAMemoryWrapper>(size_in_bytes());
    cuda_event_ = std::make_shared<CUDAEventWrapper>();
    nvtxRangePop();
}

CudaImageContainer::CudaImageContainer(std::unique_ptr<sensor_msgs::msg::Image> unique_sensor_msgs_image)
    : CudaImageContainer(
          NotNull(unique_sensor_msgs_image.get(), "unique_sensor_msgs_image cannot be nullptr").pointer->header,
          unique_sensor_msgs_image->height, unique_sensor_msgs_image->width, unique_sensor_msgs_image->encoding,
          unique_sensor_msgs_image->step)
{
    nvtxRangePushA("CudaImageContainer:CreateFromMessage");
    cuda_mem_->copy_to_device(&unique_sensor_msgs_image->data[0], size_in_bytes(), cuda_stream_->stream());
    nvtxRangePop();
}

CudaImageContainer::CudaImageContainer(const sensor_msgs::msg::Image &sensor_msgs_image)
    : CudaImageContainer(std::make_unique<sensor_msgs::msg::Image>(sensor_msgs_image))
{
}

CudaImageContainer::CudaImageContainer(const CudaImageContainer &other)
{
    nvtxRangePushA("CudaImageContainer:Copy");
    header_ = other.header_;
    height_ = other.height_;
    width_ = other.width_;
    encoding_ = other.encoding_;
    step_ = other.step_;

    // Make a new stream and have it wait on the previous one.
    cuda_stream_ = std::make_shared<CUDAStreamWrapper>();
    cuda_event_ = std::make_shared<CUDAEventWrapper>();
    cuda_event_->record(other.cuda_stream_);
    cudaStreamWaitEvent(cuda_stream_->stream(), cuda_event_->event());

    // Copy the image data over so that this is an independent copy (deep copy).
    cuda_mem_ = std::make_shared<CUDAMemoryWrapper>(size_in_bytes());
    if (cudaMemcpyAsync(other.cuda_mem_->device_memory(), cuda_mem_->device_memory(), size_in_bytes(),
                        cudaMemcpyDeviceToDevice, cuda_stream_->stream()) != cudaSuccess)
    {
        throw std::runtime_error("Failed to copy memory from the GPU");
    }
    nvtxRangePop();
}

// Equals operator makes this container become another, shallow copy only.
CudaImageContainer &CudaImageContainer::operator=(const CudaImageContainer &other)
{
    if (this == &other)
    {
        return *this;
    }

    header_ = other.header_;
    height_ = other.height_;
    width_ = other.width_;
    encoding_ = other.encoding_;
    step_ = other.step_;
    cuda_stream_ = other.cuda_stream_;
    cuda_event_ = other.cuda_event_;
    cuda_mem_ = other.cuda_mem_;

    return *this;
}

CudaImageContainer::~CudaImageContainer()
{
}

const std_msgs::msg::Header &CudaImageContainer::header() const
{
    return header_;
}

std_msgs::msg::Header &CudaImageContainer::header()
{
    return header_;
}

uint8_t *CudaImageContainer::cuda_mem()
{
    return cuda_mem_->device_memory();
}

void CudaImageContainer::get_sensor_msgs_image(sensor_msgs::msg::Image &destination) const
{
    nvtxRangePushA("CudaImageContainer:GetMsg");
    destination.header = header_;
    destination.height = height_;
    destination.width = width_;
    destination.encoding = encoding_;
    destination.step = step_;
    destination.data.resize(size_in_bytes());
    cuda_mem_->copy_from_device(&destination.data[0], size_in_bytes(), cuda_stream_->stream());
    nvtxRangePop();
}

size_t CudaImageContainer::size_in_bytes() const
{
    return height_ * step_;
}

} //  namespace shm_msgs