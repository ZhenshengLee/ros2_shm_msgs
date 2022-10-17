#include <iostream>
#include <stdlib.h>
#include <stdio.h>

__global__ void cuda_hello(){
    printf("Hello World from GPU! %d\n", threadIdx.x*gridDim.x);
}

int main() {
    printf("Hello World from CPU!\n");
    cudaSetDevice(0);
    cuda_hello<<<1,10>>>();
    uint8_t *cuda_mem_;
    if (cudaMalloc(&cuda_mem_, 1024) != cudaSuccess)
    {
        throw std::runtime_error("Failed to allocate device memory");
    }

    cudaDeviceSynchronize();
    return 0;
}