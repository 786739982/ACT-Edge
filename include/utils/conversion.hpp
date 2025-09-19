#pragma once
#include <cstdint>
#include <cstring>
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

void BGR2NV12Tensor(const uint8_t* bgr_bytes, uint32_t height, uint32_t width, uint8_t* nv12_bytes)
{
    cv::Mat rgb_mat = cv::Mat(height, width, CV_8UC3, (void*)bgr_bytes);
    cv::Mat yuv_mat;
    cv::cvtColor(rgb_mat, yuv_mat, cv::COLOR_BGR2YUV_I420);
    uint8_t* nv12_data = yuv_mat.ptr<uint8_t>();

    // copy y data
    int32_t y_size = height * width;
    memcpy(nv12_bytes, nv12_data, y_size);

    // copy uv data
    int32_t uv_height = height / 2;
    int32_t uv_width = width / 2;
    uint8_t* nv12 = nv12_bytes + y_size;
    uint8_t* u_data = nv12_data + y_size;
    uint8_t* v_data = nv12_data + y_size + uv_height * uv_width;

    for (int32_t i = 0; i < uv_width * uv_height; i++) {
        if (u_data && v_data) {
            *nv12++ = *u_data++;
            *nv12++ = *v_data++;
        }
    }
}

// void BGR2BGRTensor(std::vector<float>& chw_bgr_data, uint32_t height, uint32_t width, float* nv12_bytes)
void BGR2BGRTensor(cv::Mat bgr_mat, uint32_t height, uint32_t width, float* nv12_bytes)
{
    // cv::Mat bgr_mat = cv::Mat(height, width, CV_8UC3, (void*)bgr_bytes);
    cv::Mat bgr_mat_float;
    bgr_mat.convertTo(bgr_mat_float, CV_32F);
    bgr_mat_float /= 255.0f;

    if (!bgr_mat_float.isContinuous()) {
        bgr_mat_float = bgr_mat_float.clone(); // 确保数据连续
    }
    float* nv12_data = reinterpret_cast<float*>(bgr_mat_float.data);
    // copy y data
    int32_t data_size = height * width * 3 * 4;
    memcpy(nv12_bytes, nv12_data, data_size);

}

void GetJointTensor(std::vector<float>& joints, const hbDNNTensorProperties& tensor_properties, float* mem_addr)
{
    // Write joint data to memory
    memcpy(mem_addr, joints.data(), tensor_properties.alignedByteSize);
    
}

void convertHWCtoCHW(const cv::Mat& img, std::vector<float>& chw_data) {
    // 检查输入图像类型和通道数
    if (img.type() != CV_8UC3) {
        throw std::runtime_error("Input image must be of type CV_8UC3 (3-channel uchar)");
    }

    // 确保图像数据连续
    cv::Mat continuous_img = img.isContinuous() ? img : img.clone();

    // 图像尺寸
    int C = img.channels();
    int H = img.rows;
    int W = img.cols;

    // 预分配 CHW 数据存储
    chw_data.resize(C * H * W);

    // 直接操作内存，提高效率
    const uchar* data = continuous_img.ptr<uchar>(); // 原始 HWC 数据指针

    for (int c = 0; c < C; ++c) {
        for (int h = 0; h < H; ++h) {
            for (int w = 0; w < W; ++w) {
                // HWC 格式偏移计算
                int hwc_index = h * W * C + w * C + c;
                // CHW 格式偏移计算
                int chw_index = c * H * W + h * W + w;
                // 赋值
                chw_data[chw_index] = static_cast<float>(data[hwc_index]);
            }
        }
    }
}