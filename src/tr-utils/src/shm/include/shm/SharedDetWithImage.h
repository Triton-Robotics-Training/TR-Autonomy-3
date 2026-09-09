#ifndef SHARED_DET_W_IMAGE_H
#define SHARED_DET_W_IMAGE_H

#include "Detections.h"
#include "SharedImage.h"
#include <atomic>
#include <opencv2/opencv.hpp>
#include <string>

// Structure defining the layout of shared memory (double buffer)
struct SharedDetWithImage {
    std::atomic<int> front_idx; // 0 or 1: which buffer is current (detections + data)
    int frame[2];
    long timeStamp[2];  // time image was grabbed from camera
    unsigned long size; // size of one image buffer in bytes
    int height;
    int width;
    Detection2DArray detections_buffer[2]; // Two buffers for Detections
    unsigned char img_buffer[];            // Flexible array member, real size set at allocation
};

class SharedDetWithImageWriter {
  public:
    SharedDetWithImageWriter(const std::string &name, int height, int width, int channels,
                             const std::string &data_type);
    ~SharedDetWithImageWriter();
    bool writeDetWithImage(const cv::Mat &image, const Detection2DArray &detections, long &ts);

  private:
    std::string shm_name_;
    int shm_fd_;
    int channels_;
    int data_size_;
    SharedDetWithImage *shm_ptr_;
};

class SharedDetWithImageReader {
  public:
    SharedDetWithImageReader(const std::string &name, int height, int width, int channels,
                             const std::string &data_type);
    ~SharedDetWithImageReader();
    bool readDetWithImage(cv::Mat &image, Detection2DArray &detections, int &last_frame);
    bool isInitialized() const;
    long getTimeStamp();
    int getFrameNumber();

  private:
    std::string shm_name_;
    int shm_fd_;
    SharedDetWithImage *shm_ptr_;
    int channels_;
    int data_size_;
    int height_;
    int width_;
    bool initialized_;
    bool tryOpen();
    bool isSegmentStale() const;
};

#endif // SHARED_IMAGE_H