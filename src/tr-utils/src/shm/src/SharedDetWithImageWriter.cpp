#include "../include/shm/SharedDetWithImage.h"
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

SharedDetWithImageWriter::SharedDetWithImageWriter(const std::string &name, int height, int width,
                                                   int channels, const std::string &data_type)
    : shm_name_(name), shm_fd_(-1), shm_ptr_(nullptr), channels_(channels) {
    if (data_type == "CV_8U")
        data_size_ = 1;
    else if (data_type == "CV_16U")
        data_size_ = 2;
    else if (data_type == "CV_32F")
        data_size_ = 4;
    else if (data_type == "CV_64F")
        data_size_ = 8;
    else
        data_size_ = 1;

    size_t img_bytes = height * width * channels * data_size_;
    size_t shm_size = sizeof(SharedDetWithImage) + 2 * img_bytes;

    shm_fd_ = shm_open(shm_name_.c_str(), O_CREAT | O_EXCL | O_RDWR, 0666);
    if (shm_fd_ == -1 && errno == EEXIST) {
        // A stale segment exists (e.g. previous writer crashed without
        // cleanup). Unlink it so we always start with a fresh, properly-sized
        // segment.
        shm_unlink(shm_name_.c_str());
        shm_fd_ = shm_open(shm_name_.c_str(), O_CREAT | O_EXCL | O_RDWR, 0666);
    }
    if (shm_fd_ == -1) {
        std::cerr << "Failed to create shared memory. Error: " << strerror(errno) << std::endl;
        return;
    }
    if (ftruncate(shm_fd_, shm_size) == -1) {
        std::cerr << "Failed to set size of shared memory. Error: " << strerror(errno) << std::endl;
        close(shm_fd_);
        shm_fd_ = -1;
        return;
    }
    shm_ptr_ = static_cast<SharedDetWithImage *>(
        mmap(0, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0));
    if (shm_ptr_ == MAP_FAILED) {
        std::cerr << "Failed to map shared memory. Error: " << strerror(errno) << std::endl;
        close(shm_fd_);
        shm_fd_ = -1;
        return;
    }

    shm_ptr_->front_idx.store(0, std::memory_order_relaxed);
    shm_ptr_->frame[0] = shm_ptr_->frame[1] = 0;
    shm_ptr_->timeStamp[0] = shm_ptr_->timeStamp[1] = 0;
    shm_ptr_->size = img_bytes;
    shm_ptr_->height = height;
    shm_ptr_->width = width;
}

SharedDetWithImageWriter::~SharedDetWithImageWriter() {
    if (shm_ptr_ && shm_ptr_ != MAP_FAILED) {
        size_t shm_size = sizeof(SharedDetWithImage) - sizeof(unsigned char) + 2 * shm_ptr_->size;
        munmap(shm_ptr_, shm_size);
    }
    if (shm_fd_ != -1) {
        close(shm_fd_);
        shm_unlink(shm_name_.c_str());
    }
}

bool SharedDetWithImageWriter::writeDetWithImage(const cv::Mat &image,
                                                 const Detection2DArray &detections, long &ns) {
    if (!shm_ptr_)
        return false;
    if (image.empty() || image.total() * image.elemSize() != shm_ptr_->size)
        return false;

    int next_idx = 1 - shm_ptr_->front_idx.load(std::memory_order_acquire);

    // Step 1: Write Image Data

    // Calculate buffer addresses: buffer 0 starts at data, buffer 1 starts at
    // data + size
    unsigned char *target_img_buffer = shm_ptr_->img_buffer + (next_idx * shm_ptr_->size);

    // Write to the non-front img buffer
    memcpy(target_img_buffer, image.data, shm_ptr_->size);

    // pass in custom timestamps
    shm_ptr_->timeStamp[next_idx] = ns;

    // Copy detections
    shm_ptr_->detections_buffer[next_idx] = detections;

    // Step 2: Write Detection Data
    shm_ptr_->detections_buffer[next_idx] = detections;

    shm_ptr_->frame[next_idx] = shm_ptr_->frame[1 - next_idx] + 1;
    // Atomically publish the new buffer
    shm_ptr_->front_idx.store(next_idx, std::memory_order_release);

    return true;
}