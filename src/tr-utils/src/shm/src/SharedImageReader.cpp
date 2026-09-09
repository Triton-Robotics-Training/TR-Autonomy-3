#include "../include/shm/SharedImage.h"
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

SharedImageReader::SharedImageReader(const std::string &name, int height, int width, int channels,
                                     const std::string &data_type)
    : shm_name_(name), shm_fd_(-1), shm_ptr_(nullptr), channels_(channels), height_(height),
      width_(width), initialized_(false) {
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
    // Try to open shared memory; if it doesn't exist yet, leave uninitialized
    // and allow retry on subsequent reads.
    tryOpen();
}

SharedImageReader::~SharedImageReader() {
    if (shm_ptr_ && shm_ptr_ != MAP_FAILED) {
        size_t shm_size = sizeof(SharedImage) - sizeof(unsigned char) + 2 * shm_ptr_->size;
        munmap(shm_ptr_, shm_size);
    }
    if (shm_fd_ != -1) {
        close(shm_fd_);
    }
}

bool SharedImageReader::isInitialized() const { return initialized_; }

bool SharedImageReader::readImage(cv::Mat &image, int &last_frame) {
    // If the writer restarted it will have unlinked and recreated the segment
    // under the same name (new inode). Detect this so we remap automatically
    // rather than forever reading from the orphaned old segment.
    if (initialized_ && isSegmentStale()) {
        std::cerr << "[SharedImageReader] Segment recreated by writer, remapping..." << std::endl;
        initialized_ = false; // tryOpen() will clean up old mapping and remap
    }
    if (!isInitialized()) {
        if (!tryOpen())
            return false;
    }

    int idx = shm_ptr_->front_idx.load(std::memory_order_acquire);
    int frame = shm_ptr_->frame[idx];

    if (frame == last_frame)
        return false; // No new frame

    // Calculate buffer address
    unsigned char *source_buffer = shm_ptr_->data + (idx * shm_ptr_->size);

    // Read the image
    if (this->channels_ == 1) {
        image = cv::Mat(shm_ptr_->height, shm_ptr_->width, CV_8UC1, source_buffer).clone();
    } else {
        image = cv::Mat(shm_ptr_->height, shm_ptr_->width, CV_8UC3, source_buffer).clone();
    }
    last_frame = frame;
    return true;
}

long SharedImageReader::getTimeStamp() {
    if (!isInitialized()) {
        if (!tryOpen())
            return -1;
    }
    int idx = shm_ptr_->front_idx.load(std::memory_order_acquire);
    return shm_ptr_->timeStamp[idx];
}

int SharedImageReader::getFrameNumber() {
    if (!isInitialized()) {
        if (!tryOpen())
            return -1;
    }
    int idx = shm_ptr_->front_idx.load(std::memory_order_acquire);
    return shm_ptr_->frame[idx];
}

bool SharedImageReader::tryOpen() {
    size_t img_bytes = static_cast<size_t>(height_) * static_cast<size_t>(width_) *
                       static_cast<size_t>(channels_) * static_cast<size_t>(data_size_);
    size_t shm_size = sizeof(SharedImage) + 2 * img_bytes;

    // Clean up any existing (possibly stale) mapping before attempting to
    // reopen.
    if (shm_ptr_ && shm_ptr_ != MAP_FAILED) {
        munmap(shm_ptr_, shm_size);
        shm_ptr_ = nullptr;
    }
    if (shm_fd_ != -1) {
        close(shm_fd_);
        shm_fd_ = -1;
    }
    initialized_ = false;

    int fd = shm_open(shm_name_.c_str(), O_RDWR, 0666);
    if (fd == -1) {
        std::cerr << "Failed to map shared memory for reading. Error: " << strerror(errno)
                  << std::endl;
        initialized_ = false;
        return false;
    }
    void *mapped = mmap(0, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (mapped == MAP_FAILED) {
        std::cerr << "Failed to map shared memory for reading. Error: " << strerror(errno)
                  << std::endl;
        close(fd);
        initialized_ = false;
        return false;
    }

    shm_fd_ = fd;
    shm_ptr_ = static_cast<SharedImage *>(mapped);
    initialized_ = true;
    return true;
}

bool SharedImageReader::isSegmentStale() const {
    if (!initialized_ || shm_fd_ == -1)
        return false;
    // Open the segment by name and compare inodes. If they differ the writer
    // has unlinked the old segment and created a new one — our mapping is
    // stale.
    int fd = shm_open(shm_name_.c_str(), O_RDWR, 0666);
    if (fd == -1)
        return true; // Segment no longer exists by name (writer not running
                     // yet)
    struct stat cur_stat, new_stat;
    bool stale = true;
    if (fstat(shm_fd_, &cur_stat) == 0 && fstat(fd, &new_stat) == 0)
        stale = (cur_stat.st_ino != new_stat.st_ino);
    close(fd);
    return stale;
}
