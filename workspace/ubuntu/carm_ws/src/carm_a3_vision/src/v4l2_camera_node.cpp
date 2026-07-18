#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

#include <linux/videodev2.h>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>

namespace {

struct Buffer {
    void* start = nullptr;
    std::size_t length = 0;
};

int xioctl(const int fd, const unsigned long request, void* arg) {
    int ret = 0;
    do {
        ret = ioctl(fd, request, arg);
    } while (ret == -1 && errno == EINTR);
    return ret;
}

unsigned char clampToByte(const int value) {
    return static_cast<unsigned char>(std::max(0, std::min(255, value)));
}

void yuyvToRgb8(const unsigned char* yuyv, std::vector<unsigned char>& rgb, const int width, const int height) {
    rgb.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3U);

    std::size_t out = 0;
    const std::size_t total = static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 2U;
    for (std::size_t in = 0; in + 3 < total; in += 4) {
        const int y0 = static_cast<int>(yuyv[in + 0]);
        const int u = static_cast<int>(yuyv[in + 1]) - 128;
        const int y1 = static_cast<int>(yuyv[in + 2]);
        const int v = static_cast<int>(yuyv[in + 3]) - 128;

        const int r_add = 359 * v;
        const int g_add = -88 * u - 183 * v;
        const int b_add = 454 * u;

        rgb[out++] = clampToByte(y0 + ((r_add) >> 8));
        rgb[out++] = clampToByte(y0 + ((g_add) >> 8));
        rgb[out++] = clampToByte(y0 + ((b_add) >> 8));

        rgb[out++] = clampToByte(y1 + ((r_add) >> 8));
        rgb[out++] = clampToByte(y1 + ((g_add) >> 8));
        rgb[out++] = clampToByte(y1 + ((b_add) >> 8));
    }
}

void orientRgb8(std::vector<unsigned char>& rgb,
                const int width,
                const int height,
                const bool flip_vertical,
                const bool flip_horizontal,
                const bool rotate_180) {
    const bool do_vertical = flip_vertical ^ rotate_180;
    const bool do_horizontal = flip_horizontal ^ rotate_180;
    if (!do_vertical && !do_horizontal) {
        return;
    }

    std::vector<unsigned char> src = rgb;
    const int channels = 3;
    for (int y = 0; y < height; ++y) {
        const int src_y = do_vertical ? (height - 1 - y) : y;
        for (int x = 0; x < width; ++x) {
            const int src_x = do_horizontal ? (width - 1 - x) : x;
            const std::size_t dst_idx = (static_cast<std::size_t>(y) * width + x) * channels;
            const std::size_t src_idx = (static_cast<std::size_t>(src_y) * width + src_x) * channels;
            rgb[dst_idx + 0] = src[src_idx + 0];
            rgb[dst_idx + 1] = src[src_idx + 1];
            rgb[dst_idx + 2] = src[src_idx + 2];
        }
    }
}

}  // namespace

class V4L2CameraNode {
public:
    V4L2CameraNode() : nh_(), pnh_("~") {
        pnh_.param<std::string>("device", device_, "/dev/video0");
        pnh_.param<int>("width", width_, 640);
        pnh_.param<int>("height", height_, 480);
        pnh_.param<int>("fps", fps_, 30);
        pnh_.param<std::string>("frame_id", frame_id_, "carm_a3_camera_optical_frame");
        pnh_.param<std::string>("image_topic", image_topic_, "/carm_a3/camera/image_raw");
        pnh_.param<std::string>("camera_info_topic", camera_info_topic_, "/carm_a3/camera/camera_info");
        pnh_.param<std::string>("diagnostics_topic", diagnostics_topic_, "/carm_a3/camera/diagnostics");
        pnh_.param<bool>("flip_vertical", flip_vertical_, false);
        pnh_.param<bool>("flip_horizontal", flip_horizontal_, false);
        pnh_.param<bool>("rotate_180", rotate_180_, true);

        image_pub_ = nh_.advertise<sensor_msgs::Image>(image_topic_, 2);
        camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(camera_info_topic_, 2);
        diagnostics_pub_ = nh_.advertise<std_msgs::String>(diagnostics_topic_, 2, true);

        openDevice();
        configureDevice();
        initMmap();
        startStreaming();

        publishDiagnostics("camera_started");
        ROS_INFO("CArm A3 camera started: %s %dx%d@%d YUYV -> rgb8", device_.c_str(), width_, height_, fps_);
    }

    ~V4L2CameraNode() {
        stopStreaming();
        for (auto& buffer : buffers_) {
            if (buffer.start && buffer.start != MAP_FAILED) {
                munmap(buffer.start, buffer.length);
            }
        }
        if (fd_ >= 0) {
            close(fd_);
        }
    }

    void spin() {
        ros::Rate rate(fps_ > 0 ? fps_ : 30);
        while (ros::ok()) {
            captureOnce();
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    void openDevice() {
        fd_ = open(device_.c_str(), O_RDWR | O_NONBLOCK, 0);
        if (fd_ < 0) {
            throw std::runtime_error("failed to open " + device_ + ": " + std::strerror(errno));
        }
    }

    void configureDevice() {
        v4l2_capability cap {};
        if (xioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) {
            throw std::runtime_error("VIDIOC_QUERYCAP failed");
        }
        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) || !(cap.capabilities & V4L2_CAP_STREAMING)) {
            throw std::runtime_error("device does not support video capture streaming");
        }

        v4l2_format fmt {};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = static_cast<__u32>(width_);
        fmt.fmt.pix.height = static_cast<__u32>(height_);
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
        if (xioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
            throw std::runtime_error("VIDIOC_S_FMT YUYV failed");
        }

        width_ = static_cast<int>(fmt.fmt.pix.width);
        height_ = static_cast<int>(fmt.fmt.pix.height);
        if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV) {
            throw std::runtime_error("camera did not accept YUYV format");
        }

        v4l2_streamparm parm {};
        parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        parm.parm.capture.timeperframe.numerator = 1;
        parm.parm.capture.timeperframe.denominator = static_cast<__u32>(fps_);
        xioctl(fd_, VIDIOC_S_PARM, &parm);
    }

    void initMmap() {
        v4l2_requestbuffers req {};
        req.count = 4;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        if (xioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
            throw std::runtime_error("VIDIOC_REQBUFS failed");
        }
        if (req.count < 2) {
            throw std::runtime_error("insufficient V4L2 buffers");
        }

        buffers_.resize(req.count);
        for (std::size_t i = 0; i < buffers_.size(); ++i) {
            v4l2_buffer buf {};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = static_cast<__u32>(i);
            if (xioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
                throw std::runtime_error("VIDIOC_QUERYBUF failed");
            }
            buffers_[i].length = buf.length;
            buffers_[i].start = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
            if (buffers_[i].start == MAP_FAILED) {
                throw std::runtime_error("mmap failed");
            }
        }
    }

    void startStreaming() {
        for (std::size_t i = 0; i < buffers_.size(); ++i) {
            v4l2_buffer buf {};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = static_cast<__u32>(i);
            if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
                throw std::runtime_error("VIDIOC_QBUF failed");
            }
        }
        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
            throw std::runtime_error("VIDIOC_STREAMON failed");
        }
        streaming_ = true;
    }

    void stopStreaming() {
        if (!streaming_ || fd_ < 0) {
            return;
        }
        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl(fd_, VIDIOC_STREAMOFF, &type);
        streaming_ = false;
    }

    void captureOnce() {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd_, &fds);
        timeval tv {};
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        const int ret = select(fd_ + 1, &fds, nullptr, nullptr, &tv);
        if (ret <= 0) {
            ROS_WARN_THROTTLE(2.0, "waiting for camera frame timed out");
            publishDiagnostics("frame_timeout");
            return;
        }

        v4l2_buffer buf {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (xioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
            if (errno != EAGAIN) {
                ROS_WARN_THROTTLE(2.0, "VIDIOC_DQBUF failed: %s", std::strerror(errno));
            }
            return;
        }

        const auto* yuyv = static_cast<const unsigned char*>(buffers_[buf.index].start);
        yuyvToRgb8(yuyv, rgb_, width_, height_);
        orientRgb8(rgb_, width_, height_, flip_vertical_, flip_horizontal_, rotate_180_);
        publishFrame();

        if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
            ROS_WARN_THROTTLE(2.0, "VIDIOC_QBUF failed after capture: %s", std::strerror(errno));
        }
    }

    void publishFrame() {
        const ros::Time stamp = ros::Time::now();

        sensor_msgs::Image image;
        image.header.stamp = stamp;
        image.header.frame_id = frame_id_;
        image.height = static_cast<unsigned int>(height_);
        image.width = static_cast<unsigned int>(width_);
        image.encoding = "rgb8";
        image.is_bigendian = 0;
        image.step = static_cast<unsigned int>(width_ * 3);
        image.data = rgb_;
        image_pub_.publish(image);

        sensor_msgs::CameraInfo info;
        info.header = image.header;
        info.height = image.height;
        info.width = image.width;
        info.distortion_model = "plumb_bob";
        camera_info_pub_.publish(info);
    }

    void publishDiagnostics(const std::string& text) {
        std_msgs::String msg;
        std::ostringstream ss;
        ss << text << ",device=" << device_ << ",width=" << width_ << ",height=" << height_ << ",fps=" << fps_
           << ",rotate_180=" << (rotate_180_ ? "true" : "false");
        msg.data = ss.str();
        diagnostics_pub_.publish(msg);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher image_pub_;
    ros::Publisher camera_info_pub_;
    ros::Publisher diagnostics_pub_;

    std::string device_;
    std::string frame_id_;
    std::string image_topic_;
    std::string camera_info_topic_;
    std::string diagnostics_topic_;
    int width_ = 640;
    int height_ = 480;
    int fps_ = 30;
    bool flip_vertical_ = false;
    bool flip_horizontal_ = false;
    bool rotate_180_ = true;

    int fd_ = -1;
    bool streaming_ = false;
    std::vector<Buffer> buffers_;
    std::vector<unsigned char> rgb_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "carm_a3_usb_camera");
    try {
        V4L2CameraNode node;
        node.spin();
    } catch (const std::exception& e) {
        ROS_FATAL("carm_a3_usb_camera failed: %s", e.what());
        return 1;
    }
    return 0;
}
