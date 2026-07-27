#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <unistd.h>
#include <vector>

#include <linux/videodev2.h>

#include <camera_info_manager/camera_info_manager.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>
#include <std_msgs/msg/string.hpp>

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

        rgb[out++] = clampToByte(y0 + (r_add >> 8));
        rgb[out++] = clampToByte(y0 + (g_add >> 8));
        rgb[out++] = clampToByte(y0 + (b_add >> 8));

        rgb[out++] = clampToByte(y1 + (r_add >> 8));
        rgb[out++] = clampToByte(y1 + (g_add >> 8));
        rgb[out++] = clampToByte(y1 + (b_add >> 8));
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

class V4L2CameraNode : public rclcpp::Node {
public:
    V4L2CameraNode() : Node("carm_a3_usb_camera") {
        device_ = declare_parameter<std::string>("device", "/dev/video0");
        width_ = declare_parameter<int>("width", 640);
        height_ = declare_parameter<int>("height", 480);
        fps_ = declare_parameter<int>("fps", 30);
        frame_id_ = declare_parameter<std::string>("frame_id", "carm_a3_camera_optical_frame");
        camera_name_ = declare_parameter<std::string>("camera_name", "carm_a3_camera");
        camera_info_url_ = declare_parameter<std::string>("camera_info_url", "");
        image_topic_ = declare_parameter<std::string>("image_topic", "/carm_a3/camera/image_raw");
        camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/carm_a3/camera/camera_info");
        set_camera_info_service_ =
            declare_parameter<std::string>("set_camera_info_service", "/carm_a3/camera/set_camera_info");
        diagnostics_topic_ = declare_parameter<std::string>("diagnostics_topic", "/carm_a3/camera/diagnostics");
        flip_vertical_ = declare_parameter<bool>("flip_vertical", false);
        flip_horizontal_ = declare_parameter<bool>("flip_horizontal", false);
        rotate_180_ = declare_parameter<bool>("rotate_180", true);

        image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, rclcpp::SensorDataQoS());
        camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, 2);
        diagnostics_pub_ = create_publisher<std_msgs::msg::String>(diagnostics_topic_, rclcpp::QoS(2).transient_local());
        camera_info_manager_ =
            std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_, camera_info_url_);
        set_camera_info_srv_ = create_service<sensor_msgs::srv::SetCameraInfo>(
            set_camera_info_service_,
            std::bind(&V4L2CameraNode::setCameraInfo, this, std::placeholders::_1, std::placeholders::_2));

        openDevice();
        configureDevice();
        initMmap();
        startStreaming();

        publishDiagnostics("camera_started");
        if (camera_info_manager_->isCalibrated()) {
            RCLCPP_INFO(get_logger(), "Loaded camera calibration: name=%s url=%s", camera_name_.c_str(),
                        camera_info_url_.c_str());
        } else {
            RCLCPP_WARN(get_logger(), "Camera calibration is not loaded; /camera_info will contain default intrinsics");
        }
        RCLCPP_INFO(get_logger(), "CArm A3 camera started: %s %dx%d@%d YUYV -> rgb8", device_.c_str(), width_,
                    height_, fps_);

        const auto period = std::chrono::duration<double>(1.0 / static_cast<double>(std::max(1, fps_)));
        timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                   std::bind(&V4L2CameraNode::captureOnce, this));
    }

    ~V4L2CameraNode() override {
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
        parm.parm.capture.timeperframe.denominator = static_cast<__u32>(std::max(1, fps_));
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
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "waiting for camera frame timed out");
            publishDiagnostics("frame_timeout");
            return;
        }

        v4l2_buffer buf {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (xioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
            if (errno != EAGAIN) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "VIDIOC_DQBUF failed: %s", std::strerror(errno));
            }
            return;
        }

        const auto* yuyv = static_cast<const unsigned char*>(buffers_[buf.index].start);
        yuyvToRgb8(yuyv, rgb_, width_, height_);
        orientRgb8(rgb_, width_, height_, flip_vertical_, flip_horizontal_, rotate_180_);
        publishFrame();

        if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "VIDIOC_QBUF failed after capture: %s",
                                 std::strerror(errno));
        }
    }

    void publishFrame() {
        const rclcpp::Time stamp = now();

        sensor_msgs::msg::Image image;
        image.header.stamp = stamp;
        image.header.frame_id = frame_id_;
        image.height = static_cast<unsigned int>(height_);
        image.width = static_cast<unsigned int>(width_);
        image.encoding = "rgb8";
        image.is_bigendian = 0;
        image.step = static_cast<unsigned int>(width_ * 3);
        image.data = rgb_;
        image_pub_->publish(image);

        sensor_msgs::msg::CameraInfo info = camera_info_manager_->getCameraInfo();
        info.header = image.header;
        info.height = image.height;
        info.width = image.width;
        if (info.distortion_model.empty()) {
            info.distortion_model = "plumb_bob";
        }
        camera_info_pub_->publish(info);
    }

    void setCameraInfo(const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> req,
                       std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> res) {
        req->camera_info.header.frame_id = frame_id_;
        req->camera_info.height = static_cast<unsigned int>(height_);
        req->camera_info.width = static_cast<unsigned int>(width_);
        res->success = camera_info_manager_->setCameraInfo(req->camera_info);
        res->status_message = res->success ? "camera_info accepted" : "camera_info rejected";
        RCLCPP_INFO(get_logger(), "set_camera_info request finished: %s", res->status_message.c_str());
        publishDiagnostics("camera_info_updated");
    }

    void publishDiagnostics(const std::string& text) {
        std_msgs::msg::String msg;
        std::ostringstream ss;
        ss << text << ",device=" << device_ << ",width=" << width_ << ",height=" << height_ << ",fps=" << fps_
           << ",rotate_180=" << (rotate_180_ ? "true" : "false");
        msg.data = ss.str();
        diagnostics_pub_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_pub_;
    rclcpp::Service<sensor_msgs::srv::SetCameraInfo>::SharedPtr set_camera_info_srv_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string device_;
    std::string frame_id_;
    std::string camera_name_;
    std::string camera_info_url_;
    std::string image_topic_;
    std::string camera_info_topic_;
    std::string set_camera_info_service_;
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
    std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<V4L2CameraNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "carm_a3_usb_camera failed: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
