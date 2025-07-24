#include "amr/module/web_cam.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <iostream>

WebCam::WebCam(const std::string& device)
	: dev_name_(device) {
}

WebCam::~WebCam() {
	stopCapturing();
	closeDevice();
}

void WebCam::openDevice() {
	fd_ = open(dev_name_.c_str(), O_RDWR | O_NONBLOCK, 0);
	if (fd_ < 0) errnoExit("open");

	v4l2_capability cap{};
	if (ioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) errnoExit("VIDIOC_QUERYCAP");
	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
		throw std::runtime_error("Device does not support video capture");
	if (!(cap.capabilities & V4L2_CAP_STREAMING))
		throw std::runtime_error("Device does not support streaming I/O");
}

void WebCam::initDevice(int width, int height, uint32_t pixformat)
{
	width_ = width;
	height_ = height;
	pixformat_ = pixformat;
	v4l2_format fmt{};
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width = width_;
	fmt.fmt.pix.height = height_;
	fmt.fmt.pix.pixelformat = pixformat_;
	fmt.fmt.pix.field = V4L2_FIELD_ANY;
	if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) errnoExit("VIDIOC_S_FMT");

	initMMap();
}

void WebCam::initMMap() {
	v4l2_requestbuffers req{};
	req.count = 4;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;
	if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0) errnoExit("VIDIOC_REQBUFS");
	buffers_.resize(req.count);

	for (size_t i = 0; i < buffers_.size(); ++i) {
		v4l2_buffer buf{};
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
		if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) errnoExit("VIDIOC_QUERYBUF");

		buffers_[i].length = buf.length;
		buffers_[i].start = mmap(nullptr, buf.length,
			PROT_READ | PROT_WRITE, MAP_SHARED,
			fd_, buf.m.offset);
		if (buffers_[i].start == MAP_FAILED) errnoExit("mmap");
	}
}

void WebCam::startCapturing() {
	for (size_t i = 0; i < buffers_.size(); ++i) {
		v4l2_buffer buf{};
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
		if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) errnoExit("VIDIOC_QBUF");
	}
	v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) errnoExit("VIDIOC_STREAMON");
}

Frame WebCam::captureFrame() {
	while (true) {
		v4l2_buffer buf{};
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
			if (errno == EAGAIN) continue;
			errnoExit("VIDIOC_DQBUF");
		}
		auto length = buf.bytesused;
		auto data = new uint8_t[length];
		memcpy(data, buffers_[buf.index].start, length);
		if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) errnoExit("VIDIOC_QBUF");
		return Frame(data, length);
	}
}

void WebCam::stopCapturing() {
	if (fd_ < 0) return;
	v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ioctl(fd_, VIDIOC_STREAMOFF, &type);
}

void WebCam::closeDevice() {
	if (fd_ >= 0) {
		for (auto& buf : buffers_) {
			munmap(buf.start, buf.length);
		}
		close(fd_);
		fd_ = -1;
	}
}

void WebCam::errnoExit(const char* s) {
	perror(s);
	throw std::runtime_error(std::string(s) + " failed");
}

