/**
 * @file web_cam.h
 * @brief Declares the WebCam class for capturing video frames using the V4L2 API.
 *
 * This header defines the WebCam interface, which manages device setup,
 * memory-mapped buffer initialization, streaming control, and frame capture
 * from a specified V4L2 video device.
 */

#ifndef WEB_CAM_H
#define WEB_CAM_H

#include <string>
#include <vector>
#include <cstdint>
#include <linux/videodev2.h>
#include "amr/module/frame.h"

 /**
  * @class WebCam
  * @brief Encapsulates V4L2-based video capture functionality.
  *
  * The WebCam class provides methods to open and configure a video device,
  * set up memory-mapped buffers, start and stop streaming, and capture
  * individual frames into Frame objects.
  */
class WebCam {
public:
    /**
     * @brief Constructs a WebCam for the specified device path.
     * @param device Path to the V4L2 video device (e.g., "/dev/video0").
     */
    explicit WebCam(const std::string& device = "/dev/video0");

    /**
     * @brief Destructor that stops streaming and closes the device if open.
     */
    ~WebCam();

    /**
     * @brief Opens the video device and queries its capabilities.
     * @throws std::runtime_error on failure.
     */
    void openDevice();

    /**
     * @brief Sets the capture format (width, height, pixel format).
     * @param width       Frame width in pixels.
     * @param height      Frame height in pixels.
     * @param pixformat   FourCC pixel format (default V4L2_PIX_FMT_YUVV).
     * @throws std::runtime_error on failure.
     */
    void initDevice(int width, int height, uint32_t pixformat = V4L2_PIX_FMT_YUVV);

    /**
     * @brief Initializes memory mapping for frame buffers.
     * @throws std::runtime_error on failure.
     */
    void initMMap();

    /**
     * @brief Starts streaming by enqueuing buffers and turning on capture.
     * @throws std::runtime_error on failure.
     */
    void startCapturing();

    /**
     * @brief Dequeues a buffer, wraps it in a Frame, and re-queues it.
     * @return Captured Frame object containing image data.
     * @throws std::runtime_error on I/O errors.
     */
    Frame captureFrame();

    /**
     * @brief Stops streaming on the video device.
     */
    void stopCapturing();

    /**
     * @brief Closes the video device and releases resources.
     */
    void closeDevice();

private:
    struct Buffer {
        void* start;   /**< Pointer to mapped buffer memory. */
        size_t length;  /**< Size of the mapped buffer in bytes. */
    };

    std::string       dev_name_;   /**< Video device path. */
    int               fd_ = -1;    /**< File descriptor for the device. */
    std::vector<Buffer> buffers_;   /**< MMAP buffers for streaming. */
    int               width_ = 0;  /**< Configured frame width. */
    int               height_ = 0; /**< Configured frame height. */
    uint32_t          pixformat_ = 0; /**< Configured pixel format. */

    /**
     * @brief Helper to report an ioctl error and throw a runtime exception.
     * @param s Descriptive message of the failed operation.
     * @throws std::runtime_error always.
     */
    [[noreturn]] void errnoExit(const char* s);
};

#endif // WEB_CAM_H
