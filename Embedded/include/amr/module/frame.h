/**
 * @file frame.h
 * @brief Defines the Frame class for managing captured video frame buffers.
 *
 * The Frame class encapsulates raw frame data captured from a V4L2 webcam.
 * It owns a dynamically allocated buffer, copies the provided data into it,
 * and releases the buffer when destroyed.
 */

#ifndef FRAME_H
#define FRAME_H

#include <cstdint>
#include <cstddef>

 /**
  * @class Frame
  * @brief Encapsulates and manages video frame data.
  *
  * The Frame object owns a dynamically allocated buffer containing raw
  * frame data. On construction, it copies the provided data into its
  * internal buffer. On destruction, it frees the allocated memory.
  */
class Frame {
public:
    /**
     * @brief Constructs a Frame by copying the provided buffer.
     * @param data    Pointer to raw frame data.
     * @param length  Length of the data buffer in bytes.
     */
    Frame(const uint8_t* data, size_t length);

    /**
     * @brief Destructor that frees the internal buffer.
     */
    ~Frame();

    /**
     * @brief Returns a pointer to the internal frame data.
     * @return Const pointer to the frame data buffer.
     */
    const uint8_t* data() const;

    /**
     * @brief Returns the size of the internal data buffer.
     * @return Buffer length in bytes.
     */
    size_t size() const;

private:
    uint8_t* buffer_;   /**< Pointer to the owned frame data buffer. */
    size_t   length_;   /**< Length of the data buffer in bytes. */
};

#endif // FRAME_H
