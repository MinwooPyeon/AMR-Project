#include "amr/module/frame.h"
#include <cstring>

Frame::Frame(const uint8_t* data, size_t length)
	: length_(length) {
	buffer_ = new uint8_t[length];
	std::memcpy(buffer_, data, length);
}

Frame::~Frame(){
	delete[] buffer_;
}

const uint8_t* Frame::data() const { return buffer_; }
size_t Frame::size()const { return length_; }
