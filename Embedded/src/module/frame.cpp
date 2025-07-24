#include "amr/module/frame.h"

Frame::Frame(uint8_t* data, size_t length)
	: buffer_(data), length_(length){
}

Frame::~Frame(){
	delete[] buffer_;
}

const uint8_t* Frame::data() const { return buffer_; }
size_t Frame::size()const { return length_; }
