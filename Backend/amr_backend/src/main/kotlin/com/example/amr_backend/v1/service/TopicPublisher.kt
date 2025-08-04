package com.example.amr_backend.v1.service

import com.example.amr_backend.v1.entity.TopicMessage

interface TopicPublisher {
    fun publish(topic: String, message: TopicMessage)
}