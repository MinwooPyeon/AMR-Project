package com.example.amr_backend.v1.entity

sealed interface TopicMessage {
    data class AmrManualControlMessage(val area: String) : TopicMessage
}