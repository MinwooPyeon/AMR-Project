package com.example.amr_backend.v1.dto

sealed interface TopicMessage {
    data class AmrManualControlMessage(val area: String) : TopicMessage
}