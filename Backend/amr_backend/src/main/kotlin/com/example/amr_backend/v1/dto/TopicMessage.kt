package com.example.amr_backend.v1.dto

sealed interface TopicMessage {
    data class AmrManualControlMessage(
        val serial: String,
        val area: String
    ) : TopicMessage
}