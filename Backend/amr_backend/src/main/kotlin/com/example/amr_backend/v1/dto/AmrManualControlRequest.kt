package com.example.amr_backend.v1.dto

data class AmrManualControlRequest(
    val serial: String,
    val area: String,
)

fun AmrManualControlRequest.toAmrManualControlMessage() = TopicMessage.AmrManualControlMessage(
    serial = serial,
    area = area
)