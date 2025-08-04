package com.example.amr_backend.v1.dto

import com.example.amr_backend.v1.entity.AmrStatus
import com.example.amr_backend.v1.entity.Status

data class AmrDetailResponse(
    val name: String,
    val status: Status,
    val speed: Double,
    val model: String,
    val serial: String,
    val firmware: String,
    val ipAddress: String
)

fun AmrStatus.toAmrDetailResponse() = AmrDetailResponse(
    name = amr.name,
    status = status,
    speed = speed,
    model = amr.model,
    serial = amr.serial,
    firmware = amr.firmwareVersion,
    ipAddress = amr.ipAddress
)