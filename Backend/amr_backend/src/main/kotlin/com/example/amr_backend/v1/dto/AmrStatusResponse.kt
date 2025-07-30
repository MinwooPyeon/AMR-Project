package com.example.amr_backend.v1.dto

import com.example.amr_backend.v1.entity.AmrStatus

data class AmrStatusResponse(
    val id: Long,
    val name: String,
    val status: String,
    val speed: Double,
    val battery: Int
)

fun AmrStatus.toResponse() = AmrStatusResponse(
    id = id,
    name = amr.name,
    status = status,
    speed = speed,
    battery = batteryLevel
)