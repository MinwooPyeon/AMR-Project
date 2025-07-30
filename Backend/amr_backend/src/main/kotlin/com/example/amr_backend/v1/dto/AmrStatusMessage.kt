package com.example.amr_backend.v1.dto

import com.example.amr_backend.v1.entity.Amr
import com.example.amr_backend.v1.entity.AmrStatus

data class AmrStatusMessage(
    val serial: String,
    val status: String,
    val batteryLevel: Int,
    val x: Double,
    val y: Double,
    val speed: Double
)

fun AmrStatusMessage.toEntity(amr: Amr) = AmrStatus(
    amr = amr,
    status = status,
    batteryLevel = batteryLevel,
    x = x,
    y = y,
    speed = speed
)