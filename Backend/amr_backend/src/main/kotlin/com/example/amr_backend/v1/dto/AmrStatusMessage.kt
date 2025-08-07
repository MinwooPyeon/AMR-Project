package com.example.amr_backend.v1.dto

import com.example.amr_backend.v1.entity.Amr
import com.example.amr_backend.v1.entity.AmrStatus
import com.example.amr_backend.v1.entity.State

data class AmrStatusMessage(
    val serial: String,
    val state: State,
    val x: Double,
    val y: Double,
    val speed: Double,
    val angle: Double,
)

fun AmrStatusMessage.toAmrStatus(amr: Amr) = AmrStatus(
    amr = amr,
    state = state,
    x = x,
    y = y,
    speed = speed,
    angle = angle
)