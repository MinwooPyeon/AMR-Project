package com.example.amr_backend.v1.dto

import com.example.amr_backend.v1.entity.AmrStatus
import com.example.amr_backend.v1.entity.State

data class AmrStatusResponse(
    val id: Long,
    val name: String,
    val state: State,
    val speed: Double,
    val x: Double,
    val y: Double,
)

fun AmrStatus.toAmrStatusResponse() = AmrStatusResponse(
    id = id,
    name = amr.name,
    state = state,
    speed = speed,
    x = x,
    y = y
)