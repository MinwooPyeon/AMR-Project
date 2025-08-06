package com.example.amr_backend.v1.dto

import com.example.amr_backend.v1.entity.AmrStatus
import com.example.amr_backend.v1.entity.State

data class AmrDetailResponse(
    val id: Long,
    val name: String,
    val state: State,
    val speed: Double,
    val model: String,
    val serial: String,
    val firmware: String,
    val ipAddress: String
)

fun AmrStatus.toAmrDetailResponse() = AmrDetailResponse(
    id = id,
    name = amr.name,
    state = state,
    speed = speed,
    model = amr.model,
    serial = amr.serial,
    firmware = amr.firmwareVersion,
    ipAddress = amr.ipAddress
)