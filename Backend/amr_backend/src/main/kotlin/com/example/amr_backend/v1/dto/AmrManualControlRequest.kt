package com.example.amr_backend.v1.dto

data class AmrManualControlRequest(
    val serial: String,
    val area: String,
)

data class AmrManualControlMessage(
    val area: String
)