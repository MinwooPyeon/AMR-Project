package com.example.amr_backend.v1.dto

import org.springframework.boot.actuate.endpoint.OperationType

data class AmrManualControlMessage(
    val serial: String,
    val operationType: OperationType,
)

enum class OperationType {
    MOVE_FORWARD, MOVE_BACKWARD, TURN_RIGHT, TURN_LEFT
}