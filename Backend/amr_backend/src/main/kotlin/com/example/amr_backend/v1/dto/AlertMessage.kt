package com.example.amr_backend.v1.dto

import java.time.LocalDateTime

data class AlertInboundMessage(
    val case: Case,
    val image: ByteArray,
    val x: Double,
    val y: Double,
    val timeStamp: LocalDateTime,
) {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as AlertInboundMessage

        if (case != other.case) return false
        if (!image.contentEquals(other.image)) return false
        if (x != other.x) return false
        if (y != other.y) return false
        if (timeStamp != other.timeStamp) return false

        return true
    }

    override fun hashCode(): Int {
        var result = case.hashCode()
        result = 31 * result + image.contentHashCode()
        result = 31 * result + x.hashCode()
        result = 31 * result + y.hashCode()
        result = 31 * result + timeStamp.hashCode()
        return result
    }
}

data class AlertOutboundMessage(
    val title: String,
    val summary: String,
    val area: String,
    val image: String,
    val createdAt: LocalDateTime = LocalDateTime.now(),
)

enum class Case {
    COLLAPSE, SMOKE, EQUIPMENT
}