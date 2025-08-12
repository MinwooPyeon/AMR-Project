package com.example.amr_backend.v1.dto

import com.example.amr_backend.v1.entity.Notification
import java.time.LocalDateTime

data class AlertInboundMessage(
    val serial: String,
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
    val serial: String,
    val title: String,
    val content: String,
    val area: String,
    val case: Case,
    val image: String,
    val createdAt: LocalDateTime = LocalDateTime.now(),
)

enum class Case {
    COLLAPSE, SMOKE, EQUIPMENT
}

fun AlertOutboundMessage.toEntity() = Notification(
    serial = serial,
    title = title,
    content = content,
    area = area,
    case = case,
    image = image,
    isRead = false,
    readAt = null,
)