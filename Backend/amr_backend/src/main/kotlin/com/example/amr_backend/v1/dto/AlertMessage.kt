package com.example.amr_backend.v1.dto

import com.example.amr_backend.v1.entity.Notification
import io.swagger.v3.oas.annotations.media.Schema
import java.time.LocalDateTime

@Schema(description = "MQTT로 수신하는 알림 메시지")
data class AlertInboundMessage(
    @Schema(description = "AMR 시리얼 번호")
    val serial: String,
    @Schema(description = "알림 종류")
    val case: Case,
    @Schema(description = "알림 관련 이미지")
    val image: ByteArray,
    @Schema(description = "x 좌표")
    val x: Double,
    @Schema(description = "y 좌표")
    val y: Double,
    @Schema(description = "타임스탬프")
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

@Schema(description = "FCM으로 전송하는 알림 메시지")
data class AlertOutboundMessage(
    @Schema(description = "AMR 시리얼 번호")
    val serial: String,
    @Schema(description = "알림 제목")
    val title: String,
    @Schema(description = "알림 내용")
    val content: String,
    @Schema(description = "알림 발생 구역")
    val area: String,
    @Schema(description = "알림 종류")
    val case: Case,
    @Schema(description = "알림 관련 이미지 URL")
    val image: String,
    @Schema(description = "생성 시간")
    val createdAt: LocalDateTime = LocalDateTime.now(),
)

enum class Case {
    COLLAPSE, SMOKE, EQUIPMENT, FALL
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