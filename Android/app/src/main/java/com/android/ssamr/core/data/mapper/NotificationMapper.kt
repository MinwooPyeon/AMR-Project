package com.android.ssamr.core.data.mapper

import com.android.ssamr.core.data.local.entity.NotificationEntity
import com.android.ssamr.core.data.model.amr.response.NotificationDto
import com.android.ssamr.core.domain.model.Notification
import com.android.ssamr.core.domain.model.NotificationAction

object NotificationMapper {

    // Domain -> Entity
    fun toEntity(d: Notification): NotificationEntity =
        NotificationEntity(
            id = d.id,
            title = d.title,
            content = d.content,
            riskLevel = d.riskLevel.name,
            date = d.date,
            image = d.image,
            location = d.location,
            isRead = d.isRead,
            readAt = null
        )

    // Entity -> Domain
    fun fromEntity(e: NotificationEntity): Notification =
        Notification(
            id = e.id,
            title = e.title,
            content = e.content,
            riskLevel = NotificationAction.valueOf(e.riskLevel),
            date = e.date,
            image = e.image,
            location = e.location,
            isRead = e.isRead,
            )

    // Dto -> Domain
    fun dtoToDomain(dto: NotificationDto): Notification =
        Notification(
            id = dto.id,
            title = dto.title,
            content = dto.content,
            riskLevel = NotificationAction.valueOf(dto.riskLevel),
            date = dto.date,
            image = dto.image,
            location = dto.location,
            isRead = dto.isRead ?: false
        )

    // Domain -> Entity 리스트 업서트용
    fun domainListToEntities(list: List<Notification>): List<NotificationEntity> =
        list.map { toEntity(it) }

}