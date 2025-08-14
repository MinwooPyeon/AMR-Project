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
            case = d.case,
            createAt = d.createAt,
            image = d.image,
            area = d.area,
            isRead = d.isRead,
            readAt = null,
            serial = d.serial
        )

    // Entity -> Domain
    fun fromEntity(e: NotificationEntity): Notification =
        Notification(
            id = e.id,
            title = e.title,
            content = e.content,
            riskLevel = NotificationAction.valueOf(e.riskLevel),
            case = e.case,
            createAt = e.createAt,
            image = e.image,
            area = e.area,
            isRead = e.isRead,
            serial = e.serial
            )

    // Dto -> Domain
    fun dtoToDomain(dto: NotificationDto): Notification =
        Notification(
            id = dto.id,
            title = dto.title,
            content = dto.content,
            riskLevel = NotificationAction.DANGER,
            case = dto.case,
            createAt = dto.createAt,
            image = dto.image,
            area = dto.area,
            isRead = dto.isRead ?: false,
            serial = dto.serial
        )

    // Domain -> Entity 리스트 업서트용
    fun domainListToEntities(list: List<Notification>): List<NotificationEntity> =
        list.map { toEntity(it) }

}