package com.example.amr_backend.v1.controller

import com.example.amr_backend.v1.dto.NotificationReadUpdateRequest
import com.example.amr_backend.v1.dto.NotificationResponse
import com.example.amr_backend.v1.dto.toNotificationResponse
import com.example.amr_backend.v1.entity.Notification
import com.example.amr_backend.v1.service.NotificationService
import org.springframework.web.bind.annotation.GetMapping
import org.springframework.web.bind.annotation.PatchMapping
import org.springframework.web.bind.annotation.PathVariable
import org.springframework.web.bind.annotation.RequestBody
import org.springframework.web.bind.annotation.RequestMapping
import org.springframework.web.bind.annotation.RestController

@RestController
@RequestMapping("/api/v1/notifications")
class NotificationController(
    private val notificationService: NotificationService,
) {

    @GetMapping
    fun findAll(): List<Notification> = notificationService.findAll()

    @GetMapping("/{id}")
    fun getById(@PathVariable id: Long): NotificationResponse =
        notificationService.getById(id).toNotificationResponse()

    @PatchMapping("/{id}")
    fun updateRead(
        @PathVariable id: Long,
        @RequestBody readUpdateRequest: NotificationReadUpdateRequest,
    ): NotificationResponse =
        notificationService.updateRead(id, readUpdateRequest.isRead).toNotificationResponse()
}