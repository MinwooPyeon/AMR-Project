package com.example.amr_backend.v1.controller

import com.example.amr_backend.v1.entity.Notification
import com.example.amr_backend.v1.service.NotificationService
import org.springframework.web.bind.annotation.GetMapping
import org.springframework.web.bind.annotation.RequestMapping
import org.springframework.web.bind.annotation.RestController

@RestController
@RequestMapping("/api/v1/notifications")
class NotificationController(
    private val notificationService: NotificationService,
) {

    @GetMapping
    fun findAll(): List<Notification> = notificationService.findAll()
}