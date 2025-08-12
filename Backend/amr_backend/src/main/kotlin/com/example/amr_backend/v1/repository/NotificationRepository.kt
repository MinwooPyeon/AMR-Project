package com.example.amr_backend.v1.repository

import com.example.amr_backend.v1.entity.Notification
import org.springframework.data.jpa.repository.JpaRepository

interface NotificationRepository : JpaRepository<Notification, Long> {
}