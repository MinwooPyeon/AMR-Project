package com.example.amr_backend.v1.repository

import com.example.amr_backend.v1.entity.FcmToken
import org.springframework.data.jpa.repository.JpaRepository

interface FcmTokenRepository : JpaRepository<FcmToken, Long> {
}