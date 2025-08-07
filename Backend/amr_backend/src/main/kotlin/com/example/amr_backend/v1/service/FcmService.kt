package com.example.amr_backend.v1.service

import com.example.amr_backend.v1.entity.FcmToken
import com.example.amr_backend.v1.repository.FcmTokenRepository
import org.springframework.stereotype.Service

@Service
class FcmService(
    private val fcmTokenRepository: FcmTokenRepository,
) {
    fun saveToken(token: String) = fcmTokenRepository.save(FcmToken(token = token))
}