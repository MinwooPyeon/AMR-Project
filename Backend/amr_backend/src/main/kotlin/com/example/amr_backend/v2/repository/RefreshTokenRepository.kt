package com.example.amr_backend.v2.repository

import com.example.amr_backend.v2.entity.RefreshToken
import org.springframework.data.jpa.repository.JpaRepository

interface RefreshTokenRepository : JpaRepository<RefreshToken, Long> {
    fun findByUserIdAndToken(userId: Long, token: String): RefreshToken?
}