package com.example.amr_backend.v2.service

import com.example.amr_backend.v2.entity.RefreshToken
import com.example.amr_backend.v2.model.Token
import com.example.amr_backend.v2.repository.RefreshTokenRepository
import com.example.amr_backend.v2.repository.UserRepository
import org.springframework.beans.factory.annotation.Value
import org.springframework.http.HttpStatus
import org.springframework.security.authentication.BadCredentialsException
import org.springframework.security.crypto.password.PasswordEncoder
import org.springframework.stereotype.Service
import org.springframework.transaction.annotation.Transactional
import org.springframework.web.server.ResponseStatusException
import java.time.LocalDateTime

private const val CHECK_ID_AND_PASSWORD_AGAIN = "아이디와 비밀번호를 다시 체크해주세요."
private const val INVALID_REFRESH_TOKEN = "유효하지 않은 refresh 토큰입니다."

@Service
class AuthService(
    private val userRepository: UserRepository,
    private val jwtService: JwtService,
    private val passwordEncoder: PasswordEncoder,
    private val refreshTokenRepository: RefreshTokenRepository,
    @Value("\${jwt.refresh-token.expiry}") private val refreshTokenExpiryInMins: Long,
) {
    @Transactional
    fun login(username: String, password: String): Token {
        val user = userRepository.findByUsername(username) ?: throw BadCredentialsException(CHECK_ID_AND_PASSWORD_AGAIN)

        if (!passwordEncoder.matches(password, user.password)) {
            throw BadCredentialsException(CHECK_ID_AND_PASSWORD_AGAIN)
        }

        val accessToken = jwtService.buildJwtToken(user.id.toString(), TokenType.ACCESS)
        val refreshToken = jwtService.buildJwtToken(user.id.toString(), TokenType.REFRESH)

        val refreshTokenEntity = RefreshToken(
            user = user,
            token = refreshToken,
            expiresAt = LocalDateTime.now().plusMinutes(refreshTokenExpiryInMins)
        )
        refreshTokenRepository.save(refreshTokenEntity)

        return Token(
            accessToken = accessToken,
            refreshToken = refreshToken,
        )
    }

    @Transactional
    fun refresh(refreshToken: String): Token {
        if (!jwtService.isValidToken(refreshToken, TokenType.REFRESH)) {
            throw ResponseStatusException(HttpStatus.UNAUTHORIZED, INVALID_REFRESH_TOKEN)
        }

        val userId = jwtService.getUserIdFromToken(refreshToken)
        if (!userRepository.existsById(userId)) {
            throw ResponseStatusException(HttpStatus.UNAUTHORIZED, INVALID_REFRESH_TOKEN)
        }

        val refreshTokenFromDb =
            refreshTokenRepository.findByUserIdAndToken(userId, refreshToken) ?: throw ResponseStatusException(
                HttpStatus.UNAUTHORIZED, INVALID_REFRESH_TOKEN
            )

        val newAccessToken = jwtService.buildJwtToken(userId.toString(), TokenType.ACCESS)
        val newRefreshToken = jwtService.buildJwtToken(userId.toString(), TokenType.REFRESH)

        refreshTokenFromDb.token = newRefreshToken
        refreshTokenFromDb.expiresAt = LocalDateTime.now().plusMinutes(refreshTokenExpiryInMins)
        refreshTokenRepository.save(refreshTokenFromDb)

        return Token(
            accessToken = newAccessToken,
            refreshToken = newRefreshToken,
        )
    }
}