package com.example.amr_backend.v2.service

import com.example.amr_backend.v2.model.Token
import com.example.amr_backend.v2.repository.UserRepository
import org.springframework.security.authentication.BadCredentialsException
import org.springframework.security.crypto.password.PasswordEncoder
import org.springframework.stereotype.Service

private const val CHECK_ID_AND_PASSWORD_AGAIN = "아이디와 비밀번호를 다시 체크해주세요."

@Service
class AuthService(
    private val userRepository: UserRepository,
    private val jwtService: JwtService,
    private val passwordEncoder: PasswordEncoder,
) {
    fun login(username: String, password: String): Token {
        val user = userRepository.findByUsername(username) ?: throw BadCredentialsException(CHECK_ID_AND_PASSWORD_AGAIN)

        if (!passwordEncoder.matches(password, user.password)) {
            throw BadCredentialsException(CHECK_ID_AND_PASSWORD_AGAIN)
        }

        val accessToken = jwtService.buildJwtToken(user.id.toString(), TokenType.ACCESS)
        val refreshToken = jwtService.buildJwtToken(user.id.toString(), TokenType.REFRESH)
        return Token(
            accessToken = accessToken,
            refreshToken = refreshToken,
        )
    }
}