package com.example.amr_backend.v2.service

import io.jsonwebtoken.Claims
import io.jsonwebtoken.Jwts
import io.jsonwebtoken.security.Keys
import org.slf4j.LoggerFactory
import org.springframework.beans.factory.annotation.Value
import org.springframework.http.HttpStatus
import org.springframework.stereotype.Service
import org.springframework.web.server.ResponseStatusException
import java.util.Date

private const val SECOND = 1000L
private const val MINUTE = 60L * SECOND

@Service
class JwtService(
    @Value("\${jwt.secret-key}") private val secretKey: String,
    @Value("\${jwt.access-token.expiry}") private val accessTokenExpiryInMinutes: Int,
    @Value("\${jwt.refresh-token.expiry}") private val refreshTokenExpiryInMinutes: Int,
) {
    private val logger = LoggerFactory.getLogger(this::class.java)

    private val accessTokenExpiryInMillis = accessTokenExpiryInMinutes * MINUTE
    private val refreshTokenExpiryInMillis = refreshTokenExpiryInMinutes * MINUTE
    private val hmacShaKeyKey = Keys.hmacShaKeyFor(secretKey.toByteArray())

    fun buildJwtToken(
        userId: String,
        type: TokenType,
    ): String {
        val now = Date()
        val expiryDate = Date(now.time + type.expiryInMillis)

        return Jwts.builder()
            .subject(userId)
            .claim("type", type)
            .issuedAt(now)
            .expiration(expiryDate)
            .signWith(hmacShaKeyKey, Jwts.SIG.HS256)
            .compact()
    }

    fun isValidToken(token: String, type: TokenType): Boolean {
        val claims = parseAllClaims(token) ?: return false
        val tokenTypeString = claims["type"] as? String ?: return false
        val tokenType = TokenType.valueOf(tokenTypeString)
        return tokenType == type
    }

    fun getUserIdFromToken(token: String): Long {
        val claims = parseAllClaims(token) ?: throw ResponseStatusException(
            HttpStatus.UNAUTHORIZED,
            "유효하지 않은 토큰입니다."
        )
        return claims.subject.toLongOrNull() ?: throw ResponseStatusException(
            HttpStatus.BAD_REQUEST,
            "유효하지 않은 유저 id입니다."
        )
    }

    private fun parseAllClaims(authHeaderValue: String): Claims? {
        val token = authHeaderValue.removePrefix("Bearer ")
        val parser = Jwts.parser()
            .verifyWith(hmacShaKeyKey)
            .build()

        return try {
            parser
                .parseSignedClaims(token)
                .payload
        } catch (e: Exception) {
            logger.warn("exception while parsing JWT : {}", e.message)
            logger.warn(e.stackTraceToString())
            null
        }
    }

    private val TokenType.expiryInMillis: Long
        get() = when (this) {
            TokenType.ACCESS -> accessTokenExpiryInMillis
            TokenType.REFRESH -> refreshTokenExpiryInMillis
        }
}

enum class TokenType {
    ACCESS, REFRESH
}