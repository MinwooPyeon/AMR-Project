package com.example.amr_backend.v2.filter

import com.example.amr_backend.v2.service.JwtService
import com.example.amr_backend.v2.service.TokenType
import jakarta.servlet.FilterChain
import jakarta.servlet.http.HttpServletRequest
import jakarta.servlet.http.HttpServletResponse
import org.springframework.security.authentication.UsernamePasswordAuthenticationToken
import org.springframework.security.core.context.SecurityContextHolder
import org.springframework.stereotype.Component
import org.springframework.web.filter.OncePerRequestFilter

private const val AUTHORIZATION_HEADER = "Authorization"
private const val BEARER_PREFIX = "Bearer "

@Component
class JwtFilter(
    private val jwtService: JwtService
) : OncePerRequestFilter() {
    override fun doFilterInternal(
        request: HttpServletRequest,
        response: HttpServletResponse,
        filterChain: FilterChain
    ) {
        val authHeader = request.getHeader(AUTHORIZATION_HEADER)
        if (authHeader?.hasValidAccessToken() == true) {
            val userId = jwtService.getUserIdFromToken(authHeader)
            val authToken = UsernamePasswordAuthenticationToken.authenticated(userId, null, emptyList())
            SecurityContextHolder.getContext().authentication = authToken
        }
        filterChain.doFilter(request, response)
    }

    private fun String.hasValidAccessToken(): Boolean =
        startsWith(BEARER_PREFIX) && jwtService.isValidToken(this, TokenType.ACCESS)
}