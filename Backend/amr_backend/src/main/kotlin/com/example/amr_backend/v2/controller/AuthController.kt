package com.example.amr_backend.v2.controller

import com.example.amr_backend.v2.dto.LoginRequest
import com.example.amr_backend.v2.model.Token
import com.example.amr_backend.v2.service.AuthService
import org.springframework.web.bind.annotation.PostMapping
import org.springframework.web.bind.annotation.RequestBody
import org.springframework.web.bind.annotation.RequestMapping
import org.springframework.web.bind.annotation.RestController

@RestController
@RequestMapping("/api/v2/auth")
class AuthController(
    private val authService: AuthService,
) {
    @PostMapping("/login")
    fun login(
        @RequestBody loginRequest: LoginRequest
    ): Token = authService.login(loginRequest.username, loginRequest.password)
}