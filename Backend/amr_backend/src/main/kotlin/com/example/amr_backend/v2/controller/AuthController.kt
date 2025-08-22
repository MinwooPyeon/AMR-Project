package com.example.amr_backend.v2.controller

import com.example.amr_backend.v2.dto.LoginRequest
import com.example.amr_backend.v2.dto.RefreshRequest
import com.example.amr_backend.v2.model.Token
import com.example.amr_backend.v2.service.AuthService
import io.swagger.v3.oas.annotations.Operation
import io.swagger.v3.oas.annotations.media.Content
import io.swagger.v3.oas.annotations.media.Schema
import io.swagger.v3.oas.annotations.parameters.RequestBody as SwaggerRequestBody
import io.swagger.v3.oas.annotations.responses.ApiResponse
import io.swagger.v3.oas.annotations.responses.ApiResponses
import io.swagger.v3.oas.annotations.tags.Tag
import org.springframework.web.bind.annotation.PostMapping
import org.springframework.web.bind.annotation.RequestBody
import org.springframework.web.bind.annotation.RequestMapping
import org.springframework.web.bind.annotation.RestController

@Tag(name = "Auth", description = "인증 관련 API")
@RestController
@RequestMapping("/api/v2/auth")
class AuthController(
    private val authService: AuthService,
) {
    @Operation(
        summary = "로그인",
        requestBody = SwaggerRequestBody(
            description = "로그인 요청",
            required = true,
            content = [Content(
                mediaType = "application/json",
                schema = Schema(implementation = LoginRequest::class)
            )]
        ),
    )
    @ApiResponses(
        ApiResponse(
            responseCode = "200",
            description = "로그인 성공",
            content = [Content(
                mediaType = "application/json",
                schema = Schema(implementation = Token::class)
            )]
        ),
        ApiResponse(responseCode = "401", description = "인증 실패"),
        ApiResponse(responseCode = "500", description = "서버 오류")
    )
    @PostMapping("/login")
    fun login(
        @RequestBody loginRequest: LoginRequest
    ): Token = authService.login(loginRequest.username, loginRequest.password)

    @Operation(
        summary = "토큰 재발급",
        requestBody = SwaggerRequestBody(
            description = "토큰 재발급 요청",
            required = true,
            content = [Content(
                mediaType = "application/json",
                schema = Schema(implementation = RefreshRequest::class)
            )]
        ),
    )
    @ApiResponses(
        ApiResponse(
            responseCode = "200",
            description = "토큰 재발급 성공",
            content = [Content(
                mediaType = "application/json",
                schema = Schema(implementation = Token::class)
            )]
        ),
        ApiResponse(responseCode = "401", description = "인증 실패"),
        ApiResponse(responseCode = "500", description = "서버 오류")
    )
    @PostMapping("/refresh")
    fun refresh(
        @RequestBody refreshRequest: RefreshRequest
    ): Token = authService.refresh(refreshRequest.refreshToken)
}