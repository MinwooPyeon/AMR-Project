package com.example.amr_backend.v1.controller

import com.example.amr_backend.v1.dto.FcmTokenSaveRequest
import com.example.amr_backend.v1.service.FcmService
import io.swagger.v3.oas.annotations.Operation
import io.swagger.v3.oas.annotations.media.Content
import io.swagger.v3.oas.annotations.media.Schema
import io.swagger.v3.oas.annotations.parameters.RequestBody as SwaggerRequestBody
import io.swagger.v3.oas.annotations.responses.ApiResponse
import io.swagger.v3.oas.annotations.tags.Tag
import org.springframework.web.bind.annotation.PostMapping
import org.springframework.web.bind.annotation.RequestBody
import org.springframework.web.bind.annotation.RequestMapping
import org.springframework.web.bind.annotation.RestController

@Tag(name = "FCM", description = "FCM 관련 API")
@RestController
@RequestMapping("/api/v1/fcm")
class FcmController(
    private val fcmService: FcmService,
) {
    @Operation(
        summary = "FCM 토큰 저장",
        requestBody = SwaggerRequestBody(
            description = "FCM 토큰 정보",
            required = true,
            content = [Content(
                mediaType = "application/json",
                schema = Schema(implementation = FcmTokenSaveRequest::class)
            )]
        ),
        responses = [
            ApiResponse(
                responseCode = "200",
                description = "FCM 토큰 저장 성공"
            )
        ]
    )
    @PostMapping
    fun saveToken(@RequestBody request: FcmTokenSaveRequest) = fcmService.saveToken(request.token)
}