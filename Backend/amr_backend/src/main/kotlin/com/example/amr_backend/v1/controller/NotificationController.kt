package com.example.amr_backend.v1.controller

import com.example.amr_backend.v1.dto.NotificationReadUpdateRequest
import com.example.amr_backend.v1.dto.NotificationResponse
import com.example.amr_backend.v1.dto.toNotificationResponse
import com.example.amr_backend.v1.service.NotificationService
import io.swagger.v3.oas.annotations.Operation
import io.swagger.v3.oas.annotations.Parameter
import io.swagger.v3.oas.annotations.media.Content
import io.swagger.v3.oas.annotations.media.Schema
import io.swagger.v3.oas.annotations.responses.ApiResponse
import io.swagger.v3.oas.annotations.responses.ApiResponses
import io.swagger.v3.oas.annotations.security.SecurityRequirement
import io.swagger.v3.oas.annotations.tags.Tag
import org.springframework.web.bind.annotation.GetMapping
import org.springframework.web.bind.annotation.PatchMapping
import org.springframework.web.bind.annotation.PathVariable
import org.springframework.web.bind.annotation.RequestBody
import org.springframework.web.bind.annotation.RequestMapping
import org.springframework.web.bind.annotation.RestController

@Tag(name = "Notification", description = "알림 관련 API")
@RestController
@RequestMapping("/api/v1/notifications")
@SecurityRequirement(name = "jwtAuth")
class NotificationController(
    private val notificationService: NotificationService,
) {

    @Operation(
        summary = "모든 알림 조회",
    )
    @ApiResponses(
        ApiResponse(
            responseCode = "200",
            description = "알림 목록",
            content = [Content(
                mediaType = "application/json",
                schema = Schema(implementation = NotificationResponse::class)
            )]
        ),
        ApiResponse(responseCode = "500", description = "서버 오류")
    )
    @GetMapping
    fun findAll(): List<NotificationResponse> = notificationService.findAll().map { it.toNotificationResponse() }

    @Operation(
        summary = "특정 알림 조회",
        parameters = [
            Parameter(name = "id", description = "알림 ID", required = true)
        ],
    )
    @ApiResponses(
        ApiResponse(
            responseCode = "200",
            description = "알림 상세 정보",
            content = [Content(
                mediaType = "application/json",
                schema = Schema(implementation = NotificationResponse::class)
            )]
        ),
        ApiResponse(responseCode = "404", description = "알림을 찾을 수 없음"),
        ApiResponse(responseCode = "500", description = "서버 오류")
    )
    @GetMapping("/{id}")
    fun getById(@PathVariable id: Long): NotificationResponse =
        notificationService.getById(id).toNotificationResponse()

    @Operation(
        summary = "알림 읽음 처리",
        parameters = [
            Parameter(name = "id", description = "알림 ID", required = true)
        ],
        requestBody = io.swagger.v3.oas.annotations.parameters.RequestBody(
            description = "알림 읽음 처리 요청",
            required = true,
            content = [Content(
                mediaType = "application/json",
                schema = Schema(implementation = NotificationReadUpdateRequest::class)
            )]
        ),
    )
    @ApiResponses(
        ApiResponse(
            responseCode = "200",
            description = "알림 읽음 처리 결과",
            content = [Content(
                mediaType = "application/json",
                schema = Schema(implementation = NotificationResponse::class)
            )]
        ),
        ApiResponse(responseCode = "400", description = "잘못된 요청"),
        ApiResponse(responseCode = "404", description = "알림을 찾을 수 없음"),
        ApiResponse(responseCode = "500", description = "서버 오류")
    )
    @PatchMapping("/{id}")
    fun updateRead(
        @PathVariable id: Long,
        @RequestBody readUpdateRequest: NotificationReadUpdateRequest,
    ): NotificationResponse =
        notificationService.updateRead(id, readUpdateRequest.isRead).toNotificationResponse()
}