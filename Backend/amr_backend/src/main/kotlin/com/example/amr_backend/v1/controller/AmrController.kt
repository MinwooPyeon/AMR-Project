package com.example.amr_backend.v1.controller

import com.example.amr_backend.v1.dto.AmrManualControlRequest
import com.example.amr_backend.v1.dto.AmrStatusResponse
import com.example.amr_backend.v1.dto.toAmrDetailResponse
import com.example.amr_backend.v1.dto.toAmrManualControlMessage
import com.example.amr_backend.v1.dto.toAmrStatusResponse
import com.example.amr_backend.v1.service.AmrService
import io.swagger.v3.oas.annotations.Operation
import io.swagger.v3.oas.annotations.Parameter
import io.swagger.v3.oas.annotations.media.Content
import io.swagger.v3.oas.annotations.media.Schema
import io.swagger.v3.oas.annotations.responses.ApiResponse
import io.swagger.v3.oas.annotations.tags.Tag
import org.springframework.web.bind.annotation.GetMapping
import org.springframework.web.bind.annotation.PathVariable
import org.springframework.web.bind.annotation.PostMapping
import org.springframework.web.bind.annotation.RequestBody
import org.springframework.web.bind.annotation.RequestMapping
import org.springframework.web.bind.annotation.RestController

@Tag(name = "AMR", description = "AMR 관련 API")
@RestController
@RequestMapping("/api/v1/amrs")
class AmrController(
    private val amrService: AmrService
) {
    @Operation(
        summary = "모든 AMR의 최신 상태 조회",
        responses = [
            ApiResponse(
                responseCode = "200",
                description = "AMR 최신 상태 목록",
                content = [Content(
                    mediaType = "application/json",
                    schema = Schema(implementation = AmrStatusResponse::class)
                )]
            )
        ]
    )
    @GetMapping("/latest-statuses")
    fun findAllLatestStatuses(): List<AmrStatusResponse> =
        amrService.findAllLatestStatuses().map { it.toAmrStatusResponse() }

    @Operation(
        summary = "특정 AMR의 상세 정보 조회",
        parameters = [
            Parameter(name = "serial", description = "AMR 시리얼 번호", required = true)
        ],
        responses = [
            ApiResponse(
                responseCode = "200",
                description = "AMR 상세 정보",
                content = [Content(
                    mediaType = "application/json",
                    schema = Schema(implementation = AmrStatusResponse::class)
                )]
            )
        ]
    )
    @GetMapping("/{serial}/detail")
    fun findLatestStatusDetailBySerial(@PathVariable serial: String) =
        amrService.findLatestStatusBySerial(serial).toAmrDetailResponse()

    @Operation(
        summary = "AMR 수동 제어",
        requestBody = io.swagger.v3.oas.annotations.parameters.RequestBody(
            description = "AMR 수동 제어 요청",
            required = true,
            content = [Content(
                mediaType = "application/json",
                schema = Schema(implementation = AmrManualControlRequest::class)
            )]
        )
    )
    @PostMapping("/control")
    fun sendManualControlMessage(@RequestBody request: AmrManualControlRequest) =
        amrService.sendManualControlMessage(request.serial, request.toAmrManualControlMessage())
}