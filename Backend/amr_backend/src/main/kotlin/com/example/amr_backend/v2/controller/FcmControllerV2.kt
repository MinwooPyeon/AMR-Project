package com.example.amr_backend.v2.controller

import com.example.amr_backend.v1.dto.FcmTokenSaveRequest
import com.example.amr_backend.v1.service.FcmService
import org.springframework.web.bind.annotation.PostMapping
import org.springframework.web.bind.annotation.RequestBody
import org.springframework.web.bind.annotation.RequestMapping
import org.springframework.web.bind.annotation.RestController

@RestController
@RequestMapping("/api/v2/fcm")
class FcmControllerV2(
    private val fcmService: FcmService,
) {
    @PostMapping
    fun saveToken(@RequestBody request: FcmTokenSaveRequest) = fcmService.saveToken(request.token)
}