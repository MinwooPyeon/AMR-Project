package com.example.amr_backend.v1.controller

import com.example.amr_backend.v1.dto.AmrManualControlRequest
import com.example.amr_backend.v1.dto.AmrStatusResponse
import com.example.amr_backend.v1.dto.toAmrDetailResponse
import com.example.amr_backend.v1.dto.toAmrManualControlMessage
import com.example.amr_backend.v1.dto.toAmrStatusResponse
import com.example.amr_backend.v1.service.AmrService
import org.springframework.web.bind.annotation.GetMapping
import org.springframework.web.bind.annotation.PathVariable
import org.springframework.web.bind.annotation.PostMapping
import org.springframework.web.bind.annotation.RequestBody
import org.springframework.web.bind.annotation.RequestMapping
import org.springframework.web.bind.annotation.RestController

@RestController
@RequestMapping("/api/v1/amrs")
class AmrController(
    private val amrService: AmrService
) {
    @GetMapping("/latest-statuses")
    fun findAllLatestStatuses(): List<AmrStatusResponse> =
        amrService.findAllLatestStatuses().map { it.toAmrStatusResponse() }

    @GetMapping("/{serial}/detail")
    fun findLatestStatusDetailBySerial(@PathVariable serial: String) =
        amrService.findLatestStatusBySerial(serial).toAmrDetailResponse()

    @PostMapping("/control")
    fun sendManualControlMessage(@RequestBody request: AmrManualControlRequest) =
        amrService.sendManualControlMessage(request.serial, request.toAmrManualControlMessage())
}