package com.example.amr_backend.v1.controller

import com.example.amr_backend.v1.dto.AmrStatusResponse
import com.example.amr_backend.v1.dto.toResponse
import com.example.amr_backend.v1.repository.AmrStatusRepository
import org.springframework.web.bind.annotation.GetMapping
import org.springframework.web.bind.annotation.RequestMapping
import org.springframework.web.bind.annotation.RestController

@RestController
@RequestMapping("/api/v1/amrs")
class AmrController(
    private val amrStatusRepository: AmrStatusRepository
) {
    @GetMapping("/latest-statuses")
    fun findAllLatestStatuses(): List<AmrStatusResponse> =
        amrStatusRepository.findAllLatestStatuses().map { it.toResponse() }
}