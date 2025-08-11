package com.example.amr_backend.v1.repository

import com.example.amr_backend.v1.entity.AmrStatus

interface AmrStatusRepository {
    fun findAllLatestStatuses(): List<AmrStatus>
    fun findAmrStatusById(id: Long): AmrStatus
    fun findLatestStatusBySerial(serial: String): AmrStatus
    fun save(amrStatus: AmrStatus): AmrStatus
}