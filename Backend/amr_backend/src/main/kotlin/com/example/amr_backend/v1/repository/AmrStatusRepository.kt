package com.example.amr_backend.v1.repository

import com.example.amr_backend.v1.entity.AmrStatus
import java.util.Optional

interface AmrStatusRepository {
    fun findAllLatestStatuses(): List<AmrStatus>
    fun findLatestStatusBySerial(serial: String): Optional<AmrStatus>
    fun save(amrStatus: AmrStatus): AmrStatus
}