package com.example.amr_backend.v1.repository

import com.example.amr_backend.v1.entity.AmrStatus
import org.springframework.data.jpa.repository.JpaRepository
import org.springframework.data.jpa.repository.Query

interface AmrStatusRepository : JpaRepository<AmrStatus, Long> {
    @Query("SELECT DISTINCT ON (amr_serial) * FROM amr_status ORDER BY amr_serial, created_at DESC", nativeQuery = true)
    fun findAllLatestStatuses(): List<AmrStatus>
}