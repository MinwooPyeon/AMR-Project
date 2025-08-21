package com.example.amr_backend.v1.repository

import com.example.amr_backend.v1.entity.AmrStatus
import com.example.amr_backend.v1.exception.NoSuchAmr
import org.springframework.data.jpa.repository.JpaRepository
import org.springframework.data.jpa.repository.Query
import java.util.Optional

interface AmrStatusJpaRepository : JpaRepository<AmrStatus, Long> {
    @Query("SELECT DISTINCT ON (amr_serial) * FROM amr_status ORDER BY amr_serial, created_at DESC", nativeQuery = true)
    fun findAllLatestStatuses(): List<AmrStatus>

    fun findAllByAmrSerialIn(amrSerials: List<String>): List<AmrStatus>

    fun findTopByAmrSerialOrderByCreatedAtDesc(amrSerial: String): Optional<AmrStatus>
}

fun AmrStatusJpaRepository.findAmrStatusById(id: Long): AmrStatus = findById(id).orElseThrow {
    NoSuchAmr("ID가 ${id}인 AMR이 없습니다.")
}