package com.example.amr_backend.v1.repository

import AmrStatus
import org.springframework.data.jpa.repository.JpaRepository

interface AmrStatusRepository : JpaRepository<AmrStatus, Long> {
}