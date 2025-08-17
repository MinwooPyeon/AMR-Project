package com.example.amr_backend.v1.repository

import com.example.amr_backend.v1.entity.Amr
import org.springframework.data.jpa.repository.JpaRepository
import java.util.Optional

interface AmrRepository : JpaRepository<Amr, Long> {
    fun findBySerial(serial: String): Optional<Amr>
}