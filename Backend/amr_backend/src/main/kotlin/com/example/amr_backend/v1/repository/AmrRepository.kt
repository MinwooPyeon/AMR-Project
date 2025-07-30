package com.example.amr_backend.v1.repository

import com.example.amr_backend.v1.entity.Amr
import org.springframework.data.jpa.repository.JpaRepository

interface AmrRepository : JpaRepository<Amr, Long> {
}