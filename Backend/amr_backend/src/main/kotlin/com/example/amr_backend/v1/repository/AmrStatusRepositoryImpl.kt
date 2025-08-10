package com.example.amr_backend.v1.repository

import com.example.amr_backend.v1.entity.AmrStatus
import org.springframework.stereotype.Repository


@Repository
class AmrStatusRepositoryImpl(
    private val amrStatusJpaRepository: AmrStatusJpaRepository,
) : AmrStatusRepository {
    override fun findAllLatestStatuses(): List<AmrStatus> {
        return amrStatusJpaRepository.findAllLatestStatuses()
    }

    override fun findAmrStatusById(id: Long): AmrStatus {
        return amrStatusJpaRepository.findAmrStatusById(id)
    }

    override fun save(amrStatus: AmrStatus): AmrStatus {
        return amrStatusJpaRepository.save(amrStatus)
    }
}