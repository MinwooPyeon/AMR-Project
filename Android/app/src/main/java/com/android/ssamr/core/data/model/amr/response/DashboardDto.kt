package com.android.ssamr.core.data.model.amr.response

import com.android.ssamr.core.domain.model.DashboardAmr
import com.android.ssamr.core.domain.model.DashboardAmrStatus


data class DashboardDto(
    val id: Long,
    val name: String,
    val state: String,
    val location: String
//    val job: String
)

fun DashboardDto.toDashboardModel() : DashboardAmr = DashboardAmr(
    id = this.id,
    name = this.name,
    status = DashboardAmrStatus.from(state),
    location = this.location
//    job = this.job
)