package com.android.ssamr.core.data.model.amr.response

import com.android.ssamr.feature.dashboard.DashboardAmrStatus
import com.android.ssamr.feature.dashboard.DashboardAmrUiModel

data class DashboardDto(
    val id: Long,
    val name: String,
    val battery: Int,
    val status: DashboardAmrStatus,
    val location: String,
    val job: String
)

fun DashboardDto.toDashboardModel() : DashboardAmrUiModel = DashboardAmrUiModel(
    id = this.id,
    name = this.name,
    battery = this.battery,
    status = this.status,
    location = this.location,
    job = this.job
)