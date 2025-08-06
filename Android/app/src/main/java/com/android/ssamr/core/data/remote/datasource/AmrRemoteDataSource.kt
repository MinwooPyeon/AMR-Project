package com.android.ssamr.core.data.remote.datasource

<<<<<<< HEAD
import com.android.ssamr.core.data.model.amr.request.ManualControlRequest
=======
>>>>>>> origin/develop
import com.android.ssamr.core.data.model.amr.response.toDetailModel
import com.android.ssamr.core.data.model.amr.response.toUiModel
import com.android.ssamr.core.data.remote.service.AmrService
import com.android.ssamr.core.domain.model.AmrDetailAction
import com.android.ssamr.core.domain.model.AmrDetailStatus
import com.android.ssamr.core.domain.model.AmrStatus
<<<<<<< HEAD
import retrofit2.Response
=======
>>>>>>> origin/develop
import javax.inject.Inject

class AmrRemoteDataSource @Inject constructor(
    private val service: AmrService
) {
    suspend fun getAmrList(): List<AmrStatus> {
        val response = service.getAmrList()

        return response.map { it.toUiModel() }
    }

<<<<<<< HEAD
    suspend fun getAmrDetail(amrId: Long): AmrDetailStatus  {
=======
    suspend fun getAmrDetail(amrId: Long): AmrDetailStatus {
>>>>>>> origin/develop
        val response = service.getAmrDetail(amrId)

        return response.toDetailModel()
    }
<<<<<<< HEAD

    suspend fun manualControl(amrId: Long, destination: String): Response<Unit> {
        return service.requestControl(amrId, ManualControlRequest(destination))
    }
=======
>>>>>>> origin/develop
}