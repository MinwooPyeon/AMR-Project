package com.android.ssamr.feature.dashboard.fullscreenmap

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.android.ssamr.core.domain.model.AmrMapPosition
import com.android.ssamr.core.domain.model.AmrStatus
import com.android.ssamr.core.domain.model.DashboardAmrStatus
import com.android.ssamr.core.domain.repository.AmrRepository
import com.android.ssamr.core.domain.repository.DashboardRepository
import com.android.ssamr.core.ui.ImageDecoder
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.*
import kotlinx.coroutines.launch
import javax.inject.Inject

@HiltViewModel
class FullscreenMapViewModel @Inject constructor(
    private val dashboardRepository: DashboardRepository,
    private val amrRepository: AmrRepository
) : ViewModel() {

    private var isMapImageLoaded = false

    private val _state = MutableStateFlow(FullscreenMapState())
    val state: StateFlow<FullscreenMapState> = _state.asStateFlow()

    fun onIntent(intent: FullscreenMapIntent) {
        when (intent) {
            is FullscreenMapIntent.LoadMap -> loadMapFromRepository()
            is FullscreenMapIntent.ClickAmr -> {
                _state.update { it.copy(selectedAmrId = intent.id) }
            }
            is FullscreenMapIntent.ClearSelectedAmr -> {
                _state.update { it.copy(selectedAmrId = null) }
            }
            is FullscreenMapIntent.Close -> {
                // 필요 시 처리
            }
        }
    }

    private fun loadMapFromRepository() {
        viewModelScope.launch {
            _state.update { it.copy(isLoading = true, error = null) }

            runCatching {
                val amrStatuses = amrRepository.getAmrList()
                val amrPositions = amrStatuses.map { it.toMapPosition() }

                val mapImage = if (!isMapImageLoaded) {
                    isMapImageLoaded = true
                    val base64 = dashboardRepository.getMapImage()
                    ImageDecoder.decodeBase64ToImageBitmap(base64)
                } else {
                    _state.value.mapImage
                }

                _state.update {
                    it.copy(
                        isLoading = false,
                        amrPositions = amrPositions,
                        mapImage = mapImage
                    )
                }
            }.onFailure { e ->
                _state.update {
                    it.copy(
                        isLoading = false,
                        error = "지도 로딩 실패: ${e.message}"
                    )
                }
            }
        }
    }

    private fun AmrStatus.toMapPosition(): AmrMapPosition {
        return AmrMapPosition(
            id = id,
            name = name,
            x = locationX.toFloat(),
            y = locationY.toFloat(),
            status = when (status) {
                com.android.ssamr.core.domain.model.AmrAction.RUNNING -> DashboardAmrStatus.RUNNING
                com.android.ssamr.core.domain.model.AmrAction.CHARGING -> DashboardAmrStatus.CHARGING
                com.android.ssamr.core.domain.model.AmrAction.CHECKING -> DashboardAmrStatus.CHECKING
            }
        )
    }
}
