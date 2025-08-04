package com.android.ssamr.feature.dashboard.fullscreenmap

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.android.ssamr.core.domain.repository.DashboardRepository
import com.android.ssamr.core.data.model.amr.response.toAmrMapPositionModel
import com.android.ssamr.core.ui.ImageDecoder
import com.android.ssamr.feature.dashboard.DashboardAmrStatus
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.*
import kotlinx.coroutines.launch
import javax.inject.Inject

@HiltViewModel
class FullscreenMapViewModel @Inject constructor(
    private val repository: DashboardRepository
) : ViewModel() {

    private val USE_DUMMY_DATA = false
    private var isMapImageLoaded = false

    private val _state = MutableStateFlow(FullscreenMapState())
    val state: StateFlow<FullscreenMapState> = _state.asStateFlow()

    private val _effect = MutableSharedFlow<FullscreenMapEffect>()
    val effect: SharedFlow<FullscreenMapEffect> = _effect.asSharedFlow()

    fun onIntent(intent: FullscreenMapIntent) {
        when (intent) {
            is FullscreenMapIntent.LoadMap -> {
                loadMapFromRepository()
            }

            is FullscreenMapIntent.ClickAmr -> {
                _state.update { it.copy(selectedAmrId = intent.id) }
                emitEffect(FullscreenMapEffect.ShowAmrDetail(intent.id))
            }

            is FullscreenMapIntent.Close -> {
                emitEffect(FullscreenMapEffect.NavigateBack)
            }
        }
    }

    private fun loadMapFromRepository() {
        viewModelScope.launch {
            _state.update { it.copy(isLoading = true, error = null) }

            if (USE_DUMMY_DATA) {
                val dummyAmrs = listOf(
                    AmrMapPositionModel(1L, "AMR-001", x = 100f, y = 200f, status = DashboardAmrStatus.RUNNING),
                    AmrMapPositionModel(2L, "AMR-002", x = 300f, y = 400f, status = DashboardAmrStatus.CHARGING),
                    AmrMapPositionModel(3L, "AMR-003", x = 500f, y = 150f, status = DashboardAmrStatus.CHECK)
                )
                val dummyMap = generateDummyMapImage(720, 1280)

                _state.update {
                    it.copy(
                        isLoading = false,
                        amrPositions = dummyAmrs,
                        mapImage = dummyMap
                    )
                }
                return@launch
            }

            runCatching {
                val amrDtos = repository.getMapAmrPositions()
                val amrPositions = amrDtos.map { it.toAmrMapPositionModel() }

                val mapImage = if (!isMapImageLoaded) {
                    isMapImageLoaded = true
                    val base64 = repository.getMapImage()
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
                _state.update { it.copy(isLoading = false, error = "지도 로딩 실패: ${e.message}") }
                emitEffect(FullscreenMapEffect.ShowError("지도 로딩 실패"))
            }
        }
    }

    private fun emitEffect(effect: FullscreenMapEffect) {
        viewModelScope.launch {
            _effect.emit(effect)
        }
    }
}
