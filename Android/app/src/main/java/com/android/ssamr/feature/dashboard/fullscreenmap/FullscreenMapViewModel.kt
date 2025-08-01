package com.android.ssamr.feature.dashboard.fullscreenmap

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.android.ssamr.feature.dashboard.DashboardAmrStatus
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch

class FullscreenMapViewModel : ViewModel() {

    private val _state = MutableStateFlow(FullscreenMapState())
    val state: StateFlow<FullscreenMapState> = _state.asStateFlow()

    private val _effect = MutableSharedFlow<FullscreenMapEffect>()
    val effect: SharedFlow<FullscreenMapEffect> = _effect.asSharedFlow()

    fun onIntent(intent: FullscreenMapIntent) {
        when (intent) {
            is FullscreenMapIntent.LoadMap -> {
                loadDummyMap()
                // dto 관련 데이터 코드
            }

            is FullscreenMapIntent.ClickAmr -> {
                _state.value = _state.value.copy(selectedAmrId = intent.id)
                emitEffect(FullscreenMapEffect.ShowAmrDetail(intent.id))
            }

            is FullscreenMapIntent.Close -> {
                emitEffect(FullscreenMapEffect.NavigateBack)
            }
        }
    }

    private fun loadDummyMap() {
        viewModelScope.launch {
            _state.value = _state.value.copy(isLoading = true)

            // TODO: 실제 지도 이미지 및 SLAM 좌표 가져올 예정
            val dummyAmrs = listOf(
                AmrMapPositionModel(1L, "AMR-001", x = 100f, y = 200f, status = DashboardAmrStatus.RUNNING),
                AmrMapPositionModel(2L, "AMR-002", x = 300f, y = 400f, status = DashboardAmrStatus.CHARGING),
                AmrMapPositionModel(3L, "AMR-003", x = 500f, y = 150f, status = DashboardAmrStatus.CHECK)
            )

            // dummy map image 추가
            val dummyMap = generateDummyMapImage(width = 720, height = 1280)

            _state.value = _state.value.copy(
                isLoading = false,
                amrPositions = dummyAmrs, // 추후 백엔드에서 받아오는 AMR 정보로.
                mapImage = dummyMap // 추후 백엔드에서 받아오는 맵으로.
            )
        }
    }

    private fun emitEffect(effect: FullscreenMapEffect) {
        viewModelScope.launch {
            _effect.emit(effect)
        }
    }
}
