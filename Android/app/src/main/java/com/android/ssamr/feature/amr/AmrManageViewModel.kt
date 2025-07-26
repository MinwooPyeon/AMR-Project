package com.android.ssamr.feature.amr

import androidx.lifecycle.ViewModel
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.asStateFlow
import javax.inject.Inject

@HiltViewModel
class AmrManageViewModel @Inject constructor(
//    private val getAmrListUseCase: GetAmrListUseCase
) : ViewModel() {

    private val _state = MutableStateFlow(AmrState())
    val state: StateFlow<AmrState> = _state.asStateFlow()

    private val _effect = MutableSharedFlow<AmrEffect>()
    val effect: SharedFlow<AmrEffect>
        get() = _effect.asSharedFlow()

    init {
        fetchAmrList()
    }

    fun sendIntent(intent: AmrIntent) {
        when (intent) {
            is AmrIntent.ClickAmrCategory -> {
                _state.value = _state.value.copy(selectedCategory = intent.category)
                filterAmrListByCategory(intent.category)
            }

            is AmrIntent.ClickAmrManageCard -> {
                // TODO: AMR카드 선택시 AMRDetail 화면으로 전환
                // viewModelScope.launch { _effect.emit(AmrEffect.NavigateToAmrDetail(intent.amrId)) }
            }
        }
    }

    private fun fetchAmrList() {
        // TODO: 서버와 연결해서 List 가져오기 -> BaseViewModel 작성 후
//        viewModelScope.launch {
//            _state.value = _state.value.copy(isLoading = true)
//            try {
//                val amrList = getAmrListUseCase // 서버에서 받아오기
//                // 카테고리별 숫자 집계
//                val counts = AmrCategory.values().associateWith { cat ->
//                    when (cat) {
//                        AmrCategory.ALL -> amrList.size
//                        AmrCategory.RUNNING -> amrList.count { it.status == AmrStatus.RUNNING }
//                        AmrCategory.CHARGING -> amrList.count { it.status == AmrStatus.CHARGING }
//                        AmrCategory.CHECK -> amrList.count { it.status == AmrStatus.CHECK }
//                    }
//                }
//                _state.value = _state.value.copy(
//                    amrList = amrList, // 기본 전체 리스트
//                    categoryCounts = counts,
//                    isLoading = false,
//                    error = null
//                )
//            } catch (e: Exception) {
//                _state.value = _state.value.copy(
//                    isLoading = false,
//                    error = "AMR 리스트 로드 실패: ${e.message}"
//                )
//            }
//        }
    }

    private fun filterAmrListByCategory(category: AmrCategory) {
        val fullList = _state.value.amrList
        val filteredList = when (category) {
            AmrCategory.ALL -> fullList
            AmrCategory.RUNNING -> fullList.filter { it.status == AmrStatus.RUNNING }
            AmrCategory.CHARGING -> fullList.filter { it.status == AmrStatus.CHARGING }
            AmrCategory.CHECK -> fullList.filter { it.status == AmrStatus.CHECK }
        }
        _state.value = _state.value.copy(amrList = filteredList)
    }
}
