package com.android.ssamr.feature.more

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import kotlinx.coroutines.flow.*
import kotlinx.coroutines.launch

class MorescreenViewModel : ViewModel() {

    private val _state = MutableStateFlow(MorescreenState())
    val state: StateFlow<MorescreenState> = _state.asStateFlow()

    private val _effect = MutableSharedFlow<MorescreenEffect>()
    val effect: SharedFlow<MorescreenEffect> = _effect.asSharedFlow()

    fun onIntent(intent: MorescreenIntent) {
        when (intent) {
            is MorescreenIntent.OnSettingClick -> navigate(MorescreenDestination.SETTING)
            is MorescreenIntent.OnHelpClick -> navigate(MorescreenDestination.HELP)
            is MorescreenIntent.OnNoticeClick -> navigate(MorescreenDestination.NOTICE)
            is MorescreenIntent.OnVersionInfoClick -> navigate(MorescreenDestination.VERSION_INFO)
            is MorescreenIntent.OnProfileClick -> navigate(MorescreenDestination.PROFILE)

            is MorescreenIntent.OnProfileEdited -> updateUserProfile(intent.newProfile)
        }
    }

    private fun navigate(destination: MorescreenDestination) {
        viewModelScope.launch {
            _effect.emit(MorescreenEffect.NavigateTo(destination))
        }
    }

    private fun updateUserProfile(newProfile: UserProfileUiModel) {
        _state.update { current ->
            current.copy(userProfile = newProfile)
        }
    }

    // 초기 상태 설정용 - 예: 서버 연동 시 활용
    fun loadUserProfile() {
        _state.update {
            it.copy(
                status = MorescreenStatus.Success,
                userProfile = UserProfileUiModel(
                    name = "김철수",
                    role = "안전 관리 책임자",
                    department = "삼성전자 천안공장",
                    imageUrl = "",
                    canEdit = true
                )
            )
        }
    }
}
