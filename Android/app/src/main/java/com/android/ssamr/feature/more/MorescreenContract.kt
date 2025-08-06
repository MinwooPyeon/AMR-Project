package com.android.ssamr.feature.more

import androidx.compose.runtime.Immutable

// UI 상태 정의
sealed interface MorescreenIntent {
    data object OnSettingClick : MorescreenIntent
    data object OnHelpClick : MorescreenIntent
    data object OnNoticeClick : MorescreenIntent
    data object OnVersionInfoClick : MorescreenIntent
    data object OnProfileClick : MorescreenIntent

    data class OnProfileEdited(val newProfile: UserProfileUiModel) : MorescreenIntent
}

// 화면 상태 클래스
@Immutable
data class MorescreenState(
    val status: MorescreenStatus = MorescreenStatus.Init,
    val userProfile: UserProfileUiModel = UserProfileUiModel(),
    val appVersion: String = "v1.2.3"
)

// 단발성 효과 (예: Navigation)
sealed interface MorescreenEffect {
    data class NavigateTo(val destination: MorescreenDestination) : MorescreenEffect
}

// 상태 흐름 Enum
sealed interface MorescreenStatus {
    data object Init : MorescreenStatus
    data object Loading : MorescreenStatus
    data object Success : MorescreenStatus
    data class Failure(val message: String) : MorescreenStatus
}

// 사용자 정보
@Immutable
data class UserProfileUiModel(
    val name: String = "김철수",
    val role: String = "안전 관리 책임자",
    val department: String = "삼성전자 천안공장",
    val imageUrl: String = "",
    val canEdit: Boolean = true
)

// 화면 이동 목적지 Enum
enum class MorescreenDestination {
    SETTING,
    HELP,
    NOTICE,
    VERSION_INFO,
    PROFILE
}
