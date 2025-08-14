package com.android.ssamr.feature.notification

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.android.ssamr.core.domain.model.Notification
import com.android.ssamr.core.domain.model.NotificationAction
import com.android.ssamr.core.domain.model.NotificationCategory
import com.android.ssamr.core.domain.usecase.notification.GetLocalNotificationFlowUseCase
import com.android.ssamr.core.domain.usecase.notification.GetNotificationListUseCase
import com.android.ssamr.core.domain.usecase.notification.MarkNotificationReadLocalUseCase
import com.android.ssamr.core.domain.usecase.notification.MarkNotificationReadRemoteUseCase
import com.android.ssamr.core.domain.usecase.notification.SaveNotificationListUseCase
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.catch
import kotlinx.coroutines.flow.onStart
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch
import javax.inject.Inject

@HiltViewModel
class NotificationViewModel @Inject constructor(
    // Room에서 Flow로 전체 알림 관찰
    private val getLocalNotificationFlow: GetLocalNotificationFlowUseCase, // () -> Flow<List<Notification>>
    // 서버에서 최신 목록 가져오기
    private val getNotificationListUseCase: GetNotificationListUseCase,    // suspend () -> List<Notification>
    // Room 저장/업서트
    private val saveNotificationListUseCase: SaveNotificationListUseCase,  // suspend (List<Notification>) -> Unit
    // 읽음 처리: 로컬 즉시
    private val markNotificationReadLocal: MarkNotificationReadLocalUseCase,   // suspend (id: Long) -> Unit
    // 읽음 처리: 서버 베스트에포트
    private val markNotificationReadRemote: MarkNotificationReadRemoteUseCase  // suspend (id: Long) -> Unit
) : ViewModel() {

    private val _state = MutableStateFlow(NotificationState())
    val state: StateFlow<NotificationState> = _state.asStateFlow()

    private val _effect = MutableSharedFlow<NotificationEffect>()
    val effect: SharedFlow<NotificationEffect> = _effect

    init {
        // 1) Room Flow 구독 → UI 자동 반영
        observeLocal()
        // 2) 앱 시작 시 서버 동기화
        syncFromServer()
    }

    fun sendIntent(intent: NotificationIntent) {
        when (intent) {
            is NotificationIntent.ClickNotificationCategory -> {
                val cur = _state.value
                val filtered = filter(cur.fullNotificationList, intent.category)
                _state.value = cur.copy(
                    selectedCategory = intent.category,
                    notificationList = filtered
                )
            }
            is NotificationIntent.ClickNotificationCard -> {
                // 하이브리드: 로컬 즉시 + 서버 비동기
                markReadHybrid(intent.notificationId)
                viewModelScope.launch {
                    _effect.emit(NotificationEffect.NavigateToNotificationDetail(intent.notificationId))
                }
            }
        }
    }

    fun refresh() = syncFromServer()

    /** Room Flow 구독 */
    private fun observeLocal() {
        viewModelScope.launch {
            getLocalNotificationFlow()
                .onStart { _state.update { it.copy(isLoading = true, error = null) } }
                .catch { e ->
                    _state.update { it.copy(isLoading = false, error = e.message) }
                    _effect.emit(NotificationEffect.ShowError(e.message ?: "알림 로드 실패"))
                }
                .collect { localList ->
                    val cur = _state.value
                    val filtered = filter(localList, cur.selectedCategory)
                    _state.value = cur.copy(
                        isLoading = false,
                        fullNotificationList = localList,
                        notificationList = filtered,
                        categoryCounts = buildCounts(localList),
                        error = null
                    )
                }
        }
    }

    /** 서버 동기화: 서버 → 로컬 저장 (로컬 isRead 보존) */
    private fun syncFromServer() {
        viewModelScope.launch {
            try {
                _state.update { it.copy(isLoading = true, error = null) }
                val server = getNotificationListUseCase() // 최신 서버 목록
                // 현재 로컬 스냅샷(Flow 수집 중이라면 state 사용해도 되지만, 안전하게 state 사용)
                val local = _state.value.fullNotificationList
                val localReadMap = local.associate { it.id to it.isRead }
                // 머지 규칙: "읽었다"는 정보는 잃지 않는다.
                val merged = server.map { s ->
                    if (localReadMap[s.id] == true) s.copy(isRead = true) else s
                }
                saveNotificationListUseCase(merged) // Room 업서트 → Flow로 UI 반영
            } catch (e: Exception) {
                _state.update { it.copy(isLoading = false, error = e.message) }
                _effect.emit(NotificationEffect.ShowError(e.message ?: "동기화 실패"))
            }
        }
    }

    /** 하이브리드 읽음 처리: 로컬 즉시 + 서버 베스트에포트 */
    private fun markReadHybrid(id: Long, isRead: Boolean = true) {
        viewModelScope.launch {
            // 1) 로컬 즉시 반영 → UI 바로 회색 처리
            runCatching { markNotificationReadLocal(id) }

            // 2) 서버 동기화(실패해도 UI는 유지; 재시도 큐는 추후 도입 가능)
            runCatching { markNotificationReadRemote(id, isRead) }
                .onFailure {
                    // 필요 시: 로그/스낵바/재시도 등록
                    // _effect.emit(NotificationEffect.ShowError("읽음 동기화 실패"))
                }
        }
    }

    private fun filter(list: List<Notification>, category: NotificationCategory): List<Notification> =
        when (category) {
            NotificationCategory.ALL -> list
            NotificationCategory.DANGER -> list.filter { it.riskLevel == NotificationAction.DANGER }
            NotificationCategory.WARNING -> list.filter { it.riskLevel == NotificationAction.WARNING }
            NotificationCategory.INFORMATION -> list.filter { it.riskLevel == NotificationAction.INFORMATION }
        }

    private fun buildCounts(list: List<Notification>) = mapOf(
        NotificationCategory.ALL to list.size,
        NotificationCategory.DANGER to list.count { it.riskLevel == NotificationAction.DANGER },
        NotificationCategory.WARNING to list.count { it.riskLevel == NotificationAction.WARNING },
        NotificationCategory.INFORMATION to list.count { it.riskLevel == NotificationAction.INFORMATION }
    )
}