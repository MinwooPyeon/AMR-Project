package com.android.ssamr.feature.notificationDetail

import androidx.lifecycle.SavedStateHandle
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.android.ssamr.core.domain.usecase.notification.FetchNotificationDetailUseCase
import com.android.ssamr.core.domain.usecase.notification.MarkNotificationReadLocalUseCase
import com.android.ssamr.core.domain.usecase.notification.MarkNotificationReadRemoteUseCase
import com.android.ssamr.core.domain.usecase.notification.ObserveNotificationUseCase
import com.android.ssamr.core.domain.usecase.notification.UpsertNotificationUseCase
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch
import javax.inject.Inject

@HiltViewModel
class NotificationDetailViewModel @Inject constructor(
    savedStateHandle: SavedStateHandle,
    private val observeNotification: ObserveNotificationUseCase,
    private val fetchDetail: FetchNotificationDetailUseCase,
    private val upsertNotification: UpsertNotificationUseCase,
    private val markAsReadLocal: MarkNotificationReadLocalUseCase,
    private val markAsReadRemote: MarkNotificationReadRemoteUseCase
) : ViewModel() {

    private val notifId: Long = checkNotNull(savedStateHandle["notificationId"])

    private val _state = MutableStateFlow(
        NotificationDetailState(id = notifId, isLoading = true)
    )
    val state: StateFlow<NotificationDetailState> = _state.asStateFlow()

    private val _effect = MutableSharedFlow<NotificationDetailEffect>()
    val effect: SharedFlow<NotificationDetailEffect> = _effect

    init {
        // Room 즉시 표시
        viewModelScope.launch {
            observeNotification(notifId).collect { local ->
                _state.update {
                    it.copy(data = local ?: it.data, isLoading = false, error = null)
                }
            }
        }
        // 진입 시 1회 서버 동기화 → Room 업데이트
        viewModelScope.launch {
            try {
                val fresh = fetchDetail(notifId)
                upsertNotification(fresh)
            } catch (e: Exception) {
                _effect.emit(NotificationDetailEffect.ShowError(e.message ?: "로드 실패"))
                _state.update { it.copy(error = e.message) }
            }
        }
    }

    fun sendIntent(intent: NotificationDetailIntent) {
        when (intent) {
            NotificationDetailIntent.ClickMarkRead -> markAsRead()
            is NotificationDetailIntent.ClickPhotoView -> {
                viewModelScope.launch { _effect.emit(NotificationDetailEffect.NavigateToPhotoView(intent.url)) }
            }
        }
    }

    private fun markAsRead() {
        viewModelScope.launch {
            try {
                // 로컬 즉시 반영
                markAsReadLocal(notifId)
                // 서버 동기화 (베스트 에포트)
                runCatching { markAsReadRemote(notifId) }
            } catch (e: Exception) {
                _effect.emit(NotificationDetailEffect.ShowError("읽음 처리 실패: ${e.message}"))
            }
        }
    }
}

