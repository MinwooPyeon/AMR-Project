package com.android.ssamr.feature.notification

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.android.ssamr.core.domain.model.Notification
import com.android.ssamr.core.domain.model.NotificationAction
import com.android.ssamr.core.domain.model.NotificationCategory
import com.android.ssamr.core.domain.usecase.notification.GetNotificationListUseCase
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import javax.inject.Inject

@HiltViewModel
class NotificationViewModel @Inject constructor(
    private val getNotificationListUseCase: GetNotificationListUseCase
) : ViewModel() {

    private val _state = MutableStateFlow(NotificationState())
    val state: StateFlow<NotificationState> = _state.asStateFlow()

    private val _effect = MutableSharedFlow<NotificationEffect>()
    val effect: SharedFlow<NotificationEffect> = _effect

    init {
        fetchNotificationList()
    }

    fun sendIntent(intent: NotificationIntent) {
        when (intent) {
            is NotificationIntent.ClickNotificationCard -> {
                viewModelScope.launch { _effect.emit(NotificationEffect.NavigateToNotificationDetail(intent.notificationId)) }
            }
            is NotificationIntent.ClickNotificationCategory -> {
                val filtered = filterNotificationList(_state.value.fullNotificationList, intent.category)
                _state.value = _state.value.copy(
                    selectedCategory = intent.category,
                    notificationList = filtered,
                )
            }
        }
    }

    private fun filterNotificationList(list: List<Notification>, category: NotificationCategory): List<Notification> {
        return when (category) {
            NotificationCategory.ALL -> list
            NotificationCategory.DANGER -> list.filter { it.riskLevel == NotificationAction.DANGER }
            NotificationCategory.WARNING -> list.filter { it.riskLevel == NotificationAction.WARNING }
            NotificationCategory.INFORMATION -> list.filter { it.riskLevel == NotificationAction.INFORMATION }
        }
    }

    private fun fetchNotificationList() {

    }
}