package com.android.ssamr.feature.login

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import androidx.compose.runtime.MutableState
import androidx.compose.runtime.State
import androidx.compose.runtime.mutableStateOf
import com.android.ssamr.core.usecase.amr.LoginUseCase
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.launch
import javax.inject.Inject

@HiltViewModel
class LoginViewModel @Inject constructor(
    private val loginUseCase: LoginUseCase
) : ViewModel() {

    private val _uiState: MutableState<LoginState> = mutableStateOf(LoginState())
    val uiState: State<LoginState> = _uiState

    fun onEvent(intent: LoginIntent) {
        when (intent) {
            is LoginIntent.EmailChanged -> {
                _uiState.value = _uiState.value.copy(email = intent.email)
            }
            is LoginIntent.PasswordChanged -> {
                _uiState.value = _uiState.value.copy(password = intent.password)
            }
            is LoginIntent.LoginClicked -> {
                login()
            }
        }
    }

    private fun login() {
        val email = _uiState.value.email
        val password = _uiState.value.password

        _uiState.value = _uiState.value.copy(isLoading = true, errorMessage = null)

        viewModelScope.launch {
            val result = loginUseCase(email, password)

            _uiState.value = if (result.isSuccess) {
                _uiState.value.copy(isLoading = false, isSuccess = true)
            } else {
                _uiState.value.copy(
                    isLoading = false,
                    errorMessage = result.exceptionOrNull()?.message
                )
            }
        }
    }
}
