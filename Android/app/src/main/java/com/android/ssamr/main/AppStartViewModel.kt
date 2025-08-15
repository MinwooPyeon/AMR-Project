package com.android.ssamr.main

import android.util.Log
import androidx.lifecycle.ViewModel
import com.android.ssamr.core.domain.usecase.push.EnqueueRegisterFcmTokenUseCase
import com.google.firebase.Firebase
import com.google.firebase.messaging.FirebaseMessaging
import com.google.firebase.messaging.messaging
import dagger.hilt.android.lifecycle.HiltViewModel
import javax.inject.Inject

@HiltViewModel
class AppStartViewModel @Inject constructor(
    private val enqueueRegisterFcmToken: EnqueueRegisterFcmTokenUseCase
) : ViewModel() {

    fun syncFcmTokenOnce() {
        FirebaseMessaging.getInstance().token
            .addOnSuccessListener { token ->
                Log.d("FCM", "token (syncOnce) = $token")
                enqueueRegisterFcmToken(token)
            }
            .addOnFailureListener { e ->
                Log.e("FCM", "token fetch fail", e)
            }
    }

    fun subscribeDefaultTopics() {
        Firebase.messaging.subscribeToTopic("amr_alerts")
            .addOnCompleteListener { t ->
                if (t.isSuccessful) Log.d("FCM", "Subscribe success to arm_alerts")
                else Log.w("FCM", "Topic subscribe failed", t.exception)
            }
    }
}