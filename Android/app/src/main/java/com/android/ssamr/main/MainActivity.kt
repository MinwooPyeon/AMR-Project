package com.android.ssamr.main

import android.os.Bundle
import android.util.Log
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.ui.platform.LocalContext
import androidx.hilt.navigation.compose.hiltViewModel
import com.android.ssamr.core.permission.AskNotificationPermission
import com.android.ssamr.core.permission.openNotificationSettingsSmart
import com.android.ssamr.ui.theme.SSAMRTheme
import com.google.firebase.messaging.FirebaseMessaging
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

//        FirebaseMessaging.getInstance().token
//            .addOnCompleteListener { task ->
//                if (task.isSuccessful) {
//                    val token = task.result
//                    Log.d("FCM", "token = $token")
//                } else {
//                    Log.e("FCM", "token fail", task.exception)
//                }
//            }
        enableEdgeToEdge()
        setContent {
            SSAMRTheme {
                AppRoot()
            }
        }
    }
}

@Composable
fun AppRoot() {
    val context = LocalContext.current
    val viewModel: AppStartViewModel = hiltViewModel()

    LaunchedEffect(Unit) {
        viewModel.syncFcmTokenOnce()
    }

    AskNotificationPermission(
        onGranted = {
            // 권한 허용 시 토픽 구독 + (선택) 다시 한 번 동기화
            viewModel.subscribeDefaultTopics()
        },
        onDenied  = { context.openNotificationSettingsSmart("amr_alerts") } // 거부 시 설정으로
    )
    MainScreen()  // 네비/스크린들은 여기 아래로
}
