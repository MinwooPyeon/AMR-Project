package com.android.ssamr.core.fcm

import android.Manifest
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Build
import android.util.Log
import androidx.core.app.NotificationCompat
import androidx.core.app.NotificationManagerCompat
import androidx.core.content.ContextCompat
import com.android.ssamr.R
import com.android.ssamr.core.domain.model.Notification
import com.android.ssamr.core.domain.model.NotificationAction
import com.android.ssamr.core.domain.repository.NotificationRepository
import com.android.ssamr.main.MainActivity
import com.google.firebase.messaging.FirebaseMessagingService
import com.google.firebase.messaging.RemoteMessage
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.launch
import javax.inject.Inject

@AndroidEntryPoint
class AppFirebaseMessagingService : FirebaseMessagingService() {

    @Inject lateinit var notificationRepository: NotificationRepository
    @Inject lateinit var notificationMapper: NotificationRealtimeMapper

    private val serviceScope = CoroutineScope(SupervisorJob() + Dispatchers.IO)

    override fun onNewToken(token: String) {
        // TODO: 서버에 토큰 등록 (선택)
    }

    override fun onMessageReceived(message: RemoteMessage) {
        // 1) 데이터 메시지 처리 (우선)
        if (message.data.isNotEmpty()) {
            val notif = notificationMapper.fromFcmData(message.data)
            serviceScope.launch { notificationRepository.upsertNotifications(listOf(notif)) }
            showSystemNotification(notif)
            Log.d("FCM", "onMessageReceived data=${message.data} notif=${message.notification}")
            return
        }

        // 2) 콘솔 '알림 메시지' 처리 (title/body만 온 경우)
        message.notification?.let { n ->
            val notif = Notification(
                id = System.currentTimeMillis(),        // 임시 ID
                title = n.title ?: "알림",
                content = n.body ?: "",
                riskLevel = NotificationAction.INFORMATION,
                case = "",
                createAt = "",
                image = null,
                area = "",
                isRead = false
            )
            Log.d("FCM", "onMessageReceived data=${message.data} notif=${message.notification}")
            serviceScope.launch { notificationRepository.upsertNotifications(listOf(notif)) }
            showSystemNotification(notif)
        }
    }

    private fun showSystemNotification(n: Notification) {
        val context = this
        val channelId = "amr_alerts"
        ensureChannel(context, channelId)

        // Android 13+ 권한 체크
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            val granted = ContextCompat.checkSelfPermission(
                context,
                Manifest.permission.POST_NOTIFICATIONS
            ) == PackageManager.PERMISSION_GRANTED
            if (!granted) return // 권한 없으면 표시 스킵
        }

        // 앱이 채널/알림을 꺼둔 경우 스킵
        if (!NotificationManagerCompat.from(context).areNotificationsEnabled()) return

        val intent = Intent(context, MainActivity::class.java).apply {
            putExtra("open_notification_id", n.id)
            addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP)
        }
        val pi = PendingIntent.getActivity(
            context, n.id.toInt(), intent,
            PendingIntent.FLAG_UPDATE_CURRENT or PendingIntent.FLAG_IMMUTABLE
        )

        val builder = NotificationCompat.Builder(context, channelId)
            .setSmallIcon(R.drawable.ic_notification)
            .setContentTitle(n.title)
            .setContentText(n.content)
            .setPriority(NotificationCompat.PRIORITY_HIGH)
            .setAutoCancel(true)
            .setContentIntent(pi)

        NotificationManagerCompat.from(context).notify(n.id.toInt(), builder.build())
    }

    private fun ensureChannel(ctx: Context, id: String) {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            val mgr = ctx.getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager
            val ch = NotificationChannel(
                id,
                "AMR Alerts",
                NotificationManager.IMPORTANCE_HIGH
            )
            mgr.createNotificationChannel(ch)
        }
    }
}
