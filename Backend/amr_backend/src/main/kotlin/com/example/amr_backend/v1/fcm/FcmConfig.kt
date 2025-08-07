package com.example.amr_backend.v1.fcm

import com.google.auth.oauth2.GoogleCredentials
import com.google.firebase.FirebaseApp
import com.google.firebase.FirebaseOptions
import com.google.firebase.messaging.FirebaseMessaging
import org.springframework.beans.factory.annotation.Value
import org.springframework.context.annotation.Bean
import org.springframework.context.annotation.Configuration
import java.io.FileInputStream


@Configuration
class FcmConfig(
    @Value("\${firebase.key.location}") private val keyLocation: String
) {
    @Bean
    fun firebaseApp(): FirebaseApp {
        val existingApps = FirebaseApp.getApps()
        return if (existingApps.isEmpty()) buildNewFirebaseApp() else FirebaseApp.getInstance()
    }

    private fun buildNewFirebaseApp(): FirebaseApp {
        val serviceAccount = FileInputStream(keyLocation)
        val options = FirebaseOptions.builder()
            .setCredentials(GoogleCredentials.fromStream(serviceAccount))
            .build()

        return FirebaseApp.initializeApp(options)
    }

    @Bean
    fun firebaseMessaging(firebaseApp: FirebaseApp): FirebaseMessaging = FirebaseMessaging.getInstance(firebaseApp)
}