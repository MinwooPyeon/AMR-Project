package com.example.amr_backend.v1.fcm

import com.example.amr_backend.v1.dto.AlertOutboundMessage
import com.example.amr_backend.v1.repository.FcmTokenRepository
import com.fasterxml.jackson.databind.ObjectMapper
import com.fasterxml.jackson.module.kotlin.convertValue
import com.google.firebase.messaging.FirebaseMessaging
import com.google.firebase.messaging.Message
import com.google.firebase.messaging.Notification
import org.slf4j.LoggerFactory
import org.springframework.stereotype.Component

@Component
class FirebaseMessageHandler(
    private val firebaseMessaging: FirebaseMessaging,
    private val fcmTokenRepository: FcmTokenRepository,
    private val objectMapper: ObjectMapper,
) {
    private val logger = LoggerFactory.getLogger(this::class.java)

    fun sendAlertMessage(message: AlertOutboundMessage) {
        val title = message.title
        val body = message.content
        val payload = message.toPayload()
        send(title, body, payload)
    }

    private fun <T : Any> T.toPayload(): Map<String, String> = objectMapper.convertValue(this)

    private fun send(title: String, body: String, payload: Map<String, String>) {
        val tokens = fcmTokenRepository.findAll()
        val tokenSet = tokens.map { it.token }.toSet()
        logger.debug("tokens in set are {}", tokenSet)

        val messages = tokenSet.map {
            val notification = Notification.builder()
                .setTitle(title)
                .setBody(body)
                .build()

            Message.builder()
                .setNotification(notification)
                .putAllData(payload)
                .setToken(it)
                .build()
        }

        firebaseMessaging.sendEachAsync(messages)
    }
}