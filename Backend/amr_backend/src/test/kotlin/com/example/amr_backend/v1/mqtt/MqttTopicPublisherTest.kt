package com.example.amr_backend.v1.mqtt

import com.example.amr_backend.v1.dto.TopicMessage
import com.example.amr_backend.v1.service.TopicPublisher
import com.fasterxml.jackson.module.kotlin.jacksonObjectMapper
import io.mockk.Runs
import io.mockk.every
import io.mockk.impl.annotations.RelaxedMockK
import io.mockk.junit5.MockKExtension
import io.mockk.just
import io.mockk.slot
import io.mockk.verify
import org.eclipse.paho.client.mqttv3.MqttClient
import org.eclipse.paho.client.mqttv3.MqttMessage
import org.hamcrest.MatcherAssert.assertThat
import org.hamcrest.Matchers.`is`
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.extension.ExtendWith

@ExtendWith(MockKExtension::class)
class MqttTopicPublisherTest {
    private lateinit var mqttTopicPublisher: TopicPublisher

    @RelaxedMockK
    private lateinit var mqttClient: MqttClient

    private val objectMapper = jacksonObjectMapper()

    @BeforeEach
    fun setUp() {
        mqttTopicPublisher = MqttTopicPublisher(mqttClient = mqttClient, objectMapper = objectMapper)
    }

    @Test
    fun `publish, given a manual control message, then publishes a message to a control topic`() {
        // given
        val topic = "control"
        val serial = "AMR001"
        val area = "Area A"
        val manualControlMessage = TopicMessage.AmrManualControlMessage(serial = serial, area = area)

        val mqttMessageSlot = slot<MqttMessage>()
        every { mqttClient.publish("control", capture(mqttMessageSlot)) } just Runs

        // when
        mqttTopicPublisher.publish(topic, manualControlMessage)

        // then
        val mqttMessageContents = objectMapper.writeValueAsString(manualControlMessage)
        val capturedMqttMessage = mqttMessageSlot.captured
        verify(exactly = 1) { mqttClient.publish("control", any()) }
        assertThat(capturedMqttMessage.toString(), `is`(mqttMessageContents))
    }
}