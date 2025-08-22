package com.example.amr_backend.v1.mqtt

import com.example.amr_backend.v1.dto.AmrStatusMessage
import com.example.amr_backend.v1.entity.Amr
import com.example.amr_backend.v1.entity.AmrStatus
import com.example.amr_backend.v1.entity.State
import com.example.amr_backend.v1.repository.AmrRepository
import com.example.amr_backend.v1.repository.AmrStatusRepository
import com.fasterxml.jackson.module.kotlin.jacksonObjectMapper
import io.mockk.every
import io.mockk.impl.annotations.RelaxedMockK
import io.mockk.junit5.MockKExtension
import io.mockk.slot
import io.mockk.verify
import org.eclipse.paho.client.mqttv3.MqttClient
import org.eclipse.paho.client.mqttv3.MqttMessage
import org.hamcrest.MatcherAssert.assertThat
import org.hamcrest.Matchers.`is`
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.assertAll
import org.junit.jupiter.api.extension.ExtendWith
import org.springframework.context.ApplicationEventPublisher
import java.time.LocalDateTime
import java.util.Optional

@ExtendWith(MockKExtension::class)
class ArmStatusMessageHandlerTest {
    private lateinit var amrStatusMessageHandler: ArmStatusMessageHandler

    @RelaxedMockK
    private lateinit var amrStatusRepository: AmrStatusRepository

    @RelaxedMockK
    private lateinit var mqttClient: MqttClient

    @RelaxedMockK
    private lateinit var amrRepository: AmrRepository

    @RelaxedMockK
    private lateinit var eventPublisher: ApplicationEventPublisher

    private val objectMapper = jacksonObjectMapper()

    @BeforeEach
    fun setUp() {
        amrStatusMessageHandler = ArmStatusMessageHandler(
            amrStatusRepository = amrStatusRepository,
            mqttClient = mqttClient,
            amrRepository = amrRepository,
            objectMapper = objectMapper,
            eventPublisher = eventPublisher,
        )
    }

    @Test
    fun `messageArrived, status message from amr arrived, then saves it`() {
        // given
        val messageFromAmr = AmrStatusMessage(
            serial = "AMR001",
            state = State.CHARGING,
            x = 8.9,
            y = 10.11,
            speed = 12.13,
            angle = 14.15
        )
        val mqttMessage = MqttMessage(objectMapper.writeValueAsString(messageFromAmr).toByteArray())

        every { amrRepository.findBySerial("AMR001") } returns Optional.of(ValidAmrStatus.amr)

        val amrStatusSlot = slot<AmrStatus>()
        every { amrStatusRepository.save(capture(amrStatusSlot)) } answers { amrStatusSlot.captured }

        // when
        amrStatusMessageHandler.messageArrived("status", mqttMessage)

        // then
        val capturedAmrStatus = amrStatusSlot.captured
        assertAll(
            { assertThat("amr serial not matched", capturedAmrStatus.amr.serial, `is`("AMR001")) },
            { assertThat("x not matched", capturedAmrStatus.x, `is`(8.9)) },
            { assertThat("y not matched", capturedAmrStatus.y, `is`(10.11)) },
            { assertThat("speed not matched", capturedAmrStatus.speed, `is`(12.13)) },
            { assertThat("angle not matched", capturedAmrStatus.angle, `is`(14.15)) },
        )
    }

    @Test
    fun `messageArrived, status message from non existing amr arrived, then saves nothing`() {
        // given
        val messageFromAmr = AmrStatusMessage(
            serial = "AMR123",
            state = State.CHARGING,
            x = 8.9,
            y = 10.11,
            speed = 12.13,
            angle = 14.15
        )
        val mqttMessage = MqttMessage(objectMapper.writeValueAsString(messageFromAmr).toByteArray())

        every { amrRepository.findBySerial("AMR123") } throws Exception()

        // when
        amrStatusMessageHandler.messageArrived("status", mqttMessage)

        // then
        verify(exactly = 0) { amrRepository.save(any()) }
    }

    @Test
    fun `messageArrived, malformed message arrived, then saves nothing`() {
        // given
        val malformedMessage = """
            {
                "serial: "AMR001",
                "x": 1.2
            }
        """.trimIndent()
        val mqttMessage = MqttMessage(objectMapper.writeValueAsString(malformedMessage).toByteArray())

        every { amrRepository.findBySerial("AMR001") } returns Optional.of(ValidAmrStatus.amr)

        // when
        amrStatusMessageHandler.messageArrived("status", mqttMessage)

        // then
        verify(exactly = 0) { amrRepository.save(any()) }
    }

    companion object {
        private val ValidAmrStatus = AmrStatus(
            amr = Amr(
                id = 1878,
                name = "Irving Avila",
                ipAddress = "1.1.1.1",
                serial = "AMR001",
                model = "morbi",
                firmwareVersion = "arcu",
                lastUpdateDate = LocalDateTime.of(2025, 8, 7, 12, 30)
            ),
            state = State.CHARGING,
            x = 8.9,
            y = 10.11,
            speed = 12.13,
            angle = 14.15
        )
    }
}