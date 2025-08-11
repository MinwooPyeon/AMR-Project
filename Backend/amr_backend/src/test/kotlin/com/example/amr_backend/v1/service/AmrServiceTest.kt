package com.example.amr_backend.v1.service

import com.example.amr_backend.v1.dto.TopicMessage
import com.example.amr_backend.v1.entity.Amr
import com.example.amr_backend.v1.entity.AmrStatus
import com.example.amr_backend.v1.entity.State
import com.example.amr_backend.v1.exception.NoSuchAmr
import com.example.amr_backend.v1.repository.AmrStatusRepository
import io.mockk.every
import io.mockk.impl.annotations.InjectMockKs
import io.mockk.impl.annotations.RelaxedMockK
import io.mockk.junit5.MockKExtension
import io.mockk.verify
import org.hamcrest.MatcherAssert.assertThat
import org.hamcrest.Matchers.`is`
import org.junit.jupiter.api.assertThrows
import org.junit.jupiter.api.extension.ExtendWith
import java.time.LocalDateTime
import java.util.Optional
import kotlin.test.Test
import kotlin.test.assertTrue

@ExtendWith(MockKExtension::class)
class AmrServiceTest {
    @InjectMockKs
    private lateinit var amrService: AmrService

    @RelaxedMockK
    private lateinit var amrStatusRepository: AmrStatusRepository

    @RelaxedMockK
    private lateinit var topicPublisher: TopicPublisher

    @Test
    fun `findAllLatestStatuses, no amr status exists, then returns empty list`() {
        // given
        every { amrStatusRepository.findAllLatestStatuses() } returns emptyList()

        // when
        val result = amrService.findAllLatestStatuses()

        // then
        assertTrue(result.isEmpty(), "amr statuses are not empty")
    }

    @Test
    fun `findAllLatestStatuses, amr status repository returns amr statuses, then returns correctly`() {
        // given
        val expected = (1..5).map { ValidAmrStatus.copy(id = it.toLong()) }
        every { amrStatusRepository.findAllLatestStatuses() } returns expected

        // when
        val result = amrService.findAllLatestStatuses()

        // then
        assertThat(result, `is`(expected))
    }

    @Test
    fun `findAmrDetail, amr status repository returns status, then returns correctly`() {
        // given
        val expected = ValidAmrStatus.copy(id = 123)
        every { amrStatusRepository.findById(123) } returns Optional.of(expected)

        // when
        val result = amrService.findAmrDetail(123)

        // then
        assertThat(result, `is`(expected))
    }

    @Test
    fun `findAmrDetail, no amr of given id exists, then throws NoSuchAmr`() {
        // given
        every { amrStatusRepository.findById(123) } returns Optional.empty()

        // when
        // then
        val exception = assertThrows<NoSuchAmr> {
            amrService.findAmrDetail(123)
        }
        assertThat(exception.message, `is`("ID가 123인 AMR이 없습니다."))
    }

    @Test
    fun `sendManualControlMessage, topic publisher publish topic message`() {
        // given
        val serial = "AMR001"
        val area = "Area A"
        val message = TopicMessage.AmrManualControlMessage(
            serial = serial,
            area = area
        )

        // when
        amrService.sendManualControlMessage(serial, message)

        // then
        val expected = TopicMessage.AmrManualControlMessage(
            serial = "AMR001",
            area = "Area A"
        )
        verify(exactly = 1) {
            topicPublisher.publish("control/AMR001", expected)
        }
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