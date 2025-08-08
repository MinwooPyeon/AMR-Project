package com.example.amr_backend.v1.controller

import com.example.amr_backend.v1.dto.AmrManualControlRequest
import com.example.amr_backend.v1.dto.TopicMessage
import com.example.amr_backend.v1.entity.Amr
import com.example.amr_backend.v1.entity.AmrStatus
import com.example.amr_backend.v1.entity.State
import com.example.amr_backend.v1.exception.NoSuchAmr
import com.example.amr_backend.v1.service.AmrService
import com.fasterxml.jackson.module.kotlin.jacksonObjectMapper
import com.ninjasquad.springmockk.MockkBean
import io.mockk.Runs
import io.mockk.every
import io.mockk.just
import org.junit.jupiter.api.BeforeEach
import org.springframework.boot.test.autoconfigure.web.servlet.AutoConfigureMockMvc
import org.springframework.boot.test.autoconfigure.web.servlet.WebMvcTest
import org.springframework.http.MediaType
import org.springframework.test.web.servlet.MockMvc
import org.springframework.test.web.servlet.request.MockMvcRequestBuilders.get
import org.springframework.test.web.servlet.request.MockMvcRequestBuilders.post
import org.springframework.test.web.servlet.result.MockMvcResultHandlers
import org.springframework.test.web.servlet.result.MockMvcResultMatchers.content
import org.springframework.test.web.servlet.result.MockMvcResultMatchers.jsonPath
import org.springframework.test.web.servlet.result.MockMvcResultMatchers.status
import org.springframework.test.web.servlet.setup.DefaultMockMvcBuilder
import org.springframework.test.web.servlet.setup.MockMvcBuilders
import org.springframework.web.context.WebApplicationContext
import java.time.LocalDateTime
import kotlin.test.Test

@WebMvcTest(AmrController::class)
@AutoConfigureMockMvc
class AmrControllerTest {
    private lateinit var mockMvc: MockMvc

    @MockkBean
    private lateinit var amrService: AmrService

    private val objectMapper = jacksonObjectMapper()

    @BeforeEach
    fun setUp(wContext: WebApplicationContext) {
        mockMvc = MockMvcBuilders.webAppContextSetup(wContext)
            .alwaysDo<DefaultMockMvcBuilder>(MockMvcResultHandlers.print())
            .build()
    }

    @Test
    fun `returns latest amr statuses`() {
        // given
        val statuses = (1..5).map { ValidAmrStatus.copy(id = it.toLong()) }
        every { amrService.findAllLatestStatuses() } returns statuses

        // when
        // then
        mockMvc.perform(get("/api/v1/amrs/latest-statuses"))
            .andExpectAll(
                status().isOk,
                content().contentType(MediaType.APPLICATION_JSON),
                jsonPath("$.length()").value(5),
                jsonPath("$[0].id").value(1),
                jsonPath("$[4].id").value(5),
            )
    }

    @Test
    fun `no amr status exists, then returns empty list`() {
        // given
        every { amrService.findAllLatestStatuses() } returns emptyList()

        // when
        // then
        mockMvc.perform(get("/api/v1/amrs/latest-statuses"))
            .andExpect(status().isOk)
            .andExpect(content().contentType(MediaType.APPLICATION_JSON))
            .andExpect(jsonPath("$.length()").value(0))
    }

    @Test
    fun `returns amr detail by id`() {
        // given
        every { amrService.findAmrDetail(123) } returns ValidAmrStatus.copy(
            id = 123,
            amr = ValidAmrStatus.amr.copy(name = "sample name")
        )

        // when
        // then
        mockMvc.perform(get("/api/v1/amrs/123/detail"))
            .andExpect(status().isOk)
            .andExpect(content().contentType(MediaType.APPLICATION_JSON))
            .andExpect(jsonPath("$.id").value(123))
            .andExpect(jsonPath("$.name").value("sample name"))
    }

    @Test
    fun `given id, no amr info exists, then not found error`() {
        // given
        every { amrService.findAmrDetail(123) } throws NoSuchAmr()

        // when
        // then
        mockMvc.perform(get("/api/v1/amrs/123/detail"))
            .andExpect(status().isNotFound)
    }

    @Test
    fun `send manual control message successfully`() {
        // given
        val serial = "AMR001"
        val area = "Area B"
        val message = TopicMessage.AmrManualControlMessage(serial = serial, area = area)
        every { amrService.sendManualControlMessage("AMR001", message) } just Runs

        // when
        // then
        val requestBody = AmrManualControlRequest(serial = serial, area = area)
        mockMvc.perform(
            post("/api/v1/amrs/control")
                .contentType(MediaType.APPLICATION_JSON)
                .content(objectMapper.writeValueAsString(requestBody))
        ).andExpect(status().isOk)
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
