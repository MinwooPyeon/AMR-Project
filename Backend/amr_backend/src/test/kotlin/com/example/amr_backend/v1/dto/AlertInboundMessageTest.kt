package com.example.amr_backend.v1.dto

import com.fasterxml.jackson.datatype.jsr310.JavaTimeModule
import com.fasterxml.jackson.module.kotlin.jacksonObjectMapper
import org.hamcrest.CoreMatchers.`is`
import org.hamcrest.MatcherAssert.assertThat
import kotlin.test.Test

class AlertInboundMessageTest {
    private val objectMapper = jacksonObjectMapper()
        .registerModule(JavaTimeModule())

    @Test
    fun `parse valid JSON containing a base64-encoded png, then image should contain a png signature`() {
        // given
        // when
        val result = objectMapper.readValue(VALID_JSON, AlertInboundMessage::class.java)

        // then
        val signature = byteArrayOf(
            0x89.toByte(),
            0x50.toByte(),
            0x4E.toByte(),
            0x47.toByte(),
            0x0D.toByte(),
            0x0A.toByte(),
            0x1A.toByte(),
            0x0A.toByte(),
        )
        assertThat(result.image.take(signature.size).toByteArray(), `is`(signature))
    }

    @Test
    fun `parse valid JSON, then situation is deserialized correctly`() {
        // given
        // when
        val result = objectMapper.readValue(VALID_JSON, AlertInboundMessage::class.java)

        // then
        assertThat(result.case, `is`(Case.COLLAPSE))
    }

    companion object {
        private val VALID_JSON = """
            {
                "case": "COLLAPSE",
                "serial": "AMR001",
                "image": "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",
                "x": 128.00,
                "y": 37.50,
                "timeStamp": "2025-08-07T14:30:00"
            }
        """.trimIndent()
    }
}
