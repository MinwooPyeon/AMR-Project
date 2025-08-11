package com.example.amr_backend.v1.repository

import com.example.amr_backend.v1.entity.AmrStatus
import com.example.amr_backend.v1.repository.RedisUtil.SERIALS_KEY
import com.example.amr_backend.v1.repository.RedisUtil.getRedisKey
import org.slf4j.LoggerFactory
import org.springframework.context.event.EventListener
import org.springframework.data.redis.core.RedisTemplate
import org.springframework.data.redis.core.StringRedisTemplate
import org.springframework.stereotype.Component
import java.time.ZoneOffset

@Component
class RedisUpdater(
    private val amrStatusTemplate: RedisTemplate<String, AmrStatus>,
    private val stringTemplate: StringRedisTemplate,
) {
    private val logger = LoggerFactory.getLogger(this::class.java)

    @EventListener
    fun save(status: AmrStatus) {
        logger.debug("saving {} status into redis", status.amr.serial)
        amrStatusTemplate.opsForValue().set(getRedisKey(status.amr.serial), status)
        stringTemplate.opsForZSet().add(
            SERIALS_KEY,
            status.amr.serial,
            status.createdAt.toEpochSecond(ZoneOffset.UTC).toDouble()
        )
    }
}