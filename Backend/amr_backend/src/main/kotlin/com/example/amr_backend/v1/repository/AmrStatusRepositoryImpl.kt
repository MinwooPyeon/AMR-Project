package com.example.amr_backend.v1.repository

import com.example.amr_backend.v1.entity.AmrStatus
import org.slf4j.LoggerFactory
import org.springframework.data.redis.core.RedisTemplate
import org.springframework.data.redis.core.StringRedisTemplate
import org.springframework.stereotype.Repository
import java.time.ZoneOffset

private const val SERIALS_KEY = "amr:serials"

@Repository
class AmrStatusRepositoryImpl(
    private val amrStatusJpaRepository: AmrStatusJpaRepository,
    private val stringTemplate: StringRedisTemplate,
    private val amrStatusTemplate: RedisTemplate<String, AmrStatus>
) : AmrStatusRepository {
    private val logger = LoggerFactory.getLogger(this::class.java)

    override fun findAllLatestStatuses(): List<AmrStatus> {
        val cachedSerials = stringTemplate.opsForZSet().reverseRange(SERIALS_KEY, 0, -1)?.toList().orEmpty()
        logger.debug("cached serials : {}", cachedSerials)

        if (cachedSerials.isEmpty()) {
            val statusesFromDb = amrStatusJpaRepository.findAllLatestStatuses()
            putIntoRedis(statusesFromDb)
            return statusesFromDb
        }

        val redisKeys = cachedSerials.map { getRedisKey(it) }
        val cachedStatuses = amrStatusTemplate.opsForValue().multiGet(redisKeys).orEmpty()
        val nonNullStatuses = cachedStatuses.filterNotNull()

        if (nonNullStatuses.size == cachedSerials.size) return nonNullStatuses

        val missingSerials = cachedSerials.filterIndexed { index, _ -> cachedStatuses[index] == null }

        logger.debug("missing serials : {}", missingSerials)

        val missingStatuses = amrStatusJpaRepository.findAllByAmrSerialIn(missingSerials)
        putIntoRedis(missingStatuses)

        val associatedMissingStatuses = missingStatuses.associateBy { it.amr.serial }
        return cachedSerials.mapIndexed { index, serial ->
            cachedStatuses[index] ?: associatedMissingStatuses.getValue(serial)
        }
    }

    private fun putIntoRedis(statuses: List<AmrStatus>) {
        if (statuses.isEmpty()) return

        amrStatusTemplate.executePipelined {
            val valueOps = amrStatusTemplate.opsForValue()
            statuses.forEach { status ->
                valueOps.set(getRedisKey(status.amr.serial), status)
            }
            null
        }

        stringTemplate.executePipelined {
            val zSetOps = stringTemplate.opsForZSet()
            statuses.forEach { status ->
                zSetOps.add(
                    SERIALS_KEY,
                    status.amr.serial,
                    status.createdAt.toEpochSecond(ZoneOffset.UTC).toDouble()
                )
            }
            null
        }
    }

    private fun getRedisKey(serial: String): String = "amr:${serial}:status"

    override fun findAmrStatusById(id: Long): AmrStatus {
        return amrStatusJpaRepository.findAmrStatusById(id)
    }

    override fun save(amrStatus: AmrStatus): AmrStatus {
        return amrStatusJpaRepository.save(amrStatus)
    }
}