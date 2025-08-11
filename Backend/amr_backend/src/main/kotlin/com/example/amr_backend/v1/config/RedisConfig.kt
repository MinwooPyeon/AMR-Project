package com.example.amr_backend.v1.config

import com.example.amr_backend.v1.entity.AmrStatus
import com.fasterxml.jackson.databind.ObjectMapper
import org.springframework.beans.factory.annotation.Value
import org.springframework.context.annotation.Bean
import org.springframework.context.annotation.Configuration
import org.springframework.data.redis.connection.RedisConnectionFactory
import org.springframework.data.redis.connection.lettuce.LettuceConnectionFactory
import org.springframework.data.redis.core.RedisTemplate
import org.springframework.data.redis.core.StringRedisTemplate
import org.springframework.data.redis.serializer.Jackson2JsonRedisSerializer
import org.springframework.data.redis.serializer.StringRedisSerializer

@Configuration
class RedisConfig(
    @Value("\${spring.data.redis.host}") private val redisHost: String,
    @Value("\${spring.data.redis.port}") private val redisPort: Int
) {
    @Bean
    fun redisConnectionFactory(): RedisConnectionFactory {
        return LettuceConnectionFactory(redisHost, redisPort)
    }

    @Bean
    fun amrStatusRedisTemplate(
        objectMapper: ObjectMapper,
        redisConnectionFactory: RedisConnectionFactory,
    ): RedisTemplate<String, AmrStatus> = RedisTemplate<String, AmrStatus>().apply {
        connectionFactory = redisConnectionFactory
        keySerializer = StringRedisSerializer()
        valueSerializer = Jackson2JsonRedisSerializer(objectMapper, AmrStatus::class.java)

        isEnableDefaultSerializer = false
        afterPropertiesSet()
    }

    @Bean
    fun stringRedisTemplate(redisConnectionFactory: RedisConnectionFactory): StringRedisTemplate =
        StringRedisTemplate(redisConnectionFactory)
}