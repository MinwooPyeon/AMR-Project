package com.example.amr_backend.v1.config

import com.influxdb.client.InfluxDBClient
import com.influxdb.client.InfluxDBClientFactory
import org.springframework.beans.factory.annotation.Value
import org.springframework.context.annotation.Bean
import org.springframework.context.annotation.Configuration

@Configuration
class InfluxDBConfig {
    @Bean
    fun influxDBClient(
        @Value("\${spring.influx-db.url}") url: String,
        @Value("\${spring.influx-db.token}") token: String,
        @Value("\${spring.influx-db.org}") org: String,
        @Value("\${spring.influx-db.bucket}") bucket: String,
    ): InfluxDBClient = InfluxDBClientFactory.create(
        url, token.toCharArray(), org, bucket
    )
}