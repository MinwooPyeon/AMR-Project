package com.example.amr_backend.v1.repository


object RedisUtil {
    const val SERIALS_KEY = "amr:serials"
    fun getRedisKey(serial: String): String = "amr:${serial}:status"
}