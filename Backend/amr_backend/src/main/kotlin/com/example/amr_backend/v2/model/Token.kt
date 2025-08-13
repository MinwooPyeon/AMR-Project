package com.example.amr_backend.v2.model

data class Token(
    val accessToken: String,
    val refreshToken: String,
)