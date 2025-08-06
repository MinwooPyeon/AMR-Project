package com.example.amr_backend.v1.storage

typealias ImageUri = String

interface ImageSaver {
    fun save(image: ByteArray): ImageUri
}