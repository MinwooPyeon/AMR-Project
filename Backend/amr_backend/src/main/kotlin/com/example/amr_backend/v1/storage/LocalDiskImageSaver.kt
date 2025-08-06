package com.example.amr_backend.v1.storage

import org.springframework.stereotype.Component
import java.nio.file.Files
import java.nio.file.Paths
import java.util.UUID

@Component
class LocalDiskImageSaver : ImageSaver {
    private val uploadPath = Paths.get("images")

    override fun save(image: ByteArray): ImageUri {
        Files.createDirectories(uploadPath)

        val fileName = "${UUID.randomUUID()}.jpg"
        val filePath = uploadPath.resolve(fileName)
        Files.write(filePath, image)

        return "/images/$fileName"
    }
}