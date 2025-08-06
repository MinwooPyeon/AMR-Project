package com.example.amr_backend.v1.storage

import org.springframework.context.annotation.Configuration
import org.springframework.web.servlet.config.annotation.ResourceHandlerRegistry
import org.springframework.web.servlet.config.annotation.WebMvcConfigurer
import java.nio.file.Paths

@Configuration
class LocalImageStorageConfig : WebMvcConfigurer {
    override fun addResourceHandlers(registry: ResourceHandlerRegistry) {
        val imagePath = Paths.get("images").toUri().toString()
        registry.addResourceHandler("/images/**")
            .addResourceLocations(imagePath)
    }
}