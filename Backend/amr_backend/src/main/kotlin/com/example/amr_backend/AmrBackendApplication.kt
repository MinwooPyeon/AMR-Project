package com.example.amr_backend

import org.springframework.boot.autoconfigure.SpringBootApplication
import org.springframework.boot.runApplication
import org.springframework.data.jpa.repository.config.EnableJpaAuditing

@SpringBootApplication
@EnableJpaAuditing
class AmrBackendApplication

fun main(args: Array<String>) {
    runApplication<AmrBackendApplication>(*args)
}
