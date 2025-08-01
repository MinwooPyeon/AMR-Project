package com.example.amr_backend.v1.exception

import org.springframework.http.HttpStatus
import org.springframework.web.bind.annotation.ResponseStatus

@ResponseStatus(code = HttpStatus.NOT_FOUND)
class NoSuchAmr(message: String? = null) : RuntimeException(message)