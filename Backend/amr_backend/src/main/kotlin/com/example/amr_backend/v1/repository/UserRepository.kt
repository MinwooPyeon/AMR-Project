package com.example.amr_backend.v1.repository

import com.example.amr_backend.v1.entity.User
import org.springframework.data.jpa.repository.JpaRepository

interface UserRepository : JpaRepository<User, Long> {
}