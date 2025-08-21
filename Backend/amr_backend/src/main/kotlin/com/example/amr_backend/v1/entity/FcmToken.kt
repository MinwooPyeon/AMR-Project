package com.example.amr_backend.v1.entity

import jakarta.persistence.Column
import jakarta.persistence.Entity
import jakarta.persistence.GeneratedValue
import jakarta.persistence.GenerationType
import jakarta.persistence.Id
import jakarta.persistence.Table

@Entity
@Table(name = "fcm_token")
class FcmToken(
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "id")
    var id: Long = 0L,
    @Column(name = "token")
    var token: String,
) {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is FcmToken) return false

        if (id != other.id) return false
        if (token != other.token) return false

        return true
    }

    override fun hashCode(): Int {
        var result = id.hashCode()
        result = 31 * result + token.hashCode()
        return result
    }

    override fun toString(): String {
        return "FcmToken(id=$id, token='$token')"
    }
}
