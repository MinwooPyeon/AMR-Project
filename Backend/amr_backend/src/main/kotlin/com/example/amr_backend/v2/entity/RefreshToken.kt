package com.example.amr_backend.v2.entity

import jakarta.persistence.Column
import jakarta.persistence.Entity
import jakarta.persistence.EntityListeners
import jakarta.persistence.GeneratedValue
import jakarta.persistence.GenerationType
import jakarta.persistence.Id
import jakarta.persistence.JoinColumn
import jakarta.persistence.OneToOne
import jakarta.persistence.Table
import org.springframework.data.annotation.CreatedDate
import org.springframework.data.jpa.domain.support.AuditingEntityListener
import java.time.LocalDateTime

@Entity
@Table(name = "refresh_token")
@EntityListeners(AuditingEntityListener::class)
class RefreshToken(
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "id")
    var id: Long = 0L,
    @OneToOne
    @JoinColumn(name = "user_id")
    var user: User,
    @Column(name = "token")
    var token: String,
    @Column(name = "expires_at")
    var expiresAt: LocalDateTime,
) {
    @Column(name = "created_at")
    @CreatedDate
    lateinit var createdAt: LocalDateTime

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is RefreshToken) return false

        if (id != other.id) return false
        if (user != other.user) return false
        if (token != other.token) return false
        if (expiresAt != other.expiresAt) return false
        if (createdAt != other.createdAt) return false

        return true
    }

    override fun hashCode(): Int {
        var result = id.hashCode()
        result = 31 * result + user.hashCode()
        result = 31 * result + token.hashCode()
        result = 31 * result + expiresAt.hashCode()
        result = 31 * result + createdAt.hashCode()
        return result
    }

    override fun toString(): String {
        return "RefreshToken(id=$id, user=$user, token='$token', expiresAt=$expiresAt, createdAt=$createdAt)"
    }
}