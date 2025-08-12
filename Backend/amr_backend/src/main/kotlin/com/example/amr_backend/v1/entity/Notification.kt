package com.example.amr_backend.v1.entity

import com.example.amr_backend.v1.dto.Case
import jakarta.persistence.Column
import jakarta.persistence.Entity
import jakarta.persistence.EntityListeners
import jakarta.persistence.EnumType
import jakarta.persistence.Enumerated
import jakarta.persistence.GeneratedValue
import jakarta.persistence.GenerationType
import jakarta.persistence.Id
import jakarta.persistence.Table
import org.springframework.data.annotation.CreatedDate
import org.springframework.data.jpa.domain.support.AuditingEntityListener
import java.time.LocalDateTime

@Entity
@Table(name = "notification")
@EntityListeners(AuditingEntityListener::class)
class Notification(
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "id")
    var id: Long = 0L,
    @Column(name = "title")
    var title: String,
    @Column(name = "content")
    var content: String,
    @Column(name = "area")
    var area: String,
    @Enumerated(EnumType.STRING)
    @Column(name = "_case")
    var case: Case,
    @Column(name = "image")
    var image: String? = null,
    @Column(name = "isRead")
    var isRead: Boolean = false,
    @Column(name = "readAt", nullable = true)
    var readAt: LocalDateTime? = null,
) {
    @Column(name = "createAt")
    @CreatedDate
    lateinit var createAt: LocalDateTime

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is Notification) return false

        if (id != other.id) return false
        if (title != other.title) return false
        if (content != other.content) return false
        if (area != other.area) return false
        if (case != other.case) return false
        if (image != other.image) return false
        if (isRead != other.isRead) return false
        if (readAt != other.readAt) return false
        if (createAt != other.createAt) return false

        return true
    }

    override fun hashCode(): Int {
        var result = id.hashCode()
        result = 31 * result + title.hashCode()
        result = 31 * result + content.hashCode()
        result = 31 * result + area.hashCode()
        result = 31 * result + case.hashCode()
        result = 31 * result + (image?.hashCode() ?: 0)
        result = 31 * result + isRead.hashCode()
        result = 31 * result + (readAt?.hashCode() ?: 0)
        result = 31 * result + createAt.hashCode()
        return result
    }
}