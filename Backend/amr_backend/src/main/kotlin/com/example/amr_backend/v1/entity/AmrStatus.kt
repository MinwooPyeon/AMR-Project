package com.example.amr_backend.v1.entity

import jakarta.persistence.Column
import jakarta.persistence.Entity
import jakarta.persistence.EntityListeners
import jakarta.persistence.EnumType
import jakarta.persistence.Enumerated
import jakarta.persistence.GeneratedValue
import jakarta.persistence.GenerationType
import jakarta.persistence.Id
import jakarta.persistence.JoinColumn
import jakarta.persistence.ManyToOne
import jakarta.persistence.Table
import org.springframework.data.annotation.CreatedDate
import org.springframework.data.jpa.domain.support.AuditingEntityListener
import java.time.LocalDateTime

@Entity
@EntityListeners(AuditingEntityListener::class)
@Table(name = "amr_status")
class AmrStatus(
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "id")
    var id: Long = 0L,

    @ManyToOne
    @JoinColumn(name = "amr_serial", referencedColumnName = "serial", nullable = false)
    var amr: Amr,

    @Column(name = "state")
    @Enumerated(EnumType.STRING)
    var state: State,

    @Column(name = "x")
    var x: Double,

    @Column(name = "y")
    var y: Double,

    @Column(name = "speed")
    var speed: Double,

    @Column(name = "angle")
    var angle: Double,

    @Column(name = "_zone")
    var zone: String? = null
) {
    @CreatedDate
    @Column(name = "created_at")
    lateinit var createdAt: LocalDateTime

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is AmrStatus) return false

        if (id != other.id) return false
        if (amr != other.amr) return false
        if (state != other.state) return false
        if (x != other.x) return false
        if (y != other.y) return false
        if (speed != other.speed) return false
        if (angle != other.angle) return false
        if (zone != other.zone) return false
        if (createdAt != other.createdAt) return false

        return true
    }

    override fun hashCode(): Int {
        var result = id.hashCode()
        result = 31 * result + amr.hashCode()
        result = 31 * result + state.hashCode()
        result = 31 * result + x.hashCode()
        result = 31 * result + y.hashCode()
        result = 31 * result + speed.hashCode()
        result = 31 * result + angle.hashCode()
        result = 31 * result + (zone?.hashCode() ?: 0)
        result = 31 * result + createdAt.hashCode()
        return result
    }

    override fun toString(): String {
        return "AmrStatus(id=$id, amr=$amr, state=$state, x=$x, y=$y, speed=$speed, angle=$angle, zone=$zone, createdAt=$createdAt)"
    }
}
