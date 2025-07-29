package com.example.amr_backend.v1.entity

import jakarta.persistence.Column
import jakarta.persistence.Entity
import jakarta.persistence.EntityListeners
import jakarta.persistence.GeneratedValue
import jakarta.persistence.GenerationType
import jakarta.persistence.Id
import jakarta.persistence.Table
import org.hibernate.proxy.HibernateProxy
import org.springframework.data.annotation.CreatedDate
import org.springframework.data.jpa.domain.support.AuditingEntityListener
import java.time.LocalDateTime

@Entity
@EntityListeners(AuditingEntityListener::class)
@Table(name = "amr_status")
data class AmrStatus(
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "id")
    val id: Long = 0L,

    @Column(name = "name")
    val name: String,

    @Column(name = "status")
    val status: String,

    @Column(name = "battery_level")
    val batteryLevel: Int,

    @Column(name = "x")
    val x: Double,

    @Column(name = "y")
    val y: Double,

    @Column(name = "speed")
    val speed: Double,

    @Column(name = "serial")
    val serial: String,

    @Column(name = "firmware_version")
    val firmwareVersion: String,

    @Column(name = "last_update_date")
    val lastUpdateDate: LocalDateTime,
) {
    @CreatedDate
    @Column(name = "created_at")
    lateinit var createdAt: LocalDateTime

    final override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other == null) return false
        val oEffectiveClass =
            if (other is HibernateProxy) other.hibernateLazyInitializer.persistentClass else other.javaClass
        val thisEffectiveClass =
            if (this is HibernateProxy) this.hibernateLazyInitializer.persistentClass else this.javaClass
        if (thisEffectiveClass != oEffectiveClass) return false
        other as AmrStatus

        return id == other.id
    }

    final override fun hashCode(): Int =
        if (this is HibernateProxy) this.hibernateLazyInitializer.persistentClass.hashCode() else javaClass.hashCode()

    @Override
    override fun toString(): String {
        return this::class.simpleName + "(id = $id , name = $name , status = $status , batteryLevel = $batteryLevel , x = $x , y = $y , speed = $speed , serial = $serial , firmwareVersion = $firmwareVersion , lastUpdateDate = $lastUpdateDate )"
    }
}
