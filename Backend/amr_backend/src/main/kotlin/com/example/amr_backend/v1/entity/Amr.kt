package com.example.amr_backend.v1.entity

import jakarta.persistence.Column
import jakarta.persistence.Entity
import jakarta.persistence.GeneratedValue
import jakarta.persistence.GenerationType
import jakarta.persistence.Id
import jakarta.persistence.Table
import java.time.LocalDateTime

@Entity
@Table(name = "amr")
class Amr(
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "id")
    var id: Long = 0L,

    @Column(name = "name")
    var name: String,

    @Column(name = "ip_address", nullable = false)
    var ipAddress: String,

    @Column(name = "serial", unique = true, nullable = false)
    var serial: String,

    @Column(name = "model")
    var model: String,

    @Column(name = "firmware_version")
    var firmwareVersion: String,

    @Column(name = "last_update_date")
    var lastUpdateDate: LocalDateTime,
) {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is Amr) return false

        if (id != other.id) return false
        if (name != other.name) return false
        if (ipAddress != other.ipAddress) return false
        if (serial != other.serial) return false
        if (model != other.model) return false
        if (firmwareVersion != other.firmwareVersion) return false
        if (lastUpdateDate != other.lastUpdateDate) return false

        return true
    }

    override fun hashCode(): Int {
        var result = id.hashCode()
        result = 31 * result + name.hashCode()
        result = 31 * result + ipAddress.hashCode()
        result = 31 * result + serial.hashCode()
        result = 31 * result + model.hashCode()
        result = 31 * result + firmwareVersion.hashCode()
        result = 31 * result + lastUpdateDate.hashCode()
        return result
    }

    override fun toString(): String {
        return "Amr(id=$id, name='$name', ipAddress='$ipAddress', serial='$serial', model='$model', firmwareVersion='$firmwareVersion', lastUpdateDate=$lastUpdateDate)"
    }
}
