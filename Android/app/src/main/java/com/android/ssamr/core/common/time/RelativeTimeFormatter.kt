package com.android.ssamr.core.common.time

import android.os.Build
import androidx.annotation.RequiresApi
import java.time.*
import java.time.format.DateTimeFormatter
import java.time.format.DateTimeFormatterBuilder
import java.time.temporal.ChronoField

@RequiresApi(Build.VERSION_CODES.O)
private val ISO_WITH_MICROS: DateTimeFormatter = DateTimeFormatterBuilder()
    .appendPattern("yyyy-MM-dd'T'HH:mm:ss")
    .optionalStart()
    .appendFraction(ChronoField.NANO_OF_SECOND, 1, 9, true) // .S / .SSS / .SSSSSSSSS 모두 허용
    .optionalEnd()
    .toFormatter()

@RequiresApi(Build.VERSION_CODES.O)
private val ISO_WITH_TZ: DateTimeFormatter = DateTimeFormatter.ISO_OFFSET_DATE_TIME
@RequiresApi(Build.VERSION_CODES.O)
private val ISO_INSTANT_FMT: DateTimeFormatter = DateTimeFormatter.ISO_INSTANT
@RequiresApi(Build.VERSION_CODES.O)
private val ISO_NO_FRACTION: DateTimeFormatter = DateTimeFormatter.ofPattern("yyyy-MM-dd'T'HH:mm:ss")

/** createdAt(문자열) → "n분 전 / n시간 전 / n일 전 / 방금 전" */
@RequiresApi(Build.VERSION_CODES.O)
fun formatRelativeTimeKorean(
    createdAt: String,
    now: Instant = Instant.now(),
    zone: ZoneId = ZoneId.systemDefault()
): String {
    val createdInstant = parseToInstant(createdAt, zone) ?: return createdAt

    val duration = Duration.between(createdInstant, now)
    val seconds = duration.seconds

    // 미래 타임스탬프가 오면 너무 멀지 않다면 '방금 전' 처리
    if (seconds < 0) {
        return if (seconds > -60) "방금 전"
        else DateTimeFormatter.ofPattern("yyyy.MM.dd").withZone(zone).format(createdInstant)
    }

    return when {
        seconds < 60 -> "방금 전"
        seconds < 60 * 60 -> "${seconds / 60}분 전"
        seconds < 60 * 60 * 24 -> "${seconds / 3600}시간 전"
        else -> "${seconds / (3600 * 24)}일 전"
    }
}

/**
 * createdAt → "8월 12일 오후 3:41"
 */
@RequiresApi(Build.VERSION_CODES.O)
fun formatMonthDayTimeKorean(
    createdAt: String,
    zone: ZoneId = ZoneId.systemDefault()
): String {
    val instant = parseToInstant(createdAt, zone) ?: return createdAt
    val dateTime = instant.atZone(zone)

    val amPm = if (dateTime.hour < 12) "오전" else "오후"
    val hour12 = if (dateTime.hour % 12 == 0) 12 else dateTime.hour % 12
    val minute = dateTime.minute.toString().padStart(2, '0')

    return "${dateTime.monthValue}월 ${dateTime.dayOfMonth}일 $amPm $hour12:$minute"
}

@RequiresApi(Build.VERSION_CODES.O)
private fun parseToInstant(raw: String, zone: ZoneId): Instant? {
    // epoch millis 형태도 허용
    raw.toLongOrNull()?.let { return Instant.ofEpochMilli(it) }

    // 1) 타임존 포함 ISO (ex: 2025-08-12T03:41:12.123+09:00)
    runCatching { return OffsetDateTime.parse(raw, ISO_WITH_TZ).toInstant() }

    // 2) Z 포함 ISO Instant (ex: 2025-08-12T03:41:12Z)
    runCatching { return Instant.from(ISO_INSTANT_FMT.parse(raw)) }

    // 3) 마이크로/나노초 포함(타임존 없음) → 서버 시간을 "시스템 타임존의 LocalDateTime"으로 간주
    runCatching { return LocalDateTime.parse(raw, ISO_WITH_MICROS).atZone(zone).toInstant() }

    // 4) 초까지만 있는 ISO (타임존 없음)
    runCatching { return LocalDateTime.parse(raw, ISO_NO_FRACTION).atZone(zone).toInstant() }

    return null
}

