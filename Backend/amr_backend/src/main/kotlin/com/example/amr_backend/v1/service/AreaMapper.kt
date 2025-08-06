package com.example.amr_backend.v1.service

import org.springframework.stereotype.Service

@Service
class AreaMapper {
    // TODO : 맵 정보 생성 후 x, y 좌표에 따라 구역 이름 return하도록 변경
    fun convertCoordinateToArea(x: Double, y: Double): String = "A구역"
}