package com.android.ssamr.core.ui

import android.graphics.BitmapFactory
import android.util.Base64
import androidx.compose.ui.graphics.ImageBitmap
import androidx.compose.ui.graphics.asImageBitmap

object ImageDecoder {
    fun decodeBase64ToImageBitmap(base64: String): ImageBitmap {
        val imageBytes = Base64.decode(base64, Base64.DEFAULT)
        val bitmap = BitmapFactory.decodeByteArray(imageBytes, 0, imageBytes.size)
        return bitmap.asImageBitmap()
    }
}
