// core/permission/PermissionsRequester.kt
package com.android.ssamr.core.permission

import android.content.pm.PackageManager
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.runtime.Composable
import androidx.compose.runtime.remember
import androidx.core.content.ContextCompat
import androidx.compose.ui.platform.LocalContext

/**
 * 필요한 순간에 호출해서 권한을 요청하는 래퍼.
 *
 * 사용:
 * val request = rememberPermissionsRequester(arrayOf(Manifest.permission.ACCESS_FINE_LOCATION))
 * Button(onClick = { request.launch() }) { Text("위치 권한 요청") }
 */
class PermissionsRequester(
    private val check: () -> Boolean,
    private val launchInternal: () -> Unit
) {
    fun isGranted(): Boolean = check()
    fun launch() = launchInternal()
}

@Composable
fun rememberPermissionsRequester(
    permissions: Array<String>,
    onAllGranted: () -> Unit = {},
    onAnyDenied: (denied: List<String>) -> Unit = {}
): PermissionsRequester {
    val context = LocalContext.current

    fun currentGranted(): Boolean =
        permissions.all {
            ContextCompat.checkSelfPermission(context, it) == PackageManager.PERMISSION_GRANTED
        }

    val launcher = rememberLauncherForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { result ->
        val denied = result.filterValues { it.not() }.keys.toList()
        if (denied.isEmpty()) onAllGranted() else onAnyDenied(denied)
    }

    return remember(permissions.joinToString()) {
        PermissionsRequester(
            check = { currentGranted() },
            launchInternal = { launcher.launch(permissions) }
        )
    }
}
