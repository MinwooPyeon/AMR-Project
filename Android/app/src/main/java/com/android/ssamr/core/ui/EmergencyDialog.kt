package com.android.ssamr.core.ui

import androidx.compose.runtime.Composable
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.tooling.preview.Preview
import com.android.ssamr.R

@Composable
fun EmergencyCallSSAMRDialog(
    onCallManager: () -> Unit,
    onCallFire: () -> Unit,
    onCancel: () -> Unit,
    onDismiss: () -> Unit = onCancel

) {
    SSAMRDialog(
        iconRes = R.drawable.ic_call,          // 전화 아이콘
        iconColor = Color(0xFFFF6F61),
        iconBgColor = Color(0xFFFFEEEE),
        title = "긴급 통화",
        message = "어디로 전화하시겠습니까?",
        buttons = listOf(
            DialogButton(
                text = "관리실 (내선 100)",
                onClick = onCallManager,
                bgColor = Color(0xFF2563EB),
                iconRes = R.drawable.ic_checking   // 관리실/건물 아이콘
            ),
            DialogButton(
                text = "119 (소방서)",
                onClick = onCallFire,
                bgColor = Color(0xFFDC2626),
                iconRes = R.drawable.ic_fire    // 소방서/불 아이콘
            ),
            DialogButton(
                text = "취소",
                onClick = onCancel,
                bgColor = Color(0xFFD1D5DB)
            )
        ),
        isLoading = false,
        onDismiss = onDismiss
    )
}

// 미리보기 프리뷰
@Preview(showBackground = true)
@Composable
fun EmergencyCallSSAMRDialogPreview() {
    EmergencyCallSSAMRDialog(
        onCallManager = {},
        onCallFire = {},
        onCancel = {}
    )
}
