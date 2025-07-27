package com.android.ssamr.core.ui

import android.graphics.Paint.Align
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.heightIn
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.window.Dialog
import com.android.ssamr.R
import com.android.ssamr.ui.theme.SSAMRTheme

@Composable
fun SSAMRDialog(
    iconRes: Int,
    iconColor: Color,
    iconBgColor: Color,
    title: String,
    message: String? = null,
    buttons: List<DialogButton>,
    isLoading: Boolean = false,
    onDismiss: () -> Unit = {}
) {
    Dialog(onDismissRequest = onDismiss) {
        Surface(
            shape = RoundedCornerShape(16.dp),
            color = Color.White,
            tonalElevation = 4.dp,
            modifier = Modifier
                .width(320.dp)
                .heightIn(min = 220.dp, max = 480.dp)
        ) {
            Column(
                modifier = Modifier.padding(24.dp),
                horizontalAlignment = Alignment.CenterHorizontally
            ) {
                // Icon + 배경
                Box(
                    modifier = Modifier
                        .size(80.dp)
                        .background(iconBgColor, CircleShape),
                    contentAlignment = Alignment.Center
                ) {
                    Icon(
                        painter = painterResource(iconRes),
                        contentDescription = null,
                        modifier = Modifier
                            .size(36.dp),
                        tint = iconColor // 혹은 원본 컬러
                    )
                }
                Spacer(Modifier.height(20.dp))
                Text(
                    title,
                    style = MaterialTheme.typography.titleLarge,
                    fontWeight = FontWeight.Bold
                )
                message?.let {
                    Spacer(Modifier.height(8.dp))
                    Text(
                        it,
                        style = MaterialTheme.typography.bodyMedium,
                        color = Color(0xFF6D6D6D),
                        textAlign = TextAlign.Center
                    )
                    if (isLoading) {
                        Spacer(Modifier.height(24.dp))
                        DotsLoading(modifier = Modifier.align(Alignment.CenterHorizontally))
                    }
                }
                Spacer(Modifier.height(24.dp))
                buttons.forEachIndexed { idx, btn ->
                    Button(
                        onClick = btn.onClick,
                        modifier = Modifier
                            .fillMaxWidth()
                            .padding(top = if (idx > 0) 8.dp else 0.dp),
                        colors = ButtonDefaults.buttonColors(containerColor = btn.bgColor)
                    ) {
                        btn.iconRes?.let {
                            Icon(
                                painterResource(it),
                                contentDescription = null,
                                modifier = Modifier.size(20.dp)
                            )
                            Spacer(Modifier.width(8.dp))
                        }
                        Text(btn.text)
                    }
                }
            }
        }
    }
}

@Preview(showBackground = true)
@Composable
fun SSAMRDialogPreview() {
    SSAMRTheme {
        SSAMRDialog(
            iconRes = R.drawable.ic_home,
            iconColor = Color(0xFF2563EB),
            iconBgColor = Color(0xFFDBEAFE),
            title = "복귀 명령 전송",
            message = "AMR에 명령을 전송하고 있습니다.",
            isLoading = true,
            buttons = emptyList()
        ) {
        }
    }
}

data class DialogButton(
    val text: String,
    val onClick: () -> Unit,
    val bgColor: Color,
    val iconRes: Int? = null
)
