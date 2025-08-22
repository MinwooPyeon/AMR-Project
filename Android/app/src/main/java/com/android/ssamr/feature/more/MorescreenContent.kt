package com.android.ssamr.feature.more

import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.AccountCircle
import androidx.compose.material.icons.filled.Info
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.ColorFilter
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.res.vectorResource
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.android.ssamr.R

@Composable
fun MorescreenContent(
    state: MorescreenState,
    onIntent: (MorescreenIntent) -> Unit
) {
    Column(
        modifier = Modifier
            .fillMaxSize()
            .padding(vertical = 16.dp)
    ) {
        ProfileCardSection(profile = state.userProfile) {
            onIntent(MorescreenIntent.OnProfileClick)
        }

        Spacer(modifier = Modifier.height(16.dp))

        SettingItemCard(
            icon = ImageVector.vectorResource(id = R.drawable.ic_report),
            title = "이벤트 리포트",
            onClick = { onIntent(MorescreenIntent.OnReportClick) }
        )

        SettingItemCard(
            icon = Icons.Default.Settings,
            title = "설정",
            onClick = { onIntent(MorescreenIntent.OnSettingClick) }
        )
        SettingItemCard(
            icon = ImageVector.vectorResource(id = R.drawable.ic_help),
            title = "도움말",
            onClick = { onIntent(MorescreenIntent.OnHelpClick) }
        )
        SettingItemCard(
            icon = ImageVector.vectorResource(id = R.drawable.ic_notification),
            title = "공지사항",
            onClick = { onIntent(MorescreenIntent.OnNoticeClick) }
        )
        SettingItemCard(
            icon = Icons.Default.Info,
            title = "버전정보",
            trailingText = state.appVersion,
            onClick = { onIntent(MorescreenIntent.OnVersionInfoClick) }
        )
    }
}

@Composable
private fun ProfileCardSection(
    profile: UserProfileUiModel,
    onClick: () -> Unit
) {
    Card(
        modifier = Modifier
            .fillMaxWidth()
            .padding(horizontal = 16.dp)
            .clickable { onClick() },
        shape = MaterialTheme.shapes.medium,
        colors = CardDefaults.cardColors(containerColor = Color.White)
    ) {
        Row(
            modifier = Modifier.padding(16.dp),
            verticalAlignment = Alignment.CenterVertically
        ) {
            if (profile.imageUrl.isNotBlank()) {
                // 실제 이미지 적용 시 Coil 등 사용
                Image(
                    painter = painterResource(id = R.drawable.ic_humanprofile),
                    contentDescription = null,
                    contentScale = ContentScale.Crop,
                    modifier = Modifier
                        .size(56.dp)
                        .clip(CircleShape)
                )
            } else {
                Icon(
                    imageVector = Icons.Default.AccountCircle,
                    contentDescription = null,
                    tint = MaterialTheme.colorScheme.primary,
                    modifier = Modifier.size(56.dp)
                )
            }

            Spacer(modifier = Modifier.width(16.dp))

            Column {
                Text(profile.name, style = MaterialTheme.typography.titleMedium)
                Text(profile.role, style = MaterialTheme.typography.bodySmall, color = MaterialTheme.colorScheme.onSurfaceVariant)
                Text(profile.department, style = MaterialTheme.typography.bodySmall, color = MaterialTheme.colorScheme.onSurfaceVariant)
            }
        }
    }
}

@Composable
private fun SettingItemCard(
    icon: ImageVector,
    title: String,
    trailingText: String? = null,
    onClick: () -> Unit
) {
    Card(
        modifier = Modifier
            .fillMaxWidth()
            .padding(horizontal = 16.dp, vertical = 6.dp)
            .clickable { onClick() },
        shape = MaterialTheme.shapes.medium,
        colors = CardDefaults.cardColors(containerColor = Color.White)
    ) {
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .padding(16.dp),
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.SpaceBetween
        ) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                Icon(
                    imageVector = icon,
                    contentDescription = null,
                    tint = MaterialTheme.colorScheme.primary
                )
                Spacer(modifier = Modifier.width(12.dp))
                Text(title, style = MaterialTheme.typography.bodyLarge)
            }

            if (trailingText != null) {
                Text(
                    text = trailingText,
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant
                )
            } else {
                Icon(
                    imageVector = ImageVector.vectorResource(R.drawable.arrow_forward),
                    contentDescription = null,
                    tint = MaterialTheme.colorScheme.onSurfaceVariant
                )
            }
        }
    }
}

@Preview(showBackground = true)
@Composable
fun MorescreenContentPreview() {
    val dummyState = MorescreenState(
        status = MorescreenStatus.Success,
        userProfile = UserProfileUiModel(
            name = "김철수",
            role = "안전 관리 책임자",
            department = "삼성전자 천안공장",
            imageUrl = "" // Coil 사용 전 기본 아이콘 표시
        ),
        appVersion = "v1.2.3"
    )

    MaterialTheme {
        MorescreenContent(
            state = dummyState,
            onIntent = {} // Preview에선 빈 Intent 처리
        )
    }
}
