package com.android.ssamr.core.ui

import androidx.compose.material3.CenterAlignedTopAppBar
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.material3.TopAppBarDefaults
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.shadow
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.android.ssamr.R
import com.android.ssamr.ui.theme.SSAMRTheme

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun SSAMRTopAppBar(
    title: String,
    modifier: Modifier = Modifier,
    showBack: Boolean = true,
    onBackClick: (() -> Unit)? = null,
    actions: @Composable (() -> Unit)? = null
) {
    CenterAlignedTopAppBar(
        title = {
            Text(
                text = title,
                style = MaterialTheme.typography.titleLarge,
                color = Color.Black,
                maxLines = 1,
                overflow = TextOverflow.Ellipsis
            )
        },
        navigationIcon = {
            if (showBack && onBackClick != null) {
                IconButton(onClick = onBackClick) {
                    Icon(
                        painter = painterResource(id = R.drawable.ic_back), // 프로젝트 내 뒤로가기 아이콘 리소스
                        contentDescription = "뒤로가기"
                    )
                }
            }
        },
        actions = {
            actions?.invoke()
        },
        colors = TopAppBarDefaults.centerAlignedTopAppBarColors(
            containerColor = Color.White,
            titleContentColor = Color.Black
        ),
        modifier = modifier
            .shadow(4.dp)
    )
}

@Preview(showBackground = true)
@Composable
fun SSMARTopAppBarPreview() {
    SSAMRTheme {
        SSAMRTopAppBar(
            title = "제목",
            showBack = true,
            onBackClick = {}
        ) {}
    }
}
