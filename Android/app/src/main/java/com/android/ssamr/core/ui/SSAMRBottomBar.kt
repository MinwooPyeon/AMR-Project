package com.android.ssamr.core.ui

import androidx.compose.material3.Icon
import androidx.compose.material3.NavigationBar
import androidx.compose.material3.NavigationBarItem
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.android.ssamr.main.navigation.bottomNavScreens
import com.android.ssamr.ui.theme.SSAMRTheme

@Composable
fun SSAMRBottomBar(
    selectedRoute: String,
    onTabSelected: (String) -> Unit
) {
    NavigationBar(
        containerColor = Color.White,
        tonalElevation = 8.dp
    ) {
        bottomNavScreens.forEach { screen ->
            NavigationBarItem(
                icon = {
                    Icon(
                        painterResource(id = screen.iconRes),
                        contentDescription = screen.label
                    )
                },
                label = { Text(screen.label) },
                selected = selectedRoute == screen.route,
                onClick = { onTabSelected(screen.route) }
            )
        }
    }
}

@Preview(showBackground = true)
@Composable
fun SSAMRBottomBarPreview() {
    SSAMRTheme {
        SSAMRBottomBar(
            "more"
        ) { }
    }
}

@Preview(showBackground = true)
@Composable
fun SSAMRBottomBarPreview2() {
    SSAMRTheme {
        SSAMRBottomBar(
            "amr"
        ) { }
    }
}


//interface BottomNavScreen
//data class ProfileScreen() : BottomNavScreen

//    val navController = rememberNavController()
//    val bottomNavScreens = listOf("home")
//
//    val shouldShowBottomNav by remember {
//        derivedStateOf {
////            navController.currentDestination?.route in bottomNavScreens
//            navController.currentDestination.class is BottomNavScreen
//        }
//    }
