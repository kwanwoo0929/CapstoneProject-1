package com.example.airis

import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.alpha
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp

@Composable
fun MainScreen(onStartClick: () -> Unit) {
    var isConnected by remember { mutableStateOf(false) }

    Box(
        modifier = Modifier
            .fillMaxSize()
            .background(Color(0xFFFDFDFD))
    ) {
        Column(
            modifier = Modifier
                .fillMaxSize()
                .statusBarsPadding()
                .navigationBarsPadding()
                .padding(horizontal = 24.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            //Spacer(modifier = Modifier.height(48.dp))

            // "나의 안경" 텍스트 영역 - 고정된 높이 확보
            Box(
                modifier = Modifier.height(48.dp),
                contentAlignment = Alignment.BottomCenter
            ) {
                if (isConnected) {
                    Text(
                        text = "나의 안경",
                        fontSize = 24.sp,
                        fontWeight = FontWeight.Bold,
                        color = Color.Black
                    )
                }
            }

            Spacer(modifier = Modifier.height(16.dp))

            // 안경 이미지
            Image(
                painter = painterResource(id = R.drawable.main_glasses),
                contentDescription = "안경",
                modifier = Modifier.size(192.dp)
            )

            Spacer(modifier = Modifier.height(16.dp))

            // 연결 상태
            Box(
                modifier = Modifier
                    .fillMaxWidth()
                    .height(50.dp)
                    .background(
                        color = Color(0xFFD9D9D9),
                        shape = RoundedCornerShape(32.dp)
                    )
            ) {
                Row(
                    modifier = Modifier
                        .fillMaxSize()
                        .padding(horizontal = 20.dp),
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Icon(
                        painter = painterResource(
                            if (isConnected) R.drawable.ic_sign_connected
                            else R.drawable.ic_sign_not_connected
                        ),
                        contentDescription = if (isConnected) "연결됨" else "연결 안됨",
                        modifier = Modifier.size(16.dp),
                        tint = if (isConnected) Color(0xFF4CAF50) else Color(0xFFDC3545)
                    )

                    Spacer(modifier = Modifier.width(12.dp))

                    Text(
                        text = if (isConnected) "연결됨" else "연결되지 않음",
                        fontSize = 14.sp,
                        fontWeight = FontWeight.Medium
                    )

                    Spacer(modifier = Modifier.weight(1f))

                    if (isConnected) {
                        Icon(
                            painter = painterResource(R.drawable.ic_battery_full),
                            contentDescription = "배터리 아이콘",
                            modifier = Modifier.size(24.dp)
                        )

                        Spacer(modifier = Modifier.width(8.dp))

                        Text(
                            text = "98 %",
                            fontSize = 14.sp,
                            fontWeight = FontWeight.Medium
                        )
                    }
                }
            }

            Spacer(modifier = Modifier.height(16.dp))

            // 위치 정보
            Row(
                modifier = Modifier.fillMaxWidth(),
                verticalAlignment = Alignment.CenterVertically
            ) {
                Icon(
                    painter = painterResource(R.drawable.ic_current_location),
                    contentDescription = "위치 아이콘",
                    modifier = Modifier.size(24.dp)
                )

                Spacer(modifier = Modifier.width(8.dp))

                Text(
                    text = "The Metropolitan Museum of Art",
                    fontSize = 14.sp,
                    fontWeight = FontWeight.Medium,
                    maxLines = 1,
                    overflow = TextOverflow.Ellipsis,
                    modifier = Modifier.weight(1f)
                )

                Spacer(modifier = Modifier.width(8.dp))

                Box(
                    modifier = Modifier
                        .width(55.dp)
                        .height(25.dp)
                        .clip(RoundedCornerShape(4.dp))
                        .background(color = Color(0xFFF0F0F0))
                        .clickable { },
                    contentAlignment = Alignment.Center
                ) {
                    Text(
                        text = "변경하기",
                        fontSize = 12.sp,
                        fontWeight = FontWeight.Medium
                    )
                }
            }

            Spacer(modifier = Modifier.height(16.dp))

            // 카메라 테스트
            MenuBox(
                title = "카메라 테스트",
                description = "카메라가 작품을 제대로\n인식하는지 확인해보세요",
                iconRes = R.drawable.main_cameratest,
                enabled = isConnected
            )

            Spacer(modifier = Modifier.height(16.dp))

            // 마이 히스토리
            MenuBox(
                title = "마이 히스토리",
                description = "지금까지 관람한 작품들을\n확인해보세요",
                iconRes = R.drawable.main_myhistory
            )

            Spacer(modifier = Modifier.height(24.dp))

            // 연결 버튼
            Button(
                onClick = {
                    isConnected = !isConnected
                    onStartClick()
                },
                modifier = Modifier
                    .fillMaxWidth()
                    .height(50.dp),
                shape = RoundedCornerShape(12.dp),
                colors = ButtonDefaults.buttonColors(
                    containerColor = Color(0xFFE0E0E0),
                    contentColor = Color.Black
                )
            ) {
                Text(
                    text = if (isConnected) "연결 끊기" else "눌러서 연결하기",
                    fontSize = 16.sp,
                    fontWeight = FontWeight.Medium
                )
            }
        }
    }
}

@Composable
private fun MenuBox(
    title: String,
    description: String,
    iconRes: Int,
    enabled: Boolean = true
) {
    Box(
        modifier = Modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(16.dp))
            .height(120.dp)
            .background(
                color = Color(0xFFF0F0F0),
                shape = RoundedCornerShape(16.dp)
            )
            .clickable(enabled = enabled) {  }
    ) {
        Row(
            modifier = Modifier
                .fillMaxSize()
                .padding(24.dp),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Column(
                modifier = Modifier.alpha(if (enabled) 1f else 0.4f)
            ) {
                Text(
                    text = title,
                    fontSize = 20.sp,
                    fontWeight = FontWeight.Medium
                )

                Spacer(modifier = Modifier.height(12.dp))

                Text(
                    text = description,
                    fontSize = 14.sp,
                    fontWeight = FontWeight.Medium,
                    lineHeight = 15.sp
                )
            }

            Image(
                painter = painterResource(id = iconRes),
                contentDescription = title,
                modifier = Modifier
                    .size(64.dp)
                    .alpha(if (enabled) 1f else 0.2f)
            )
        }
    }
}

@Preview(showBackground = true, showSystemUi = true)
@Composable
fun MainScreenPreview() {
    MaterialTheme {
        MainScreen(
            onStartClick = { }
        )
    }
}