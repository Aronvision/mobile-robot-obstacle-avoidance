void odometry3() {
  if (x >= 25) {
    turn(30, -90);
  }
  if (x >= 25 && y >= 70) {
    motor(20, 20, 500); //500밀리초만큼 각 모터 20rpm으로 감
    motor(0, 0, 200); //멈춤
    sn_distance(); //소나센서값읽기
    sonor_in_check(); //소나값읽힐때 조건
    line_out_check3(); //코스밖으로 나가기직전에 조건
  }
}



void sn_distance() { //소나센서 거리계산 및 값 저장함수
  unsigned long duration[5];
  for (int i = 0; i < 5; i++) {
    digitalWrite(ECHO1 + (2 * i), LOW); //에코센서 핀 31,33,35,37,39
    delayMicroseconds(2);
    digitalWrite(TRIG1 + (2 * i), HIGH); //트리거핀 30,32,34,36,38
    delayMicroseconds(10);
    digitalWrite(TRIG1 + (2 * i), LOW);
    duration[i] = pulseIn(ECHO1 + (2 * i), HIGH, 4000);
    sn[i] = (double)340 * duration[i] / 20000;//배열로저장
    for (int i = 0; i < 3; i++) {
      if (sn[i] == 0) {
        sn[i] = 15;//타임아웃을 주었기 때문에 너무 먼 거리는 0으로 저장되었으므로 그럴경우 15로 다시 저장
      }
    }
  }
}



void sonor_in_check() { //소나센서로 장애물 인식할경우 조건

  if (sn[0] <= 12 && sn[1] <= 12 && sn[2] <= 12 ) { //3개 모두 장애물 탐지했을경우
    motor(-30, -30, 500);
    if (sn[0] == max(sn[0], sn[2])) turn(30, -90); //왼쪽이 가장클경우 우회전
    else if (sn[2] == max(sn[0], sn[2])) turn(30, 90); //오른쪽이 클경우 좌회전
  }
  else if (sn[0] > 12 && sn[1] <= 12 && sn[2] > 12 ) { //중앙소나센서만 인식할경우 우회전
    turn(30, -90);
  }
  else if (sn[0] <= 12 && sn[1] > 12 && sn[2] > 12 ) { //좌측 소나센서만 인식할경우 뒤로갔다가 우회전
    motor(-30, -30, 300);

    turn(30, -90);
  }
  else if (sn[0] > 12 && sn[1] > 12 && sn[2] <= 12 ) { //우측 소나센서만 인식할경우 뒤로갔다가 좌회전
    motor(-30, -30, 300);

    turn(30, 90);
  }
}


void line_out_check3() { //코스밖으로 나가기직전 오도메트리활용 조건
  if (x >= 0 && y >= 150) { // 우측 맵 밖으로 나가기 직전 조건 
    turn(30,90);  
  }
  if (final_y >= 90 && final_x < 145) { //우측코스밖나가기직전
    rightcorner();
    motor(10, 10, 1000);
  }
  if (x >= 180) { //위쪽 맵 밖으로 나가기 직전 조건 
    turn(30,90);  
    
  }
  if (x >= 180 && y>= 30 { //최종목적지 
    turn(30,180);
    sendMotor(-30,-30);
    delay(3000)
  }
}
 
