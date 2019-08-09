#include <RC100.h>
#include "snake_parameter.h"

#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
		
/* Control table defines */
#define BaudRate 3
#define LimitTemperature  12
#define P_GOAL_POSITION    30
#define P_GOAL_SPEED    32

#define ledred 1
#define ledgreen 2
#define ledblue 4
#define ledcyan ( ledgreen|ledblue )
#define ledmagenta ( ledblue|ledred )
#define ledyellow ( ledred|ledgreen )
#define ledwhite ( ledred|ledgreen|ledblue )

#define TIMER1_PERIOD ((uint32)10000) // microsec

RC100 Controller;
Dynamixel Dxl(DXL_BUS_SERIAL1);

void timer1_interrupt_handler();

double targety[UNIT_NUM];//+-150[deg]
double targetp[UNIT_NUM];//+-150[deg]

enum {
  snake,
  side,
  helix,
  other
}
mode, present_mode;
// modeが変更されたら，モード初期化の処理を1回だけ実行し
// preset_modeを変更する

// ボタンのデータ格納
int RcvData = 0;
double up = 0, right = 0;
int t = 0;

void setup() {
  //XL-320の初期化をちょっと待つ
  delay(1000);

  Dxl.begin(3);

  delay(500);   // このdelayがないと，100以上のIDを持つDxl
                // が動かない！

  // 一旦接続されている全てのDxlを点灯->全て消灯
  //Dxl.ledOn(BROADCAST_ID, ledwhite);
  //delay(500);
  //Dxl.ledOff(BROADCAST_ID);

  for(int i = 0; i < JOINT_NUM; i++) {
    Dxl.jointMode(CommandParameters[2*i]);
    Dxl.maxTorque(CommandParameters[2*i],512);
    Dxl.goalSpeed(CommandParameters[2*i], 512);
    Dxl.goalTorque(CommandParameters[2*i], 512);
    Dxl.alarmShutdown(CommandParameters[2*i], 2);
  }
  //delay(100);

  Controller.begin(1);
  //delay(100);

  RcvData = 0;
  mode = other;
  present_mode = other;

  initsnake();
  SerialUSB.print("SYSTEM INIT OK!!!\r\n");

  // タイマ1設定
  //// channeは1を使う
  const int channel = 1;
  //// タイマ停止
  Timer1.pause();
  //// 周期設定
  Timer1.setPeriod(TIMER1_PERIOD);
  //// コンペアチャンネルの設定
  Timer1.setCompare(channel, Timer1.getOverflow());
  //// 割り込み関数設定
  Timer1.attachInterrupt(channel, timer1_interrupt_handler);
  //// タイマのリフレッシュ(設定の適応)
  Timer1.refresh();
  //// タイマ再開
  Timer1.resume();
}

void loop() {
  if ( RcvData & RC100_BTN_1 ) {
    mode = snake;
    //Dxl.ledOn(BROADCAST_ID, ledblue);
  } 
  else if ( RcvData & RC100_BTN_2 ) {
    mode = side;
    //Dxl.ledOn(BROADCAST_ID, ledcyan);
  } 
  else if ( RcvData & RC100_BTN_3 ) {
    mode = helix;
    //Dxl.ledOn(BROADCAST_ID, ledgreen);
  }
  else if ( RcvData & RC100_BTN_4 ) {
    //Dxl.wheelMode(BROADCAST_ID);
    mode = other;
    //Dxl.ledOn(BROADCAST_ID, ledyellow);
  }
  //delay(100);
  unsigned char color;
  if (mode == snake ) color = ledblue;
  if (mode == side ) color = ledcyan;
  if (mode == helix ) color = ledgreen;
  if (mode == other ) color = ledwhite;

  if ( RcvData & RC100_BTN_U ) {
    up += 1;
  } 
  else if ( RcvData & RC100_BTN_D ) { 
    up -= 1;
  } 
  else if ( RcvData & RC100_BTN_R ) {
    right += 1;
  } 
  else if ( RcvData & RC100_BTN_L ) {
    right -= 1;
  }

  //モード初期化
  if ( present_mode != mode ){
    present_mode = mode; 
    up = 0;
    right = 0;
    //Dxl.writeWord( BROADCAST_ID, P_GOAL_SPEED, 512);// 
    //Dxl.writeWord( BROADCAST_ID, P_GOAL_POSITION, 512);// 初期設定　軸の位置　０度
    //initsnake();
    //delay(500);
    for ( int i = 0; i< UNIT_NUM; i++){
      targety[i] = 0;
      targetp[i] = 0;
    }

    if ( mode == snake ) {
      snakemode();
    } 
    else if ( mode == side ) {
      sidemode();
    } 
    else if ( mode == helix ) {
      helixmode();
    }
    else if ( mode == other ) {
      othermode();
    }

    settargetang();
    
    //だんだんうごかす      
    for ( int ui = 0 ; ui < JOINT_NUM ; ui++ ) {
      Dxl.goalPosition( CommandParameters[2*ui], CommandParameters[2*ui +1]);
      Dxl.ledOn( CommandParameters[2*ui], color);

      delay(30);
    }
  }
  else { // モード初期化不要の場合
    if ( mode == snake ) {
      snakemode();
    } 
    else if ( mode == side ) {
      sidemode();
    } 
    else if ( mode == helix ) {
      helixmode();
    }
    else if ( mode == other ) {
      othermode();
    }

    settargetang();

    //いっきにうごかす
    //byte syncWrite(byte start_addr, byte num_of_data, int *param, int array_length); 
    /********************************************************************************************************************************************************************************************/
    // for(int i = 0; i < 30; i++) {
  
    // Dxl.syncWrite( P_GOAL_POSITION, 1, CommandParameters,200);

    //だんだんうごかす      
    for ( int ui = 0 ; ui < JOINT_NUM ; ui++ ) {
      Dxl.goalPosition( CommandParameters[2*ui], CommandParameters[2*ui +1]);
      //Dxl.ledOn( CommandParameters[2*ui], color);
      // delay(3);
    }
  }
}

void snakemode() {

  double A = 60; 
  double w = 0.1;
  double phi = 1.5;
  //t = t++;
  t = up;
  for ( int i = 0; i< UNIT_NUM; i++){
    targetp[i] = 0;
    targety[i] =  A * sin ( w * t + phi * i );
  }

}


void sidemode () {

  double A = 90; 
  double w = 0.1;
  double phi = 1.8;
  //t = t++;
  t = up;
  for ( int i = 0; i< UNIT_NUM; i++){
    targety[i] = A * sin ( w * t + phi * i );
    targetp[i] = A * sin ( w * t + phi * i - M_PI/4 );
  }

}

void helixmode() {

  //double ds = 0.045;
  static double ds = 0.010;
  //double a = 0.009;
  double a = 0.012;
  //double b = 0.12/(2*M_PI);
  //double b = 0.0042;
  double b = 0.0062;
  double w = 0.030;
  double kap, tau;

  kap = a/(a*a + b*b);
  tau = b/(a*a + b*b);

  //t = t++;
  t = up;
  //ds = ds + right*0.001;
  
  
  
  //double psi;
  //psi = psi + t*0.01;
  //kap_p = -kappa*sin(psi);
  //kap_y = kappa*cos(psi);

  for ( int i = 0; i< UNIT_NUM; i++){
    targety[i] = 2.0*kap*ds*cos( (double)i  * 2.0 * ds * tau + w * (double)t )*360.0/(double)(2.0*M_PI);
    targetp[i] = -2.0*kap*ds*sin( ((double)i * 2.0 ) * ds * tau + w * (double)t )*360.0/(double)(2.0*M_PI);
  }

}

void othermode() {

  for ( int i = 0; i< UNIT_NUM; i++){
    targety[i] = 0;
    targetp[i] = 0;
  }
  //Dxl.writeWord( BROADCAST_ID, P_GOAL_SPEED, 512);// 
  //Dxl.writeWord( BROADCAST_ID, P_GOAL_POSITION, 512);// 初期設定　軸の位置　０度

}


void settargetang () {
  //目標角度設定　0-1023
  for ( int ui = 0 ; ui < UNIT_NUM ; ui++ ) {
    CommandParameters[2*2*ui + 1] = 512 + targetp[ui]/150.0*511; //ピッチ軸
    CommandParameters[2*2*ui + 3] = 512 + targety[ui]/150.0*511; //ヨー軸
  }
}

void initsnake() {
  unsigned char color;

  if (mode == snake ) color = ledblue;
  if (mode == side ) color = ledcyan;
  if (mode == helix ) color = ledgreen;
  if (mode == other ) color = ledwhite;

  for ( int ui = 0 ; ui < JOINT_NUM ; ui++ ) {
    Dxl.goalPosition(CommandParameters[2*ui], 512);
    Dxl.ledOn(CommandParameters[2*ui], color);
    delay(30);
  }
}

void timer1_interrupt_handler() {
  // コントローラのデータ取得
  if(Controller.available()) {
    RcvData = Controller.readData();
  }
}