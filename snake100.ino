#include <RC100.h>

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

#define IS_YAW(i) ((i%2 == 0) == odd_joint_is_yaw)
#define GOAL_POSITION_PARAM_ZERO 512
#define GOAL_POSITION_PARAM_PAR_ONE_RADIAN (512 / ((5.0/6.0)*PI))

RC100 Controller;
Dynamixel Dxl(DXL_BUS_SERIAL1);

// サーボモータの個数に関する定義
#define UNIT_NUM 50               // サーボモータ2個で1ユニット．
#define JOINT_NUM (UNIT_NUM * 2)  // 100個サーボモータをつないだとき，
                                  // UNIT_NUM 50, JOINT_NUM 100

void timer1_interrupt_handler();

double target_joint_angles[JOINT_NUM] = {0};
bool odd_joint_is_yaw = false;

int CommandParameters[200]=
{
  1,   0,
  101, 0,
  2,   0,
  102, 0,
  3,   0,
  103, 0,
  4,   0,
  104, 0,
  5,   0,
  105, 0,

  6,   0,
  106, 0,
  7,   0,
  107, 0,
  8,   0,
  108, 0,
  9,   0,
  109, 0,
  10,  0,
  110, 0,

  11,  0,
  111, 0,
  12,  0,
  112, 0,
  13,  0,
  113, 0,
  14,  0,
  114, 0,
  15,  0,
  115, 0,
  
  16,  0,
  116, 0,
  17,  0,
  117, 0,
  18,  0,
  118, 0,
  19,  0,
  119, 0,
  20,  0,
  120, 0,

  21,  0,
  121, 0,
  22,  0,
  122, 0,
  23,  0,
  123, 0,
  24,  0,
  124, 0,
  25,  0,
  125, 0,
  
  26,  0,
  126, 0,
  27,  0,
  127, 0,
  28,  0,
  128, 0,
  29,  0,
  129, 0,
  30,  0,
  130, 0,
  
  31,  0,
  131, 0,
  32,  0,
  132, 0,
  33,  0,
  133, 0,
  34,  0,
  134, 0,
  35,  0,
  135, 0,
  
  36,  0,
  136, 0,
  37,  0,
  137, 0,
  38,  0,
  138, 0,
  39,  0,
  139, 0,
  40,  0,
  140, 0,

  41,  0,
  141, 0,
  42,  0,
  142, 0,
  43,  0,
  143, 0,
  44,  0,
  144, 0,
  45,  0,
  145, 0,

  46,  0,
  146, 0,
  47,  0,
  147, 0,
  48,  0,
  148, 0,
  49,  0,
  149, 0,
  50,  0,
  150, 0,
};

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

  delay(3000);
  // ここのdelayは大事!
  // 100以上のIDを持つDynamixelと通信できない問題に対して，ここにdelay(500)を追加
  // [2019年10月26日]
  // 60リンク以上つないだ時にDynamixelとまったく通信できなくなる問題に対して，
  // delay(500)をdelay(3000)に変更し，ヘビの先頭に1000uFの電解コンデンサを追加

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
    for ( int i = 0; i< JOINT_NUM; i++){
      target_joint_angles[i] = 0;
    }

    if ( mode == snake ) {
      snakemode();
    } 
    else if ( mode == side ) {
      sidemode();
    } 
    else if ( mode == helix ) {
      pedalmode();
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
      pedalmode();
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

  double A = 1.0 /3.0 * PI;      // 最大の曲率
  double w = 0.1;                // 動作速度パラメータ
  double phi = 0.0015 /* [m] */; // リンク長
  double L = 0.015 /* [m] */;    // 1周期分の長さ

  t = up;

  for ( int i = 0; i< JOINT_NUM; i++){
    if(IS_YAW(i)) {
      target_joint_angles[i] = A * sin(w*t + phi*i * (2*PI / L));
    }
    else {
      target_joint_angles[i] = 0;
    }
  }
}


void sidemode () {

  double A = 90; 
  double w = 0.1;
  double phi = 1.8;
  //t = t++;
  t = up;
  for ( int i = 0; i< UNIT_NUM; i++){
    // targety[i] = A * sin ( w * t + phi * i );
    // targetp[i] = A * sin ( w * t + phi * i - M_PI/4 );
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
    // targety[i] = 2.0*kap*ds*cos( (double)i  * 2.0 * ds * tau + w * (double)t )*360.0/(double)(2.0*M_PI);
    // targetp[i] = -2.0*kap*ds*sin( ((double)i * 2.0 ) * ds * tau + w * (double)t )*360.0/(double)(2.0*M_PI);
  }

}

void othermode() {

  for ( int i = 0; i< UNIT_NUM; i++){
    // targety[i] = 0;
    // targetp[i] = 0;
  }
  //Dxl.writeWord( BROADCAST_ID, P_GOAL_SPEED, 512);// 
  //Dxl.writeWord( BROADCAST_ID, P_GOAL_POSITION, 512);// 初期設定　軸の位置　０度

}


void pedalmode() {
 
 double l_p=0.5; //縦波の波長
 double l_y=0.1; //横波の波長
 double alpha_p=M_PI/6; //縦波における体と推進方向の最大角度
 double alpha_y=M_PI/6; //横波における体と推進方向の最大角度
 double T=10; //ペダルウェーブの周期
 double v=0.003; //ペダルウェーブの移動速度
 double phi=0.045; //1リンクごとの長さ
 
 t=up;

// kappa_y=(2*M_PI/l_)*alpha_*sin(2*M_PI*(s-v*t)/l_);
 //kappa_p = (2*M_PI/l_p_)*alpha_p_*sin(2*M_PI*(s/l_p_ + t/t_p_));
 for ( int i = 0; i< JOINT_NUM; i++){

   if(IS_YAW(i)){
    target_joint_angles[i]=alpha_y*sin(2*M_PI*(i*phi-v*1)/l_y);
   }else{
    target_joint_angles[i]=alpha_p*sin(2*M_PI*((i*phi)/l_p+ t/T));
   }
 }

}

void settargetang () {
  //目標角度設定　0-1023
  for (int i = 0; i < JOINT_NUM; i++) {
    CommandParameters[2 * i + 1] =
      GOAL_POSITION_PARAM_ZERO - target_joint_angles[i] * GOAL_POSITION_PARAM_PAR_ONE_RADIAN; //ピッチ軸
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