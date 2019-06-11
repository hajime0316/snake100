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

RC100 Controller;
Dynamixel Dxl(DXL_BUS_SERIAL1);


#define nou 50
#define JOINT_NUM nou*2
double targety[nou];//+-150[deg]
double targetp[nou];//+-150[deg]


int CommandParameters[200]=
{
  21, 0, 
  121, 0,
  22, 0,  
  122, 0,
  23, 0,
  123, 0,
  24, 0,
  124, 0,
  25,0,
  125, 0,
  
  26, 0, 
  126, 0,
  27, 0,  
  127, 0,
  28, 0,
  128, 0,
  29, 0,
  129, 0,
  30, 0,
  130, 0,
    
  31, 0, 
  131, 0,
  32, 0,  
  132, 0,
  33, 0,
  133, 0,
  34, 0,
  134, 0,
  35, 0,
  135, 0,
  
  36, 0, 
  136, 0,
  37, 0,  
  137, 0,
  38, 0,
  138, 0,
  39, 0,
  139, 0,
  40, 0,
  140, 0,
  
  41, 0, 
  141, 0,
  42, 0,  
  142, 0,
  43, 0,
  143, 0,
  44, 0,
  144, 0,
  45, 0,
  145, 0,
  
  46, 0, 
  146, 0,
  47, 0,  
  147, 0,
  48, 0,
  148, 0,
  49, 0,
  149, 0,
  50, 0,
  150, 0,
    
  51, 0, 
  151, 0,
  52, 0,  
  152, 0,
  53, 0,
  153, 0,
  54, 0,
  154, 0,
  55, 0,
  155, 0,
  
  56, 0, 
  156, 0,
  57, 0,  
  157, 0,
  58, 0,
  158, 0,
  59, 0,
  159, 0,
  60, 0,
  160, 0,
  
    1, 0, 
  101, 0,
  2, 0,  
  102, 0,
  3, 0,
  103, 0,
  4, 0,
  104, 0,
  5, 0,
  105, 0,
  
   6, 0, 
  106, 0,
  7, 0,  
  107, 0,
  8, 0,
  108, 0,
  9, 0,
  109, 0,
  10, 0,
  110, 0
};

enum {
  snake,
  side,
  helix,
  other
}
mode, premode;
/*
 * premode = present mode 現在のモード
 * モード変更したい場合はmode変数を変更する
 */

//// ボタンのデータ格納
int RcvData = 0;
double up = 0, right = 0;
int t = 0;

void setup() {
  //XL-320の初期化をちょっと待つ
  delay(1000);

  Dxl.begin(3);

  //  Dxl.writeByte( BROADCAST_ID, LimitTemperature, 50); //初期設定　温度リミット
  //  delay(100);
  Dxl.writeWord( BROADCAST_ID, P_GOAL_SPEED, 512);// 
  Dxl.writeWord( BROADCAST_ID, P_GOAL_POSITION, 512);// 初期設定　軸の位置　０度
  delay(100);
  //  Dxl.writeWord( BROADCAST_ID, P_GOAL_SPEED, 0 );//初期設定　最高速度
  //  delay(100);

  int count = 0;
  for(count = 0; count<=7;count++){
    //Dxl.ledOn(ID_NUM, count);// XL-320 only movement XL-320 it has RGB LED 
    Dxl.ledOn(BROADCAST_ID, count);// XL-320 only movement XL-320 it has RGB LED 
    delay(100);
    //Dxl.ledOff(ID_NUM);//All Dynamixel LED off
    Dxl.ledOff(BROADCAST_ID);//All Dynamixel LED off
    delay(100);
  }

  //Dxl.jointMode(ID_NUM); //jointMode() is to use position mode
  Dxl.jointMode(BROADCAST_ID); //jointMode() is to use position mode
  //delay(100);

  Controller.begin(1);
  //delay(100);

  RcvData = 0;
  mode = other;
  premode = other;

  initsnake();
  SerialUSB.print("SYSTEM INIT OK!!!\r\n");
}

void loop() {
  // コントローラのデータ取得
  if(Controller.available()) {
    RcvData = Controller.readData();
  }

  if ( RcvData & RC100_BTN_1 ) {
    mode = snake;
    SerialUSB.print("buttonState = RC100_BTN_1\r\n");
    //Dxl.ledOn(BROADCAST_ID, ledblue);
  } 
  else if ( RcvData & RC100_BTN_2 ) {
    mode = side;
    SerialUSB.print("buttonState = RC100_BTN_2\r\n");
    //Dxl.ledOn(BROADCAST_ID, ledcyan);
  } 
  else if ( RcvData & RC100_BTN_3 ) {
    mode = helix;
    SerialUSB.print("buttonState = RC100_BTN_3\r\n");
    //Dxl.ledOn(BROADCAST_ID, ledgreen);
  }
  else if ( RcvData & RC100_BTN_4 ) {
    //Dxl.wheelMode(BROADCAST_ID);
    mode = other;
    SerialUSB.print("buttonState = RC100_BTN_4\r\n");
    //Dxl.ledOn(BROADCAST_ID, ledyellow);
  }
  //delay(100);
  unsigned char color;
  if (mode == snake ) color = ledblue;
  if (mode == side ) color = ledcyan;
  if (mode == helix ) color = ledgreen;
  if (mode == other ) color = ledwhite;

  if ( RcvData & RC100_BTN_U ) {
    SerialUSB.print("buttonState = RC100_BTN_U\r\n");
    up += 1;
  } 
  else if ( RcvData & RC100_BTN_D ) { 
    SerialUSB.print("buttonState = RC100_BTN_D\r\n");
    up -= 1;
  } 
  else if ( RcvData & RC100_BTN_R ) {
    SerialUSB.print("buttonState = RC100_BTN_R\r\n");
    right += 1;
  } 
  else if ( RcvData & RC100_BTN_L ) {
    SerialUSB.print("buttonState = RC100_BTN_L\r\n");
    right -= 1;
  }

  //モード初期化
  if ( premode != mode ){
    premode = mode; 
    up = 0;
    right = 0;
    SerialUSB.print("\r\n\r\nSnakeMode = ");
    SerialUSB.println(mode);
    SerialUSB.print("\r\n\r\n");
    //Dxl.writeWord( BROADCAST_ID, P_GOAL_SPEED, 512);// 
    //Dxl.writeWord( BROADCAST_ID, P_GOAL_POSITION, 512);// 初期設定　軸の位置　０度
    //initsnake();
    //delay(500);
    for ( int i = 0; i< nou; i++){
      targety[i] = 0;
      targetp[i] = 0;
    }

    if ( mode == snake ) {
      //SerialUSB.print("\r\nSnakeMode == snake\r\n\r\n");
      snakemode();
    } 
    else if ( mode == side ) {
      //SerialUSB.print("\r\nSnakeMode == side\r\n\r\n");
      sidemode();
    } 
    else if ( mode == helix ) {
      //SerialUSB.print("\r\nSnakeMode == helix\r\n\r\n");
      helixmode();
    }
    else if ( mode == other ) {
      //SerialUSB.print("\r\nSnakeMode == other\r\n\r\n");
      othermode();
    }

    settargetang();
    
    //だんだんうごかす      
    for ( int ui = 0 ; ui < JOINT_NUM ; ui++ ) {
      Dxl.goalPosition( CommandParameters[2*ui], CommandParameters[2*ui +1]);
      Dxl.ledOn( CommandParameters[2*ui], color);

      // delay(3);
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
  for ( int i = 0; i< nou; i++){
    targety[i] =  A * sin ( w * t + phi * i );
    targetp[i] = 0;
  }

}


void sidemode () {

  double A = 90; 
  double w = 0.1;
  double phi = 1.8;
  //t = t++;
  t = up;
  for ( int i = 0; i< nou; i++){
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

  for ( int i = 0; i< nou; i++){
    targety[i] = 2.0*kap*ds*cos( (double)i  * 2.0 * ds * tau + w * (double)t )*360.0/(double)(2.0*M_PI);
    targetp[i] = -2.0*kap*ds*sin( ((double)i * 2.0 ) * ds * tau + w * (double)t )*360.0/(double)(2.0*M_PI);
  }

}

void othermode() {

  for ( int i = 0; i< nou; i++){
    targety[i] = 0;
    targetp[i] = 0;
  }
  //Dxl.writeWord( BROADCAST_ID, P_GOAL_SPEED, 512);// 
  //Dxl.writeWord( BROADCAST_ID, P_GOAL_POSITION, 512);// 初期設定　軸の位置　０度

}


void settargetang () {

  //目標角度設定　0-1023
  for ( int ui = 0 ; ui < nou ; ui++ ) {
    //CommandParameters[2*ui -1] = 512 + targety[ui-1]/150.0*511; //ヨー軸
    //CommandParameters[14+ 2*ui -1] = 512 + targetp[ui-1]/150.0*511; //ピッチ軸
    
    CommandParameters[2*2*ui + 1] = 512 + targety[ui]/150.0*511; //ヨー軸
    CommandParameters[2*2*ui + 3] = 512 + targetp[ui]/150.0*511; //ピッチ軸
    /*SerialUSB.print("\r\n\r\nCommandParameters[");
    SerialUSB.print(2*2*ui + 1);
    SerialUSB.print("] = ");
    SerialUSB.print(CommandParameters[2*2*ui + 1]);
     
    SerialUSB.print("     CommandParameters[");
    SerialUSB.print(2*2*ui + 3);
    SerialUSB.print("] = ");
    SerialUSB.println(CommandParameters[2*2*ui + 3]);
    SerialUSB.print("\r\n\r\n");*/
  }

}

void initsnake() {

  unsigned char color;

  if (mode == snake ) color = ledblue;
  if (mode == side ) color = ledcyan;
  if (mode == helix ) color = ledgreen;
  if (mode == other ) color = ledwhite;
//    for ( int ui = 0 ; ui < nou ; ui++ ) {
//    CommandParameters[2*2*ui + 1] = 512; //ヨー軸
//    CommandParameters[2*2*ui + 3] = 512; //ピッチ軸
//  }
//       if ( Controller.available() ) {
//    Dxl.syncWrite( P_GOAL_POSITION, 1, CommandParameters,100);
//    } 
//     if ( Controller.available() ) {
//    Dxl.syncWrite( P_GOAL_POSITION, 1, &CommandParameters[100],100);
//    } 
    
  for ( int ui = 0 ; ui < nou*2 ; ui++ ) {
    Dxl.goalPosition(CommandParameters[2*ui], 512);
   // Dxl.goalPosition(CommandParameters[2*ui]+100,  512);

    Dxl.ledOn(CommandParameters[2*ui], color);
    //Dxl.ledOn(CommandParameters[2*ui]+100, color);
//          SerialUSB.print("\r\n\r\nCommandParameters[2*ui] = ");
//      SerialUSB.println(CommandParameters[2*ui]);
//      SerialUSB.print("\r\n\r\n");
    //Dxl.ledOn(ui+100, color);
    //      //delay(dtms/nou);
    delay(30);
  }
  Dxl.ledOn(BROADCAST_ID, color);
}