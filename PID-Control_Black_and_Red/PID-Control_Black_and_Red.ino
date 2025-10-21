// Ver. 2.4 2025/10/21
//不明な点があったら芦澤まで

float Target_Black = -18.0; //目標温度
float Target_Red = -18.0; //目標温度
const int Senser_Port_Black = 0;
const int Senser_Port_Red = 1;
const int Output_Port_Black = 9; //PWM Port
const int Output_Port_Red = 10;
bool echo_on = false;

//PID制御パラメータ
//----- 芦澤、西 における初期設定値 --------------------
//float Kp_Black = 300.0; //比例ゲイン
//float Ki_Black = 11.0; //積分ゲイン
//float Kd_Black = 100.0; //微分ゲイン
//----- 芦澤、西 における初期設定値 END --------------------
//----- ジーグラ・ニコルス限界感度法における初期設定値  2025/7/23 変更--------------------
//----- Ti, TdとKi, Kd を混同していたのを修正 2025/1021 変更---------------------------
float Ku_Black = 95.0;
float Pu_Black = 4.35; 
float Kp_Black = 0.60*Ku_Black;     //比例ゲイン 57.0
float Ti_Black = 0.50*Pu_Black;     //2.175
float Td_Black = 0.125*Pu_Black;    //0.54375
float Ki_Black = Kp_Black/Ti_Black; //積分ゲイン 26.3 
float Kd_Black = Kp_Black*Td_Black; //微分ゲイン 30.75
//Black channel

//-30℃で1200/50/0
//-18℃で1500/75/10
//----- 芦澤、西 における初期設定値 --------------------
//float Kp_Red = 400.0; //比例ゲイン
//float Ki_Red = 15.0; //積分ゲイン
//float Kd_Red = 33.0; //微分ゲイン
//----- 芦澤、西 における初期設定値 END --------------------
//----- ジーグラ・ニコルス限界感度法における初期設定値  2025/7/23 変更--------------------
//----- Ti, TdとKi, Kd を混同していたのを修正 2025/1021 変更---------------------------
float Ku_Red = 120.0;
float Pu_Red = 1.11; 
float Kp_Red = 0.60*Ku_Red;     //比例ゲイン 72
float Ti_Red = 0.50*Pu_Red;     //0.555
float Td_Red = 0.125*Pu_Red;    //0.139
float Ki_Red = Kp_Red/Ti_Red; //積分ゲイン 129.7 
float Kd_Red = Kp_Red*Td_Red; //微分ゲイン 48.0
//Red channel

float P_Black = 0;
float P_Red = 0;
float I_Black = 0;
float I_Red = 0;
float D_Black =0;
float D_Red =0;
float preP_Black = 0;
float preP_Red = 0;

float x_Black = 0.0;
float x_Red = 0.0;
long  duty_Black, duty_Red;
float dt;
float pretime = 0;//過去の時間

int count = 0;
unsigned long curr;
unsigned long prev;
double tempsum_Black = 0;
double tempsum_Red = 0;
double number_Black;
double temp_Black;
double number_Red;
double temp_Red;
bool verbose = false;
String params[8];
int index;

int i;

void(*resetFunc)(void) = 0;

void setup() {
  echo_on = false;
//  Serial.begin(9600);   //シリアル通信を9600 bpsで開始
  Serial.begin(115200);   //シリアル通信を115200 bpsで開始
}

void loop() {
  curr = prev = millis();
  tempsum_Black = 0;
  tempsum_Red = 0;
  
  while ((curr - prev) <= 1000) {
    //サーミスタの出力を測定
    x_Black = x_Red = 0;
    for (i = 0; i < 80; i++) {
      x_Black += analogRead(Senser_Port_Black); //アナログピン0でサーミスタのV_outを読み取る
      x_Red += analogRead(Senser_Port_Red);
    }
    //電圧[V]に変換
    x_Black = x_Black * 5.0 / 80.0 / 1023.0;
    x_Red = x_Red * 5.0 / 80.0 / 1023.0;
    //温度に変換
    number_Black = x_Black / (5.0 - x_Black);
    temp_Black = 1.0 / (1.0 / 298.0 + (1 / 3250.0) * log(number_Black)) - 273.0;
    number_Red = x_Red / (5.0 - x_Red);
    temp_Red = 1.0 / (1.0 / 298.0 + (1 / 3250.0) * log(number_Red)) - 273.0;

    count++;
    tempsum_Black += temp_Black; //温度の合計
    tempsum_Red += temp_Red;

    //PID制御
    P_Black = -(Target_Black - temp_Black);
    P_Red = -(Target_Red - temp_Red);

    dt = (micros() - pretime) / 1000000.0; //時間の変化量をs単位表示
    if (dt < 0) {
      dt = (micros() - (4294967295 - pretime)) / 1000000.0;
    }
    pretime = micros();//過去時間の更新
    
    I_Black += P_Black * dt; if(I_Black > 255.0/Ki_Black)I_Black = 255.0/Ki_Black; // I_Black, I_Red に上限設定 200.0 250805
    I_Red += P_Red * dt; if(I_Red > 255.0/Ki_Red)I_Red = 255.0/Ki_Red;             // 上限値を変更 255.0/Ki_Black,Red 

    D_Black = (P_Black - preP_Black) / dt;
    D_Red = (P_Red - preP_Red) / dt; // この行がなかった（バグ）2025/7/23 修正 by Muroo

    preP_Black = P_Black; //過去Pの更新
    preP_Red = P_Red;

    //ゲート電圧の出力値
    duty_Black = Kp_Black * P_Black + Ki_Black * I_Black + Kd_Black * D_Black;
    duty_Red = Kp_Red * P_Red + Ki_Red * I_Red + Kd_Red * D_Red;
    
    if (duty_Black > 255.0) {
      duty_Black = 255;
    }
    if (duty_Red > 255.0) {
      duty_Red = 255;
    }
    
    if (duty_Black < 0.0 || temp_Black > 30.0) {
      duty_Black = 0;
    }
    if (duty_Red < 0.0 || temp_Red > 30.0) {
      duty_Red = 0;
    }
    //Serial.println(duty);
    analogWrite(Output_Port_Black, duty_Black);
    analogWrite(Output_Port_Red, duty_Red);
    curr = millis();
  }

//  Serial.println(tempsum / count);//温度を出力
  if(echo_on == true){
    //Serial.print(count);
    //Serial.print("\t");
    Serial.print(tempsum_Black / count);
    Serial.print("\t");
    Serial.println(tempsum_Red / count);
    if(verbose == true)
    {
      Serial.print(Target_Black);
      Serial.print("\t");
      Serial.print(temp_Black);
      Serial.print("\t");
      Serial.print(P_Black);
      Serial.print("\t");
      Serial.print(I_Black);
      Serial.print("\t");
      Serial.print(D_Black);
      Serial.print("\t");
      Serial.println(duty_Black);

      Serial.print(Target_Red);
      Serial.print("\t");
      Serial.print(temp_Red);
      Serial.print("\t");
      Serial.print(P_Red);
      Serial.print("\t");
      Serial.print(I_Red);
      Serial.print("\t");
      Serial.print(D_Red);
      Serial.print("\t");
      Serial.println(duty_Red);
    }
  }
  count = 0;
}

//シリアル割り込み処理
void serialEvent(){
  char ch;
  String myString;
  if(Serial.available()>0) {
    ch=Serial.read();
    switch(ch){
      case 'h':
        myString = Serial.readString();
        Serial.println("c: Rreturn \"PID_Controller\"");
        Serial.println("e: Enable report");
        Serial.println("s: Stop report");
        Serial.println("r: Reset and return \"Reset\"");
        Serial.println("l: Show current PID parameters");
        Serial.println("p: Set PID parameters; given by 8 numbers separated by \",\", and return \"P\"");
        Serial.println("v: Set verbose mode\",\" and return \"V\"");
        Serial.println("t: Set terse mode\",\" and return \"T\"");
        Serial.println("i: Force to clear integrals and return \"I\"");
        Serial.println("h: Show this message");
        Serial.flush();
        break;
      case 'c':
        myString = Serial.readString();
        Serial.println("PID_Controller");
        Serial.flush();
        break;
      case 'e':
        myString = Serial.readString();
        echo_on = true;
        Serial.flush();
        //I_Black = I_Red = 0.0; // I をクリア
        break;
      case 's':
        myString = Serial.readString();
        echo_on = false;
        Serial.flush();
        break;
      case 'r':
        myString = Serial.readString();
        Serial.println("Reset"); // Arduinoリセット, "Reset"を返す
        Serial.flush();
        resetFunc();
        break;
      case 'l':
        myString = Serial.readString();
        echo_on = false;
        Serial.print(Target_Black);// Black: 目的温度を返す
        Serial.print(",");
        Serial.print(Kp_Black); // Black: 制御パラメーターを返す
        Serial.print(",");
        Serial.print(Ki_Black);
        Serial.print(",");
        Serial.print(Kd_Black);
        Serial.print(",");
        Serial.print(Target_Red);// Red: 目的温度を返す
        Serial.print(",");
        Serial.print(Kp_Red); // Red: 制御パラメーターを返す
        Serial.print(",");
        Serial.print(Ki_Red);
        Serial.print(",");
        Serial.println(Kd_Red);
        Serial.flush();
        break;
      case 'p':
        myString = Serial.readString();
        index = split(myString, ',', params,8);
        Target_Black = params[0].toFloat();params[0]="";// Black: 目的温度を返す
        Kp_Black = params[1].toFloat();params[1]=""; // Black: 制御パラメーターを返す
        Ki_Black = params[2].toFloat();params[2]="";
        Kd_Black = params[3].toFloat();params[3]="";
        Target_Red = params[4].toFloat();params[4]="";// Red: 目的温度を返す
        Kp_Red = params[5].toFloat();params[5]=""; // Red: 制御パラメーターを返す
        Ki_Red = params[6].toFloat();params[6]="";
        Kd_Red = params[7].toFloat();params[7]="";
        Serial.println("P");
        Serial.flush();
        myString = Serial.readString();
        break;
      case 'v': // Set verbose mode
        myString = Serial.readString();
        verbose = true;
        Serial.println("V");
        Serial.flush();
        break;
      case 't': //Set terse mode
        myString = Serial.readString();
        verbose = false;
        Serial.println("T");
        Serial.flush();
        break; 
      case 'i': //Reset integlals
        myString = Serial.readString();
        I_Black = I_Red = 0.0;
        Serial.println("I");
        Serial.flush();
        break;
      default:
        myString = Serial.readString();
        Serial.println("Invalid order");
    }
  }
}

int split(String data, char delimiter, String *dst, int number_of_params){ // 文字列を区切り文字で分割
    index = 0;
    int datalength = data.length();
    for (int i = 0; i < datalength; i++) {
        char tmp = data.charAt(i);
        if ( tmp == delimiter )
        {
            index++;
            if(index > number_of_params-1)
            {
              Serial.println("Too many parameters");
              return(index);
            }
        }
        else
        {
          dst[index] += tmp;
        }   
    }
    return (index + 1);
}
