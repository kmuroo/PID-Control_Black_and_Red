//不明な点があったら芦澤まで


float Target_Black = -18.0; //目標温度
float Target_Red = -18.0; //目標温度
const int Senser_Port_Black = 0;
const int Senser_Port_Red = 1;
const int Output_Port_Black = 9; //PWM Port
const int Output_Port_Red = 10;
bool echo_on = false;

//PID制御パラメータ
float Kp_Black = 300.0; //比例ゲイン
float Ki_Black = 11.0; //積分ゲイン
float Kd_Black = 100.0; //微分ゲイン
//Black channel
//-30℃で1200/50/0
//-18℃で1500/75/10
float Kp_Red = 400.0; //比例ゲイン
float Ki_Red = 15.0; //積分ゲイン
float Kd_Red = 33.0; //微分ゲイン
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
long duty_Black, duty_Red;
float dt;
float pretime = 0;//過去の時間

void(*resetFunc)(void) = 0;

void setup() {
  echo_on = false;
//  Serial.begin(9600);   //シリアル通信を9600 bpsで開始
  Serial.begin(115200);   //シリアル通信を115200 bpsで開始
}

void loop() {
  int count = 0;
  unsigned long curr = millis();
  unsigned long prev = millis();
  double tempsum_Black = 0;
  double tempsum_Red = 0;

  while ((curr - prev) <= 1000) {

    //サーミスタの出力を測定
    x_Black = x_Red = 0;
    for (int i = 0; i < 80; i++) {
      x_Black += analogRead(Senser_Port_Black); //アナログピン0でサーミスタのV_outを読み取る
      x_Red += analogRead(Senser_Port_Red);
    }
    //電圧[V]に変換
    x_Black = x_Black * 5.0 / 80.0 / 1023.0;
    x_Red = x_Red * 5.0 / 80.0 / 1023.0;
    //温度に変換
    double number_Black = x_Black / (5.0 - x_Black);
    double temp_Black = 1.0 / (1.0 / 298.0 + (1 / 3250.0) * log(number_Black)) - 273.0;
    double number_Red = x_Red / (5.0 - x_Red);
    double temp_Red = 1.0 / (1.0 / 298.0 + (1 / 3250.0) * log(number_Red)) - 273.0;

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
    
    I_Black += P_Black * dt;
    I_Red += P_Red * dt;

    D_Black = (P_Black - preP_Black) / dt;

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
    Serial.print(tempsum_Black / count);
    Serial.print("\t");
    Serial.println(tempsum_Red / count);
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
        //myString = Serial.readString();
        Serial.println("c: Rreturn \"PID_Controller\"");
        Serial.println("e: Enable report");
        Serial.println("s: Stop report");
        Serial.println("r: Reset and return \"Reset\"");
        Serial.println("l: Show current PID parameters");
        Serial.println("p: Put PID parameters; given by 7 numbers separated \",\" and return \"P\"");
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
        break;
      case 's':
        myString = Serial.readString();
        echo_on = false;
        Serial.flush();
        break;
      case 'r':
        myString = Serial.readString();
        Serial.println("Reset"); // Arduinoリセット "Reset"を返す
        Serial.flush();
        resetFunc();
        break;
      case 'l':
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
        //Serial.println(myString);
        String cmds[8] = {"\0"}; // 分割された文字列を格納する配列 
        int index = split(myString, ',', cmds);
        Target_Black = cmds[0].toInt();// Black: 目的温度を返す
        Kp_Black = cmds[1].toInt(); // Black: 制御パラメーターを返す
        Ki_Black = cmds[2].toInt();
        Kd_Black = cmds[3].toInt();
        Target_Red = cmds[4].toInt();// Red: 目的温度を返す
        Kp_Red = cmds[5].toInt(); // Red: 制御パラメーターを返す
        Ki_Red = cmds[6].toInt();
        Kd_Red = cmds[7].toInt();
        Serial.println("P");
        Serial.flush();
        break;
      default:
        myString = Serial.readString();
        //Serial.println("Interapted!");
    }
  }
}

int split(String data, char delimiter, String *dst){ // 文字列を区切り文字で分割
    int index = 0; 
    int datalength = data.length();
    
    for (int i = 0; i < datalength; i++) {
        char tmp = data.charAt(i);
        if ( tmp == delimiter ) {
            index++;
        }
        else dst[index] += tmp;
    }
    
    return (index + 1);
}
