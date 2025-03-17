/*
* Codigo para ESP32 S3 
  Firmware 3.0.5
 *  Na primeira vez que for usar esse programa no seu computador, coloque  os diretorios SparkFun_ADS1015_Arduino_Library e NimbleBLE-Arduino 
 *  no diretorio de bibliotecas do Arduino ID (Documentos/Arduino/Libraries). 
 *  O procedimento é copiar e colocar os diretorios.
 */

/*
  PWM gerado pela ESP32
  ADS 1015 com biblioteca . Ganho 1 . Alimentacao em 3.3 V . 3300 SPS. Biblioteca SparkFun_ADS1015_Arduino_Library
  Leitura de sinal unipolar de corrente, dp eletrodo 1 dp eletrodo 2 --> calculo de DP entre Eletrodo 1 e 2 via codigo
  1 LM741 gera sinal bipolar simetrico
  2 TL072 gera sinal diferencial de corrente e inviduais dos eletrodo
  1,5 MCP6002 transforma sinal bipolar em unipolar (I,EP1 e Ep2). Unico par de resistor R3 e R4 para os dois sinais -- 
  Sinal Corrente e Eletrodo 2 e 3 no Canal 0 , 1  e 2 no ADS1015
  ESP32 lê sinal e converte usando equação obtida experimentalmente (fatores obtidos no Excel)
  Sensor de Umidade de Solo com 555 (sem ligar Pino 1 no GND com capacitor 1 uF) ==Canal 3 ADS1015
  Sensor PT100 em pino analogico da ESP32 (D14)
  Divisor de tensão para ler tensão de bateria (D3)
  Bluetooth BLE Bliblioteca NimBLE -> NIMBLE
  Recebe comando para medir CEa e Umida no formato C10@1 , em que 10 é frequencia e 1 é ID
  Envia dados da bateria no formato BXX , onde XX é a tensão, a cada 10s
  Envia dados de CEa e Umidade formato C10,20
  Envia dados de Temperatura  no formato T10 , a cada 10 s
  Analise de sinal para detecção de contato eletrodos - 0 A 100 %
  Aplica filtro de outlier nos dados dos canais remove maiores de 1.5 *mediana
  Calcula valor medio do absoluto do sinal para calcular CEA
  Salva sinais e analise em cartao de memoria SD

  Reinicia a cada 50 leituras
  Filtro da mediana nos sinais
*/

//Sensor
String name = "SSS";
float RES_I = 180.0;
float COR_A[] = { -6.269, -6.0382, -6.0917 };
float COR_B[] = { 9.941, 9.538, 9.61 };
float COEF_UM[2] = { -26.51895, 3.93791 };
float factor_umi = 1.0;
float COEF_TMPT[2] = { 176.366, -260.29578 };


//filtragem do sinal DP com base na mediana H e L
#include <algorithm>
#include <vector>

// watchdog
#include "esp_task_wdt.h"

//salvar sinal para analise
#include "FS.h"
#include "SD.h"
#include "SPI.h"
char *file_sinal = "/signal.txt";     //sinal
char *file_analise = "/analise.txt";  //analise do sinal
String dataS;                         //para linha para arquivo
String id = "";                       //id do ponto enviado pelo App
uint8_t id_test = 0;                  //id para modo continuo
bool save_sd = false;                 //alerta de funcionamento do cartao
//Bluetooth
#include "NimBLEDevice.h"
NimBLECharacteristic *pCharacteristic;
bool deviceConnected = false;
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"         // UART service UUID
#define CHARACTERISTIC_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  //Precisa ser compativel com o App
String msg = "";                                                    //mensagem que será recebida e processada

//PWM para gerar sinal a ser emitido no solo
#define PIN_PWM 9  //
uint8_t chn = 0;   //canal 0
uint8_t res = 14;  //resolucao 14 bit ===0 a 2^14= 16384 //ESP32 S3 não funciona com 15 ou 16 bit. Para pequenas frequencias a resolução precisa ser grande

//ADS1015 para ler sinais com precisao
#include <SparkFun_ADS1015_Arduino_Library.h>
#include <Wire.h>
ADS1015 ads;  //ADS1015
#define I2C_SDA 6
#define I2C_SCL 7


//parametro para aquisição de dados
float freq_signal = 0;       //frequencia do sinal emitido no solo --> do App
float dt_sampling = 0.0;     //taxa de amostragem
uint8_t time_acq_rate = 10;  //10x a frequencia do sinal
#define N_ACQ 40             //40 AQUISICOES   (4 ondas completas)
unsigned long t_accq = 0;    //para fazer tempo de aquisição sem delay
bool acquisition = false;    //está adquirindo CEa e Umi?

//calculos de CEA : Canal 0 , 1  e 2 do ADS
float signal_cea[N_ACQ][4];  //row , column (4) - corrente,pot E2,potE1 , pot dif
float cea = 0.0;             //guardar CEa
char soil_contact = 'N';     //contato com solo

//Mede tensao da bateria
#define PIN_BAT 3             //
float vb = 0.0;               //tensao bateria
unsigned long last_time = 0;  //para fazer leitura enquanto naão mede CEa

//Umidade do solo
//Canal 3 do ADS1015
float v_umi = 0.0;          //tensao umidade
float umi = 0.0;            //umidade
unsigned long last_t_wait;  //para medir tempo de leitura

//leds
#define led1 39
bool state = LOW;

// Pt100  pino analogico da ESP32 S3
#define pin_temp 19  //
float temp = 0.0;    //temperatura do solo

//Modulo Rele CEa e Umi
#define pin_r_cea 18
#define pin_r_umi 17

//Tempo de processo para forçar um RESET
unsigned long time_start;
uint8_t contador = 0;  //contador de medições


//declara funcoes
void ProcessMsg(String msg);
void runAcqCea();
void CalcCEA();
float GetTemp();
float voltage_bat();
void appendFile(fs::FS &fs, const char *path, const char *message);
float calculateMedian(std::vector<float> &data);

//Conectado/Desconectado
class MyServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *pServer) {
    deviceConnected = true;
  };
  void onDisconnect(NimBLEServer *pServer) {
    deviceConnected = false;
  }
};

//Receive Information
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();
    ProcessMsg(rxValue.c_str());
  }
};

//SETUP//
void setup() {

  //Define frequencia do CPU em 240 MHz
  setCpuFrequencyMhz(240);

  //start serial
  Serial.begin(115200);
  delay(1000);

  //Iniciar cartao sd
  if (!SD.begin(SS)) {
    Serial.println(F("Cartao Nao Iniciado"));
    save_sd = false;
  } else save_sd = true;

  //Modulo Rele
  pinMode(pin_r_cea, OUTPUT);     //rele como saida
  digitalWrite(pin_r_cea, HIGH);  //desliga rele
  pinMode(pin_r_umi, OUTPUT);     //rele como saida
  digitalWrite(pin_r_umi, HIGH);  //desliga rele

  //Pinos de entrada analogica
  pinMode(PIN_BAT, INPUT);
  pinMode(pin_temp, INPUT);

  //LED
  pinMode(led1, OUTPUT);
  digitalWrite(led1, LOW);

  //configura pin pwm
  pinMode(PIN_PWM, OUTPUT);
  digitalWrite(PIN_PWM, LOW);


  //ADS1115
  Wire.begin(I2C_SDA, I2C_SCL, 1000000);  //é necessário iniciar o Wire , ESP32 S3 qualquer pino pode ser usado 1 mhz
  //Wire.setClock(3400000);
  while (!ads.begin(0x48)) {
    Serial.println(F("Falha iniciar ADS."));
    delay(1000);
  }
  //Ajusta ganho e frequência de aquisição
  ads.setGain(ADS1015_CONFIG_PGA_1);              //gain:1, input range: ± 4.096V
  ads.setSampleRate(ADS1015_CONFIG_RATE_3300HZ);  //- 0X00C0 : 3300Hz
  ads.setMode(ADS1015_CONFIG_MODE_SINGLE);        //não pode ser continuou mode
  ads.useConversionReady(false);

  // BLE
  NimBLEDevice::init("SSS");


  NimBLEServer *pServer = NimBLEDevice::createServer();            // Configura o dispositivo como Servidor BLE
  pServer->setCallbacks(new MyServerCallbacks());                  //Configura para receber calls
  NimBLEService *pService = pServer->createService(SERVICE_UUID);  // Cria o servico UART
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  pCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();  // Inicia o serviço
  pServer->getAdvertising()->start();

  //Cria cabeçalho do arquivo , se o arquivo nao existe no SD
  if (not SD.exists(file_sinal)) {
    appendFile(SD, file_sinal, "id;rep,freq,vi,ep1,ep2,dp,\r\n");
  }

  //Cria cabeçalho do arquivo , se o arquivo nao existe no SD
  if (not SD.exists(file_analise)) {
    appendFile(SD, file_analise, "id;rep,freq,i_nh,nl,rnlh,vh,vl,rvlm,mean,ep1_nh,nl,rnlh,vh,vl,rvlm,mean,ep2_nh,nl,rnlh,vh,vl,rvlm,mean,dp_nh,nl,rnlh,vh,vl,rvlm,mean,cea,contact\r\n");
  }

  //Habilita o watchdog configurando o timeout para 10 segundos :: permite reiniciar a ESP32 automaticamente em caso de congelamento
  esp_task_wdt_config_t config = {
    .timeout_ms = 10 * 1000,  //  10 seconds
    .trigger_panic = true,    // Trigger panic if watchdog timer is not reset
  };
  esp_task_wdt_reconfigure(&config);
  esp_task_wdt_add(NULL);


  //Bate Rele para Indicar bom Funcionammento
  bool state = HIGH;
  for (uint8_t i = 0; i < 2; i++) {
    state = !state;
    digitalWrite(pin_r_cea, state);  //desliga rele
    delay(1000);
    digitalWrite(pin_r_umi, state);  //desliga rele
    delay(1000);
  }

}  //fim do void SETUP()


//Loop Tradicional
void loop() {

  //Se está conectado  : mede tensao de bateria (esperando mensagem em segundo plano
  if (deviceConnected) {

    //se tiver estouro de tempo (s)
    if (millis() - last_time > 5000) {

      //pisca led
      last_time = millis();
      state = !state;
      digitalWrite(led1, state);

      //obtem dados de bateria e temperatura -- se não estiver adquirindo
      if (!acquisition) {

        //Tensao Bateria
        vb = voltage_bat();  //
        //Envia resposta para o aplicativo
        msg = "B" + String(vb, 1);
        pCharacteristic->setValue(msg);
        pCharacteristic->notify();  // Envia o valor para o aplicativo!
        //Aguarda
        delay(100);

        //Temperatura
        temp = GetTemp();
        //Serial.print(F("Temperatura(ºC) : ")); Serial.println(temp, 3);
        //Envia resposta para o aplicativo
        msg = "T" + String(temp, 1);
        pCharacteristic->setValue(msg);
        pCharacteristic->notify();  // Envia o valor para o aplicativo!
        //Aguarda
        delay(100);

        //status do SD
        if (!save_sd) msg = "SD FAIL";
        else msg = "SD OK";
        pCharacteristic->setValue(msg);
        pCharacteristic->notify();  // Envia o valor para o aplicativo!

        //nome do dispostivo
        msg = "NM" + name;
        pCharacteristic->setValue(msg);
        pCharacteristic->notify();  // Envia o valor para o aplicativo!
      }

      //Induz CEa continuo (freq 10Hz)
      //String command = "C10@" + String(id_test);
      //ProcessMsg(command);
      //id_test = id_test + 1;
    }
  }

  // Reseta o temporizador do watchdog (evita congelamento)
  esp_task_wdt_reset();
}

//Funcao Executada Quando Mensagem é Recebina pelo BLE
void ProcessMsg(String msg) {

  //Se Recebe comando para medir
  if (msg.indexOf("C") > -1) {  //se tem a palavra C   C10@1   10 é frequencia   1 é ID
    contador = contador + 1;
    //Serial.print(contador);
    //Serial.print(",");
    time_start = millis();
    //Define acquisition para não ler Temperatura e Bateria
    acquisition = true;                 //seta como inicio de aquisicao --> nao mede temp e vb
    msg.remove(0, 1);                   //remove o primeiro caracteres C
    uint8_t idf = msg.indexOf("@", 0);  //acha o separadores
    String a = msg.substring(0, idf);   //extrai variavel entre separador
    freq_signal = a.toFloat();          //converte string na frequencia em float
    msg.remove(0, idf + 1);             //remove a frequencia e @
    id = msg;                           //id é o resto de msg --> usado para salvar sinal
    //Serial.print(id);
    //Liga Rele CEA para emitir sinal no solo
    digitalWrite(pin_r_umi, HIGH);  //desliga  rele umidade
    digitalWrite(pin_r_cea, LOW);   //liga rele cea
                                    //

    //inicia PWM
    ledcAttach(PIN_PWM, freq_signal, res);
    ledcWrite(PIN_PWM, 8192);  //pwm 50% duty cicle

    //Configura intervalo de leitura -- máxima frequencia de leitura é 680 Hz
    if (freq_signal <= 62) time_acq_rate = 10;                          //menor que 62 8x freq
    else if (freq_signal > 62 && freq_signal <= 83) time_acq_rate = 6;  //entre 62 e 83 6x freq
    else time_acq_rate = 4;                                             //maior que 83
    dt_sampling = 1000.0 / (float(time_acq_rate) * freq_signal);        // milisegundos;
    delay(1000);                                                        //espera até estabilizar PWM

    //Chama funcao de aquisicao do sinal e salva no vetor float v[NACQ][4]
    runAcqCea();
    //Serial.print(F(", "));
    //Serial.print(millis() - time_start);

    //Função que calcula a CEA e simultaneamente analisa o sinal
    CalcCEA();
    //Serial.print(F(","));
    //Serial.print(millis() - time_start);

    //desliga PWM
    ledcWrite(PIN_PWM, 0);

    //Mede umidade
    digitalWrite(pin_r_umi, LOW);   //liga rele umidade
    digitalWrite(pin_r_cea, HIGH);  //desliga rele cea
    delay(1000);                    //espera solo carregar como capacitor

    //Incia aquisicicao
    v_umi = 0;  //zera valor
      //media de 10 valores a cada 10 ms
    for (uint8_t i = 0; i < 10; i++) {
      v_umi = v_umi + (ads.getSingleEndedMillivolts(3) / 1000.0);  //umidade no canal A3
      delay(10);
    }

    v_umi = v_umi / 10.0;
    //Serial.print(F("Tensao Umidade (V) : "));  Serial.println (v_umi, 2);
    //Equação Calibração
    umi = COEF_UM[0] * log(v_umi / factor_umi) + COEF_UM[1];
    if (umi < 0) umi = 0;
    //Serial.print(F("Umidade (%) : ")); Serial.println (umi, 1);

    //Serial.print(F(", "));
    //Serial.print(millis() - time_start);

    //Desliga Rele não emitir sinal no solo
    digitalWrite(pin_r_umi, HIGH);  //desliga rele
    digitalWrite(pin_r_cea, HIGH);  //desliga rele

    //Envia para aplicativo
    msg = "C" + String(cea, 1) + ", " + String(umi, 1) + ";" + String(v_umi, 2) + ", " + soil_contact;  //
    pCharacteristic->setValue(msg);
    pCharacteristic->notify();  // Envia o valor para o aplicativo!
    //Serial.print("MSG : "); Serial.println(msg);  Serial.println(".......");

    //Retorna a leitura de temperatura e tensao de bateria
    acquisition = false;

    //Depois de calcar CEA e enviar para aplicativo salva o dado no SD
    for (uint8_t i = 1; i < N_ACQ; i++) {  //ignora primeiro valor
      dataS = id + ", " + String(freq_signal) + ", ";
      for (uint8_t j = 0; j < 4; j++) {
        //Serial.print(signal_cea[i][j], 2);
        //Serial.print(F("\t"));
        dataS = dataS + String(signal_cea[i][j], 2) + ", ";
      }
      //Serial.println();
      dataS = dataS + "\n";
      appendFile(SD, file_sinal, dataS.c_str());
    }
    //Serial.print(F(","));
    //Serial.println(millis() - time_start);

    //Indica que ESP está ficando lenta ==> força reinicialização
    //if (millis() - time_start > 10000) ESP.restart();
    if (contador > 30) ESP.restart();
  }
}  //Fim da funcao ProcessaMsg
//Final do Arquivo
