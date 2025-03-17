//Adquire sinal para calcular CEA
void runAcqCea() {
  uint8_t n_actual = 0;  ////acquisicao atual

  //enquanto nao atinge N_ACQ
  while (n_actual < N_ACQ) {

    //se teve estouro de tempo
    if (millis() - t_accq > dt_sampling) {

      //resete timer
      t_accq = millis();

      //para cada um dos tres canais
      for (uint8_t j = 0; j < 3; j++) {  //chanel 0 (current)  1 (E2) 2 (E3)
        signal_cea[n_actual][j] = COR_A[j] * (ads.getSingleEndedMillivolts(j) / 1000.0) + COR_B[j];
      }

      //Dados diferencial eletrodos calculado
      signal_cea[n_actual][3] = signal_cea[n_actual][1] - signal_cea[n_actual][2];

      //Proximo ponto
      n_actual++;
    }
  }
}


//Calcula CEA e simultaneamente analisa sinal

void CalcCEA() {

  soil_contact = 'S';                          //Inicialmente tem contato
  float mean_cea[4] = { 0.0, 0.0, 0.0, 0.0 };  //guarda media do absoluto dos 4 sinais
  //para gerar linha de anlise
  dataS = id + "," + String(freq_signal, 1) + ",";  //para arquivo

  //para cada sinal  I, EP1,EP2, DP
  for (uint8_t j = 0; j < 4; j++) {

    Serial.print(j);
    Serial.print("\t");
    uint8_t nl = 0, nh = 0;    //numero de pontos em H e L
    float vl = 0.0, vh = 0.0;  //tensao média em H e L

    float rn_lh = 0.0;  //relacao numero pontos L e H
    float rv_lh = 0.0;  //relacao tensao em L e H

    //inicialmente,  separa o sinal em positivo e negativo para calcular a mediana
    std::vector<float> signal_h;
    std::vector<float> signal_l;

    for (uint8_t k = 0; k < N_ACQ; k++) {
      if (signal_cea[k][j] > 0) {
        signal_h.push_back(signal_cea[k][j]);
      } else {
        signal_l.push_back(signal_cea[k][j]);
      }
    }
    //calcula mediana em L e H
    float vh_median = calculateMedian(signal_h);
    float vl_median = calculateMedian(signal_l);
    Serial.print("Mediana :");
    Serial.print(vh_median);
    Serial.print("\t");
    Serial.print(vl_median);

    //varre sinal original e filtra se valor menor que 1.5 *mediana
    //calcula media do absoluto do sinal sem os outlier
    //calcula media e quantidade H e L sem os outiler
    for (uint8_t k = 0; k < N_ACQ; k++) {
      if (signal_cea[k][j] > 0 and signal_cea[k][j] < 1.25 * vh_median and signal_cea[k][j] > 0.75 * vh_median ) {          //se positivo e não diverger da mediana
        mean_cea[j] = mean_cea[j] + abs(signal_cea[k][j]);                        //para media
        nh = nh + 1;                                                              //conta NH
        vh = vh + signal_cea[k][j];                                               //soma para média VH
      } else if (signal_cea[k][j] <= 0 and signal_cea[k][j] > 1.25 * vl_median and signal_cea[k][j] < 0.75 * vl_median) {  //se negativo e não diverger da mediana
        mean_cea[j] = mean_cea[j] + abs(signal_cea[k][j]);                        //para media
        nl = nl + 1;                                                              //conta NL
        vl = vl + signal_cea[k][j];                                               //soma para média VH
      }
    }

    //calcula média dos sinais filtrados
    mean_cea[j] = mean_cea[j] / float(nh + nl);
    Serial.print(mean_cea[j]);
    Serial.print("\t");

    //relacao rnlh --numero de pontos
    if (nh > 0) rn_lh = float(nl) / float(nh);
    else rn_lh = 99.9;  //para evitar inf
    Serial.print(rn_lh, 2);
    Serial.print("\t");

    //media vh
    if (nh > 0) vh = vh / float(nh);
    else vh = 99.9;  //para evitar inf

    //media vl
    if (nl > 0) vl = vl / float(nl);
    else vl = -99.9;  //para evitar inf

    //relacao rnlh --numero de valores  de H e L
    rv_lh = vl / vh;
    Serial.print(rv_lh, 2);
    Serial.println("\t");

    //indicadores de mal contato
    if (rn_lh < 0.5 or rn_lh > 1.5) soil_contact = 'N';     //0.5 < RN < 1.5
    if (rv_lh < -2.0 or rv_lh > -0.10) soil_contact = 'N';  //-0.25 < RV < -1.75

    //MEAN EP1 ou MEAN EP2 < 8.0 (caso especifico para solo com baixa CEa sem contato no GND)
    if ((j == 1 or j == 2) and mean_cea[j] > 8.0) soil_contact = 'N';

    //gera linha para analise posterior
    dataS = dataS + String(nh) + "," + String(nl) + "," + String(rn_lh, 2) + ",";
    dataS = dataS + String(vh, 2) + "," + String(vl, 2) + "," + String(rv_lh, 2) + ",";
    dataS = dataS + String(mean_cea[j], 3) + ",";
  }
  //CEA= i(mA)/(2 * PI * a (m) * DP (V))
  cea = (1000.0 * mean_cea[0] / float(RES_I)) / (2.0 * 3.14159 * 0.3 * mean_cea[3]);
  //limitação do número de caracter enviado para o App
  if (cea > 100) cea = 99.9;

  //salva analise no arquivo
  dataS = dataS + String(cea, 1) + "," + soil_contact + "\n";
  Serial.print(dataS);
  appendFile(SD, file_analise, dataS.c_str());
}

// Function to calculate the median of an array, ignoring zero values
// Function to calculate the median of a vector
float calculateMedian(std::vector<float> &data) {
  // Ensure the vector is not empty
  if (data.empty()) {
    return 0.0;  // Return 0 for an empty vector (you can choose a different value)
  }

  // Sort the vector in ascending order
  std::sort(data.begin(), data.end());

  // Calculate the median
  size_t size = data.size();
  if (size % 2 == 0) {
    // If the size is even, return the average of the middle two elements
    return (data[size / 2 - 1] + data[size / 2]) / 2.0;
  } else {
    // If the size is odd, return the middle element
    return data[size / 2];
  }
}

//mede e tensão de uma bateria responsavel pela fonte positiva -- > /Executado no Nucleo 1
float voltage_bat() {

  float value = 0;
  uint8_t R1 = 99;  //kohm
  uint8_t R2 = 33;  //kohm

  //usa media de 10 valores
  for (int i = 0; i < 10; i++) {
    value = value + 3.3 * analogRead(PIN_BAT) / 4096.0;
    delay(10);
  }
  value = value / 10.0;
  return (value * float(R1 + R2)) / (float(R2)) / 3.0;  //calcula tensão na entrada no divisor de tensão e divide por 3 ==> tensão de uma bateria
}

//obtem tensao gerada pelo PT100
float GetTemp() {

  float temp = 0;
  int n = 20;

  //media de 20 valores a cada 10 ms
  for (int i = 0; i < n; i++) {
    temp = temp + 3.3 * analogRead(pin_temp) / 4096.0;
    delay(10);
  }
  temp = temp / float(n);

  //Serial.print(F("Voltage Temp: ")); Serial.println(temp, 3);
  return COEF_TMPT[0] * temp + COEF_TMPT[1];
}

//Salva sinal
void appendFile(fs::FS &fs, const char *path, const char *message) {
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    save_sd = false;
    return;  //se não abri arquivo , retorna
  } else save_sd = true;
  file.print(message);
  file.close();
}

//Final do Arquivo
