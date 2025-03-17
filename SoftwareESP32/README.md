# SoilSmartSensor

# SoftwareESP32

Esse diretorio contém ao software desenvolvido para a placa de aquisição de dados do Soil Smart Sensor. 

A placa de aquisição do Soil Smart Sensor utiliza como base a placa de desenvolvimento ESP32 S3.

Documentação da placa em https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/index.html



O software foi desenvolvido no Arduino IDE 2.3.2 com firmware para a familia de placas ESP32 versão 3.0.5

Detalhamento dos arquivos e diretórios

/SmartSensor_ESP32  --> contém os arquivos do software desenvolvido para ESP32 S3
 SmartSensor_ESP32.ino  --> arquivo principal com o software
 functions.ino            --> arquivo auxiliar com funções utilizadas.

/NimBLE-Arduino    --> contém a biblioteca para uso do Bluetooth Low Energy

/Sparkfun_ADS1015_Arduino_Library --> contém a biblioteca para uso do módulo ADC ADS1015
