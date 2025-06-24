#include "Communication_Driver.h"
#include <ArduinoJson.h> 
#include <Arduino.h>
#include "Config.h"
#include "RTC_1302.h" 
static const char* aws_root_ca_pem_driver PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";

static const char* aws_certificate_pem_driver PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----
MIIDWTCCAkGgAwIBAgIUZsE3kA6rU2DsT3uO+rZN0b8EiTUwDQYJKoZIhvcNAQEL
BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g
SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTI1MDYxNDEzMzYx
MFoXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0
ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAMNxxGLHblTiVFe84HWD
LdJpxNBEN3ZDZjtXwHstmtH90IJ6ZzzJsHu2hsnczBfyd0xl+ns6z+01d1YiKm/y
n4vvlOXYqGu5PM2hWATz3oIpKFMafSnWVRvHLMVhiJjmoYkLLeb1SOXDeTpPThWK
f9abQ2Bq3R9gEj/ymI4H9QXlwzi1eLOft9hNl97Tverlvp8xPxIDnXVjXEwi7f0D
d0RVmAZgAUEhRfZf/l7T7w85XsfCfOsY31ZL63BlaiX9vr6SyI0qyJ0f6bpU4xFE
5UbHdVOPyD1oZx8bDt3aAwPVDg5g3gcAUBPPDuym5/VQ/SwiYEFbaqTpuLngoqxY
13cCAwEAAaNgMF4wHwYDVR0jBBgwFoAUN2CGaGWYNIJua3VdjQajbZa+nrQwHQYD
VR0OBBYEFMO33GCk+1ZYcJchfL6iy92kMZZZMAwGA1UdEwEB/wQCMAAwDgYDVR0P
AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQCxZTf4KPVPIaKIZt8jl1BuFn+/
AiC3a2bi/2JpkSR4G30kO2cQTksMbe4QZP7qx0gmlbIhHlECOYe95YXYhBQXh8N0
QGtApQWUnArQcOQASbYyzCnsWVWelGkelclTwWLngW1rTVURkLp4l1fO61xYl64h
/vXjfLePRlwBkSNNoeSwUhmxvyuDzNqo/OxFanTxX+QSLt63QVgrXTfnIhHnUTj8
18WZl9BzhEEMcvuDzr7ftAI5Bp2++sYOwgmNcAfFZAoky7Jk44uP0ECh1GLDQs+n
6ElelzYUSmJ045N1+ZqcaGf4kcRnzzmf2LBiBodU4hHhTYpjb198j5ZHFNgk
-----END CERTIFICATE-----
)KEY";

static const char* aws_private_pem_driver PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
MIIEpgIBAAKCAQEAw3HEYsduVOJUV7zgdYMt0mnE0EQ3dkNmO1fAey2a0f3Qgnpn
PMmwe7aGydzMF/J3TGX6ezrP7TV3ViIqb/Kfi++U5dioa7k8zaFYBPPegikoUxp9
KdZVG8csxWGImOahiQst5vVI5cN5Ok9OFYp/1ptDYGrdH2ASP/KYjgf1BeXDOLV4
s5+32E2X3tO96uW+nzE/EgOddWNcTCLt/QN3RFWYBmABQSFF9l/+XtPvDzlex8J8
6xjfVkvrcGVqJf2+vpLIjSrInR/pulTjEUTlRsd1U4/IPWhnHxsO3doDA9UODmDe
BwBQE88O7Kbn9VD9LCJgQVtqpOm4ueCirFjXdwIDAQABAoIBAQCnqkuNagUOsPMm
KZRte1Gt9qqu+DaUMBHdytQHWAI/L3zW0mcKUQQjvVw4hEBaMtmBIuMVIEkL+Bge
PkUlsICKrceQvqMNduXgig9WgFdcVi6PzGOfpo1wvRd9ZMSoVwIw7PQ99CLLaTvw
d+RGuoAc/Sl553M23VYKofWZst3SZd4X10W485JTRXlGWCVs9BP6u5Mz/ctpPNzA
hx6t4foqZR57dRpe5OLREZBF3l5/yg/e4BIWw/4UyHYksJNSo7zjKDsKxem8tWqh
aCSLqCXM9T9u8NfCpQeAD5S3zk1Jq0a50RwTufKSCZ5yqJ9xOdkiijUKRBPzkVco
czKe3l/xAoGBAPSdBjofXcAd8d53QgviLzv0yiIEPVi8ca+lWB51wS6Wap4qOGmO
9KmuL7Pe0SuiIAaHuTcHj3+Dtxj+YAq6y94Kr+RuUh+KHjCCOotUQo/7kv1BdNBR
VRYcTx31AkJxvhwpDvbE6vG1rugdIx4/uwbUAeCues9pg4S5YtokN+5VAoGBAMyK
0Affs4XqwKSPXked2PBVqEgfIvNbpOLk5qBVJDaSzRxoi9mAXRrys6ixnMQbw5aE
ku0jwdBqO9FImaYb42zaAIhdiGsi6TYNAZc9mjorN55BJktHVKXMiJ7Qn47+Xcst
kIfvFZqi4Xe0Y9dJxi5QBmHPip6RFCzfTgEhQ2KbAoGBALdHL/M8rTHVPfRBxurA
fkOeFhWGt7BVAZlpeazY7fXKHfiECeCNjCPWr0dGG9NlljlOWCgzMgGI3xeLAMsJ
StcDiubNAhs/kX8nFFZwu67GAExhTvUG+4mljGIVwaunLZpL6TpcMO0KBBNGQZnY
hnmHQCDomnvtwu0fOkHYAVBBAoGBAJy+a3j9NqeTJBl3PHqJPeu+2HycT75KJmsx
moKXfG7NxJzWiQUeVzsFbiQCJ+4okJPR+H39ftLl9f0FM6mViYyq6umVM6tWT3z6
LpPlw0B8SdzedCgP+5Aq0y9RFFUSHHJM8osY7mdYFowTEseyY0quTJW747Br4RgZ
777x7J+tAoGBALasrp2S/YwTgx4kG+F/4FQG/aEwCthHuaQHgbOAwtfnK0URFe1N
pStv0QV3SvXYTloMKoacOUvwcrLU4LIEDxHtXM6rB9KMlKdY0Gcl+nHv/IRYyq1p
v0vzU+zKtXPkmO+jBXSVHTWdXIhXdthn+NQ90VIC4DF0JN/joHYOisv2
-----END RSA PRIVATE KEY-----
)KEY";

// Initialize static member
Communication_Driver* Communication_Driver::_instance = nullptr;

Communication_Driver::Communication_Driver(HardwareSerial& modemSerialPort, NpkSensor& npkSensor, BME280Sensor& bmeSensor, RTC_Module& rtcModule)
    : _modemSerial(modemSerialPort), 
      _npkSensor(npkSensor), 
      _bmeSensor(bmeSensor),
      _rtc(rtcModule),
      _modem(_modemSerial), // TinyGsm'yi seri portla başlat
      _gsmClient(_modem),   // TinyGsmClient'ı modemle başlat
      _secureClient(&_gsmClient), // SSLClient'ı TinyGsmClient ile başlatın
      _mqttClient(_secureClient),  // PubSubClient'ı SSLClient ile başlatın
      _gps_lat(0.0f),       // GPS değişkenlerini başlat
      _gps_lon(0.0f),
      _gps_speed(0.0f),
      _gps_alt(0.0f),
      _gps_year(0),
      _gps_month(0),
      _gps_day(0),
      _gps_hour(0),
      _gps_minute(0),
      _gps_second(0),
      _gps_fix_available(false),
      _sup_bat_external(0.0f), 
      _sup_solar_external(0.0f)
{
    _instance = this; // Statik örnek işaretçisini bu nesneye ayarlayın
    memset(_sup_4v, 0, sizeof(_sup_4v)); // Modem pil voltajı dizesini başlat
}
bool Communication_Driver::pwrmodem() {
    // Modemi PWR_sim808 aktif hale getir
    digitalWrite(PWR_sim808, LOW); 
    delay(1000); 
    digitalWrite(PWR_sim808, HIGH);  

    int retry = 0;
    const int max_retries = 5;
    while (!_modem.testAT(1000U) && retry < max_retries) {
        LOG_WARN("AT komutuna yanıt yok, tekrar deneniyor... (%d)", retry + 1);
        digitalWrite(PWR_sim808, LOW); 
        delay(1000); 
        digitalWrite(PWR_sim808, HIGH); 
        delay(1500);
        retry++;
    }

    if (retry >= max_retries) {
        LOG_ERROR("Modem %d denemeden sonra yanıt vermiyor!", max_retries);
        return false; 
    }
    LOG_INFO("Modem AT yanıtı alındı.");

    return true; // Başarılı
}
bool Communication_Driver::setupModem() {
    LOG_INFO("Modem başlatılıyor...");
    
    if (!pwrmodem()) {
        LOG_ERROR("Modem güç kontrolü başarısız!");
        return false; 
    }

    // SIM card PIN control
    if (strlen(SIM_CARD_PIN) > 0) {
        LOG_INFO("SIM PIN giriliyor...");
        if (!_modem.simUnlock(SIM_CARD_PIN)) {
            LOG_ERROR("SIM PIN kilidi açılamadı!");
            return false; 
        }
        LOG_INFO("SIM PIN başarıyla girildi.");
    }
    // Check SIM card status
    SimStatus simStatus = _modem.getSimStatus();
    int retry = 0; 
    while (simStatus != SIM_READY && retry < 5) { 
        LOG_WARN("SIM Kart hazır değil: %d, bekleniyor...", simStatus);
        delay(1000);
        simStatus = _modem.getSimStatus();
        retry++;
    }
    if (simStatus != SIM_READY) {
        LOG_ERROR("SIM Kart %d denemeden sonra hazır değil!", retry);
        return false;
    }
    LOG_INFO("SIM Kart hazır.");
    LOG_INFO("Modem ayarları tamamlandı.");
    return true;
}

int Communication_Driver::restartModem() {
    LOG_INFO("Modem yeniden başlatılıyor...");

    // 1. Modem power down (using power control pin)
    LOG_INFO("Modem kapatılıyor (PWR_sim808 pini LOW)...");
    _modem.poweroff(); // Modemi kapatın (SIM800 serisi için poweroff())
    delay(3000); // Modemin kapanması için zaman verin

    LOG_INFO("Modem açılıyor (PWR_sim808 pini HIGH)...");
    digitalWrite(PWR_sim808, LOW); 
    delay(1000); 
    digitalWrite(PWR_sim808, HIGH); 
    delay(2000);

    
    LOG_INFO("Modem yeniden başlatılıyor ve kuruluyor...");
    if (this->setupModem()) {
        LOG_INFO("Modem başarıyla yeniden başlatıldı ve kuruldu.");
        return 0; // Success
    } else {
        LOG_ERROR("Modem yeniden başlatma ve kurulumu başarısız!");
        return 1; // Failure
    }
}

bool Communication_Driver::enableGPS() {

    setupModem();
    LOG_INFO("GPS etkinleştiriliyor...");
    if (_modem.enableGPS()) { // TinyGsmGPS.tpp içindeki public enableGPS() kullanılır
        LOG_INFO("GPS başarıyla etkinleştirildi.");
        return true;
    } else {
        LOG_ERROR("GPS etkinleştirilemedi.");
        return false;
    }
}

bool Communication_Driver::disableGPS() {
    LOG_INFO("GPS devre dışı bırakılıyor...");
    if (_modem.disableGPS()) { // TinyGsmGPS.tpp içindeki public disableGPS() kullanılır
        LOG_INFO("GPS başarıyla devre dışı bırakıldı.");
        return true;
    } else {
        LOG_ERROR("GPS devre dışı bırakılamadı.");
        return false;
    }
}

bool Communication_Driver::getGPSLocation(float* lat, float* lon, float* speed, float* alt, int* year, int* month, int* day, int* hour, int* minute, int* second) {
    LOG_INFO("GPS konumu alınıyor...");
    // Geçici değişkenler kullanarak doğrudan sınıf üyelerine yazmaktan kaçının
    // eğer getGPS başarısız olursa eski değerler korunur.
    float temp_lat, temp_lon, temp_speed, temp_alt;
    int temp_year, temp_month, temp_day, temp_hour, temp_minute, temp_sec;

    // TinyGsmGPS.tpp içindeki public getGPS() kullanılır
    _gps_fix_available = _modem.getGPS(&temp_lat, &temp_lon, &temp_speed, &temp_alt, nullptr, nullptr, nullptr, &temp_year, &temp_month, &temp_day, &temp_hour, &temp_minute, &temp_sec);

    if (_gps_fix_available) {
        // Sınıf üyelerini güncelle
        _gps_lat = temp_lat;
        _gps_lon = temp_lon;
        _gps_speed = temp_speed;
        _gps_alt = temp_alt;
        _gps_year = temp_year;
        _gps_month = temp_month;
        _gps_day = temp_day;
        _gps_hour = temp_hour;
        _gps_minute = temp_minute;
        _gps_second = temp_sec;

        // Fonksiyon argümanlarını (pointer ise) güncelle
        if (lat) *lat = _gps_lat;
        if (lon) *lon = _gps_lon;
        if (speed) *speed = _gps_speed;
        if (alt) *alt = _gps_alt;
        if (year) *year = _gps_year;
        if (month) *month = _gps_month;
        if (day) *day = _gps_day;
        if (hour) *hour = _gps_hour;
        if (minute) *minute = _gps_minute;
        if (second) *second = _gps_second;

        snprintf(Location, sizeof(Location), "%.6f,%.6f", _gps_lat, _gps_lon);
        LOG_INFO("GPS Konumu: Lat=%.6f, Lon=%.6f, Speed=%.2f, Alt=%.2f", _gps_lat, _gps_lon, _gps_speed, _gps_alt);
        LOG_INFO("GPS Tarih/Saat: %04d-%02d-%02d %02d:%02d:%02d", _gps_year, _gps_month, _gps_day, _gps_hour, _gps_minute, _gps_second);
        return true;
    } else {
        LOG_WARN("GPS konumu alınamadı veya fix yok.");
        // İsteğe bağlı olarak, pointer'ları varsayılan değerlere ayarlayın
        if (lat) *lat = 0.0f;
        if (lon) *lon = 0.0f;
        // ... diğerleri için de
        return false;
    }
}

bool Communication_Driver::readGPSWithRetry(int maxRetries) {
    int retryCount = 0;
    while (retryCount < maxRetries) {
        if (getGPSLocation(&_gps_lat, &_gps_lon, &_gps_speed, &_gps_alt, &_gps_year, &_gps_month, &_gps_day, &_gps_hour, &_gps_minute, &_gps_second)) {
            LOG_INFO("GPS Konumu alındı: Lat=%.6f, Lon=%.6f", _gps_lat, _gps_lon);
            return true; // Başarılı
        } else {
            LOG_WARN("GPS konumu alınamadı. Deneme %d/%d", retryCount + 1, maxRetries);
            delay(2000); // Bekleme süresi
            retryCount++;
        }
    }
    return false; // Tüm denemeler başarısız oldu
}
bool Communication_Driver::connectGPRS() {
    if (_modem.isGprsConnected()) {
        LOG_INFO("GPRS zaten bağlı.");
        return true;
    }

    LOG_INFO("GPRS bağlantısı kuruluyor...");
    LOG_INFO("Ağa bağlanılıyor...");
    if (!_modem.waitForNetwork(60000L)) { // Ağ kaydı için 1 dakikaya kadar bekleyin
        LOG_ERROR("Ağ bağlantısı zaman aşımına uğradı!");
        // burada modemi yeniden kurmayı deneyin veya sistemin yeniden başlat
        setupModem(); 
        return false;
    }
    LOG_INFO("Ağ bağlantısı OK. Sinyal kalitesi: %d", _modem.getSignalQuality());

    LOG_INFO("GPRS'e bağlanılıyor (APN: %s)...", APN);
    if (!_modem.gprsConnect(APN, APN_USERNAME, APN_PASSWORD)) {
        LOG_ERROR("GPRS bağlantısı başarısız!");
        delay(5000); // Tekrar denemeden veya başarısız olmadan önce bekleyin
        return false;
    }
    LOG_INFO("GPRS Bağlantısı OK. IP: %s", _modem.getLocalIP().c_str());
    return true;
}

bool Communication_Driver::connectMQTT() {
    LOG_INFO("Modem başlatılıyor...");
    connectGPRS(); // 🔹 **GPRS Bağlantısını Kur**
const int MAX_MODEM_RETRIES = 5;
int modemRetries = 0;

while ((!_modem.isNetworkConnected() || !_modem.isGprsConnected()) && modemRetries < MAX_MODEM_RETRIES) {
    LOG_WARN("Modem veya GPRS bağlı değil. (%d/%d) Kurulum ve bağlantı yeniden deneniyor...", modemRetries + 1, MAX_MODEM_RETRIES);

    if (!setupModem()) {
        LOG_ERROR("Modem kurulumu başarısız (deneme %d)", modemRetries + 1);
    } else if (!connectGPRS()) {
        LOG_ERROR("GPRS bağlantısı başarısız (deneme %d)", modemRetries + 1);
    } else {
        break; // Başarılı bağlantı
    }

    modemRetries++;
    delay(2000); // Gecikme ile modem stabilize olabilir
}

if (!_modem.isNetworkConnected() || !_modem.isGprsConnected()) {
    LOG_ERROR("Modem/GPRS bağlantısı %d denemeden sonra başarısız oldu.", MAX_MODEM_RETRIES);
    return false;
}
    
    LOG_INFO("SSL İstemcisi ayarlanıyor...");
    _secureClient.setCACert(aws_root_ca_pem_driver);
    _secureClient.setCertificate(aws_certificate_pem_driver);
    _secureClient.setPrivateKey(aws_private_pem_driver);

    // 🔹 **MQTT Sunucusunu ve Portunu Ayarla**
    LOG_INFO("MQTT bağlantısı kuruluyor...");
    _mqttClient.setServer(AWS_IOT_ENDPOINT, AWS_IOT_PORT);
    LOG_INFO("MQTT sunucusuna bağlanılıyor: %s:%d", AWS_IOT_ENDPOINT, AWS_IOT_PORT);
    _mqttClient.setCallback(Communication_Driver::staticMqttCallback); 

    int retries = 0;
    while (!_mqttClient.connected() && retries < 5) { // 5 kez bağlanmayı deneyin
        LOG_INFO("MQTT Bağlantı denemesi %d...", retries + 1);
        
         // Client ID ile bağlan
        if (_mqttClient.connect(MQTT_CLIENT_ID)) {
            LOG_INFO("MQTT Bağlandı!");
            // Subscribe to the topic
            if (_mqttClient.subscribe(AWS_IOT_SUBSCRIBE_TOPIC)) {
                LOG_INFO("MQTT konusuna abone olundu: %s", AWS_IOT_SUBSCRIBE_TOPIC);
            } else {
                LOG_ERROR("MQTT konusuna abone olma başarısız!");
               // Abonelik şimdilik başarısız olsa bile devam et veya false döndür
            }
            return true; // Başarıyla bağlandı
        } 
        else {
            LOG_ERROR("MQTT bağlantı hatası, rc=%d. Detaylar için PubSubClient dokümantasyonuna bakın.", _mqttClient.state());
            // Common error codes:
            // -4: MQTT_CONNECTION_TIMEOUT
            // -3: MQTT_CONNECTION_LOST
            // -2: MQTT_CONNECT_FAILED
            // -1: MQTT_DISCONNECTED
            //  1: MQTT_CONNECT_BAD_PROTOCOL
            //  2: MQTT_CONNECT_BAD_CLIENT_ID
            //  3: MQTT_CONNECT_UNAVAILABLE
            //  4: MQTT_CONNECT_BAD_CREDENTIALS
            //  5: MQTT_CONNECT_UNAUTHORIZED
            retries++;
            delay(2000); 
        }
    }

    if (!_mqttClient.connected()) {
        LOG_ERROR("MQTT bağlantısı %d denemede başarısız oldu.", retries);
        return false;
    }
    return true;
}

int Communication_Driver::restartGPRS() {
    LOG_INFO("GPRS yeniden başlatılıyor...");

    LOG_INFO("Mevcut GPRS bağlantısı kesiliyor...");
    if (_modem.isGprsConnected()) {
        _modem.gprsDisconnect();
        LOG_INFO("GPRS bağlantısı kesildi.");
    } else {
        LOG_INFO("GPRS zaten bağlı değil.");
    }
    delay(1000); 

    LOG_INFO("Yeni GPRS bağlantısı kuruluyor...");
    if (this->connectGPRS()) { // Sınıfın metodunu kullanarak GPRS'e bağlanmayı deneyin
        LOG_INFO("GPRS başarıyla yeniden bağlandı.");
        return 0; 
    } else {
        LOG_ERROR("GPRS yeniden bağlanma başarısız!");
        return 1; 
    }
}


String Communication_Driver::createCsvDataLine() {
    String csv_row = "";
    char buffer[20]; // For float to string conversion
    // 1. measurement_id
    csv_row += String(wakeCounter);
    csv_row += ",";
    // 2. measurement_time
    csv_row += _rtc.getTimestamp();
    csv_row += ",";
    // 3. soil_nitrogen
    dtostrf(_npkSensor.getAzot(), 4, 1, buffer); csv_row += buffer; csv_row += ",";
    // 4. soil_phosphorus
    dtostrf(_npkSensor.getFosfor(), 4, 1, buffer); csv_row += buffer; csv_row += ",";
    // 5. soil_potassium
    dtostrf(_npkSensor.getPotasyum(), 4, 1, buffer); csv_row += buffer; csv_row += ",";
    // 6. soil_humidity (NPK Nem)
    dtostrf(_npkSensor.getNem(), 4, 1, buffer); csv_row += buffer; csv_row += ",";
    // 7. soil_temperature (NPK Sicaklik)
    dtostrf(_npkSensor.getSicaklik(), 4, 1, buffer); csv_row += buffer; csv_row += ",";
    // 8. soil_electrical_conductivity (NPK EC)
    dtostrf(_npkSensor.getEC(), 4, 2, buffer); csv_row += buffer; csv_row += ",";
    // 9. soil_ph (NPK pH)
    dtostrf(_npkSensor.getPH(), 4, 2, buffer); csv_row += buffer; csv_row += ",";
    // 10. weather_air_temperature (BME280 Temp)
    dtostrf(_bmeSensor.getTemperature(), 4, 1, buffer); csv_row += buffer; csv_row += ",";
    // 11. weather_air_humidity (BME280 Hum)
    dtostrf(_bmeSensor.getHumidity(), 4, 1, buffer); csv_row += buffer; csv_row += ",";
    // 12. weather_air_pressure (BME280 Press)
    dtostrf(_bmeSensor.getPressure(), 6, 2, buffer); csv_row += buffer; csv_row += ","; // Assuming pressure is in Pa, convert to hPa
    // 14. system_solar_panel_voltage (Placeholder)
    dtostrf(_sup_solar_external, 4, 2, buffer); csv_row += buffer; csv_row += ",";
    // 13. system_battery_voltage (_sup_bat_external)
    dtostrf(_sup_bat_external, 4, 2, buffer); csv_row += buffer; csv_row += ",";
    // 15. system_supply_voltage_1 (_sup_4v - modem battery)
    csv_row += String(_sup_4v); csv_row; // _sup_4v is already a char array (string)
    LOG_INFO("CSV Data Line: %s", csv_row.c_str());
    return csv_row;
}

String Communication_Driver::createJsonPayloadForAWS(const String& csv_content) {
    JsonDocument doc; 

    doc["hash_code"] = "a1b2c3d4e5f67890a1b2c3d4e5f67890a1b2c3d4e5f67890a1b2c3d4e5f67890";
    JsonObject header = doc["header"].to<JsonObject>();
    header["transmit_time"] = _rtc.getTimestamp();
    header["device_id"] = DEVICE_ID; // From Config.h
    header["measurement_period"] = (unsigned long)(uykusuresi / 1000000ULL); // Saniye cinsinden
    header["transmit_period"] = (unsigned long)(uykusuresi / 1000000ULL) * MAX_WAKECOUNTER; // Saniye cinsinden
    header["measurement_count"] = MAX_WAKECOUNTER;
    header["status"] = "active";
    doc["csv"] = csv_content; // csv_content zaten kendi başlık satırını içerir ve \n

    String jsonBuffer;
    serializeJson(doc, jsonBuffer);

    LOG_INFO("AWS JSON Yükü Oluşturuldu. Uzunluk: %d", jsonBuffer.length());
    LOG_DEBUG("AWS JSON: %s", jsonBuffer.c_str());
    return jsonBuffer;
}

bool Communication_Driver::publishData(const char* payload) {
    if (!_mqttClient.connected()) {
        LOG_ERROR("MQTT bağlı değil, veri yayınlanamıyor.");
        return false;
    }

    LOG_INFO("MQTT Konusuna gönderiliyor: %s", AWS_IOT_PUBLISH_TOPIC);
    LOG_DEBUG("Payload: %s", payload); // payload artık bir parametre

    const int maxRetries = 5; // Orijinal publishData ile tutarlı
    for (int attempt = 1; attempt <= maxRetries; ++attempt) {
        if (_mqttClient.publish(AWS_IOT_PUBLISH_TOPIC, payload)) { // payload'ı doğrudan kullan
            LOG_INFO("Mesaj %d. denemede başarıyla yayınlandı.", attempt);
            return true;
        } else {
            LOG_WARN("MQTT mesaj yayınlama %d. denemede başarısız!", attempt);
            delay(500);
        }
    }

    LOG_ERROR("MQTT mesaj yayınlama %d denemeden sonra başarısız oldu.", maxRetries);
    return false;
}

void Communication_Driver::mqttLoop() {
    if (_mqttClient.connected()) {
        _mqttClient.loop();
    }
}

bool Communication_Driver::isMqttConnected() {
    return _mqttClient.connected();
}

bool Communication_Driver::isGprsConnectedDriver() {
    return _modem.isGprsConnected();
}

void Communication_Driver::disconnect() {
    if (_mqttClient.connected()) {
        _mqttClient.disconnect();
        LOG_INFO("MQTT bağlantısı kapatıldı.");
    }
    if (_modem.isGprsConnected()) {
        _modem.gprsDisconnect();
        LOG_INFO("GPRS bağlantısı kapatıldı.");
    }
    
    LOG_INFO("Modem kapatılıyor...");
    if (_modem.poweroff()) { // SIM800 serisi için poweroff() 
        LOG_INFO("Modem başarıyla kapatıldı.");
    } else {
        LOG_WARN("Modem kapatılamadı veya zaten kapalıydı.");
    }
    delay(500); // Modeme kapanması için zaman verin
}

void Communication_Driver::updateModemBatteryStatus() {
    int voltageMV = _modem.getBattVoltage(); // Voltajı mV olarak döndürür
    if (voltageMV > 0) {
        float voltageV = voltageMV > 0 ? voltageMV / 1000.0 : 0.0;
        dtostrf(voltageV, 5, 2, _sup_4v); // SUP_4V'yi doldur
        LOG_INFO("Modem Batarya Voltajı: %sV", _sup_4v);
    } else {
        LOG_WARN("Modem batarya voltajı okunamadı.");
        strcpy(_sup_4v, "N/A");
    }
}

float Communication_Driver::readAndProcessBatteryVoltage() {
    int rawValue = analogRead(BAT_VOLTAGE_PIN);
    float voltage_adc = (rawValue / ADC_MAX_VALUE) * ADC_VREF;
    _sup_bat_external = voltage_adc * (VOLTAGE_DIVIDER_R1 + VOLTAGE_DIVIDER_R2) / VOLTAGE_DIVIDER_R2;
    _sup_bat_external = round(_sup_bat_external * 100.0) / 100.0;
    LOG_INFO("=> ADC Voltajı (Okunan Değer): %.2fV, Hesaplanan Pil Voltajı: %.2fV", voltage_adc, _sup_bat_external);
    return _sup_bat_external;
}

float Communication_Driver::readAndProcessSolarVoltage() {
    int solar_rawValue = analogRead(SOLAR_VOLTAGE_PIN);
    float solar_voltage_adc = (solar_rawValue / ADC_MAX_VALUE) * ADC_VREF;
    _sup_solar_external = solar_voltage_adc * (SOLAR_VOLTAGE_DIVIDER_R1 + SOLAR_VOLTAGE_DIVIDER_R2) / SOLAR_VOLTAGE_DIVIDER_R2;
    _sup_solar_external = round(_sup_solar_external * 100.0) / 100.0;
    LOG_INFO("=> ADC Voltajı (Okunan Değer): %.2fV, Hesaplanan solar Voltajı: %.2fV", solar_voltage_adc, _sup_solar_external);
    return _sup_solar_external;
}

// Statik geri çağırma işlevi sarmalayıcısı
void Communication_Driver::staticMqttCallback(char* topic, byte* payload, unsigned int length) {
    if (_instance) {
        _instance->handleMqttCallback(topic, payload, length);
    } else {
        LOG_ERROR("Communication_Driver örneği MQTT geri araması için ayarlanmadı!");
    }
}

// Actual MQTT message handler
void Communication_Driver::handleMqttCallback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Mesaj geldi [");
    Serial.print(topic);
    Serial.print("] ");
    JsonDocument doc;
    deserializeJson(doc, payload);
    const char* message = doc["message"];
    Serial.println(message);
    // Gelen mesaja göre işlem yapabilirsiniz 
}
