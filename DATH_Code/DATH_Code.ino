//////////////////////////////////////////////////////      LIBRARIES      //////////////////////////////////////////////////////
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <HTTPClient.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <Ticker.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>


//////////////////////////////////////////////////      GLOBAL VARIABLES      //////////////////////////////////////////////////
#define ONE_WIRE_BUS 2
#define triac_pin 15
#define zero_detector_pin 25
#define chip_peltier_pin 26
#define humi_dht11_pin 0
#define fans_pin 27


TaskHandle_t wifi_blynk_task;

Ticker triacTicker;

// OneWire communicate for temperature sensors
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature temp_sensors(&oneWire);
DeviceAddress tempDeviceAddress;

// Setup for humidity sensor
DHT dht(humi_dht11_pin, DHT11);

// I2C address 0x27, 20 columns and 4 rows
LiquidCrystal_I2C lcd(0x27, 20, 4);

// WidgetLED in Blynk
WidgetLED online_state_led(16);
WidgetLED chip_peltier_state_led(14);
WidgetLED fans_state_led(27);


String apps_script_id = "AKfycbybTYy9wGJTRs-dmWQw4ZskiAZzFxYfVozP5y6nCBGIrAlaHhXKoU11ZIKKgA38npD_";

char* blynk_net = "ashthieu.ddns.net";
uint16_t blynk_port = 1713;
char* blynk_token = "47B-Uh9rQKGpXbJ-MfNyUv_QpsEFQrdC";

char* wifi_network = "SaKe Quan";
char* wifi_password = "10021954";


// Variables for writing to google sheet
String apps_script_param = "";
const unsigned long sampling_time_sheet_best = 10 * 1000;  // 10s
unsigned long sampling_time_sheet = sampling_time_sheet_best;
unsigned long previous_millis_sheet = 0;

// Variables for p controller
const unsigned long sampling_time_p_best = 1 * 1000;  // 1s
unsigned long sampling_time_p = sampling_time_p_best;
unsigned long previous_millis_p = 0;
const float kp_best = -1200.0;
float kp = kp_best;

// Variables for timer
const unsigned long runtime_offline = 0;  // 0h
unsigned long runtime = runtime_offline;
unsigned long previous_millis_timer = 0;
signed long remaining_time = 0;

// Create array to store values ​​in order
// (humi, temp_avg, temp1, temp2, temp3, temp4, temp5, temp6, temp7, temp8,
// firing_delay, alpha, humi_setpoint_offline, temp_setpoint_offline, chip_peltier)
String sample[15];
float sample_float[15];

int online_mode = 0;
int fans_state = 0;
int thermistor_state = 0;
int num_temp_sensors;
int firing_delay_max = 8500;
volatile int firing_delay = 0;  // us
const float humi_setpoint_offline = 50.0;
const float temp_setpoint_offline = 50.0;


/////////////////////////////////////////////////////      FUNCTIONS      ////////////////////////////////////////////////////
void wait_function(int waiting_time_ms) {
  unsigned long curr_time = millis();
  while (!((millis() - curr_time) >= waiting_time_ms))
    ;
}


void turn_off_outputs() {
  firing_delay = firing_delay_max;
  sample_float[10] = firing_delay / 1000.0;
  sample[10] = String(sample_float[10], 2);
  sample_float[11] = firing_delay * 180.0 / 10000.0;
  sample[11] = String(sample_float[11], 2);

  digitalWrite(chip_peltier_pin, HIGH);
  digitalWrite(fans_pin, LOW);
  fans_state = 0;
  thermistor_state = 0;
  sample_float[14] = 0;
  sample[14] = String(sample_float[14], 2);
}


void print_to_lcd() {
  lcd.clear();

  for (int i = 0; i < 11; i++) {
    if (i == 0) {
      lcd.setCursor(14, 1);
      lcd.print("H: " + String(sample_float[i], 0) + "%");
    } else if (i == 1) {
      lcd.setCursor(14, 0);
      lcd.print("T : " + String(sample_float[i], 0));
    } else if (i >= 2 && i <= 5) {
      lcd.setCursor(0, i - 2);
      lcd.print("T" + String(i - 1) + ": " + String(sample_float[i], 0));
    } else if ((i >= 6 && i <= 9)) {
      lcd.setCursor(7, i - 6);
      lcd.print("T" + String(i - 1) + ": " + String(sample_float[i], 0));
    } else {
      lcd.setCursor(14, 2);
      lcd.print("f: " + String(sample_float[i], 1));
    }
  }

  lcd.setCursor(14, 3);
  lcd.print("P: " + String(digitalRead(chip_peltier_pin) == LOW ? "ON" : "OFF"));
}


void control_temp_using_p() {
  thermistor_state = 1;
  unsigned long current_millis_p = millis();

  if (current_millis_p - previous_millis_p >= sampling_time_p) {
    firing_delay = firing_delay + kp * (sample_float[13] - sample_float[1]);
    firing_delay = constrain(firing_delay, 0, firing_delay_max);

    previous_millis_p = current_millis_p;
  }

  sample_float[10] = firing_delay / 1000.0;
  sample[10] = String(sample_float[10], 2);
  sample_float[11] = firing_delay * 180.0 / 10000.0;
  sample[11] = String(sample_float[11], 2);
}


void turn_on_fans() {
  digitalWrite(fans_pin, HIGH);
  fans_state = 1;
}


void control_chip_peltier() {
  if (sample_float[0] >= sample_float[12]) {
    digitalWrite(chip_peltier_pin, LOW);
    sample_float[14] = 1;
  } else {
    digitalWrite(chip_peltier_pin, HIGH);
    sample_float[14] = 0;
  }
  sample[14] = String(sample_float[14], 2);
}


void read_humi_sensor() {
  float humi = dht.readHumidity();
  sample_float[0] = !isnan(humi) ? humi : sample_float[0];
  sample[0] = String(sample_float[0], 2);
}


void read_temp_sensors() {
  float sum_temp = 0.0;

  temp_sensors.requestTemperatures();
  num_temp_sensors = temp_sensors.getDeviceCount();
  for (int i = 2; i < 9; i++) {
    sample_float[i] = temp_sensors.getTempCByIndex(i - 2);
    sample[i] = String(sample_float[i], 2);
    sum_temp += !(round(sample_float[i]) == -127.0) ? sample_float[i] : 0.0;
  }

  float temp = dht.readTemperature();
  sample_float[9] = !isnan(temp) ? temp : sample_float[9];
  sample[9] = String(sample_float[9], 2);
  sum_temp += !isnan(sample_float[9]) ? sample_float[9] : 0.0;
  num_temp_sensors += !isnan(sample_float[9]) ? 1 : 0;

  sample_float[1] = !(num_temp_sensors == 0) ? sum_temp / num_temp_sensors : 0;
  sample[1] = String(sample_float[1], 2);
}


void print_temp_address() {
  for (int i = 0; i < num_temp_sensors; i++) {
    if (temp_sensors.getAddress(tempDeviceAddress, i)) {
      Serial.print("Found device " + String(i) + " with address: ");
      for (uint8_t j = 0; j < 8; j++) {
        Serial.print(tempDeviceAddress[j] < 16 ? "0" : "");
        Serial.print(tempDeviceAddress[j], HEX);
      }
      Serial.println();
    } else Serial.println("Found ghost device at " + String(i));
  }
}


void print_to_blynk() {
  if (!Blynk.connected()) {
    // Serial.println("Not connected to Blynk server\n");
    Blynk.connect();
  } else {
    // Serial.println("Pushed data to Blynk server\n");
    Blynk.run();

    Blynk.syncVirtual(V15);

    for (int i = 0; i < 14; i++)
      Blynk.virtualWrite(i, sample_float[i]);

    int(sample_float[14]) == 1 ? chip_peltier_state_led.on() : chip_peltier_state_led.off();
    online_mode == 1 ? online_state_led.on() : online_state_led.off();
    fans_state == 1 ? fans_state_led.on() : fans_state_led.off();

    remaining_time = runtime + previous_millis_timer - millis();
    Blynk.virtualWrite(18, remaining_time >= 0 ? remaining_time / 60000 : 0);

    Blynk.virtualWrite(22, kp);
    Blynk.virtualWrite(24, int(sampling_time_sheet / 1000));
    Blynk.virtualWrite(26, int(sampling_time_p / 1000));
  }
}


void write_to_google_sheet() {
  HTTPClient http;
  unsigned long current_millis_sheet = millis();
  if (current_millis_sheet - previous_millis_sheet >= sampling_time_sheet) {
    apps_script_param = "humi=" + sample[0] + "&temp=" + sample[1];
    apps_script_param += "&temp1=" + sample[2] + "&temp2=" + sample[3];
    apps_script_param += "&temp3=" + sample[4] + "&temp4=" + sample[5];
    apps_script_param += "&temp5=" + sample[6] + "&temp6=" + sample[7];
    apps_script_param += "&temp7=" + sample[8] + "&temp8=" + sample[9];
    apps_script_param += "&firing=" + sample[10] + "&alpha=" + sample[11];
    apps_script_param += "&humi_setpoint=" + sample[12] + "&temp_setpoint=" + sample[13];
    apps_script_param += "&chip_peltier=" + sample[14];
    Serial.println(apps_script_param);

    String url = "https://script.google.com/macros/s/" + apps_script_id + "/exec?" + apps_script_param;
    http.begin(url.c_str());
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    Serial.println("HTTP Status Code: " + String(http.GET()));
    Serial.println("Push Status: " + String(http.getString()));

    previous_millis_sheet = current_millis_sheet;
  }
}


void connect_to_wifi() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(wifi_network, wifi_password);
    unsigned long start_time = millis();
    Serial.print("Connecting");

    while ((WiFi.status() != WL_CONNECTED) && (millis() - start_time < 5000)) {
      Serial.print(".");
      wait_function(500);
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nSuccessfully connected to: " + String(wifi_network));
      Serial.println("IP address: " + String(WiFi.localIP()));
    } else Serial.println("\nNot connected to wifi");
  }
}


void wifi_blynk_task_code(void* pvParameters) {
  for (;;) {
    connect_to_wifi();
    print_to_blynk();
  }
}


void trigger_triac() {
  if (thermistor_state == 1) {
    digitalWrite(triac_pin, HIGH);
    delayMicroseconds(10);  // Small delay to ensure TRIAC fires properly
    digitalWrite(triac_pin, LOW);
  } else digitalWrite(triac_pin, LOW);
}


void IRAM_ATTR zero_cross() {
  triacTicker.once_us(firing_delay, trigger_triac);
}


////////////////////////////////////////////////////      BLYNK_WRITE      /////////////////////////////////////////////////////
// Read online_state from blynk
BLYNK_WRITE(V15) {
  online_mode = param.asInt();
  if (online_mode == 0) {
    sample_float[12] = humi_setpoint_offline;
    sample[12] = String(sample_float[12], 2);
    sample_float[13] = temp_setpoint_offline;
    sample[13] = String(sample_float[13], 2);

    runtime = runtime_offline;
    previous_millis_timer = millis();

    kp = kp_best;
    sampling_time_sheet = sampling_time_sheet_best;
    sampling_time_p = sampling_time_p_best;
  }
  Serial.println("Online mode: " + String(online_mode));
}


// Read runtime from blynk
BLYNK_WRITE(V17) {
  runtime = (online_mode == 1) ? param.asInt() * 1000 : runtime;
  previous_millis_timer = millis();
}


// Read humidity setpoint from blynk
BLYNK_WRITE(V19) {
  sample_float[12] = (online_mode == 1) ? param.asFloat() : sample_float[12];
  sample[12] = String(sample_float[12], 2);
}


// Read temperature setpoint from blynk
BLYNK_WRITE(V20) {
  sample_float[13] = (online_mode == 1) ? param.asFloat() : sample_float[13];
  sample[13] = String(sample_float[13], 2);
}


// Read kp from blynk
BLYNK_WRITE(V21) {
  kp = (online_mode == 1) ? param.asInt() : kp;
}


// Read sampling_time_sheet from blynk
BLYNK_WRITE(V23) {
  sampling_time_sheet = (online_mode == 1) ? param.asInt() * 1000 : sampling_time_sheet;
}


// Read sampling_time_p from blynk
BLYNK_WRITE(V25) {
  sampling_time_p = (online_mode == 1) ? param.asInt() * 1000 : sampling_time_p;
}


////////////////////////////////////////////////////////      SETUP      //////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);

  pinMode(zero_detector_pin, INPUT_PULLUP);
  pinMode(triac_pin, OUTPUT);
  pinMode(chip_peltier_pin, OUTPUT);
  pinMode(fans_pin, OUTPUT);

  digitalWrite(triac_pin, LOW);
  digitalWrite(chip_peltier_pin, HIGH);  // Trigger low level
  digitalWrite(fans_pin, LOW);           // Trigger high level

  attachInterrupt(zero_detector_pin, zero_cross, FALLING);

  connect_to_wifi();
  xTaskCreatePinnedToCore(wifi_blynk_task_code, "wifi_blynk_task", 10000, NULL, 1, &wifi_blynk_task, 1);

  // Connect to humidity sensor
  dht.begin();
  wait_function(2000);

  // Connect to temperature sensors
  temp_sensors.begin();
  wait_function(2000);
  num_temp_sensors = temp_sensors.getDeviceCount();
  Serial.println("\nLocating devices...Found " + String(num_temp_sensors) + " devices.");
  print_temp_address();

  lcd.init();
  lcd.backlight();

  Blynk.config(blynk_token, blynk_net, blynk_port);
  Blynk.connect();

  sample_float[12] = humi_setpoint_offline;
  sample[12] = String(sample_float[12], 2);
  sample_float[13] = temp_setpoint_offline;
  sample[13] = String(sample_float[13], 2);
}


////////////////////////////////////////////////////////      LOOP      //////////////////////////////////////////////////////
void loop() {
  read_temp_sensors();
  read_humi_sensor();

  if ((millis() - previous_millis_timer) < runtime) {
    control_chip_peltier();
    turn_on_fans();
    control_temp_using_p();

    write_to_google_sheet();
    Serial.println("System is running\n");
    Serial.println("Firing delay: " + String(firing_delay));
  } else {
    turn_off_outputs();
    Serial.println("System stopped\n");
    Serial.println("Firing delay: " + String(firing_delay));
  }

  print_to_lcd();
}
