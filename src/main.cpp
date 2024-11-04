// TODO
// Hardware initialization
// Component test functions

#include <Arduino.h>
#include <esp_camera.h>
#include <thijs_rplidar.h>
#include <lidar.h>
#include <driver/i2s.h>
#include <BluetoothSerial.h>
#include <Button2.h>
#include <LittleFS.h>

// camera
#define XCLK_CAM 38  // External clock
#define SIOD_CAM 2   // I2C SDA
#define SIOC_CAM 1   // I2C SCL
#define D7_CAM 4     // D7
#define D6_CAM 5     // D6
#define D5_CAM 6     // D5
#define D4_CAM 7     // D4
#define D3_CAM 15    // D3
#define D2_CAM 16    // D2
#define D1_CAM 46    // D1
#define D0_CAM 8     // D0
#define VSYNC_CAM 41 // VSYNC
#define HREF_CAM 42  // HREF
#define PCLK_CAM 39  // PCLK

// lidar
#define TX 17        // Lidar UART TX
#define RX 18        // Lidar UART RX
#define MOTO_CTRL 19 // Lidar motor pwn

// haptic feedback
#define MOTORL2 10 // Left-most motor pin
#define MOTORL1 11 // Middle-left motor pin
#define MOTORM 12  // Middle motor pin
#define MOTORR1 13 // Middle-right motor pin
#define MOTORR2 14 // RIght-most motor pin

// mic
#define I2S_SD 20 // I2S
#define I2S_SCK 9 // I2S
#define I2S_WS 3  // I2S Word Select

// speaker
#define SPEAKER 21

// button
#define BUTTON 45 // Input button

// bluetooth
BluetoothSerial SerialBT;

// function declarations

//**LiDAR begin
lidarMotorHandler motorHandler(MOTO_CTRL);
RPlidar lidar(Serial2);
//keepSpinning = true;
// uint16_t debugPrintCounter = 0;
// const uint16_t debugPrintThreshold = 48; // print data every (this many) datapoints (if you are getting CRC errors, there may be buffer overflow, try setting this to like 48+ (or uncommenting printing entirely))
//**LiDAR end

//**Camera begin
void camera_setup(camera_config_t *config)
{
  //camera_config_t config;
  config->ledc_channel = LEDC_CHANNEL_0;
  config->ledc_timer = LEDC_TIMER_0;
  config->pin_d0 = D0_CAM;
  config->pin_d1 = D1_CAM;
  config->pin_d2 = D2_CAM;
  config->pin_d3 = D3_CAM;
  config->pin_d4 = D4_CAM;
  config->pin_d5 = D5_CAM;
  config->pin_d6 = D6_CAM;
  config->pin_d7 = D7_CAM;
  config->pin_xclk = XCLK_CAM;
  config->pin_pclk = PCLK_CAM;
  config->pin_vsync = VSYNC_CAM;
  config->pin_href = HREF_CAM;
  config->pin_sccb_sda = SIOD_CAM;
  config->pin_sccb_scl = SIOC_CAM;
  // config.pin_pwdn = PWDN_GPIO_NUM;
  // config.pin_reset = RESET_GPIO_NUM;
  config->xclk_freq_hz = 20000000;       // XCLK 20MHz for the OV2640
  config->pixel_format = PIXFORMAT_JPEG; // Camera output format

  // Frame size: QVGA (320x240), SVGA (800x600), or VGA (640x480)
  config->frame_size = FRAMESIZE_VGA;
  config->jpeg_quality = 12; // JPEG quality (lower is better)
  config->fb_count = 1;
  // Initialize the camera
  if (esp_camera_init(config) != ESP_OK)
  {
    Serial.println("Camera init failed");
  }
  else
  {
    Serial.println("Camera init succeeded");
  }
}
//**Camera end

//**Mic begin
//  I2S pins and settings
void i2s_setup()
{
  // I2S configuration struct
  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX), // Master mode, RX
      .sample_rate = 16000,                                // Sample rate in Hz
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,        // Bits per sample
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,         // Left channel (single channel)
      .communication_format = I2S_COMM_FORMAT_I2S_MSB,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // Interrupt level
      .dma_buf_count = 8,                       // Number of buffers
      .dma_buf_len = 64,                        // Buffer length
      .use_apll = false,                        // No audio PLL
      .tx_desc_auto_clear = false,              // Auto-clear TX descriptor
      .fixed_mclk = 0};

  // I2S pin configuration
  i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_SCK,             // BCK pin
      .ws_io_num = I2S_WS,               // WS pin (word select)
      .data_out_num = I2S_PIN_NO_CHANGE, // Not used for microphone
      .data_in_num = I2S_SD              // SD pin (data input)
  };

  // Install and start the I2S driver
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_set_clk(I2S_NUM_0, 16000, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO);
}
//**Mic end

void setup()
{
  Serial.begin(115200);

  //**camera setup start**
  camera_config_t config;
  camera_setup(&config);
  //**camera setup end**

  // bluetooth setup
  SerialBT.begin("ESP32test"); // Bluetooth device name

  // lidar setup
  motorHandler.init();
  lidar.init(RX, TX);
  lidar.postParseCallback = dataHandler; // set dat handler function
  lidar.printLidarInfo();
  //  lidar.printLidarHealth();
  //  lidar.printLidarSamplerate();
  //  lidar.printLidarConfig();
  if (!lidar.connectionCheck())
  {
    Serial.println("connectionCheck() failed");
    while (1)
    {}
  }
  delay(10);
  motorHandler.setPWM(200);
  // bool startSuccess = lidar.startStandardScan();
  // bool startSuccess = lidar.startExpressScan(EXPRESS_SCAN_WORKING_MODE_LEGACY);
  bool startSuccess = lidar.startExpressScan(EXPRESS_SCAN_WORKING_MODE_BOOST);
  //  Serial.print("startSuccess: "); Serial.println(startSuccess);

  // mic setup
  i2s_setup();

  // haptic feedback setup
  pinMode(MOTORL2, OUTPUT);
  pinMode(MOTORL1, OUTPUT);
  pinMode(MOTORM, OUTPUT);
  pinMode(MOTORR1, OUTPUT);
  pinMode(MOTORR2, OUTPUT);

  // speaker setup
  ledcSetup(0, 1000, 8);     // Timer 0, 1kHz frequency, 8-bit resolution
  ledcAttachPin(SPEAKER, 0); // Attach speaker pin to channel 0
}

void loop()
{
  // put your main code here, to run repeatedly:
  // if (SerialBT.available())
  // {
  //   Serial.write(SerialBT.read());
  // }
}