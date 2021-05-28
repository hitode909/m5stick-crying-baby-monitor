#include <M5StickC.h>
#include <driver/i2s.h>

#define WAVE_LEN 256
#define DISPLAY_WIDTH 160
#define DISPLAY_HEIGHT 80

#define PIN_CLK  0
#define PIN_DATA 34
#define READ_LEN (2 * WAVE_LEN)
uint8_t BUFFER[READ_LEN] = {0};

#define SAMPLE_PER_SECOND 1
#define SAMPLING_SECOND 60*60*12
#define SAMPLES_MAX SAMPLING_SECOND * SAMPLE_PER_SECOND
const int samplesMax = SAMPLES_MAX;

int16_t *adcBuffer = NULL;

TFT_eSprite img = TFT_eSprite(&M5.Lcd);

void i2sInit()
{
  // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2s.html#_CPPv418i2s_driver_install10i2s_port_tPK12i2s_config_tiPv
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
    .sample_rate =  44100,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // I2S_BITS_PER_SAMPLE_8BIT にしたい?
    .channel_format = I2S_CHANNEL_FMT_ALL_RIGHT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 2,
    .dma_buf_len = 128,
  };

  i2s_pin_config_t pin_config;
  pin_config.bck_io_num   = I2S_PIN_NO_CHANGE;
  pin_config.ws_io_num    = PIN_CLK;
  pin_config.data_out_num = I2S_PIN_NO_CHANGE;
  pin_config.data_in_num  = PIN_DATA;


  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_set_clk(I2S_NUM_0, 44100, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
}



void mic_record_task (void* arg)
{
  size_t bytesread;
  while (1) {
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2s.html#_CPPv48i2s_read10i2s_port_tPv6size_tP6size_t10TickType_t
    i2s_read(I2S_NUM_0, (char*) BUFFER, READ_LEN, &bytesread, (100 / portTICK_RATE_MS));
    adcBuffer = (int16_t *)BUFFER;

    analyze();
    showSignal();
    updateClock();
    // sleepForStep();
  }
}

// variables for visualize
unsigned short maxHistory[SAMPLES_MAX] = {0};
float plotValues[DISPLAY_WIDTH] = {0.0};
float annotations[DISPLAY_WIDTH] = {0.0};
uint16_t annotationColors[DISPLAY_WIDTH] = {WHITE};
// TODO: change to zero-base
int frameCount = 0 + SAMPLES_MAX; // count up to infinity

void setup() {
  M5.begin();
  M5.Lcd.setRotation(3);
  M5.Axp.ScreenBreath(8); // 7 ~ 15
  M5.Lcd.fillScreen(WHITE);
  M5.Lcd.setTextColor(BLACK, WHITE);
  M5.Lcd.setTextSize(1);

  img.createSprite(DISPLAY_WIDTH, DISPLAY_HEIGHT);
  img.fillSprite(TFT_WHITE);

  Serial.begin(115200);
  Serial.println("Setup done");

  i2sInit();
  xTaskCreate(mic_record_task, "mic_record_task", 2048, NULL, 1, NULL);

  // for(int i = 0; i < SAMPLES_MAX; i++) {
  //  maxHistory[i] = i % 100;
  // }
}

int frameCountPrev = 0;
float inputVolume = 0.0;
void analyze() {
  float sum = 0.0;
  for (int n = 0; n < WAVE_LEN; n++) {
    float absValue = abs(adcBuffer[n]);
    sum += absValue / WAVE_LEN;
  }

  // to ignore noise on initialize
  inputVolume = min(inputVolume + 0.001, 1.0);

  sum *= inputVolume;
  if (frameCount != frameCountPrev) {
    maxHistory[frameCount % samplesMax] = 0;
    frameCountPrev = frameCount;
  }
  unsigned int prev = maxHistory[frameCount % samplesMax];

  // XXX: scale down to avoid overflow
  maxHistory[frameCount % samplesMax] += (int)(sum / 100);
  if (prev > maxHistory[frameCount % samplesMax]) {
    Serial.print("overflow ");
    Serial.print(prev);
    Serial.print(" -> ");
    Serial.println(maxHistory[frameCount % samplesMax]);

  }
}

unsigned short maxAt(int i) {
  // while (i > SAMPLES_MAX) i-= SAMPLES_MAX;
  return maxHistory[i % samplesMax];

}

float maxInRange(int from, int to) {
  float maxValue = maxAt(from);
  for (int i = from; i < to; i++) {
    float current = maxAt(i);
    if (current > maxValue) maxValue = current;
  }
  return maxValue;
}

#define HEADER_HEIGHT 8
#define GRAPH_HEIGHT DISPLAY_HEIGHT - (HEADER_HEIGHT * 2)

float factor = 1.0;

float seconds = 60;
#define SCALES_LENGTH 4
unsigned int scales[SCALES_LENGTH][3] = {
  {60 * 6, 5, 1},
  {60 * 30, 5, 1},
  {60 * 60 * 3, 60, 5},
  {60 * 60 * 12, 60, 15},
};
int scaleIndex = 0;

void showSignal() {
  scaleIndex = (frameCount / 10) % SCALES_LENGTH;
  seconds = ceil(seconds * 0.95 + (float)scales[scaleIndex][0] * 0.05);
  showSignalInner(seconds, scales[scaleIndex][1], scales[scaleIndex][2]);
  M5.Lcd.setCursor(0, 0);
  // M5.Lcd.printf("%d    ", (int)seconds);
  // Serial.println(scaleIndex);
  // Serial.println((int)seconds);
}

void showSignalInner(unsigned int targetSeconds, unsigned int annotateLong, unsigned int annotateShort) {
  img.fillSprite(TFT_WHITE);

  int availableSamplesCount = targetSeconds * SAMPLE_PER_SECOND;
  int sampleAvailableFrom = max(frameCount - availableSamplesCount, 0);
  for (double n = 0; n < DISPLAY_WIDTH; n++) {
    int indexFrom = sampleAvailableFrom + floor(availableSamplesCount * (n / (DISPLAY_WIDTH)));
    int indexTo = sampleAvailableFrom + floor(availableSamplesCount * ((n + 1) / (DISPLAY_WIDTH)));

    float currentValue = maxInRange(
                           indexFrom,
                           indexTo
                         );
    plotValues[(int)n] = currentValue;
    int minFrom = floor((frameCount - indexFrom) / (60 * SAMPLE_PER_SECOND));
    int minTo = floor((frameCount - indexTo) / (60 * SAMPLE_PER_SECOND));
    bool minChanged = minFrom != minTo;
    if (minChanged) {
      if (minFrom / annotateLong != minTo / annotateLong) {
        annotations[(int)n] = HEADER_HEIGHT;
        annotationColors[(int)n] = BLACK;
      } else if (minFrom / annotateShort != minTo / annotateShort) {
        annotations[(int)n] = floor(HEADER_HEIGHT * 0.8);
        annotationColors[(int)n] = DARKGREY;
      } else {
        annotations[(int)n] = 0;
        annotationColors[(int)n] = WHITE;
        // annotations[(int)n] = floor(HEADER_HEIGHT * 0.5);
        // annotationColors[(int)n] = DARKGREY;
      }
    } else {
      annotations[(int)n] = 0;
      annotationColors[(int)n] = WHITE;
    }
  }

  float valueMax = 0.0;
  // to use recent values, loop from n = DISPLAY_WIDTH * 0.5
  for (int n = 0; n < DISPLAY_WIDTH; n++) {
    valueMax = max(valueMax, plotValues[n]);
  }

  // low pass
  factor = (0.01 * factor) + ((valueMax / DISPLAY_HEIGHT) * 0.9 * 0.99);

  for (int n = 0; n < DISPLAY_WIDTH; n++) {
    int y = max(ceil(GRAPH_HEIGHT - plotValues[n] / factor), 0.0);
    img.drawFastVLine(n, y / 2 + HEADER_HEIGHT, (GRAPH_HEIGHT - y), LIGHTGREY);

    // annotation
    img.drawFastVLine(n, 0, annotations[n], annotationColors[n]);
    img.drawFastVLine(n, DISPLAY_HEIGHT - annotations[n], annotations[n], annotationColors[n]);
  }

  img.setTextFont(4); // 1:8px 2:16px 4:26px
  img.setTextColor(BLACK);
  img.setCursor(5, (80 - 26) / 2);
  if (seconds >= 3600) {
    img.printf("%.0fhr", round(seconds / 3600));
  } else {
    img.printf("%.0fmin", round(seconds / 60));
  }

  img.pushSprite(0, 0);
}

void updateClock() {
  frameCount =  millis() / (1000 / SAMPLE_PER_SECOND) + SAMPLES_MAX;
}

void sleepForStep() {
  vTaskDelay(1000 / SAMPLE_PER_SECOND);
}

void loop() {
  vTaskDelay(1000 / portTICK_RATE_MS);
}
