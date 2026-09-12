/*
Class: MTRE 4400 DSBiomedical IoT Technologies
Section: 07
Term: Fall 2026
Primary Investigator: Dr. Sandip Das
Name: Christine Marie Lirazan
Project: Smart Watch
Week: 02
*/

// Section: Libraries and hardware interfaces

#include <QMI8658.h>
#include <Wire.h>
#include <Arduino.h>
#include "pin_config.h"
#include <lvgl.h>
#include "Arduino_GFX_Library.h"
#include "Arduino_DriveBus_Library.h"
#include "lv_conf.h"
#include "SensorQMI8658.hpp"
#include "HWCDC.h"

HWCDC USBSerial;


// Section: Global variables

uint32_t screenWidth;
uint32_t screenHeight;
uint32_t bufSize;

lv_display_t *disp;
lv_color_t *disp_draw_buf;

SensorQMI8658 qmi;

IMUdata acc;
IMUdata gyr;


// Section: IMU filtering settings

const float FILTER_ALPHA = 0.02;
const float GYRO_DEADBAND = 1.5;

// Description: FILTER_ALPHA controls the strength of the low-pass filter applied to the IMU readings.

// Reasoning: A low filter coefficient provides stronger smoothing to reduce visible sensor noise while maintaining real-time updates.

// Description: GYRO_DEADBAND defines the minimum gyroscope magnitude displayed as motion.

// Reasoning: Small gyroscope readings near zero are treated as sensor noise and set to zero.


// Section: Gyroscope calibration

const int CALIBRATION_SAMPLES = 1000;

float gyroBiasX = 0.0;
float gyroBiasY = 0.0;
float gyroBiasZ = 0.0;

// Configuration: The gyroscope bias is calculated using 1000 stationary samples.

// Reasoning: Averaging a large number of samples provides a more stable estimate of the sensor's resting bias.


// Section: Filtered sensor values

float filteredAccX = 0.0;
float filteredAccY = 0.0;
float filteredAccZ = 0.0;

float filteredGyrX = 0.0;
float filteredGyrY = 0.0;
float filteredGyrZ = 0.0;

bool accFilterInitialized = false;
bool gyroFilterInitialized = false;


// Section: LVGL labels

lv_obj_t *accTitle;

lv_obj_t *accXLabel;
lv_obj_t *accXValue;
lv_obj_t *accXUnit;

lv_obj_t *accYLabel;
lv_obj_t *accYValue;
lv_obj_t *accYUnit;

lv_obj_t *accZLabel;
lv_obj_t *accZValue;
lv_obj_t *accZUnit;

lv_obj_t *gyroTitle;

lv_obj_t *gyroXLabel;
lv_obj_t *gyroXValue;
lv_obj_t *gyroXUnit;

lv_obj_t *gyroYLabel;
lv_obj_t *gyroYValue;
lv_obj_t *gyroYUnit;

lv_obj_t *gyroZLabel;
lv_obj_t *gyroZValue;
lv_obj_t *gyroZUnit;


// Section: Display setup

// Warning: DIRECT_RENDER_MODE should remain disabled for the current display configuration.

Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  LCD_CS,
  LCD_SCLK,
  LCD_SDIO0,
  LCD_SDIO1,
  LCD_SDIO2,
  LCD_SDIO3
);

Arduino_GFX *gfx = new Arduino_CO5300(
  bus,
  LCD_RESET,
  0,
  LCD_WIDTH,
  LCD_HEIGHT,
  22,
  0,
  0,
  0
);


// Section: Touch setup

std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus =
  std::make_shared<Arduino_HWIIC>(
    IIC_SDA,
    IIC_SCL,
    &Wire
  );

void Arduino_IIC_Touch_Interrupt(void);

std::unique_ptr<Arduino_IIC> FT3168(
  new Arduino_FT3x68(
    IIC_Bus,
    FT3168_DEVICE_ADDRESS,
    DRIVEBUS_DEFAULT_VALUE,
    TP_INT,
    Arduino_IIC_Touch_Interrupt
  )
);

void Arduino_IIC_Touch_Interrupt(void) {
  FT3168->IIC_Interrupt_Flag = true;
}


// Section: LVGL logging

#if LV_USE_LOG != 0

void my_print(
  lv_log_level_t level,
  const char *buf
) {
  LV_UNUSED(level);

  USBSerial.println(buf);
  USBSerial.flush();
}

#endif


// Function: Provides the current system time to LVGL.

uint32_t millis_cb(void) {
  return millis();
}


// Function: Sends the rendered LVGL display buffer to the physical display.

void my_disp_flush(
  lv_display_t *disp,
  const lv_area_t *area,
  uint8_t *px_map
) {

#ifndef DIRECT_RENDER_MODE

  uint32_t w = lv_area_get_width(area);
  uint32_t h = lv_area_get_height(area);

  gfx->draw16bitRGBBitmap(
    area->x1,
    area->y1,
    (uint16_t *)px_map,
    w,
    h
  );

#endif

  lv_disp_flush_ready(disp);
}


// Function: Reads touch coordinates from the FT3168 touch controller and updates LVGL input state.

void my_touchpad_read(
  lv_indev_t *indev,
  lv_indev_data_t *data
) {

  int32_t touchX =
    FT3168->IIC_Read_Device_Value(
      FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X
    );

  int32_t touchY =
    FT3168->IIC_Read_Device_Value(
      FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y
    );

  if (FT3168->IIC_Interrupt_Flag == true) {

    FT3168->IIC_Interrupt_Flag = false;

    data->state = LV_INDEV_STATE_PR;

    data->point.x = touchX;
    data->point.y = touchY;

  } else {

    data->state = LV_INDEV_STATE_REL;
  }
}


// Function: Adjusts invalidated LVGL display areas to even pixel boundaries.

void rounder_event_cb(lv_event_t *e) {

  lv_area_t *area =
    (lv_area_t *)lv_event_get_param(e);

  uint16_t x1 = area->x1;
  uint16_t x2 = area->x2;

  uint16_t y1 = area->y1;
  uint16_t y2 = area->y2;

  area->x1 = (x1 >> 1) << 1;
  area->y1 = (y1 >> 1) << 1;

  area->x2 = ((x2 >> 1) << 1) + 1;
  area->y2 = ((y2 >> 1) << 1) + 1;
}


// Function: Calculates the resting gyroscope bias while the device is stationary.

void calibrateGyroscope() {

  USBSerial.println();
  USBSerial.println("--------------------------------");
  USBSerial.println("Gyroscope calibration starting");
  USBSerial.println("KEEP DEVICE COMPLETELY STILL");
  USBSerial.println("--------------------------------");

  // Warning: The device must remain completely stationary during calibration.

  delay(2000);

  float sumX = 0.0;
  float sumY = 0.0;
  float sumZ = 0.0;

  int samples = 0;

  while (samples < CALIBRATION_SAMPLES) {

    if (qmi.getDataReady()) {

      if (qmi.getGyroscope(
            gyr.x,
            gyr.y,
            gyr.z
          )) {

        sumX += gyr.x;
        sumY += gyr.y;
        sumZ += gyr.z;

        samples++;
      }
    }

    delay(1);
  }

  gyroBiasX =
    sumX / CALIBRATION_SAMPLES;

  gyroBiasY =
    sumY / CALIBRATION_SAMPLES;

  gyroBiasZ =
    sumZ / CALIBRATION_SAMPLES;

  USBSerial.println();
  USBSerial.println("Gyroscope calibration complete.");

  USBSerial.print("Gyro Bias X: ");
  USBSerial.println(gyroBiasX, 3);

  USBSerial.print("Gyro Bias Y: ");
  USBSerial.println(gyroBiasY, 3);

  USBSerial.print("Gyro Bias Z: ");
  USBSerial.println(gyroBiasZ, 3);

  USBSerial.println("--------------------------------");
  USBSerial.println();
}


// Function: Creates a formatted accelerometer row containing the axis label, sensor value, and unit.

void createAccelRow(
  lv_obj_t **label,
  lv_obj_t **value,
  lv_obj_t **unit,
  const char *labelText,
  const char *unitText,
  int yPosition
) {

  *label =
    lv_label_create(
      lv_scr_act()
    );

  lv_label_set_text(
    *label,
    labelText
  );

  lv_obj_set_width(
    *label,
    80
  );

  lv_obj_set_style_text_align(
    *label,
    LV_TEXT_ALIGN_LEFT,
    0
  );

  lv_obj_set_style_text_font(
    *label,
    &lv_font_montserrat_18,
    0
  );

  lv_obj_align(
    *label,
    LV_ALIGN_CENTER,
    -60,
    yPosition
  );

  *value =
    lv_label_create(
      lv_scr_act()
    );

  lv_label_set_text(
    *value,
    "0.00"
  );

  lv_obj_set_width(
    *value,
    100
  );

  lv_label_set_long_mode(
    *value,
    LV_LABEL_LONG_CLIP
  );

  lv_obj_set_style_text_align(
    *value,
    LV_TEXT_ALIGN_CENTER,
    0
  );

  lv_obj_set_style_text_font(
    *value,
    &lv_font_montserrat_18,
    0
  );

  lv_obj_align(
    *value,
    LV_ALIGN_CENTER,
    0,
    yPosition
  );

  *unit =
    lv_label_create(
      lv_scr_act()
    );

  lv_label_set_text(
    *unit,
    unitText
  );

  lv_obj_set_width(
    *unit,
    70
  );

  lv_obj_set_style_text_align(
    *unit,
    LV_TEXT_ALIGN_CENTER,
    0
  );

  lv_obj_set_style_text_font(
    *unit,
    &lv_font_montserrat_18,
    0
  );

  lv_obj_align(
    *unit,
    LV_ALIGN_CENTER,
    60,
    yPosition
  );
}


// Function: Creates a formatted gyroscope row containing the axis label, sensor value, and unit.

void createGyroRow(
  lv_obj_t **label,
  lv_obj_t **value,
  lv_obj_t **unit,
  const char *labelText,
  const char *unitText,
  int yPosition
) {

  *label =
    lv_label_create(
      lv_scr_act()
    );

  lv_label_set_text(
    *label,
    labelText
  );

  lv_obj_set_width(
    *label,
    80
  );

  lv_obj_set_style_text_align(
    *label,
    LV_TEXT_ALIGN_LEFT,
    0
  );

  lv_obj_set_style_text_font(
    *label,
    &lv_font_montserrat_18,
    0
  );

  lv_obj_align(
    *label,
    LV_ALIGN_CENTER,
    -60,
    yPosition
  );

  *value =
    lv_label_create(
      lv_scr_act()
    );

  lv_label_set_text(
    *value,
    "0.00"
  );

  lv_obj_set_width(
    *value,
    100
  );

  lv_label_set_long_mode(
    *value,
    LV_LABEL_LONG_CLIP
  );

  lv_obj_set_style_text_align(
    *value,
    LV_TEXT_ALIGN_CENTER,
    0
  );

  lv_obj_set_style_text_font(
    *value,
    &lv_font_montserrat_18,
    0
  );

  lv_obj_align(
    *value,
    LV_ALIGN_CENTER,
    0,
    yPosition
  );

  *unit =
    lv_label_create(
      lv_scr_act()
    );

  lv_label_set_text(
    *unit,
    unitText
  );

  lv_obj_set_width(
    *unit,
    70
  );

  lv_obj_set_style_text_align(
    *unit,
    LV_TEXT_ALIGN_CENTER,
    0
  );

  lv_obj_set_style_text_font(
    *unit,
    &lv_font_montserrat_18,
    0
  );

  lv_obj_align(
    *unit,
    LV_ALIGN_CENTER,
    60,
    yPosition
  );
}


// Function: Initializes the display, touch controller, IMU, calibration routine, and LVGL interface.

void setup() {

#ifdef DEV_DEVICE_INIT

  DEV_DEVICE_INIT();

#endif

  USBSerial.begin(115200);

  USBSerial.println(
    "Arduino_GFX LVGL Simple IMU Display"
  );


  // Section: Initialize display

  if (!gfx->begin()) {

    USBSerial.println(
      "gfx->begin() failed!"
    );
  }

  gfx->fillScreen(
    RGB565_BLACK
  );


  // Section: Initialize I2C

  Wire.begin(
    IIC_SDA,
    IIC_SCL
  );


  // Section: Initialize touch controller

  while (FT3168->begin() == false) {

    USBSerial.println(
      "FT3168 initialization fail"
    );

    delay(2000);
  }

  USBSerial.println(
    "FT3168 initialization successfully"
  );

  FT3168->IIC_Write_Device_State(
    FT3168->Arduino_IIC_Touch::Device::TOUCH_POWER_MODE,
    FT3168->Arduino_IIC_Touch::Device_Mode::TOUCH_POWER_MONITOR
  );


  // Section: Initialize QMI8658

  if (!qmi.begin(
        Wire,
        QMI8658_L_SLAVE_ADDRESS,
        IIC_SDA,
        IIC_SCL
      )) {

    USBSerial.println(
      "Failed to find QMI8658 - check your wiring!"
    );

    while (1) {
      delay(1000);
    }
  }

  USBSerial.println(
    "QMI8658 initialized successfully"
  );


  // Section: Configure accelerometer

  qmi.configAccelerometer(
    SensorQMI8658::ACC_RANGE_4G,
    SensorQMI8658::ACC_ODR_1000Hz,
    SensorQMI8658::LPF_MODE_0
  );

  qmi.enableAccelerometer();

  // Configuration: The accelerometer uses a ±4 g measurement range with a 1000 Hz output data rate.

  // Reasoning: The selected range provides sufficient measurement capacity for normal device motion while maintaining appropriate sensitivity.


  // Section: Configure gyroscope

  qmi.configGyroscope(
    SensorQMI8658::GYR_RANGE_512DPS,
    SensorQMI8658::GYR_ODR_896_8Hz,
    SensorQMI8658::LPF_MODE_0
  );

  qmi.enableGyroscope();

  // Configuration: The gyroscope uses a ±512 °/s measurement range with an 896.8 Hz output data rate.

  // Reasoning: The selected range provides sufficient angular velocity range for expected motion without unnecessarily reducing measurement sensitivity.

  USBSerial.println(
    "Accelerometer and gyroscope configured"
  );


  // Section: Calibrate gyroscope

  calibrateGyroscope();


  // Section: Initialize LVGL

  lv_init();

  lv_tick_set_cb(
    millis_cb
  );

#if LV_USE_LOG != 0

  lv_log_register_print_cb(
    my_print
  );

#endif


  // Section: Configure display dimensions

  screenWidth =
    gfx->width();

  screenHeight =
    gfx->height();

#ifdef DIRECT_RENDER_MODE

  bufSize =
    screenWidth *
    screenHeight;

#else

  bufSize =
    screenWidth *
    50;

  // Configuration: LVGL uses a 50-row partial rendering buffer.

  // Reasoning: Partial rendering reduces memory usage while providing sufficient performance for the sensor display.

#endif


  // Section: Allocate LVGL display buffer

#ifdef ESP32

#if defined(DIRECT_RENDER_MODE) && \
    (defined(CANVAS) || \
     defined(RGB_PANEL) || \
     defined(DSI_PANEL))

  disp_draw_buf =
    (lv_color_t *)gfx->getFramebuffer();

#else

  disp_draw_buf =
    (lv_color_t *)heap_caps_malloc(
      bufSize * 2,
      MALLOC_CAP_INTERNAL |
      MALLOC_CAP_8BIT
    );

  if (!disp_draw_buf) {

    disp_draw_buf =
      (lv_color_t *)heap_caps_malloc(
        bufSize * 2,
        MALLOC_CAP_8BIT
      );
  }

#endif

#else

  disp_draw_buf =
    (lv_color_t *)malloc(
      bufSize * 2
    );

#endif


  if (!disp_draw_buf) {

    USBSerial.println(
      "LVGL display buffer allocation failed!"
    );

  } else {


    // Section: Create LVGL display

    disp =
      lv_display_create(
        screenWidth,
        screenHeight
      );

    lv_display_set_flush_cb(
      disp,
      my_disp_flush
    );

#ifdef DIRECT_RENDER_MODE

    lv_display_set_buffers(
      disp,
      disp_draw_buf,
      NULL,
      bufSize * 2,
      LV_DISPLAY_RENDER_MODE_DIRECT
    );

#else

    lv_display_set_buffers(
      disp,
      disp_draw_buf,
      NULL,
      bufSize * 2,
      LV_DISPLAY_RENDER_MODE_PARTIAL
    );

#endif


    // Section: Configure touch input

    lv_indev_t *indev =
      lv_indev_create();

    lv_indev_set_type(
      indev,
      LV_INDEV_TYPE_POINTER
    );

    lv_indev_set_read_cb(
      indev,
      my_touchpad_read
    );


    // Section: Configure display rendering

    lv_display_add_event_cb(
      disp,
      rounder_event_cb,
      LV_EVENT_INVALIDATE_AREA,
      NULL
    );


    // Section: Create accelerometer title

    accTitle =
      lv_label_create(
        lv_scr_act()
      );

    lv_label_set_text(
      accTitle,
      "ACCELEROMETER"
    );

    lv_obj_set_style_text_font(
      accTitle,
      &lv_font_montserrat_24,
      0
    );

    lv_obj_align(
      accTitle,
      LV_ALIGN_CENTER,
      0,
      -120
    );


    // Section: Create accelerometer rows

    createAccelRow(
      &accXLabel,
      &accXValue,
      &accXUnit,
      "X",
      "g",
      -80
    );

    createAccelRow(
      &accYLabel,
      &accYValue,
      &accYUnit,
      "Y",
      "g",
      -50
    );

    createAccelRow(
      &accZLabel,
      &accZValue,
      &accZUnit,
      "Z",
      "g",
      -20
    );


    // Section: Create gyroscope title

    gyroTitle =
      lv_label_create(
        lv_scr_act()
      );

    lv_label_set_text(
      gyroTitle,
      "GYROSCOPE"
    );

    lv_obj_set_style_text_font(
      gyroTitle,
      &lv_font_montserrat_24,
      0
    );

    lv_obj_align(
      gyroTitle,
      LV_ALIGN_CENTER,
      0,
      25
    );


    // Section: Create gyroscope rows

    createGyroRow(
      &gyroXLabel,
      &gyroXValue,
      &gyroXUnit,
      "Roll",
      "°/s",
      65
    );

    createGyroRow(
      &gyroYLabel,
      &gyroYValue,
      &gyroYUnit,
      "Pitch",
      "°/s",
      95
    );

    createGyroRow(
      &gyroZLabel,
      &gyroZValue,
      &gyroZUnit,
      "Yaw",
      "°/s",
      125
    );
  }

  USBSerial.println(
    "Setup done"
  );
}


// Function: Continuously updates the LVGL interface and displays filtered IMU measurements.

void loop() {

  // Section: Process LVGL interface

  lv_task_handler();

  delay(5);


  // Section: Read IMU data

  if (qmi.getDataReady()) {


    // Section: Process accelerometer

    if (qmi.getAccelerometer(
          acc.x,
          acc.y,
          acc.z
        )) {


      // Note: The first accelerometer reading initializes the filter to prevent startup ramping.

      if (!accFilterInitialized) {

        filteredAccX = acc.x;
        filteredAccY = acc.y;
        filteredAccZ = acc.z;

        accFilterInitialized = true;

      } else {

        filteredAccX =
          FILTER_ALPHA * acc.x +
          (1.0 - FILTER_ALPHA) *
          filteredAccX;

        filteredAccY =
          FILTER_ALPHA * acc.y +
          (1.0 - FILTER_ALPHA) *
          filteredAccY;

        filteredAccZ =
          FILTER_ALPHA * acc.z +
          (1.0 - FILTER_ALPHA) *
          filteredAccZ;
      }


      // Section: Format accelerometer values

      char accXBuffer[20];
      char accYBuffer[20];
      char accZBuffer[20];

      snprintf(
        accXBuffer,
        sizeof(accXBuffer),
        "%.2f",
        filteredAccX
      );

      snprintf(
        accYBuffer,
        sizeof(accYBuffer),
        "%.2f",
        filteredAccY
      );

      snprintf(
        accZBuffer,
        sizeof(accZBuffer),
        "%.2f",
        filteredAccZ
      );


      lv_label_set_text(
        accXValue,
        accXBuffer
      );

      lv_label_set_text(
        accYValue,
        accYBuffer
      );

      lv_label_set_text(
        accZValue,
        accZBuffer
      );


      // Section: Output accelerometer data

      USBSerial.print(
        "{ACCEL: "
      );

      USBSerial.print(
        filteredAccX
      );

      USBSerial.print(",");

      USBSerial.print(
        filteredAccY
      );

      USBSerial.print(",");

      USBSerial.print(
        filteredAccZ
      );

      USBSerial.println(
        "}"
      );
    }


    // Section: Process gyroscope

    if (qmi.getGyroscope(
          gyr.x,
          gyr.y,
          gyr.z
        )) {


      // Section: Remove gyroscope bias

      float gyroX =
        gyr.x - gyroBiasX;

      float gyroY =
        gyr.y - gyroBiasY;

      float gyroZ =
        gyr.z - gyroBiasZ;

      // Reasoning: Subtracting the calibrated bias reduces the sensor's stationary offset before filtering.


      // Section: Apply gyroscope deadband

      if (fabs(gyroX) < GYRO_DEADBAND) {
        gyroX = 0.0;
      }

      if (fabs(gyroY) < GYRO_DEADBAND) {
        gyroY = 0.0;
      }

      if (fabs(gyroZ) < GYRO_DEADBAND) {
        gyroZ = 0.0;
      }

      // Reasoning: The deadband suppresses small residual measurements that remain after bias calibration.


      // Section: Initialize gyroscope filter

      if (!gyroFilterInitialized) {

        filteredGyrX = gyroX;
        filteredGyrY = gyroY;
        filteredGyrZ = gyroZ;

        gyroFilterInitialized = true;

      } else {

        // Section: Apply low-pass filter

        filteredGyrX =
          FILTER_ALPHA * gyroX +
          (1.0 - FILTER_ALPHA) *
          filteredGyrX;

        filteredGyrY =
          FILTER_ALPHA * gyroY +
          (1.0 - FILTER_ALPHA) *
          filteredGyrY;

        filteredGyrZ =
          FILTER_ALPHA * gyroZ +
          (1.0 - FILTER_ALPHA) *
          filteredGyrZ;
      }


      // Section: Format gyroscope values

      char gyroXBuffer[20];
      char gyroYBuffer[20];
      char gyroZBuffer[20];

      snprintf(
        gyroXBuffer,
        sizeof(gyroXBuffer),
        "%.2f",
        filteredGyrX
      );

      snprintf(
        gyroYBuffer,
        sizeof(gyroYBuffer),
        "%.2f",
        filteredGyrY
      );

      snprintf(
        gyroZBuffer,
        sizeof(gyroZBuffer),
        "%.2f",
        filteredGyrZ
      );


      lv_label_set_text(
        gyroXValue,
        gyroXBuffer
      );

      lv_label_set_text(
        gyroYValue,
        gyroYBuffer
      );

      lv_label_set_text(
        gyroZValue,
        gyroZBuffer
      );


      // Section: Output gyroscope data

      USBSerial.print(
        "{GYRO: "
      );

      USBSerial.print(
        filteredGyrX
      );

      USBSerial.print(",");

      USBSerial.print(
        filteredGyrY
      );

      USBSerial.print(",");

      USBSerial.print(
        filteredGyrZ
      );

      USBSerial.println(
        "}"
      );
    }
  }
}