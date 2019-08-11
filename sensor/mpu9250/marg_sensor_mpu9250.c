#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "../marg_sensor.h"
#include "mpu9250_hw.h"
#include "math.h"

#define MPU9250_DEBUG_INIT              (0)
#define MPU9250_DEBUG_DATA              (1)
#define ENABLE_TIMING_TEST              (0)

#define _I2C_NUMBER(num)                I2C_NUM_##num
#define I2C_NUMBER(num)                 _I2C_NUMBER(num)
#define I2C_MASTER_NUM                  I2C_NUMBER(CONFIG_MPU9250_I2C_PORT_NUM)
#define MPU9250_DEV_ADDR                CONFIG_MPU9250_DEV_ADDR
#define WRITE_BIT                       I2C_MASTER_WRITE
#define READ_BIT                        I2C_MASTER_READ
#define ACK_CHECK_EN                    (0x1)
#define ACK_CHECK_DIS                   (0x0)
#define ACK_VAL                         (0x0)
#define NACK_VAL                        (0x1)

#define MPU9250_STACK_SIZE              (2048)

#define MPU9250_NOTIFY_INT_FLAG         (0x00000001)

#if(ENABLE_TIMING_TEST == 1)
#define TIMING_IO_0                     (22)
#define TIMING_IO_1                     (23)
#endif /* #if(ENABLE_TIMING_TEST == 1) */

enum AscaleDef {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};
#if defined(CONFIG_MPU9250_ACCEL_FS_2G)
#define ACCEL_FS_DEF        (AFS_2G)
#define ACCEL_SCALE_DEF     ((float)(2.0f / 32768.0f))
#elif defined(CONFIG_MPU9250_ACCEL_FS_4G)
#define ACCEL_FS_DEF        (AFS_4G)
#define ACCEL_SCALE_DEF     ((float)(4.0f / 32768.0f))
#elif defined(CONFIG__MPU9250_ACCEL_FS_8G)
#define ACCEL_FS_DEF        (AFS_8G)
#define ACCEL_SCALE_DEF     ((float)(8.0f / 32768.0f))
#elif defined(CONFIG_MPU9250_ACCEL_FS_16G)
#define ACCEL_FS_DEF        (AFS_16G)
#define ACCEL_SCALE_DEF     ((float)(16.0f / 32768.0f))
#else
#error "Accelerometer Full-Scale configuration not defined!"
#endif
#define ACCEL_SCALE(x)  (((float)(1 << x)) * 2.0f / 32768.0f)

enum GscaleDef {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

#if defined(CONFIG_MPU9250_GYRO_FS_250DPS)
#define GYRO_FS_DEF         (GFS_250DPS)
#define GYRO_SCALE_DEF      ((float)(250.f / 32768.0f))
#elif defined(CONFIG_MPU9250_GYRO_FS_500DPS)
#define GYRO_FS_DEF         (GFS_500DPS)
#define GYRO_SCALE_DEF      ((float)(500.f / 32768.0f))
#elif defined(CONFIG_MPU9250_GYRO_FS_1000DPS)
#define GYRO_FS_DEF         (GFS_1000DPS)
#define GYRO_SCALE_DEF      ((float)(1000.f / 32768.0f))
#elif defined(CONFIG_MPU9250_GYRO_FS_2000DPS)
#define GYRO_FS_DEF         (GFS_2000DPS)
#define GYRO_SCALE_DEF      ((float)(2000.f / 32768.0f))
#else
#error "Gyrometer Full-Scale configuration not defined!"
#endif
#define GYRO_SCALE(x)   (((float)(1 << x)) * 250.f / 32768.0f)

enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};
#if defined(CONFIG_AK8963_OUTPUT_14BIT)
#define MAG_RES_DEF     (MFS_14BITS)
#define MAG_SCALE_DEF   (10.0f * 4912.0f / 8190.0f)
#elif defined(CONFIG_AK8963_OUTPUT_16BIT)
#define MAG_RES_DEF     (MFS_16BITS)
#define MAG_SCALE_DEF   (10.0f * 4912.0f / 32760.0f)
#else
#error "Magnetormeter output size not defined"
#endif
#define MAG_SCALE(x)    (10.0f * 4912.0f / (8190.0f * (1.0f + ((float)x * 3.0f))))

#if defined(CONFIG_AK8963_SINGLE_MODE)
#define MAG_MODE_DEF    (0x01)
#elif defined(CONFIG_AK8963_CONT_MODE_8HZ)
#define MAG_MODE_DEF    (0x02)
#elif defined(CONFIG_AK8963_CONT_MODE_100HZ)
#define MAG_MODE_DEF    (0x06)
#elif defined(CONFIG_AK8963_EXT_TRIG_MODE)
#define MAG_MODE_DEF    (0x04)
#else
#endif

static const char *TAG = "mpu9250";
static DRAM_ATTR TaskHandle_t mpu9250_task_handle = NULL;
static bool bInit = false;

#ifdef CONFIG_MARG_STRICT_FIX16
#error "TODO!"
#else
static float accelScale = 0.0f;
static float gyroScale = 0.0f;
static float magScale = 0.0f;
static dataXYZ_t gyroBias = {0.0f};
static dataXYZ_t accelBias = {0.0f};
static dataXYZ_t magCal = {0.0f};
static dataXYZ_t magBias = {0.0f};
static dataXYZ_t accelRawData = {0.0};
static dataXYZ_t gyroRawData = {0.0};
static dataXYZ_t magRawData = {0.0};
#endif /* #ifdef CONFIG_MARG_STRICT_FIX16 */

static int _i2c_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = CONFIG_MPU9250_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;       /* external Pull-up resistor is present in board */
    conf.scl_io_num = CONFIG_MPU9250_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;       /* external Pull-up resistor is present in board */
    conf.master.clk_speed = CONFIG_MPU9250_I2C_CLK_FREQ;
    i2c_param_config(i2c_master_port, &conf);
    if(ESP_OK != i2c_driver_install(i2c_master_port, I2C_MODE_MASTER, 0, 0, 0)) {
        return -1;
    }
#if(MPU9250_DEBUG_INIT == 1)
    ESP_LOGI(TAG, "I2C initialized.");
#endif /* #if(MPU9250_DEBUG_INIT == 1) */
    return 0;
}

static int _i2c_write(unsigned char slave_addr,
        unsigned char reg_addr,
        unsigned char length,
        unsigned char *data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if(ESP_OK != i2c_master_start(cmd)) {
    i2c_cmd_link_delete(cmd);
    return -1;
    }
    if(ESP_OK != i2c_master_write_byte(cmd, (slave_addr << 1) | WRITE_BIT, ACK_CHECK_EN)) {
    i2c_cmd_link_delete(cmd);
    return -1;
    }
    if(ESP_OK != i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN)) {
    i2c_cmd_link_delete(cmd);
    return -1;
    }
    if(ESP_OK != i2c_master_write(cmd, data, length, ACK_CHECK_EN)) {
    i2c_cmd_link_delete(cmd);
    return -1;
    }
    if(ESP_OK != i2c_master_stop(cmd)) {
    i2c_cmd_link_delete(cmd);
    return -1;
    }
    if(ESP_OK != i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS)) {
    i2c_cmd_link_delete(cmd);
    return -1;
    }
    i2c_cmd_link_delete(cmd);
    return 0;
}

static int _i2c_write_byte(unsigned char slave_addr,
        unsigned char reg_addr,
        unsigned char data) {
    return(_i2c_write(slave_addr, reg_addr, 1, &data));
}

static int _i2c_read(unsigned char slave_addr,
                   unsigned char reg_addr,
                   unsigned char length,
                   unsigned char *data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if(ESP_OK != i2c_master_start(cmd)) {
        i2c_cmd_link_delete(cmd);
        return -1;
    }
    if(ESP_OK != i2c_master_write_byte(cmd, slave_addr << 1 | WRITE_BIT, ACK_CHECK_EN)) {
        i2c_cmd_link_delete(cmd);
        return -1;
    }
    if(ESP_OK != i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN)) {
        i2c_cmd_link_delete(cmd);
        return -1;
    }
    if(ESP_OK != i2c_master_stop(cmd)) {
        i2c_cmd_link_delete(cmd);
        return -1;
    }
    if(ESP_OK != i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS)) {
        i2c_cmd_link_delete(cmd);
        return -1;
    }
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    if(ESP_OK != i2c_master_start(cmd)) {
        i2c_cmd_link_delete(cmd);
        return -1;
    }
    if(ESP_OK != i2c_master_write_byte(cmd, slave_addr << 1 | READ_BIT, ACK_CHECK_EN)) {
        i2c_cmd_link_delete(cmd);
        return -1;
    }
    if(length > 1) {
        if(ESP_OK != i2c_master_read(cmd, data, length - 1, ACK_VAL)) {
            i2c_cmd_link_delete(cmd);
            return -1;
        }
    }
    if(ESP_OK != i2c_master_read_byte(cmd, data + length - 1, NACK_VAL)) {
        i2c_cmd_link_delete(cmd);
        return -1;
    }
    if(ESP_OK != i2c_master_stop(cmd)) {
        i2c_cmd_link_delete(cmd);
        return -1;
    }
    if(ESP_OK != i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS)) {
        i2c_cmd_link_delete(cmd);
        return -1;
    }
    i2c_cmd_link_delete(cmd);
    return 0;
}

static uint8_t _i2c_read_byte(unsigned char slave_addr, unsigned char reg_addr, esp_err_t *status)
{
    uint8_t retval = 0;
    if(!status) {
        *status = ESP_OK;
    }
    if(!_i2c_read(slave_addr, reg_addr, 1, &retval)) {
        if(!status) {
            *status = ESP_FAIL;
        }
    }
    return (retval);
}

static esp_err_t _ak8963_write_byte(uint8_t reg_addr, uint8_t data)
{
    if(_i2c_write_byte(MPU9250_DEV_ADDR, ADDR_I2C_SLV0_ADDR, AK8963_I2C_ADDR)) return ESP_FAIL;
    if(_i2c_write_byte(MPU9250_DEV_ADDR, ADDR_I2C_SLV0_REG, reg_addr)) return ESP_FAIL;
    if(_i2c_write_byte(MPU9250_DEV_ADDR, ADDR_I2C_SLV0_DO, data)) return ESP_FAIL;
    if(_i2c_write_byte(MPU9250_DEV_ADDR, ADDR_I2C_SLV0_CTRL, 0x81)) return ESP_FAIL;
    vTaskDelay(1);
    return ESP_OK;
}

static esp_err_t _ak8963_read(uint8_t reg_addr, uint8_t length, uint8_t *pData)
{
    uint8_t ctrlVal = 0;
    if((pData == (uint8_t *)0) || (length > 7)) {
        return ESP_ERR_INVALID_ARG;
    }
    if(length == 0) {
        return ESP_OK;
    }
    if(_i2c_write_byte(MPU9250_DEV_ADDR, ADDR_I2C_SLV0_ADDR, AK8963_I2C_ADDR | 0x80)) return ESP_FAIL;
    if(_i2c_write_byte(MPU9250_DEV_ADDR, ADDR_I2C_SLV0_REG, reg_addr)) return ESP_FAIL;
    ctrlVal = 0x80 | length;
    if(_i2c_write_byte(MPU9250_DEV_ADDR, ADDR_I2C_SLV0_CTRL, ctrlVal)) return ESP_FAIL;
    vTaskDelay(1);
    if(_i2c_read(MPU9250_DEV_ADDR, ADDR_EXT_SENS_DATA_00, length, pData)) return ESP_FAIL;

    return ESP_OK;
}
static uint8_t _ak8963_read_byte(uint8_t reg_addr, esp_err_t * status)
{
    uint8_t retval = 0;
    if(status) {
        *status = ESP_OK;
    }
    if(_i2c_write_byte(MPU9250_DEV_ADDR, ADDR_I2C_SLV0_ADDR, AK8963_I2C_ADDR | 0x80)) {
        if(status) {
            *status = ESP_FAIL;
            return 0;
        }
    }
    if(_i2c_write_byte(MPU9250_DEV_ADDR, ADDR_I2C_SLV0_REG, reg_addr)) {
        if(status) {
            *status = ESP_FAIL;
            return 0;
        }
    }
    if(_i2c_write_byte(MPU9250_DEV_ADDR, ADDR_I2C_SLV0_CTRL, 0x81)) {
        if(status) {
            *status = ESP_FAIL;
            return 0;
        }
    }
    vTaskDelay(1);
    retval = _i2c_read_byte(MPU9250_DEV_ADDR, ADDR_EXT_SENS_DATA_00, status);
    return(retval);

}

static esp_err_t _mpu9250_run_selfTest(dataXYZ_t *pAccel, dataXYZ_t *pGyro)
{
#ifdef CONFIG_MARG_STRICT_FIX16
    ESP_LOGE(TAG, "Fix16 calculation not yet supported!");
#else
    uint8_t rawData[14] = {0};
    uint8_t selfTest[6] = {0};
    int32_t gyroAvg[3] = {0};
    int32_t accAvg[3] = {0};
    int32_t gyroSTAvg[3] = {0};
    int32_t accSTAvg[3] = {0};
    float factoryTrim[6] = {0.0f};
    TickType_t xLastWakeTime = 0;
    int32_t ii;
    const uint16_t N_SAMPLES = 200;

    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_SMPLRT_DIV, 0x00);   // Set gyro sample rate to 1 kHz
    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_CONFIG, 0x02);       // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_GYRO_CONFIG, 1);     // Set full scale range for the gyro to 250 dps
    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_ACCEL_CONFIG_2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_ACCEL_CONFIG, 1);    // Set full scale range for the accelerometer to 2 g

    xLastWakeTime = xTaskGetTickCount();
    // get average of N_SAMPLES
    for(ii = 0; ii < N_SAMPLES; ii++) {
        vTaskDelayUntil(&xLastWakeTime, 1);
        _i2c_read(MPU9250_DEV_ADDR, ADDR_ACCEL_XOUT_H, 14, rawData);
        accAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
        accAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        accAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
        // Temperature rawData[6] and rawData[7]
        gyroAvg[0] += (int16_t)(((int16_t)rawData[8] << 8) | rawData[9]);
        gyroAvg[1] += (int16_t)(((int16_t)rawData[10] << 8) | rawData[11]);
        gyroAvg[2] += (int16_t)(((int16_t)rawData[12] << 8) | rawData[13]);
    }
    for(ii = 0; ii < 3; ii++) {
        accAvg[ii] /= N_SAMPLES;
        gyroAvg[ii] /= N_SAMPLES;
    }
    /* Enable Self Test and get average of N_SAMPLES */
    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_GYRO_CONFIG, 0xE0);
    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_ACCEL_CONFIG, 0xE0);
    vTaskDelay(25);

    xLastWakeTime = xTaskGetTickCount();
    for(ii = 0; ii < N_SAMPLES; ii++) {
        vTaskDelayUntil(&xLastWakeTime, 1);
        _i2c_read(MPU9250_DEV_ADDR, ADDR_ACCEL_XOUT_H, 14, rawData);
        accSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
        accSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        accSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
        // Temperature rawData[6] and rawData[7]
        gyroSTAvg[0] += (int16_t)(((int16_t)rawData[8] << 8) | rawData[9]);
        gyroSTAvg[1] += (int16_t)(((int16_t)rawData[10] << 8) | rawData[11]);
        gyroSTAvg[2] += (int16_t)(((int16_t)rawData[12] << 8) | rawData[13]);
    }
    for(ii = 0; ii < 3; ii++) {
        accSTAvg[ii] /= N_SAMPLES;
        gyroSTAvg[ii] /= N_SAMPLES;
    }
    /* Configure the gyro and accelerometer for normal operation */
    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_GYRO_CONFIG, 0x00);
    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_ACCEL_CONFIG, 0x00);
    vTaskDelay(25);

    /* Retrieve accelerometer and gyro factory Self-Test Code */
    _i2c_read(MPU9250_DEV_ADDR, ADDR_SELF_TEST_X_ACCEL, 3, &selfTest[0]);
    _i2c_read(MPU9250_DEV_ADDR, ADDR_SELF_TEST_X_GYRO, 3, &selfTest[3]);
    for(ii = 0; ii < 6; ii++) {
        factoryTrim[ii] = (float)(2620.0f * powf(1.01f, ((float)selfTest[ii] - 1.0f)));
    }
    /* Calculate factory self-test value from self-test code */
    pAccel->x = (100.0f * ((float)(accSTAvg[0] - accAvg[0])) / factoryTrim[0]) - 100.0f;
    pAccel->y = (100.0f * ((float)(accSTAvg[1] - accAvg[1])) / factoryTrim[1]) - 100.0f;
    pAccel->z = (100.0f * ((float)(accSTAvg[2] - accAvg[2])) / factoryTrim[2]) - 100.0f;
    pGyro->x = (100.0f * ((float)(gyroSTAvg[0] - gyroAvg[0])) / factoryTrim[3]) - 100.0f;
    pGyro->y = (100.0f * ((float)(gyroSTAvg[1] - gyroAvg[1])) / factoryTrim[4]) - 100.0f;
    pGyro->z = (100.0f * ((float)(gyroSTAvg[2] - gyroAvg[2])) / factoryTrim[5]) - 100.0f;

#endif /* #ifdef CONFIG_MARG_STRICT_FIX16 */
    return ESP_OK;
}

static esp_err_t _mpu9250_calGyroAccel(dataXYZ_t *pGyroBias, dataXYZ_t *pAccelBias)
{
    uint8_t data[12] = {0}; /* data array to hold accelerometer and gyro x, y, z, data */
    uint16_t ii = 0;
    uint16_t packet_count = 0;
    uint16_t fifo_count = 0;
    int32_t gyro_bias[3] = {0};
    int32_t accel_bias[3] = {0};
    const uint16_t  gyrosensitivity  = 131;    // = 131 LSB/degrees/sec
    const uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g


    /* reset device */
    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_PWR_MGMT_1, 0x80);
    vTaskDelay(100);

    /* get stable time source; Auto select clock source to be PLL gyroscope reference if ready
       else use the internal oscillator, bits 2:0 = 001 */
    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_PWR_MGMT_1, 0x01);
    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_PWR_MGMT_2, 0x00);
    vTaskDelay(200);

    /* Configure device for bias calculation */
    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_INT_ENABLE, 0x00);   // Disable all interrupts
    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_FIFO_EN, 0x00);      // Disable FIFO
    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_PWR_MGMT_1, 0x00);   // Turn on internal clock source
    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_I2C_MST_CTRL, 0x00); // Disable I2C master
    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_USER_CTRL, 0x0C);    // Reset FIFO and DMP
    vTaskDelay(15);
    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

    /* Configure FIFO to capture accelerometer and gyro data for bias calculation */
    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_USER_CTRL, 0x40);   // Enable FIFO
    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
    vTaskDelay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes
    /* At end of sample accumulation, turn off FIFO sensor read */
    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
    _i2c_read(MPU9250_DEV_ADDR, ADDR_FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count/12;   // How many sets of full gyro and accelerometer data for averaging
#if(MPU9250_DEBUG_INIT == 1)
    ESP_LOGI(TAG, "%s: packet_count = %d", __func__, packet_count);
#endif /* #if(MPU9250_DEBUG_INIT == 1) */
    for (ii = 0; ii < packet_count; ii++) {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        _i2c_read(MPU9250_DEV_ADDR, ADDR_FIFO_R_W, 12, &data[0]); // read data for averaging
        accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
        accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
        gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
        gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
        gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

        accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0]  += (int32_t) gyro_temp[0];
        gyro_bias[1]  += (int32_t) gyro_temp[1];
        gyro_bias[2]  += (int32_t) gyro_temp[2];
    }
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
    if(accel_bias[2] > 0L) {
        accel_bias[2] -= (int32_t) accelsensitivity;   // Remove gravity from the z-axis accelerometer bias calculation
    } else {
        accel_bias[2] += (int32_t) accelsensitivity;
    }

    /* Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup */
    data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1]/4)       & 0xFF;
    data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2]/4)       & 0xFF;

    /* Push gyro biases to hardware registers */
    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_XG_OFFSET_H, data[0]);
    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_XG_OFFSET_L, data[1]);
    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_YG_OFFSET_H, data[2]);
    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_YG_OFFSET_L, data[3]);
    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_ZG_OFFSET_H, data[4]);
    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_ZG_OFFSET_L, data[5]);

    pGyroBias->x = (float)gyro_bias[0] / (float)gyrosensitivity;
    pGyroBias->y = (float)gyro_bias[1] / (float)gyrosensitivity;
    pGyroBias->z = (float)gyro_bias[2] / (float)gyrosensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.
    int32_t accel_bias_reg[3] = {0}; // A place to hold the factory accelerometer trim biases
    _i2c_read(MPU9250_DEV_ADDR, ADDR_XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    _i2c_read(MPU9250_DEV_ADDR, ADDR_YA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    _i2c_read(MPU9250_DEV_ADDR, ADDR_ZA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

    uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t mask_bit[3] = {0}; // Define array to hold mask bit for each accelerometer bias axis
    for(ii = 0; ii < 3; ii++) {
        if((accel_bias_reg[ii] & mask)) {
            mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
        }
    }
    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1]/8);
    accel_bias_reg[2] -= (accel_bias[2]/8);

    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0])      & 0xFF;
    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1])      & 0xFF;
    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2])      & 0xFF;
    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

//    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_XA_OFFSET_H, data[0]);
//    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_XA_OFFSET_L, data[1]);
//    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_YA_OFFSET_H, data[2]);
//    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_YA_OFFSET_L, data[3]);
//    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_ZA_OFFSET_H, data[4]);
//    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_ZA_OFFSET_L, data[5]);

    pAccelBias->x = (float)accel_bias[0] / (float)accelsensitivity;
    pAccelBias->y = (float)accel_bias[1] / (float)accelsensitivity;
    pAccelBias->z = (float)accel_bias[2] / (float)accelsensitivity;

    return ESP_OK;
}

static esp_err_t _mpu9250_check_id(void)
{
    uint8_t id = 0;
    esp_err_t retval = ESP_OK;
    id = _i2c_read_byte(MPU9250_DEV_ADDR, ADDR_WHO_AM_I, &retval);
    if(retval) {
        ESP_LOGE(TAG, "I2C error");
        return (retval);
    }
    if((id != MPU9250_WHOIAM_VAL) && (id != MPU9255_WHOIAM_VAL)) {
        ESP_LOGE(TAG, "Device not found! ID:0x%0X", id);
        return ESP_FAIL;
    }
#if(MPU9250_DEBUG_INIT == 1)
    if(id == MPU9250_WHOIAM_VAL) {
        ESP_LOGI(TAG, "MPU9250 found.");
    } else if(id == MPU9255_WHOIAM_VAL) {
        ESP_LOGI(TAG, "MPU9255 found.");
    }
#endif /* #if(MPU9250_DEBUG_INIT == 1) */
    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_INT_PIN_CFG, 0x02); // Bypass MPU9250 master and directly access AK8963
    id = _i2c_read_byte(AK8963_I2C_ADDR, AK8963_REG_WIA, &retval);
    if(retval) {
        ESP_LOGE(TAG, "I2C error");
        return (retval);
    }
#if(MPU9250_DEBUG_INIT == 1)
    if(id == AK8963_WHOIAM_VAL) {
        ESP_LOGI(TAG, "AK8963 found in BY-PASS mode.");
    } else {
        ESP_LOGE(TAG, "AK8963 not found! ID: 0x%02X", id);
        return ESP_FAIL;
    }
#endif /* #if(MPU9250_DEBUG_INIT == 1) */
    return ESP_OK;
}

static esp_err_t _mpu9250_init(void)
{
    int i = 0;
    const uint8_t const init_conf[][3] = {
            /* Register, Value, Delay_ms */
            {ADDR_PWR_MGMT_1,   0x00, 100},  /* Wake up */
            {ADDR_PWR_MGMT_1,   0x01, 200},  /* clock source */
            {ADDR_CONFIG,       0x03,   0},
            {ADDR_SMPLRT_DIV,   0x04,   0},
            {ADDR_GYRO_CONFIG,  (GYRO_FS_DEF << 3), 0},
            {ADDR_ACCEL_CONFIG, (ACCEL_FS_DEF << 3), 0},
            {ADDR_ACCEL_CONFIG_2, 0x03, 0},
            {ADDR_INT_PIN_CFG,  0xD2,   0},  /* Interrupt Pin config: Active Low, Open Drain, bypass enabled */
            {ADDR_INT_ENABLE,   0x01, 100},  /* Data Ready */
            {ADDR_I2C_MST_CTRL, 0x1D,   0},  /* I2C multi-master / 400kHz and add Stop bit between read */
            {ADDR_USER_CTRL,    0x20,   0},  /* I2C master mode */
//            {ADDR_I2C_MST_DELAY_CTRL, 0x81, 0},
//            {ADDR_I2C_SLV4_CTRL, 0x01,  0},
            {0xff,              0xff, 0xff}   /* Watermark for end of entry */
        };
    for(i = 0; init_conf[i][0] != 0xFF; i++) {
        if(_i2c_write_byte(MPU9250_DEV_ADDR, init_conf[i][0], init_conf[i][1])) {
            ESP_LOGE(TAG, "Initialization failed!");
            return ESP_FAIL;
        }
        vTaskDelay(init_conf[i][2] / portTICK_RATE_MS);
    }
    return ESP_OK;
}

static esp_err_t _mpu9250_fetchData(dataXYZ_t *pGyroData, dataXYZ_t *pAccelData, dataXYZ_t *pMagData)
{
    uint8_t rawData[21];  /* 6(Acc) + 2(Temp) + 6(Gyro) + 7(Mag + Sts2) */
    if((pGyroData == NULL) || (pAccelData == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }
    if(_i2c_read(MPU9250_DEV_ADDR, ADDR_ACCEL_XOUT_H, 21, rawData)) {
        return ESP_FAIL;
    }
    pAccelData->x = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
    pAccelData->y = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
    pAccelData->z = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
    // Temperature rawData[6] and rawData[7]
    pGyroData->x = (int16_t)(((int16_t)rawData[8] << 8) | rawData[9]);
    pGyroData->y = (int16_t)(((int16_t)rawData[10] << 8) | rawData[11]);
    pGyroData->z = (int16_t)(((int16_t)rawData[12] << 8) | rawData[13]);

    if(!(rawData[20] & 0x08)) {
        /* Overflow flag is clear */
        pMagData->x = (float)(((int16_t)rawData[15] << 8) | rawData[14]);
        pMagData->y = (float)(((int16_t)rawData[17] << 8) | rawData[16]);
        pMagData->z = (float)(((int16_t)rawData[19] << 8) | rawData[18]);
    }
    return ESP_OK;
}

static esp_err_t _ak8963_init(dataXYZ_t *pMagBias)
{
    uint8_t rawData[3] = {0};
    /* Reset AK8963 */
    if(_ak8963_write_byte(AK8963_REG_CNTL2, 0x01)) return ESP_FAIL;
    vTaskDelay(50);
    /* Power Down */
    if(_ak8963_write_byte(AK8963_REG_CNTL1, 0x00)) return ESP_FAIL;
    vTaskDelay(50);
    /* Enter Fuze Mode */
    if(_ak8963_write_byte(AK8963_REG_CNTL1, 0x0F)) return ESP_FAIL;
    /* Read Sensitivity */
    if(_ak8963_read(AK8963_REG_ASAX, 3, &rawData[0])) return ESP_FAIL;
    pMagBias->x = (float)(rawData[0] - 128) / 256.0f + 1.0f;
    pMagBias->y = (float)(rawData[1] - 128) / 256.0f + 1.0f;
    pMagBias->z = (float)(rawData[2] - 128) / 256.0f + 1.0f;
    /* Power Down */
    if(_ak8963_write_byte(AK8963_REG_CNTL1, 0x00)) return ESP_FAIL;
    if(_ak8963_write_byte(AK8963_REG_CNTL1, (MAG_RES_DEF << 4) | MAG_MODE_DEF)) return ESP_FAIL;
    return ESP_OK;
}

static void IRAM_ATTR _mpu9250_int_isr_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken;
    if((uint32_t) arg == CONFIG_MPU9250_INT_IO) {
#if(ENABLE_TIMING_TEST == 1)
        gpio_set_level(TIMING_IO_0, 0);
#endif /* ENABLE_TIMING_TEST */
        xTaskNotifyFromISR(mpu9250_task_handle, MPU9250_NOTIFY_INT_FLAG, eSetBits, &xHigherPriorityTaskWoken);
        if(xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
}

static void _mpu9250_task(void *pArg)
{
    uint8_t tempVar = 0;
    esp_err_t status = ESP_OK;
    uint32_t intStatusFlag = 0;

    /* reset */
    _i2c_write_byte(MPU9250_DEV_ADDR, ADDR_PWR_MGMT_1, 0x80);
    vTaskDelay(100);

    /* Check MPU9250 Device ID */
    if(ESP_OK != _mpu9250_check_id()) {
        vTaskDelete(NULL);
    }

    /***** Sensor self-check *************************************************/
#ifdef CONFIG_MPU9250_SELFTEST_AUTORUN
    dataXYZ_t accelSelfTest;
    dataXYZ_t gyroSelfTest;
    _mpu9250_run_selfTest(&accelSelfTest, &gyroSelfTest);
#if(MPU9250_DEBUG_INIT == 1)
    ESP_LOGI(TAG, "Accel Trim delta (%f%%, %f%%, %f%%)", accelSelfTest.x, accelSelfTest.y, accelSelfTest.z);
    ESP_LOGI(TAG, "Gyro Trim delta (%f%%, %f%%, %f%%)", gyroSelfTest.x, gyroSelfTest.y, gyroSelfTest.z);
#endif /* #if(MPU9250_DEBUG_INIT == 1) */
#endif

    /* Initialize Scale */
    accelScale = ACCEL_SCALE_DEF;
    gyroScale = GYRO_SCALE_DEF;
    magScale = MAG_SCALE_DEF;

    /* Calibrate gyro and accelerometer */
    _mpu9250_calGyroAccel(&gyroBias, &accelBias);
#if(MPU9250_DEBUG_INIT == 1)
    ESP_LOGI(TAG, "Accel Bias (%f, %f, %f)", accelBias.x, accelBias.y, accelBias.z);
    ESP_LOGI(TAG, "Gyro Bias (%f, %f, %f)", gyroBias.x, gyroBias.y, gyroBias.z);
#endif /* #if(MPU9250_DEBUG_INIT == 1) */

    /* Init MPU9250 Device */
    if(ESP_OK != _mpu9250_init()) {
        vTaskDelete(NULL);
    }

    /* Sanity Check */
    tempVar = _ak8963_read_byte(AK8963_REG_WIA, &status);
    if(status) {
        ESP_LOGE(TAG, "AK8963 i2c error");
    }
    if(AK8963_WHOIAM_VAL == tempVar) {
        ESP_LOGI(TAG, "AK8963 found.");
    } else {
        ESP_LOGE(TAG, "AK8963 not found! ID: 0x%02X", tempVar);
        vTaskDelete(NULL);
    }

    /* Init AK8963 Device */
    if(ESP_OK != _ak8963_init(&magBias)) {
        ESP_LOGE(TAG, "AK8963 init failed!");
        vTaskDelete(NULL);
    }

    /* Calibrate Magnetometer */
    /// TODO:

    /* FINALLY, setup external sensor data read */
    const uint8_t const extSens0Cfg[3] = {
            AK8963_I2C_ADDR | 0x80,
            AK8963_REG_HX_LH,
            0x87
    };
    _i2c_write(MPU9250_DEV_ADDR, ADDR_I2C_SLV0_ADDR, 3, extSens0Cfg);

    /* Increase I2C Bus Speed */
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = CONFIG_MPU9250_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;       /* external Pull-up resistor is present in board */
    conf.scl_io_num = CONFIG_MPU9250_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;       /* external Pull-up resistor is present in board */
    conf.master.clk_speed = 400000;
    i2c_param_config(I2C_MASTER_NUM, &conf);

    /* install gpio isr service */
    gpio_set_intr_type(CONFIG_MPU9250_INT_IO, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(CONFIG_MPU9250_INT_IO, _mpu9250_int_isr_handler, (void*)CONFIG_MPU9250_INT_IO);

    // test
    tempVar = 0;
    while(1) {
        xTaskNotifyWait(0x00, ULONG_MAX, &intStatusFlag, portMAX_DELAY);
#if(ENABLE_TIMING_TEST == 1)
        gpio_set_level(TIMING_IO_0, 1);
#endif /* ENABLE_TIMING_TEST */
        if(intStatusFlag & MPU9250_NOTIFY_INT_FLAG) {
#if(ENABLE_TIMING_TEST == 1)
            gpio_set_level(TIMING_IO_1, 0);
#endif /* ENABLE_TIMING_TEST */
            /* Read Sensor Data */
            _mpu9250_fetchData(&gyroRawData, &accelRawData, &magRawData);
#if(MPU9250_DEBUG_DATA == 1)
            // test -->
            tempVar++;
            if((tempVar % 20) == 0) {
                ESP_LOGI(TAG, "Accel (%f, %f, %f), Gyro (%f, %f, %f), Mag (%f, %f, %f)",
                        accelRawData.x, accelRawData.y, accelRawData.z,
                        gyroRawData.x, gyroRawData.y, gyroRawData.z,
                        magRawData.x, magRawData.y, magRawData.z);
            }
            // <-- test
#endif /* MPU9250_DEBUG_INIT */
#if(ENABLE_TIMING_TEST == 1)
            gpio_set_level(TIMING_IO_1, 1);
#endif
        }
    }
}

int marg_sensor_init(void)
{
    gpio_config_t io_conf;

    /* initialize i2c interface */
    if(_i2c_init()) {
#if(MPU9250_DEBUG_INIT == 1)
        ESP_LOGE(TAG, "I2C init failed!");
#endif /* #if(MPU9250_DEBUG_INIT == 1) */
        return -1;
    }

    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << CONFIG_MPU9250_INT_IO);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    if(ESP_OK != gpio_config(&io_conf)) {
        return -1;
    }

#if (ENABLE_TIMING_TEST == 1)
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = ((1ULL << TIMING_IO_0) | (1ULL << TIMING_IO_1));
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);
#endif /* ENABLE_TIMING_TEST */

#if defined(CONFIG_MARG_PINNED_TO_CORE_0)
    if(pdPASS != xTaskCreatePinnedToCore(
                _mpu9250_task,            /* the task function */
                "mpu9250",                /* the name of the task */
                MPU9250_STACK_SIZE,                     /* stack size */
                NULL,                     /* the 'pvParameters' parameter */
                CONFIG_MARG_TASK_PRIORITY,/* FreeRTOS priority */
                &mpu9250_task_handle,     /* task handle */
                PRO_CPU_NUM)) {
#elif defined(CONFIG_MARG_PINNED_TO_CORE_1)
    if(pdPASS != xTaskCreatePinnedToCore(
                _mpu9250_task,            /* the task function */
                "mpu9250",                /* the name of the task */
                MPU9250_STACK_SIZE,       /* stack size */
                NULL,                     /* the 'pvParameters' parameter */
                CONFIG_MARG_TASK_PRIORITY,/* FreeRTOS priority */
                &mpu9250_task_handle,     /* task handle */
                APP_CPU_NUM)) {
#else
    if(pdPASS != xTaskCreate(
                _mpu9250_task,            /* the task function */
                "mpu9250",                /* the name of the task */
                MPU9250_STACK_SIZE,       /* stack size */
                NULL,                     /* the 'pvParameters' parameter */
                CONFIG_MARG_TASK_PRIORITY,/* FreeRTOS priority */
                &mpu9250_task_handle)) {
#endif
#if(MPU9250_DEBUG_INIT == 1)
        ESP_LOGE(TAG, "Task creation failed!");
#endif /* #if(MPU9250_DEBUG_INIT == 1) */
        return -1;
    }

    return 0;
}

int marg_sensor_enable(void)
{
    return 0;
}
