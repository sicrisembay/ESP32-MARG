#ifndef _MPU_9250_HW_H_
#define _MPU_9250_HW_H_

/* MPU9250 Register Address Definition */
#define ADDR_SELF_TEST_X_GYRO       (0x00)
#define ADDR_SELF_TEST_Y_GYRO       (0x01)
#define ADDR_SELF_TEST_Z_GYRO       (0x02)
#define ADDR_SELF_TEST_X_ACCEL      (0x0D)
#define ADDR_SELF_TEST_Y_ACCEL      (0x0E)

#define ADDR_SELF_TEST_Z_ACCEL      (0x0F)
#define ADDR_XG_OFFSET_H            (0x13)
#define ADDR_XG_OFFSET_L            (0x14)
#define ADDR_YG_OFFSET_H            (0x15)
#define ADDR_YG_OFFSET_L            (0x16)
#define ADDR_ZG_OFFSET_H            (0x17)
#define ADDR_ZG_OFFSET_L            (0x18)
#define ADDR_SMPLRT_DIV             (0x19)
#define ADDR_CONFIG                 (0x1A)
#define ADDR_GYRO_CONFIG            (0x1B)
#define ADDR_ACCEL_CONFIG           (0x1C)
#define ADDR_ACCEL_CONFIG_2         (0x1D)
#define ADDR_LP_ACCEL_ODR           (0x1E)
#define ADDR_WOM_THR                (0x1F)

#define ADDR_FIFO_EN                (0x23)
#define ADDR_I2C_MST_CTRL           (0x24)
#define ADDR_I2C_SLV0_ADDR          (0x25)
#define ADDR_I2C_SLV0_REG           (0x26)
#define ADDR_I2C_SLV0_CTRL          (0x27)
#define ADDR_I2C_SLV1_ADDR          (0x28)
#define ADDR_I2C_SLV1_REG           (0x29)
#define ADDR_I2C_SLV1_CTRL          (0x2A)
#define ADDR_I2C_SLV2_ADDR          (0x2B)
#define ADDR_I2C_SLV2_REG           (0x2C)
#define ADDR_I2C_SLV2_CTRL          (0x2D)
#define ADDR_I2C_SLV3_ADDR          (0x2E)
#define ADDR_I2C_SLV3_REG           (0x2F)

#define ADDR_I2C_SLV3_CTRL          (0x30)
#define ADDR_I2C_SLV4_ADDR          (0x31)
#define ADDR_2C_SLV4_REG            (0x32)
#define ADDR_2C_SLV4_DO             (0x33)
#define ADDR_I2C_SLV4_CTRL          (0x34)
#define ADDR_I2C_SLV4_DI            (0x35)
#define ADDR_I2C_MST_STATUS         (0x36)
#define ADDR_INT_PIN_CFG            (0x37)
#define ADDR_INT_ENABLE             (0x38)
#define ADDR_INT_STATUS             (0x3A)
#define ADDR_ACCEL_XOUT_H           (0x3B)
#define ADDR_ACCEL_XOUT_L           (0x3C)
#define ADDR_ACCEL_YOUT_H           (0x3D)
#define ADDR_ACCEL_YOUT_L           (0x3E)
#define ADDR_ACCEL_ZOUT_H           (0x3F)

#define ADDR_ACCEL_ZOUT_L           (0x40)
#define ADDR_TEMP_OUT_H             (0x41)
#define ADDR_TEMP_OUT_L             (0x42)
#define ADDR_GYRO_XOUT_H            (0x43)
#define ADDR_GYRO_XOUT_L            (0x44)
#define ADDR_GYRO_YOUT_H            (0x45)
#define ADDR_GYRO_YOUT_L            (0x46)
#define ADDR_GYRO_ZOUT_H            (0x47)
#define ADDR_GYRO_ZOUT_L            (0x48)
#define ADDR_EXT_SENS_DATA_00       (0x49)
#define ADDR_EXT_SENS_DATA_01       (0x4A)
#define ADDR_EXT_SENS_DATA_02       (0x4B)
#define ADDR_EXT_SENS_DATA_03       (0x4C)
#define ADDR_EXT_SENS_DATA_04       (0x4D)
#define ADDR_EXT_SENS_DATA_05       (0x4E)
#define ADDR_EXT_SENS_DATA_06       (0x4F)

#define ADDR_EXT_SENS_DATA_07       (0x50)
#define ADDR_EXT_SENS_DATA_08       (0x51)
#define ADDR_EXT_SENS_DATA_09       (0x52)
#define ADDR_EXT_SENS_DATA_10       (0x53)
#define ADDR_EXT_SENS_DATA_11       (0x54)
#define ADDR_EXT_SENS_DATA_12       (0x55)
#define ADDR_EXT_SENS_DATA_13       (0x56)
#define ADDR_EXT_SENS_DATA_14       (0x57)
#define ADDR_EXT_SENS_DATA_15       (0x58)
#define ADDR_EXT_SENS_DATA_16       (0x59)
#define ADDR_EXT_SENS_DATA_17       (0x5A)
#define ADDR_EXT_SENS_DATA_18       (0x5B)
#define ADDR_EXT_SENS_DATA_19       (0x5C)
#define ADDR_EXT_SENS_DATA_20       (0x5D)
#define ADDR_EXT_SENS_DATA_21       (0x5E)
#define ADDR_EXT_SENS_DATA_22       (0x5F)

#define ADDR_EXT_SENS_DATA_23       (0x60)
#define ADDR_I2C_SLV0_DO            (0x63)
#define ADDR_I2C_SLV1_DO            (0x64)
#define ADDR_I2C_SLV2_DO            (0x65)
#define ADDR_I2C_SLV3_DO            (0x66)
#define ADDR_I2C_MST_DELAY_CTRL     (0x67)
#define ADDR_SIGNAL_PATH_RESET      (0x68)
#define ADDR_MOT_DETECT_CTRL        (0x69)
#define ADDR_USER_CTRL              (0x6A)
#define ADDR_PWR_MGMT_1             (0x6B)
#define ADDR_PWR_MGMT_2             (0x6C)

#define ADDR_FIFO_COUNTH            (0x72)
#define ADDR_FIFO_COUNTL            (0x73)
#define ADDR_FIFO_R_W               (0x74)
#define ADDR_WHO_AM_I               (0x75)
#define ADDR_XA_OFFSET_H            (0x77)
#define ADDR_XA_OFFSET_L            (0x78)
#define ADDR_YA_OFFSET_H            (0x7A)
#define ADDR_YA_OFFSET_L            (0x7B)
#define ADDR_ZA_OFFSET_H            (0x7D)
#define ADDR_ZA_OFFSET_L            (0x7E)
/* End: MPU9250 Register Address Definition */

#define MPU9250_WHOIAM_VAL          (0x71)
#define MPU9255_WHOIAM_VAL          (0x73)

typedef union {
    uint8_t reg;
    struct field {
        uint8_t reserved0 : 1;
        uint8_t fifo_mode : 1;
        uint8_t ext_sync_set : 3;
        uint8_t dlpf_cfg : 3;
    };
} mpu_config_t;

/**
 * AK8963 (Included in MPU-9250 package.)
 */
#define AK8963_WHOIAM_VAL                       (0x48)
#define AK8963_I2C_ADDR                         (0x0c)
/* Registers. */
#define AK8963_REG_WIA                          (0x00)
#define AK8963_REG_INFO                         (0x01)
#define AK8963_REG_ST1                          (0x02)
#define AK8963_REG_HX_LH                        (0x03)  //2 Bytes
#define AK8963_REG_HY_LH                        (0x05)  //2 Bytes
#define AK8963_REG_HZ_LH                        (0x07)  //2 Bytes
#define AK8963_REG_ST2                          (0x09)
#define AK8963_REG_CNTL1                        (0x0A)
#define AK8963_REG_CNTL2                        (0x0B)
#define AK8963_REG_ASTC                         (0x0C)
/* --- */
#define AK8963_REG_ASAX                         (0x10)
#define AK8963_REG_ASAY                         (0x11)
#define AK8963_REG_ASAZ                         (0x12)

#endif // End _MPU_9250_HW_H_
