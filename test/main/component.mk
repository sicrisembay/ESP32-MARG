COMPONENT_OBJS += marg_test_main.o
COMPONENT_SRCDIRS += .

# Component to test
COMPONENT_OBJS += ../../src/marg.o
COMPONENT_SRCDIRS += ../../src

ifdef CONFIG_MARG_SENSOR_MPU9250
COMPONENT_OBJS += ../../sensor/mpu9250/marg_sensor_mpu9250.o
COMPONENT_SRCDIRS += ../../sensor/mpu9250
endif

