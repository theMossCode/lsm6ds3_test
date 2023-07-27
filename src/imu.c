#include "imu.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(SENSOR, LOG_LEVEL_DBG);

#define IMU_NODE        DT_NODELABEL(imu0)

const struct device *imu_device = DEVICE_DT_GET(IMU_NODE);

struct sensor_value accl_x, accl_y, accl_z;
struct sensor_value gyro_x, gyro_y, gyro_z;

static int get_accl()
{
    if(sensor_sample_fetch_chan(imu_device, SENSOR_CHAN_ACCEL_XYZ) < 0){
        LOG_WRN("Sensor accl fetch fail");
        return -EIO;
    }

    if(sensor_channel_get(imu_device, SENSOR_CHAN_ACCEL_X, &accl_x) < 0){
        LOG_WRN("get X acceleration fail");
        return -EINVAL;
    }

    if(sensor_channel_get(imu_device, SENSOR_CHAN_ACCEL_Y, &accl_y) < 0){
        LOG_WRN("get Y acceleration fail");
        return -EINVAL;
    }

    if(sensor_channel_get(imu_device, SENSOR_CHAN_ACCEL_Z, &accl_z) < 0){
        LOG_WRN("get Z acceleration fail");
        return -EINVAL;
    }

    return 0;
}

#if (CONFIG_LSM6DS3_TRIGGER)

static void lsm6ds3_trigger_handler(const struct device *dev, const struct sensor_trigger *trig)
{
    if(get_accl() < 0){
        LOG_WRN("Accel data fetch fail");
        return;
    }

    LOG_DBG("Get accel trigger, x:%f y:%f z:%f", sensor_value_to_double(&accl_x), sensor_value_to_double(&accl_y), sensor_value_to_double(&accl_z));
}

#endif

int imu_init(double odr)
{
    if(!(device_is_ready(imu_device))){
        LOG_WRN("IMU device not ready");
        return -ENODEV;
    }

    #if (CONFIG_LSM6DS3_ACCL_ODR == 0)
    struct sensor_value odr_attr;
    sensor_value_from_double(&odr_attr, odr);

    if(sensor_attr_set(imu_device, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0){
        LOG_WRN("Set sampling frequency fail");
        return -EINVAL;
    }
    #else
    ARG_UNUSED(odr);
    #endif

    #if (CONFIG_LSM6DS3_TRIGGER)
   
    struct sensor_trigger trig = {
        .type = SENSOR_TRIG_DATA_READY,
        .chan = SENSOR_CHAN_ACCEL_XYZ
    };

    if(sensor_trigger_set(imu_device, &trig, lsm6ds3_trigger_handler) < 0){
        LOG_WRN("Data ready trigger set fail");
        return -EIO;
    }

    #endif

    return 0;
}

int fetch_imu_accl(float *x, float *y, float *z)
{
    #if (!CONFIG_LSM6DS3_TRIGGER)

    get_accl();

    LOG_DBG("Sample get complete x=%d.%d, y=%d.%d, z=%d.%d", accl_x.val1, accl_x.val2, accl_y.val1, accl_y.val2, accl_z.val1, accl_z.val2);

    #endif

    *x = (float)sensor_value_to_double(&accl_x);
    *y = (float)sensor_value_to_double(&accl_y);
    *z = (float)sensor_value_to_double(&accl_z);

    return 0;
}