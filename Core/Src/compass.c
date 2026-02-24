#include "compass.h"
#include "attitude.h"
#include <math.h>
#include <string.h>

#define COMPASS_I2C_TIMEOUT_MS      5u
#define COMPASS_QMC5883L_ADDR_7BIT  0x0Du
#define COMPASS_QMC5883L_ADDR_8BIT  (COMPASS_QMC5883L_ADDR_7BIT << 1)

#define QMC5883_REG_X_LSB           0x00u
#define QMC5883_REG_STATUS          0x06u
#define QMC5883_REG_CTRL1           0x09u
#define QMC5883_REG_CTRL2           0x0Au
#define QMC5883_REG_SET_RESET       0x0Bu
#define QMC5883_REG_CHIP_ID         0x0Du

#define QMC5883_CTRL1_200HZ_8G_CONT 0x1Du
#define QMC5883_CTRL2_SOFT_RESET    0x80u

#define COMPASS_MAHA_MIN_SAMPLES    30u
#define COMPASS_MAHA_VAR_EPS        1.0f
#define DEG_PER_RAD                 57.2957795f
#define RAD_PER_DEG                 0.0174532925f

static I2C_HandleTypeDef *compass_i2c = NULL;
static uint8_t compass_addr_8bit = COMPASS_QMC5883L_ADDR_8BIT;
static bool compass_initialized = false;
static uint8_t compass_chip_id = 0u;

static int16_t last_raw_mx = 0;
static int16_t last_raw_my = 0;
static int16_t last_raw_mz = 0;

static float cal_soft_x = 1.0f;
static float cal_soft_y = 1.0f;
static float cal_soft_z = 1.0f;
static float cal_hard_x = 0.0f;
static float cal_hard_y = 0.0f;
static float cal_hard_z = 0.0f;

static uint32_t compass_last_read_ms = 0u;
static uint32_t compass_read_ok_count = 0u;
static uint32_t compass_read_fail_count = 0u;
static HAL_StatusTypeDef compass_last_hal_status = HAL_OK;

static float maha_mean[3] = {0.0f, 0.0f, 0.0f};
static float maha_m2[3] = {0.0f, 0.0f, 0.0f};
static uint32_t maha_count = 0u;
static const uint8_t compass_candidate_addr_7bit[] = {0x0Du, 0x2Cu, 0x1Eu, 0x0Cu, 0x0Eu};

static float soft_or_identity(float v)
{
    return (fabsf(v) < 0.0001f) ? 1.0f : v;
}

static bool read_reg(uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (compass_i2c == NULL || buf == NULL || len == 0u) {
        compass_last_hal_status = HAL_ERROR;
        return false;
    }

    compass_last_hal_status = HAL_I2C_Mem_Read(compass_i2c,
                                               compass_addr_8bit,
                                               reg,
                                               I2C_MEMADD_SIZE_8BIT,
                                               buf,
                                               len,
                                               COMPASS_I2C_TIMEOUT_MS);
    if (compass_last_hal_status != HAL_OK) {
        compass_read_fail_count++;
        return false;
    }
    return true;
}

static bool write_reg(uint8_t reg, uint8_t val)
{
    if (compass_i2c == NULL) {
        compass_last_hal_status = HAL_ERROR;
        return false;
    }

    compass_last_hal_status = HAL_I2C_Mem_Write(compass_i2c,
                                                compass_addr_8bit,
                                                reg,
                                                I2C_MEMADD_SIZE_8BIT,
                                                &val,
                                                1u,
                                                COMPASS_I2C_TIMEOUT_MS);
    if (compass_last_hal_status != HAL_OK) {
        compass_read_fail_count++;
        return false;
    }
    return true;
}

static void calibrate_vector(int16_t mx, int16_t my, int16_t mz, float out[3])
{
    out[0] = ((float)mx - cal_hard_x) * soft_or_identity(cal_soft_x);
    out[1] = ((float)my - cal_hard_y) * soft_or_identity(cal_soft_y);
    out[2] = ((float)mz - cal_hard_z) * soft_or_identity(cal_soft_z);
}

static void maha_update(const float v[3])
{
    float dx;
    float dy;
    float dz;

    maha_count++;

    dx = v[0] - maha_mean[0];
    maha_mean[0] += dx / (float)maha_count;
    maha_m2[0] += dx * (v[0] - maha_mean[0]);

    dy = v[1] - maha_mean[1];
    maha_mean[1] += dy / (float)maha_count;
    maha_m2[1] += dy * (v[1] - maha_mean[1]);

    dz = v[2] - maha_mean[2];
    maha_mean[2] += dz / (float)maha_count;
    maha_m2[2] += dz * (v[2] - maha_mean[2]);
}

static bool compass_try_init_address(uint8_t addr_7bit, uint8_t *chip_id_out)
{
    uint8_t raw[6];
    uint8_t chip = 0u;

    compass_addr_8bit = (uint8_t)(addr_7bit << 1);
    compass_last_hal_status = HAL_I2C_IsDeviceReady(compass_i2c,
                                                    compass_addr_8bit,
                                                    3u,
                                                    COMPASS_I2C_TIMEOUT_MS);
    if (compass_last_hal_status != HAL_OK) {
        return false;
    }

    if (!write_reg(QMC5883_REG_CTRL2, QMC5883_CTRL2_SOFT_RESET)) {
        return false;
    }
    HAL_Delay(2);
    if (!write_reg(QMC5883_REG_SET_RESET, 0x01u)) {
        return false;
    }
    if (!write_reg(QMC5883_REG_CTRL1, QMC5883_CTRL1_200HZ_8G_CONT)) {
        return false;
    }
    if (!read_reg(QMC5883_REG_X_LSB, raw, sizeof(raw))) {
        return false;
    }

    compass_last_hal_status = HAL_I2C_Mem_Read(compass_i2c,
                                               compass_addr_8bit,
                                               QMC5883_REG_CHIP_ID,
                                               I2C_MEMADD_SIZE_8BIT,
                                               &chip,
                                               1u,
                                               COMPASS_I2C_TIMEOUT_MS);
    if (compass_last_hal_status != HAL_OK) {
        chip = 0u;
    }

    if (chip_id_out != NULL) {
        *chip_id_out = chip;
    }

    return true;
}

void Compass_Init(I2C_HandleTypeDef *i2c_handle)
{
    uint8_t chip = 0u;
    uint32_t i;

    compass_i2c = i2c_handle;
    compass_initialized = false;
    compass_chip_id = 0u;
    last_raw_mx = 0;
    last_raw_my = 0;
    last_raw_mz = 0;
    compass_last_read_ms = 0u;
    compass_read_ok_count = 0u;
    compass_read_fail_count = 0u;
    compass_last_hal_status = HAL_OK;
    maha_count = 0u;
    memset(maha_mean, 0, sizeof(maha_mean));
    memset(maha_m2, 0, sizeof(maha_m2));

    if (compass_i2c == NULL) {
        return;
    }

    for (i = 0u; i < (uint32_t)(sizeof(compass_candidate_addr_7bit) / sizeof(compass_candidate_addr_7bit[0])); i++) {
        if (compass_try_init_address(compass_candidate_addr_7bit[i], &chip)) {
            compass_chip_id = chip;
            compass_initialized = true;
            return;
        }
    }

    compass_addr_8bit = COMPASS_QMC5883L_ADDR_8BIT;
    compass_read_fail_count++;
}

void Compass_LoadCalibration(float softX, float softY, float softZ,
                             float hardX, float hardY, float hardZ)
{
    cal_soft_x = soft_or_identity(softX);
    cal_soft_y = soft_or_identity(softY);
    cal_soft_z = soft_or_identity(softZ);
    cal_hard_x = hardX;
    cal_hard_y = hardY;
    cal_hard_z = hardZ;
}

bool Compass_ReadRaw(int16_t *mx, int16_t *my, int16_t *mz)
{
    uint8_t raw[6];

    if (mx == NULL || my == NULL || mz == NULL) {
        return false;
    }

    if (!compass_initialized) {
        if (compass_i2c == NULL) {
            return false;
        }
        Compass_Init(compass_i2c);
        if (!compass_initialized) {
            return false;
        }
    }

    if (!read_reg(QMC5883_REG_X_LSB, raw, sizeof(raw))) {
        if (compass_i2c == NULL) {
            return false;
        }
        Compass_Init(compass_i2c);
        if (!compass_initialized || !read_reg(QMC5883_REG_X_LSB, raw, sizeof(raw))) {
            return false;
        }
    }

    *mx = (int16_t)(((uint16_t)raw[1] << 8) | raw[0]);
    *my = (int16_t)(((uint16_t)raw[3] << 8) | raw[2]);
    *mz = (int16_t)(((uint16_t)raw[5] << 8) | raw[4]);

    last_raw_mx = *mx;
    last_raw_my = *my;
    last_raw_mz = *mz;
    compass_last_read_ms = HAL_GetTick();
    compass_read_ok_count++;
    return true;
}

float Compass_ComputeHeading(int16_t mx, int16_t my, int16_t mz)
{
    float corr[3];
    float heading;
    float xh;
    float yh;
    AttitudeAngles a = Attitude_GetAngles();
    float roll = a.roll * RAD_PER_DEG;
    float pitch = a.pitch * RAD_PER_DEG;

    calibrate_vector(mx, my, mz, corr);

    xh = corr[0] * cosf(pitch) + corr[2] * sinf(pitch);
    yh = corr[0] * sinf(roll) * sinf(pitch) +
         corr[1] * cosf(roll) -
         corr[2] * sinf(roll) * cosf(pitch);

    if (fabsf(xh) < 1e-3f && fabsf(yh) < 1e-3f) {
        xh = corr[0];
        yh = corr[1];
    }

    heading = atan2f(yh, xh) * DEG_PER_RAD;
    if (heading < 0.0f) {
        heading += 360.0f;
    }
    return heading;
}

bool Compass_CheckMahalanobis(int16_t mx, int16_t my, int16_t mz, float threshold)
{
    float corr[3];
    float dx;
    float dy;
    float dz;
    float vx;
    float vy;
    float vz;
    float score;
    float thr;
    bool pass;

    if (!compass_initialized) {
        return false;
    }

    if (threshold <= 0.0f) {
        return true;
    }

    calibrate_vector(mx, my, mz, corr);

    if (maha_count < COMPASS_MAHA_MIN_SAMPLES) {
        maha_update(corr);
        return true;
    }

    vx = (maha_m2[0] / (float)(maha_count - 1u)) + COMPASS_MAHA_VAR_EPS;
    vy = (maha_m2[1] / (float)(maha_count - 1u)) + COMPASS_MAHA_VAR_EPS;
    vz = (maha_m2[2] / (float)(maha_count - 1u)) + COMPASS_MAHA_VAR_EPS;

    dx = corr[0] - maha_mean[0];
    dy = corr[1] - maha_mean[1];
    dz = corr[2] - maha_mean[2];
    score = (dx * dx) / vx + (dy * dy) / vy + (dz * dz) / vz;

    thr = threshold * threshold;
    pass = (score <= thr);
    if (pass) {
        maha_update(corr);
    }
    return pass;
}

bool Compass_IsStub(void)
{
    return false;
}

uint8_t Compass_GetChipId(void)
{
    return compass_chip_id;
}

uint8_t Compass_GetAddress7bit(void)
{
    return (uint8_t)(compass_addr_8bit >> 1);
}

uint32_t Compass_GetLastReadMs(void)
{
    return compass_last_read_ms;
}

uint32_t Compass_GetReadOkCount(void)
{
    return compass_read_ok_count;
}

uint32_t Compass_GetReadFailCount(void)
{
    return compass_read_fail_count;
}

HAL_StatusTypeDef Compass_GetLastHalStatus(void)
{
    return compass_last_hal_status;
}
