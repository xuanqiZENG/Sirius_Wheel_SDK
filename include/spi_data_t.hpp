#ifndef SPI_DATA_T
#define SPI_DATA_T

#include <cstdint>

// 9 * 4 = 36
typedef struct {
    float q_abad[4];
    float q_hip[4];
    float q_knee[4];
    float qd_abad[4];
    float qd_hip[4];
    float qd_knee[4];
    float tau_abad[4];
    float tau_hip[4];
    float tau_knee[4];
    uint32_t flags[2];
} spi_data_t;

#endif
