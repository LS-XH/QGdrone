#include "drv_imuDrv_bmi088.hpp"
#include "drv_spi1.hpp"

#include "basic.hpp"

// spi接口定义
#define spi_write spi1_write
#define spi_writeNread spi1_writeNread

// 引脚定义
#define GYRO_CS_GPIO GPIOE
#define GYRO_CS_PIN 10
#define ACCEL_CS_GPIO GPIOE
#define ACCEL_CS_PIN 9

// 轴向定义
static const int8_t axis_index[] = {2, -1, 3};

static inline void pu_Gyro_CS()
{
	GYRO_CS_GPIO->BSRR = 1<<(GYRO_CS_PIN+0);
}
static inline void pd_Gyro_CS()
{
	GYRO_CS_GPIO->BSRR = 1<<(GYRO_CS_PIN+16);
}

static inline void pu_Accel_CS()
{
	ACCEL_CS_GPIO->BSRR = 1<<(ACCEL_CS_PIN+0);
}
static inline void pd_Accel_CS()
{
	ACCEL_CS_GPIO->BSRR = 1<<(ACCEL_CS_PIN+16);
}

imuDrv_bmi088::imuDrv_bmi088() : driver_type(IMU_DRIVER_TYPE_NULL),
                                 freqGyro(2000), freqAccel(1000),
                                 name("bmi088"),
                                 gyro_sensitivity(0.00106526443603169529841533860381),
                                 accel_sensitivity(GravityAcc / 1365.0)
{
	// gyro cs
	set_register( GYRO_CS_GPIO->MODER , 0b01 , 2*GYRO_CS_PIN , 2 );
	set_register( GYRO_CS_GPIO->OTYPER , 0 , 1*GYRO_CS_PIN , 1 );
	set_register( GYRO_CS_GPIO->OSPEEDR , 2 , 2*GYRO_CS_PIN , 2 );
	pu_Gyro_CS();

	// accel cs
	set_register( ACCEL_CS_GPIO->MODER , 0b01 , 2*ACCEL_CS_PIN , 2 );
	set_register( ACCEL_CS_GPIO->OTYPER , 0 , 1*ACCEL_CS_PIN , 1 );
	set_register( ACCEL_CS_GPIO->OSPEEDR , 2 , 2*ACCEL_CS_PIN , 2 );
	pu_Accel_CS();
}

IMU_DRIVER_TYPE imuDrv_bmi088::init()
{
	// gyro cs
	set_register( GYRO_CS_GPIO->MODER , 0b01 , 2*GYRO_CS_PIN , 2 );
	set_register( GYRO_CS_GPIO->OTYPER , 0 , 1*GYRO_CS_PIN , 1 );
	set_register( GYRO_CS_GPIO->OSPEEDR , 2 , 2*GYRO_CS_PIN , 2 );
	pu_Gyro_CS();

	// accel cs
	set_register( ACCEL_CS_GPIO->MODER , 0b01 , 2*ACCEL_CS_PIN , 2 );
	set_register( ACCEL_CS_GPIO->OTYPER , 0 , 1*ACCEL_CS_PIN , 1 );
	set_register( ACCEL_CS_GPIO->OSPEEDR , 2 , 2*ACCEL_CS_PIN , 2 );
	pu_Accel_CS();

	os_delay(0.01);

	#ifdef DCACHE_SIZE
		#define TX_BUF_SIZE DCACHE_SIZE
		#define RX_BUF_SIZE TX_BUF_SIZE
		Aligned_DMABuf uint8_t tx_buf[TX_BUF_SIZE];
		uint8_t* rx_buf = tx_buf;
	#else
		#define TX_BUF_SIZE 32
		#define RX_BUF_SIZE TX_BUF_SIZE
		uint8_t tx_buf[TX_BUF_SIZE];
		uint8_t* rx_buf = tx_buf;
	#endif

	/*加速度计初始化*/
		// 使加速度计进入spi模式
		pd_Accel_CS();
		tx_buf[0] = (1 << 7) | 0;
		spi_writeNread(
			tx_buf, 3, TX_BUF_SIZE,
			rx_buf, RX_BUF_SIZE
		);
		pu_Accel_CS();
		os_delay(0.01);

		// 读取序列号
		pd_Accel_CS();
		tx_buf[0] = (1 << 7) | 0;
		spi_writeNread(
			tx_buf, 3, TX_BUF_SIZE,
			rx_buf, RX_BUF_SIZE
		);
		pu_Accel_CS();
		if (rx_buf[2] != 0x1e)
			return IMU_DRIVER_TYPE_NULL;
		os_delay(0.01);

		// 复位
		pd_Accel_CS();
		tx_buf[0] = (0 << 7) | 0x7e;
		tx_buf[1] = 0xb6;
		spi_write(
			tx_buf, 2, TX_BUF_SIZE
		);
		pu_Accel_CS();
		os_delay(0.1);

		// 使加速度计进入spi模式
		pd_Accel_CS();
		tx_buf[0] = (1 << 7) | 0;
		spi_writeNread(
			tx_buf, 3, TX_BUF_SIZE,
			rx_buf, RX_BUF_SIZE
		);
		pu_Accel_CS();
		os_delay(0.01);

		// 打开加速度计
		pd_Accel_CS();
		tx_buf[0] = (0 << 7) | 0x7d;
		tx_buf[1] = 0x04;
		spi_write(
			tx_buf, 2, TX_BUF_SIZE
		);
		pu_Accel_CS();
		os_delay(0.01);

		// 加速度计ODR 800hz
		pd_Accel_CS();
		tx_buf[0] = (0 << 7) | 0x40;
		tx_buf[1] = (0x8 << 4) | (0xB << 0);
		spi_write(
			tx_buf, 2, TX_BUF_SIZE
		);
		pu_Accel_CS();
		os_delay(0.01);

		// 加速度计量程24g
		pd_Accel_CS();
		tx_buf[0] = (0 << 7) | 0x41;
		tx_buf[1] = 0x03;
		spi_write(
			tx_buf, 2, TX_BUF_SIZE
		);
		pu_Accel_CS();
		os_delay(0.01);

		// 进入Active模式
		pd_Accel_CS();
		tx_buf[0] = (0 << 7) | 0x7c;
		tx_buf[1] = 0x00;
		spi_write(
			tx_buf, 2, TX_BUF_SIZE
		);
		pu_Accel_CS();
		os_delay(0.01);
	/*加速度计初始化*/

	/*陀螺仪初始化*/
		// 读取序列号
		pd_Gyro_CS();
		tx_buf[0] = (1 << 7) | 0;
		spi_writeNread(
			tx_buf, 2, TX_BUF_SIZE,
			rx_buf, RX_BUF_SIZE
		);
		pu_Gyro_CS();
		if (rx_buf[1] != 0x0f)
			return IMU_DRIVER_TYPE_NULL;
		os_delay(0.01);

		// 复位
		pd_Gyro_CS();
		tx_buf[0] = (0 << 7) | 0x14;
		tx_buf[1] = 0xb6;
		spi_write(
			tx_buf, 2, TX_BUF_SIZE
		);
		pu_Gyro_CS();
		os_delay(0.1);

		// GYRO_LPM1(0x11): normal mode
		pd_Gyro_CS();
		tx_buf[0] = (0 << 7) | 0x11;
		tx_buf[1] = 0x00;
		spi_write(
			tx_buf, 2, TX_BUF_SIZE
		);
		pu_Gyro_CS();
		os_delay(0.01);

		// GYRO_RANGE(0x0f): 2000deg/s
		pd_Gyro_CS();
		tx_buf[0] = (0 << 7) | 0x0f;
		tx_buf[1] = 0x00;
		spi_write(
			tx_buf, 2, TX_BUF_SIZE
		);
		pu_Gyro_CS();
		os_delay(0.01);

		// GYRO_BANDWIDTH(0x10): ODR=2000hz bandwidth=230hz
		pd_Gyro_CS();
		tx_buf[0] = (0 << 7) | 0x10;
		tx_buf[1] = 0x02;
		spi_write(
			tx_buf, 2, TX_BUF_SIZE
		);
		pu_Gyro_CS();
		os_delay(0.01);
	/*陀螺仪初始化*/

	driver_type = IMU_DRIVER_TYPE_GYRO_ACCEL;
	return driver_type;
}

bool imuDrv_bmi088::sample(uint8_t sId, 
	vector3<int32_t> *rx_data1, vector3<int32_t> *rx_data2, vector3<int32_t> *rx_data3, 
	double *temperature, 
	IMU_DATA_STATUS *status1, IMU_DATA_STATUS *status2, IMU_DATA_STATUS *status3)
{
	#ifdef DCACHE_SIZE
		#define TX_BUF_SIZE DCACHE_SIZE
		#define RX_BUF_SIZE TX_BUF_SIZE
		Aligned_DMABuf uint8_t tx_buf[TX_BUF_SIZE];
		uint8_t* rx_buf = tx_buf;
	#else
		#define TX_BUF_SIZE 32
		#define RX_BUF_SIZE TX_BUF_SIZE
		uint8_t tx_buf[TX_BUF_SIZE];
		uint8_t* rx_buf = tx_buf;
	#endif
	
	if (sId == 0)
	{ // 陀螺
		struct __BMI088_GyroData
		{
			uint8_t rsv1;
			int16_t gyro[3]; // 0x02-0x07
		} __attribute__((__packed__));

		pd_Gyro_CS();
		tx_buf[0] = (1 << 7) | 0x02;
		spi_writeNread(
			tx_buf, sizeof(__BMI088_GyroData), TX_BUF_SIZE,
			rx_buf, RX_BUF_SIZE
		);
		pu_Gyro_CS();

		__BMI088_GyroData *rData = (__BMI088_GyroData *)rx_buf;
		if (axis_index[0] > 0)
			rx_data1->x = rData->gyro[axis_index[0] - 1];
		else
			rx_data1->x = -rData->gyro[-axis_index[0] - 1];
		if (axis_index[1] > 0)
			rx_data1->y = rData->gyro[axis_index[1] - 1];
		else
			rx_data1->y = -rData->gyro[-axis_index[1] - 1];
		if (axis_index[2] > 0)
			rx_data1->z = rData->gyro[axis_index[2] - 1];
		else
			rx_data1->z = -rData->gyro[-axis_index[2] - 1];
		*temperature = -1000;
		*status1 = IMU_DATA_STATUS_HEALTHY;

		return true;
	}
	else if (sId == 1)
	{ // 加速度
		struct __BMI088_AccelData
		{
			uint8_t rsv1[2];
			int16_t acc[3];            // 0x12-0x17
			uint32_t sensor_time : 24; // 0x18-0x1a
			uint8_t rsv2[2];           // 0x1b-0x1c
			uint8_t acc_int_stat;      // 0x1d
			uint8_t rsv3[4];           // 0x1e-0x21
			uint16_t temperature;      // 0x22-0x23
		} __attribute__((__packed__));

		pd_Accel_CS();
		tx_buf[0] = (1 << 7) | 0x12;
		spi_writeNread(
			tx_buf, sizeof(__BMI088_AccelData), TX_BUF_SIZE,
			rx_buf, RX_BUF_SIZE
		);
		pu_Accel_CS();

		__BMI088_AccelData *rData = (__BMI088_AccelData *)rx_buf;
		if (axis_index[0] > 0)
			rx_data2->x = rData->acc[axis_index[0] - 1];
		else
			rx_data2->x = -rData->acc[-axis_index[0] - 1];
		if (axis_index[1] > 0)
			rx_data2->y = rData->acc[axis_index[1] - 1];
		else
			rx_data2->y = -rData->acc[-axis_index[1] - 1];
		if (axis_index[2] > 0)
			rx_data2->z = rData->acc[axis_index[2] - 1];
		else
			rx_data2->z = -rData->acc[-axis_index[2] - 1];
		*temperature = -1000;
		*status2 = IMU_DATA_STATUS_HEALTHY;
		return true;
	}

	return false;
}