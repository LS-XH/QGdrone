#include "drv_imuDrv_icm20689.hpp"
#include "drv_spi1.hpp"

#include "basic.hpp"

// spi接口定义
#define spi_write spi1_write
#define spi_writeNread spi1_writeNread

// 引脚定义
#define CS_GPIO GPIOE
#define CS_PIN 11

// 轴向定义
static const int8_t axis_index[] = {2, -1, 3};

static inline void pu_CS()
{
	CS_GPIO->BSRR = 1<<(CS_PIN+0);
}
static inline void pd_CS()
{
	CS_GPIO->BSRR = 1<<(CS_PIN+16);
}

imuDrv_icm20689::imuDrv_icm20689() : driver_type(IMU_DRIVER_TYPE_NULL),
                                 freqGyro(1000), freqAccel(1000),
                                 name("bmi20689"),
                                 gyro_sensitivity(0.00106526443603169529841533860381),
                                 accel_sensitivity(GravityAcc / 2048.0)
{
	// cs
	set_register( CS_GPIO->MODER , 0b01 , 2*CS_PIN , 2 );
	set_register( CS_GPIO->OTYPER , 0 , 1*CS_PIN , 1 );
	set_register( CS_GPIO->OSPEEDR , 2 , 2*CS_PIN , 2 );
	pu_CS();
}

IMU_DRIVER_TYPE imuDrv_icm20689::init()
{
	// cs
	set_register( CS_GPIO->MODER , 0b01 , 2*CS_PIN , 2 );
	set_register( CS_GPIO->OTYPER , 0 , 1*CS_PIN , 1 );
	set_register( CS_GPIO->OSPEEDR , 2 , 2*CS_PIN , 2 );
	pu_CS();

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

	
	// 校验Chip ID
	tx_buf[0] = (1<<7) | 117;
	pd_CS();
	spi_writeNread(
		tx_buf, 2, TX_BUF_SIZE,
		rx_buf, RX_BUF_SIZE
	);
	pu_CS();
	os_delay(0.01);
	if( rx_buf[1] != 0x98 )
		return IMU_DRIVER_TYPE_NULL;
	
	// 复位
	tx_buf[0] = (0<<7) | 107;
	tx_buf[1] = (1<<7);
	pd_CS();
	spi_write(
		tx_buf, 2, TX_BUF_SIZE
	);
	pu_CS();
	os_delay(0.3);
	
	// 设置 ICM20689 时钟源，Auto selects the best available clock source
	tx_buf[0] = (0<<7) | 107;
	tx_buf[1] = (1<<0);
	pd_CS();
	spi_write(
		tx_buf, 2, TX_BUF_SIZE
	);
	pu_CS();
	os_delay(0.1);
	
	// 设置 ICM20689 陀螺量程:±2000dps
	tx_buf[0] = (0<<7) | 27;
	tx_buf[1] = (3<<3);
	pd_CS();
	spi_write(
		tx_buf, 2, TX_BUF_SIZE
	);
	pu_CS();
	os_delay(0.01);
	
	// 设置 ICM20689  陀螺DLPF： 250hz
	tx_buf[0] = (0<<7) | 26;
	tx_buf[1] = 0;
	pd_CS();
	spi_write(
		tx_buf, 2, TX_BUF_SIZE
	);
	pu_CS();
	os_delay(0.01);

	// 设置 ICM20689 加计量程: ± 16G
	tx_buf[0] = (0<<7) | 28;
	tx_buf[1] = (3<<3);
	pd_CS();
	spi_write(
		tx_buf, 2, TX_BUF_SIZE
	);
	pu_CS();
	os_delay(0.01);

	//设置 ICM20689 加速度计DLPF： 1046.0  hz
	tx_buf[0] = (0<<7) | 29;
	tx_buf[1] = (0<<3) | (0<<0);
	pd_CS();
	spi_write(
		tx_buf, 2, TX_BUF_SIZE
	);
	pu_CS();
	os_delay(0.01);

	//使能 ICM20689 加计和陀螺
	tx_buf[0] = (0<<7) | 107;
	tx_buf[1] = 0;
	pd_CS();
	spi_write(
		tx_buf, 2, TX_BUF_SIZE
	);
	pu_CS();
	os_delay(0.01);

	driver_type = IMU_DRIVER_TYPE_GYROACCEL;
	return driver_type;
}

bool imuDrv_icm20689::sample(uint8_t sId, 
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
	
	struct __SENSOR_Data
	{
		uint8_t rsv1;
		int16_t acc[3];
		int16_t temperature;
		int16_t gyro[3];
	} __attribute__((__packed__));
	if (sId == 0)
	{ // 陀螺+加速度
		pd_CS();
		tx_buf[0] = (1<<7) | 59;
		spi_writeNread(
			tx_buf, sizeof(__SENSOR_Data), TX_BUF_SIZE,
			rx_buf, RX_BUF_SIZE
		);
		pu_CS();

		__SENSOR_Data *rData = (__SENSOR_Data *)rx_buf;
		rData->gyro[0] = __REV16(rData->gyro[0]);
		rData->gyro[1] = __REV16(rData->gyro[1]);
		rData->gyro[2] = __REV16(rData->gyro[2]);
		rData->acc[0] = __REV16(rData->acc[0]);
		rData->acc[1] = __REV16(rData->acc[1]);
		rData->acc[2] = __REV16(rData->acc[2]);
			
			
		*temperature = -1000;
		
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
		*status1 = IMU_DATA_STATUS_HEALTHY;
		
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
		
		*status2 = IMU_DATA_STATUS_HEALTHY;

		return true;
	}

	return false;
}