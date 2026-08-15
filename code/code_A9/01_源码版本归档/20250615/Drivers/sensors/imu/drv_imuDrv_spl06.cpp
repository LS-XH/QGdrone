#include "drv_imuDrv_spl06.hpp"
#include "drv_spi1.hpp"

#include "basic.hpp"

// spi接口定义
#define spi_write spi1_write
#define spi_writeNread spi1_writeNread

// 引脚定义
#define CS_GPIO GPIOC
#define CS_PIN 4

static inline void pu_CS()
{
	CS_GPIO->BSRR = 1<<(CS_PIN+0);
}
static inline void pd_CS()
{
	CS_GPIO->BSRR = 1<<(CS_PIN+16);
}

/*SPL06校准数据*/
	void imuDrv_spl06::spl06_pressure_rateset(uint8_t u8SmplRate, uint8_t u8OverSmpl)
	{
		uint8_t reg = 0;
		switch (u8SmplRate)
		{
			case 2:
				reg |= (1 << 4);
				break;
			case 4:
				reg |= (2 << 4);
				break;
			case 8:
				reg |= (3 << 4);
				break;
			case 16:
				reg |= (4 << 4);
				break;
			case 32:
				reg |= (5 << 4);
				break;
			case 64:
				reg |= (6 << 4);
				break;
			case 128:
				reg |= (7 << 4);
				break;
			case 1:
			default:
				break;
		}
		switch (u8OverSmpl)
		{
			case 2:
				reg |= 1;
				coefficients.KP = 1.0 / 1572864;
				break;
			case 4:
				reg |= 2;
				coefficients.KP = 1.0 / 3670016;
				break;
			case 8:
				reg |= 3;
				coefficients.KP = 1.0 / 7864320;
				break;
			case 16:
				coefficients.KP = 1.0 / 253952;
				reg |= 4;
				break;
			case 32:
				coefficients.KP = 1.0 / 516096;
				reg |= 5;
				break;
			case 64:
				coefficients.KP = 1.0 / 1040384;
				reg |= 6;
				break;
			case 128:
				coefficients.KP = 1.0 / 2088960;
				reg |= 7;
				break;
			case 1:
			default:
				coefficients.KP = 1.0 / 524288;
				break;
		}

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

		pd_CS();
		tx_buf[0] = (0 << 7) | 0x06;
		tx_buf[1] = reg;
		spi_write(
			tx_buf, 2, TX_BUF_SIZE
		);
		pu_CS();
		os_delay(0.01);

		if (u8OverSmpl > 8)
		{
			pd_CS();
			tx_buf[0] = (1 << 7) | 0x09;
			spi_writeNread(
				tx_buf, 2, TX_BUF_SIZE,
				rx_buf, RX_BUF_SIZE
			);
			pu_CS();
			os_delay(0.01);

			pd_CS();
			tx_buf[0] = (0 << 7) | 0x09;
			tx_buf[1] = rx_buf[1] | (1 << 2);
			spi_write(
				tx_buf, 2, TX_BUF_SIZE
			);
			pu_CS();
			os_delay(0.01);
		}
	}
	void imuDrv_spl06::spl06_temperature_rateset(uint8_t u8SmplRate, uint8_t u8OverSmpl)
	{
		uint8_t reg = 0;
		switch (u8SmplRate)
		{
			case 2:
				reg |= (1 << 4);
				break;
			case 4:
				reg |= (2 << 4);
				break;
			case 8:
				reg |= (3 << 4);
				break;
			case 16:
				reg |= (4 << 4);
				break;
			case 32:
				reg |= (5 << 4);
				break;
			case 64:
				reg |= (6 << 4);
				break;
			case 128:
				reg |= (7 << 4);
				break;
			case 1:
			default:
				break;
		}
		switch (u8OverSmpl)
		{
			case 2:
				reg |= 1;
				coefficients.KT = 1.0f / 1572864;
				break;
			case 4:
				reg |= 2;
				coefficients.KT = 1.0f / 3670016;
				break;
			case 8:
				reg |= 3;
				coefficients.KT = 1.0f / 7864320;
				break;
			case 16:
				coefficients.KT = 1.0f / 253952;
				reg |= 4;
				break;
			case 32:
				coefficients.KT = 1.0f / 516096;
				reg |= 5;
				break;
			case 64:
				coefficients.KT = 1.0f / 1040384;
				reg |= 6;
				break;
			case 128:
				coefficients.KT = 1.0f / 2088960;
				reg |= 7;
				break;
			case 1:
			default:
				coefficients.KT = 1.0f / 524288;
				break;
		}

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

		pd_CS();
		tx_buf[0] = (0 << 7) | 0x07;
		tx_buf[1] = (1 << 7) | reg;
		spi_write(
			tx_buf, 2, TX_BUF_SIZE
		);
		pu_CS();
		os_delay(0.01);

		if (u8OverSmpl > 8)
		{
			pd_CS();
			tx_buf[0] = (1 << 7) | 0x09;
			spi_writeNread(
				tx_buf, 2, TX_BUF_SIZE,
				rx_buf, RX_BUF_SIZE
			);
			pu_CS();
			os_delay(0.01);

			pd_CS();
			tx_buf[0] = (0 << 7) | 0x09;
			tx_buf[1] = rx_buf[1] | (1 << 2);
			spi_write(
				tx_buf, 2, TX_BUF_SIZE
			);
			pu_CS();
			os_delay(0.01);
		}
	}
/*SPL06校准数据*/

imuDrv_spl06::imuDrv_spl06() : driver_type(IMU_DRIVER_TYPE_NULL),
                               freq(33),
                               delay(0.02), trust(300), lt_trust(50),
                               name("spl06")
{
	// cs
	set_register( CS_GPIO->MODER , 0b01 , 2*CS_PIN , 2 );
	set_register( CS_GPIO->OTYPER , 0 , 1*CS_PIN , 1 );
	set_register( CS_GPIO->OSPEEDR , 2 , 2*CS_PIN , 2 );
	pu_CS();
}

IMU_DRIVER_TYPE imuDrv_spl06::init()
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
	uint8_t retryCnt = 0;
	
	// 读取ID
	do
	{
		pd_CS();
		tx_buf[0] = (1 << 7) | 0x0d;
		tx_buf[1] = 0xff;
		spi_writeNread(
			tx_buf, 2, TX_BUF_SIZE,
			rx_buf, RX_BUF_SIZE
		);
		pu_CS();
		os_delay(0.1);
	}while(rx_buf[1]==0xff && retryCnt<20);
	if( rx_buf[1]==0xff )
		return IMU_DRIVER_TYPE_NULL;;
	retryCnt = 0;
	
	// 复位
	pd_CS();
	tx_buf[0] = (0 << 7) | 0x0c;
	tx_buf[1] = (1 << 7) | (0b1001);
	spi_write(
		tx_buf, 2, TX_BUF_SIZE
	);
	pu_CS();
	os_delay(0.1);

	spl06_pressure_rateset(64, 32);    // pressure 64 samples per sec , 32 times over sampling
	spl06_temperature_rateset(128, 2); // temperature 128 samples per sec , 2 times over sampling

	// 连续采样模式
	pd_CS();
	tx_buf[0] = (0<<7) | 0x08;
	tx_buf[1] = 0b111;
	spi_write(
		tx_buf, 2, TX_BUF_SIZE
	);
	pu_CS();
	os_delay(0.1);

	// 获取校准数据
	pd_CS();
	tx_buf[0] = (1 << 7) | 0x10;
	spi_writeNread(
		tx_buf, 19, TX_BUF_SIZE,
		rx_buf, RX_BUF_SIZE
	);
	pu_CS();
	os_delay(0.01);
	coefficients.c0 = ( rx_buf[1] << 4 ) | ( rx_buf[2] >> 4 );
	coefficients.c0 = (coefficients.c0 & 0x0800) ? (0xF000 | coefficients.c0) : coefficients.c0;
	coefficients.c1 = ((rx_buf[2] & 0xf) << 8) | (rx_buf[3]);
	coefficients.c1 = (coefficients.c1 & 0x0800) ? (0xF000 | coefficients.c1) : coefficients.c1;
	coefficients.c00 = (rx_buf[4] << 12) | (rx_buf[5] << 4) | (rx_buf[6] >> 4);
	coefficients.c00 = (coefficients.c00 & 0x080000) ? (0xFFF00000 | coefficients.c00) : coefficients.c00;
	coefficients.c10 = ((rx_buf[6] & 0xf) << 16) | (rx_buf[7] << 8) | (rx_buf[8] >> 0);
	coefficients.c10 = (coefficients.c10 & 0x080000) ? (0xFFF00000 | coefficients.c10) : coefficients.c10;
	coefficients.c01 = (rx_buf[9] << 8) | (rx_buf[10] << 0);
	coefficients.c11 = (rx_buf[11] << 8) | (rx_buf[12] << 0);
	coefficients.c20 = (rx_buf[13] << 8) | (rx_buf[14] << 0);
	coefficients.c21 = (rx_buf[15] << 8) | (rx_buf[16] << 0);
	coefficients.c30 = (rx_buf[17] << 8) | (rx_buf[18] << 0);

	driver_type = IMU_DRIVER_TYPE_ALT;
	return driver_type;
}

bool imuDrv_spl06::sample(uint8_t sId, 
	vector3<int32_t> *rx_data1, vector3<int32_t> *rx_data2, vector3<int32_t> *rx_data3, 
	double *temperature, 
	IMU_DATA_STATUS *status1, IMU_DATA_STATUS *status2, IMU_DATA_STATUS *status3)
{
	struct __SPL06_Data
	{
		uint8_t rsv1;
		unsigned int pressure : 24;
		unsigned int temperature : 24;
	} __attribute__((__packed__));

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

	// 读取传感器数据
	pd_CS();
	tx_buf[0] = (1<<7) | 0x00;
	spi_writeNread(
		tx_buf, sizeof(__SPL06_Data), TX_BUF_SIZE,
		rx_buf, RX_BUF_SIZE
	);
	pu_CS();
	
	// 处理数据
	__SPL06_Data *datap = (__SPL06_Data *)rx_buf;
	int32_t buf32[2];
	buf32[0] = __REV(datap->pressure) >> 8;
	buf32[1] = __REV(datap->temperature) >> 8;

	buf32[0] = ( buf32[0] & 0x800000 ) ? (0xFF000000|buf32[0]) : buf32[0];
	buf32[1] = ( buf32[1] & 0x800000 ) ? (0xFF000000|buf32[1]) : buf32[1];

	double fPsc = buf32[0] * coefficients.KP;
	double fTsc = buf32[1] * coefficients.KT;
	double qua2 = coefficients.c10 + fPsc * (coefficients.c20 + fPsc * coefficients.c30);
	double qua3 = fTsc * fPsc * (coefficients.c11 + fPsc * coefficients.c21);

	double pressure = coefficients.c00 + fPsc * qua2 + fTsc * coefficients.c01 + qua3;
	*temperature = coefficients.c0 * 0.5 + coefficients.c1 * fTsc;
		
	// 通过气压计算高度
	if (pressure > 0)
	{
		// double velx = get_VelocityENU_Ctrl_x();
		// double vely = get_VelocityENU_Ctrl_y();
		trust = 300; // + constrain( 0.5*safe_sqrt( velx*velx + vely*vely ), 150.0 );
		lt_trust = 50;

		*status1 = IMU_DATA_STATUS_HEALTHY;
		rx_data1->y = pressure*1000;
		rx_data1->z = 44300000 * (1.0 - pow(pressure / 101325.0, 1.0 / 5.256));
	}
	else
		*status1 = IMU_DATA_STATUS_ERROR;

	return true;
}