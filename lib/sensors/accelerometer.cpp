#include "accelerometer.hpp"

namespace hyped::sensors {

Accelerometer::Accelerometer(io::I2c &i2c, core::ILogger &log) : log_(log), i2c_(i2c)
{
}

Accelerometer::~Accelerometer()
{
}

    std::optional<float> Accelerometer::read() 
    {
        
        do
		{
			/*Wait till the value is ready to read*/
			ITDS_getdataReadyState(&DRDY);
		} while (DRDY == ITDS_disable);

		ITDS_getRawAccelerationX(&XRawAcc);
		XRawAcc = XRawAcc >> 2; /* shifted by 2 as 14bit resolution is used in high performance mode */
		float XAcceleration = (float) (XRawAcc);
		XAcceleration = XAcceleration / 1000; /* mg to g */
		XAcceleration = XAcceleration * 1.952; /* Multiply with sensitivity 1.952 in high performance mode, 14bit, and full scale +-16g */
		printf("Acceleration X-axis %f g \r\n ", XAcceleration);

		ITDS_getRawAccelerationY(&YRawAcc);
		YRawAcc = YRawAcc >> 2;
		float YAcceleration = (float) (YRawAcc);
		YAcceleration = YAcceleration / 1000;
		YAcceleration = (YAcceleration * 1.952);
		printf("Acceleration Y-axis %f g \r\n ", YAcceleration);

		ITDS_getRawAccelerationZ(&ZRawAcc);
		ZRawAcc = ZRawAcc >> 2;
		float ZAcceleration = (float) (ZRawAcc);
		ZAcceleration = ZAcceleration / 1000;
		ZAcceleration = ZAcceleration * 1.952;
		printf("Acceleration Z-axis %f g \r\n ", ZAcceleration);

       
    }

    bool Accelerometer::configure()
    {
        i2c.writeByte(ADDR, 0x20, 0x64);
        i2c.writeByte(ADDR, 0x21, 0xC);
        i2c.writeByte(ADDR, 0x25, 0x30);
        return true; 
    }

}  // namespace hyped::sensors