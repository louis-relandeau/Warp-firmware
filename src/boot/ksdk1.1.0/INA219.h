void		initINA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts);

WarpStatus	readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes);
WarpStatus	writeSensorRegisterINA219(uint8_t deviceRegister,
					uint8_t payloadBtye);
WarpStatus	configureSensorINA219();
void		printSensorDataINA219(bool hexModeFlag);

// uint16_t readCurrent();
// void printSensorDataINA219(bool hexModeFlag);