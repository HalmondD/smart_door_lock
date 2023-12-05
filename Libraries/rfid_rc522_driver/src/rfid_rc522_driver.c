#include "rfid_rc522_driver.h"

/////////////////////////////////////////////////////////////////////////////////////
// Basic interface functions for communicating with the MFRC522
/////////////////////////////////////////////////////////////////////////////////////
SPI_HandleTypeDef* ptr_rfid_rc522_connect_mode;

/**
 * @brief 	Writes a byte (one byte) to the specified register in the MFRC522 chip.
 * 			The interface is described in the datasheet section 8.1.2.
 * @param	write_register		The register to write to. One of the rfid_rc522_register_address enums.
 * @param	write_data			The values to write.
 */
void rfid_rc522_write_one_data(uint8_t write_register, uint8_t write_data)
{
    uint8_t transmit_data_array[2] =
    {
        write_register,
        write_data
    };

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_SPI_Transmit(ptr_rfid_rc522_connect_mode, transmit_data_array, 2, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
}

/**
 * @brief 	Writes a number of bytes to the specified register in the MFRC522 chip.
 * 			The interface is described in the datasheet section 8.1.2.
 * @param	write_register		The register to write to. One of the rfid_rc522_register_address enums.
 * @param	write_data_array	The values to write. Byte array.
 * @param	write_byte_count	number of ONLY data byte to write
 */
void rfid_rc522_write_many_data(uint8_t write_register, uint8_t *write_data_array, uint8_t write_byte_count)
{
	uint8_t transmit_byte_count = write_byte_count + 1;
	uint8_t transmit_data_array[transmit_byte_count];

	transmit_data_array[0] = write_register;

	for (uint8_t i = 1; i <= transmit_byte_count; i++)
	{
		transmit_data_array[i] = write_data_array[i - 1];
	}

	//Calculate the timeout for hal spi, 500000 Bytes/s, this is from the speed of 4Mb/s
	//+1 so that we can generous.
	uint8_t timeout_ms = (transmit_byte_count / 500000) + 1;


	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_Transmit(ptr_rfid_rc522_connect_mode, transmit_data_array, transmit_byte_count, timeout_ms);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
}

/**
 * @brief 	Read a byte (one byte) from the specified register in the MFRC522 chip.
 * 			The interface is described in the datasheet section 8.1.2.
 * @param	read_register		The register to read from. One of the rfid_rc522_register_address enums.
 * @param	read_data			The read value.
 * @return	read_data - 1 byte.
 */
uint8_t rfid_rc522_read_one_data(uint8_t read_register)
{
    uint8_t read_address_array[2] =
    {
        READ_MASK | read_register,
        END_OF_SPI_READ
    };

    uint8_t read_data_array[2];

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(ptr_rfid_rc522_connect_mode, read_address_array, read_data_array, 2, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

    return read_data_array[1];
}

/**
 * @brief 	Read a number of bytes from the specified register in the MFRC522 chip.
 * 			The interface is described in the datasheet section 8.1.2.
 * @param	read_register		The register to read from. One of the rfid_rc522_register_address enums.
 * @param	read_data_array		Pointer to the read data array
 * @param	read_byte_count		number of ONLY data byte to read
 * @param	rxAlign_value		used for reception of bit-oriented frames: defines the bit position
 * 								for the first bit received to be stored in the FIFO buffer.
 * @return	read_data_array with the data from the read register
 */
void rfid_rc522_read_many_data(uint8_t read_register, uint8_t *read_data_array, uint8_t read_byte_count, uint8_t rxAlign_value)
{
	uint8_t transmit_byte_count = read_byte_count + 1;
	uint8_t transmit_data_array[transmit_byte_count];

	transmit_data_array[transmit_byte_count - 1] = END_OF_SPI_READ;
	for (uint8_t i = 0; i <= (transmit_byte_count - 2); i++)
	{
		transmit_data_array[i] = READ_MASK | read_register;
	}

	//Calculate the timeout for hal spi, 500000 Bytes/s, this is from the speed of 4Mb/s
	//+1 so that we can generous.
	uint8_t timeout_ms = (transmit_byte_count / 500000) + 1;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(ptr_rfid_rc522_connect_mode, transmit_data_array, read_data_array, read_byte_count, timeout_ms);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

	//Read the first byte of the read data according to the rxAlign_value
	uint8_t rxAlign_mask = 0xff << rxAlign_value;
	read_data_array[0] = read_data_array[0] & rxAlign_mask;
}

/**
 * @brief	Clears the bits given in mask from register reg.
 * @param	register_address	The register to clear the bits according to the bit mask.
 * @param	bit_mask			The bit mask.
 */
void rfid_rc522_clear_register_bit_mask(uint8_t register_address, uint8_t bit_mask)
{
    uint8_t register_data = rfid_rc522_read_one_data(register_address);

    rfid_rc522_write_one_data(register_address, register_data & (~bit_mask));
}

/**
 * @brief	Sets the bits given in mask in register reg.
 * @param	register_address	The register to update. One of the rfid_rc522_register enums.
 * @param	bit_mask			The bits to set.
 */
void rfid_rc522_set_register_bit_mask(uint8_t register_address, uint8_t bit_mask)
{ 
	uint8_t register_data = rfid_rc522_read_one_data(register_address);

	rfid_rc522_write_one_data(register_address, register_data | bit_mask); // set bit mask		
}

/**
 * @brief	Get the bits given in mask in register reg.
 * @param	register_address	The register to get data. One of the rfid_rc522_register enums.
 * @param	bit_mask			The bits to get data.
 * @return	The register data with the bit mask apply.
 */
uint8_t rfid_rc522_get_register_bit_mask(uint8_t register_address, uint8_t bit_mask)
{
    return rfid_rc522_read_one_data(register_address) & bit_mask;
}

/**
 * @brief   Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
 * @param   data_to_crc			In: Pointer to the data to transfer to the FIFO for CRC calculation.
 * @param   data_byte_count  	In: The number of bytes to transfer.
 * @param   result_array  		Out: Pointer to result buffer. Result is written to result[0..1], low byte first.
 * @return  STATUS_OK on success, STATUS_??? otherwise.
 */
enum rfid_rc522_status_code rfid_rc522_calculate_crc(uint8_t *data_to_crc, uint8_t data_byte_count, uint8_t *result_array)
{
	// Stop any active command.
	rfid_rc522_write_one_data(CommandReg, Idle);
	// Clear the CRCIRq interrupt request bit
	rfid_rc522_write_one_data(DivIrqReg, 0x04);
	// FlushBuffer = 1, FIFO initialization
	rfid_rc522_write_one_data(FIFOLevelReg, 0x80);
	// Write data to the FIFO
	rfid_rc522_write_many_data(FIFODataReg, data_to_crc, data_byte_count);
	// Start the calculation
	rfid_rc522_write_one_data(CommandReg, CalcCRC);
	
	// Wait for the CRC calculation to complete. Check for the register to
	// indicate that the CRC calculation is complete in a loop. If the
	// calculation is not indicated as complete in ~90ms, then time out
	// the operation.
	const uint32_t complete_time = HAL_GetTick() + 90;

	do
	{
		// DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
		uint8_t CRCIRq_value = rfid_rc522_get_register_bit_mask(DivIrqReg, 0x04);
		if (CRCIRq_value)
		{	// CRCIRq bit set - calculation done
			// Stop calculating CRC for new content in the FIFO.
			rfid_rc522_write_one_data(CommandReg, Idle);
			// Transfer the result from the registers to the result buffer
			result_array[0] = rfid_rc522_read_one_data(CRCResultRegL);
			result_array[1] = rfid_rc522_read_one_data(CRCResultRegH);
			return STATUS_OK;
		}
	}
	while (HAL_GetTick() < complete_time);

	// 89ms passed and nothing happened. Communication with the MFRC522 might be down.
	return STATUS_TIMEOUT;
}

/////////////////////////////////////////////////////////////////////////////////////
// Functions for manipulating the MFRC522
/////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief	Initializes the MFRC522 chip.
 */
void rfid_rc522_init(SPI_HandleTypeDef* connect_mode)
{
	ptr_rfid_rc522_connect_mode = connect_mode;

	//Do a soft reset
    rfid_rc522_soft_reset();

    // Reset baud rates
	rfid_rc522_write_one_data(TxModeReg, 0x00);
	rfid_rc522_write_one_data(RxModeReg, 0x00);

	// Reset ModWidthReg
	rfid_rc522_write_one_data(ModWidthReg, 0x26);

	// When communicating with a PICC we need a timeout if something goes wrong.
	// f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
	// TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
	// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
	rfid_rc522_write_one_data(TModeReg, 0x80);
	// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
	rfid_rc522_write_one_data(TPrescalerReg, 0xA9);
	// Reload timer with 0x7D0 = 2000, ie 50ms before timeout.
	rfid_rc522_write_one_data(TReloadRegH, 0x07);
	rfid_rc522_write_one_data(TReloadRegL, 0xD0);
	
	// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	rfid_rc522_write_one_data(TxASKReg, 0x40);
	// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
	rfid_rc522_write_one_data(ModeReg, 0x3D);
	// Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
	rfid_rc522_antenna_on();
}

/**
 * @brief Performs a soft reset on the MFRC522 chip and waits for it to be ready again.
 */
void rfid_rc522_soft_reset()
{
    rfid_rc522_write_one_data(CommandReg, SoftReset);
    // Issue the SoftReset command.
	// The datasheet does not mention how long the SoftRest command takes to complete.
	// But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg) 
	// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74μs. Let us be generous: 50ms.

    uint8_t count = 0;

    do
    {
        gettick_delay_ms(50);
    }
    while ((rfid_rc522_read_one_data(CommandReg) & (1 << 4)) && (++count) < 3);
}

/**
 * @brief	Turns the antenna on by enabling pins TX1 and TX2.
 * 			After a reset these pins are disabled.
 */
void rfid_rc522_antenna_on()
{
    uint8_t TxnRFEn_bit = rfid_rc522_get_register_bit_mask(TxControlReg, 0x03);

    if (TxnRFEn_bit != 0x03)
        rfid_rc522_set_register_bit_mask(TxControlReg, 0x03);
}

/**
 * @brief	Turns the antenna off by disabling pins TX1 and TX2.
 */
void rfid_rc522_antenna_off()
{
	rfid_rc522_clear_register_bit_mask(TxControlReg, 0x03);
}

/**
 * @brief	Get the current MFRC522 Receiver Gain (RxGain[2:0]) value.
 * 			See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 * NOTE: 	Return value scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.
 * 
 * @return 	Value of the RxGain, scrubbed to the 3 bits used.
 */
uint8_t rfid_rc522_get_antenna_gain()
{
	// (0x07<<4) = 01110000b
	return rfid_rc522_get_register_bit_mask(RFCfgReg, (0x07 << 4));
}

/**
 * @brief	Set the MFRC522 Receiver Gain (RxGain) to value specified by given RxGain_value.
 * 			See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 * @param 	RxGain_value 	RxGain is from the rfid_rc522_RxGain enum
 * 							the enum has already scrubbed with (0x07<<4) = 01110000b
 * 							as RCFfgReg may use reserved bits.
 */
void rfid_rc522_set_antenna_gain( uint8_t RxGain_value)
{
	if (rfid_rc522_get_antenna_gain() != RxGain_value)
    {	
        // only bother if there is a change
		rfid_rc522_clear_register_bit_mask(RFCfgReg, (0x07<<4));

		rfid_rc522_set_register_bit_mask(RFCfgReg, RxGain_value);	// only set RxGain[2:0] bits
	}
}

/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with PICCs
/////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief	Executes the Transceive command.
 * 			CRC validation can only be done if backData and backLen are specified.
 * @param	transmit_data			Pointer to the data to transfer to the FIFO.
 * @param	transmit_byte_count		Number of bytes to transfer to the FIFO.
 * @param	receive_data			nullptr or pointer to buffer if data should be read back after executing the command.
 * @param	receive_byte_count		In: Max number of bytes to write to *backData. Out: The number of bytes returned.
 * @param	valid_bit				In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default nullptr.
 * @param	rxAlign_value			In: Defines the bit position in backData[0] for the first bit received. Default 0.
 * @param	check_crc				In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
 * @return 	STATUS_OK on success, STATUS_??? otherwise.
 */
enum rfid_rc522_status_code rfid_rc522_transceive_data(uint8_t *transmit_data, uint8_t transmit_byte_count, uint8_t *receive_data, uint8_t *receive_byte_count, uint8_t *valid_bit, uint8_t rxAlign_value, bool check_crc)
{
	uint8_t waitIRq_bit = 0x30;		// RxIRq and IdleIRq
	return rfid_rc522_communicate_with_rfid_card(Transceive, waitIRq_bit, transmit_data, transmit_byte_count, receive_data, receive_byte_count, valid_bit, rxAlign_value, check_crc);
}

/**
 * @brief	Transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
 * 			CRC validation can only be done if backData and backLen are specified.
 * @param	command					The command to execute. One of the PCD_Command enums.
 * @param	waitIRq_bit				The bits in the ComIrqReg register that signals successful completion of the command.
 * @param	transmit_data			Pointer to the data to transfer to the FIFO.
 * @param	transmit_byte_count		Number of bytes to transfer to the FIFO.
 * @param	receive_data			nullptr or pointer to buffer if data should be read back after executing the command.
 * @param	receive_byte_count		In: Max number of bytes to write to *backData. Out: The number of bytes returned.
 * @param	valid_bit				In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default nullptr.
 * @param	rxAlign_value			In: Defines the bit position in backData[0] for the first bit received. Default 0.
 * @param	check_crc				In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
 * @return 	STATUS_OK on success, STATUS_??? otherwise.
 */
enum rfid_rc522_status_code rfid_rc522_communicate_with_rfid_card(uint8_t command, uint8_t waitIRq_bit, uint8_t *transmit_data, uint8_t transmit_byte_count, uint8_t *receive_data, uint8_t *receive_byte_count, uint8_t *valid_bit, uint8_t rxAlign_value, bool check_crc)
{
	// Prepare values for BitFramingReg
	uint8_t txLastBits_bit = (valid_bit != 0) ? *valid_bit : 0;
	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
	uint8_t BitFramingReg_value = (rxAlign_value << 4) + txLastBits_bit;
	
	// Stop any active command.
	rfid_rc522_write_one_data(CommandReg, Idle);
	// Clear all seven interrupt request bits
	rfid_rc522_write_one_data(ComIrqReg, 0x7F);
	// FlushBuffer = 1, FIFO initialization
	rfid_rc522_write_one_data(FIFOLevelReg, 0x80);
	// Write sendData to the FIFO
	rfid_rc522_write_many_data(FIFODataReg, transmit_data, transmit_byte_count);
	// Bit adjustments
	rfid_rc522_write_one_data(BitFramingReg, BitFramingReg_value);

	// Execute the command
	rfid_rc522_write_one_data(CommandReg, command);
	if (command == Transceive)
	{
		// BitFramingReg[7] = StartSend = 1, transmission of data starts
		rfid_rc522_set_register_bit_mask(BitFramingReg, 0x80);
	}
	
	// In rfid_rc522_init() we set the TAuto flag in TModeReg. This means the timer
	// automatically starts when the rfid_rc522 stops transmitting.
	//
	// Wait here for the command to complete. The bits specified in the
	// `waitIRq` parameter define what bits constitute a completed command.
	// When they are set in the ComIrqReg register, then the command is
	// considered complete. If the command is not indicated as complete in
	// ~36ms, then consider the command as timed out.
	const uint32_t complete_time = HAL_GetTick() + 55;
	bool command_is_completed = false;

	do 
	{
		// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
		uint8_t ComIrqReg_value = rfid_rc522_read_one_data(ComIrqReg);
		// One of the interrupts that signal success has been set.
		if (ComIrqReg_value & waitIRq_bit)
		{
			command_is_completed = true;
			break;
		}
		// Timer interrupt - nothing received in 25ms
		if (ComIrqReg_value & 0x01)
		{
			return STATUS_TIMEOUT;
		}
	}
	while (HAL_GetTick() < complete_time);

	// 36ms and nothing happened. Communication with the MFRC522 might be down.
	if (!command_is_completed)
	{
		return STATUS_INTERNAL_ERROR;
	}
	
	// ERROR HANDLING
	// Stop now if any errors except collisions were detected.
	// ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
	uint8_t ErrorReg_value = rfid_rc522_read_one_data(ErrorReg);
	
	// Tell about BufferOvfl ParityErr ProtocolErr error
	if (ErrorReg_value & 0x13)	// 0x13 = BufferOvfl ParityErr ProtocolErr
	{	 						
		return STATUS_ERROR;
	}

	// DATA RETURNING FROM FIFO TO MCU
	// If the caller wants data back, get it from the MFRC522.
	uint8_t _validBits = 0;
	if (receive_data && receive_byte_count)
	{
		// Number of bytes in the FIFO
		uint8_t FIFODataReg_byte_count = rfid_rc522_read_one_data(FIFOLevelReg);
		
		if (FIFODataReg_byte_count > *receive_byte_count)
		{
			return STATUS_NO_ROOM;
		}

		// Number of bytes returned
		*receive_byte_count = FIFODataReg_byte_count;
		// Get received data from FIFO
		rfid_rc522_read_many_data(FIFODataReg, receive_data, FIFODataReg_byte_count, rxAlign_value);

		/**
		 * @brief _validBits:
		 * RxLastBits[2:0] indicates the number of valid bits in the last received byte.
		 * If this value is 000b, the whole byte is valid.
		 */
		_validBits = rfid_rc522_get_register_bit_mask(ControlReg, 0x07);
		if (valid_bit)
		{
			*valid_bit = _validBits;
		}
	}

	// Tell about collisions
	if (ErrorReg_value & 0x08)	// 0x08 = CollErr
	{	
		return STATUS_COLLISION;
	}

	// Perform CRC_A validation if requested.
	if (receive_data && receive_byte_count && check_crc)
	{
		// In this case a MIFARE Classic NAK is not OK.
		if (*receive_byte_count == 1 && _validBits == 4)
		{
			return STATUS_MIFARE_NACK;
		}
		// We need at least the CRC_A value and all 8 bits of the last byte must be received.
		if (*receive_byte_count < 2 || _validBits != 0)
		{
			return STATUS_CRC_WRONG;
		}
		// Verify CRC_A - do our own calculation and store the control in crc_control_buffer.
		uint8_t crc_control_buffer[2];
		enum rfid_rc522_status_code status = rfid_rc522_calculate_crc(&receive_data[0], *receive_byte_count - 2, &crc_control_buffer[0]);
		if (status != STATUS_OK)
		{
			return status;
		}
		if ((receive_data[*receive_byte_count - 2] != crc_control_buffer[0]) || (receive_data[*receive_byte_count - 1] != crc_control_buffer[1]))
		{
			return STATUS_CRC_WRONG;
		}
	}
	
	return STATUS_OK;
}

/**
 * @brief	a REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
 * 			Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * @param	buffer_ATQA			The buffer to store the ATQA (Answer to request) in
 * @param	buffer_byte_count	Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
 * @return 	STATUS_OK on success, STATUS_??? otherwise.
 */
enum rfid_rc522_status_code rfid_card_request_A(uint8_t *buffer_ATQA, uint8_t *buffer_byte_count)
{
	return rfid_card_REQA_or_WUPA(PICC_CMD_REQA, buffer_ATQA, buffer_byte_count);
}

/**
 * @brief 	Transmits a Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
 * 			Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * @param	buffer_ATQA			The buffer to store the ATQA (Answer to request) in
 * @param	buffer_byte_count	Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
 * @return 	STATUS_OK on success, STATUS_??? otherwise.
 */
enum rfid_rc522_status_code rfid_card_wake_up_A(uint8_t *buffer_ATQA, uint8_t *buffer_byte_count)
{
	return rfid_card_REQA_or_WUPA(PICC_CMD_WUPA, buffer_ATQA, buffer_byte_count);
}

/**
 * @brief 	Transmits REQA or WUPA commands.
 * 			Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * @param	command				The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
 * @param 	buffer_ATQA			The buffer to store the ATQA (Answer to request) in
 * @param	buffer_byte_count	Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */ 
enum rfid_rc522_status_code rfid_card_REQA_or_WUPA(uint8_t command, uint8_t *buffer_ATQA, uint8_t *buffer_byte_count)
{
	uint8_t valid_bit;
	enum rfid_rc522_status_code command_status;
	
	// The ATQA response is 2 bytes long.
	if (buffer_ATQA == NULL || *buffer_byte_count < 2)
	{
		return STATUS_NO_ROOM;
	}

	// ValuesAfterColl=1 => Bits received after collision are cleared.
	rfid_rc522_clear_register_bit_mask(CollReg, 0x80);
	// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]	
	valid_bit = 7;
	command_status = rfid_rc522_transceive_data(&command, 1, buffer_ATQA, buffer_byte_count, &valid_bit, 0, false);

	if (command_status != STATUS_OK)
	{
		return command_status;
	}
	if (*buffer_byte_count != 2 || valid_bit != 0)	// ATQA must be exactly 16 bits.
	{		
		return STATUS_ERROR;
	}
	return STATUS_OK;
}

/**
 * @brief 	Transmits SELECT/ANTICOLLISION commands to select a single PICC.
 * 			Before calling this function the PICCs must be placed in the READY(*) state by calling PICC_RequestA() or PICC_WakeupA().
 * 			On success:
 * 			- The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
 * 			- The UID size and value of the chosen PICC is returned in *uid along with the SAK.
 * 
 * 			A PICC UID consists of 4, 7 or 10 bytes.
 * 			Only 4 bytes can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
 * 			UID size	Number of UID bytes		Cascade levels		Example of PICC
 * 			========	===================		==============		===============
 * 			single				 4						1				MIFARE Classic
 * 			double				 7						2				MIFARE Ultralight
 * 			triple				10						3				Not currently in use?
 * @param	uid	            Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
 *                          If set you must also supply uid->size. You must set the uid_bit_count, if give it 0 the command will run in loop.
 * @return 	STATUS_OK on success, STATUS_??? otherwise.
 */
/*enum rfid_rc522_status_code rfid_card_read_serial(struct UID *uid)
{
	bool uid_is_complete;
	bool select_is_done;
	bool is_use_cascade_tag;
	uint8_t uid_bit_count = 0;
	uint8_t cascade_level = 1;
	enum rfid_rc522_status_code result_status;
	uint8_t count;
	uint8_t check_bit;
	uint8_t card_command_index;
	uint8_t uid_data_index;				    // The first index in uid->data_aray[] that is used in the current Cascade Level.
	int8_t current_level_uid_bit_count;		// The number of only known UID bits in the current Cascade Level, no counting command.
	uint8_t card_command_array[9];			// The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
	uint8_t card_command_byte_count;		// The number of bytes used in the card_command_array, ie the number of bytes to transfer to the FIFO.
	uint8_t rxAlign_value;					// Used in BitFramingReg. Defines the bit position for the first bit received.
	uint8_t txLastBits_value;				// Used in BitFramingReg. The number of valid bits in the last transmitted byte. 
	uint8_t *card_response_ptr;
	uint8_t card_response_byte_count;
	
	// Description of card_command_array structure:
	//		Byte 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
	//		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits. 
	//		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
	//		Byte 3: UID-data
	//		Byte 4: UID-data
	//		Byte 5: UID-data
	//		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
	//		Byte 7: CRC_A
	//		Byte 8: CRC_A
	// The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
	//
	// Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
	//		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
	//		========	=============	=====	=====	=====	=====
	//		 4 bytes		1			uid0	uid1	uid2	uid3
	//		 7 bytes		1			CT		uid0	uid1	uid2
	//						2			uid3	uid4	uid5	uid6
	//		10 bytes		1			CT		uid0	uid1	uid2
	//						2			CT		uid3	uid4	uid5
	//						3			uid6	uid7	uid8	uid9
	
	// Sanity checks
	if (uid_bit_count > 80) {
		return STATUS_INVALID;
	}
	
	// Prepare MFRC522
	rfid_rc522_clear_register_bit_mask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	
	// Repeat Cascade Level loop until we have a complete UID.
	uid_is_complete = false;
	while (uid_is_complete == false)
	{
		// Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
		switch (cascade_level)
		{
			case 1:
				card_command_array[0] = PICC_CMD_SEL_CL1;
				uid_data_index = 0;
				is_use_cascade_tag = uid_bit_count && uid->byte_count > 4;	// When we know that the UID has more than 4 bytes
				break;
			
			case 2:
				card_command_array[0] = PICC_CMD_SEL_CL2;
				uid_data_index = 3;     // Look at the table, if use 2 level, the first index of the data only on the 2nd level is 3.
				is_use_cascade_tag = uid_bit_count && uid->byte_count > 7;	// When we know that the UID has more than 7 bytes
				break;
			
			case 3:
				card_command_array[0] = PICC_CMD_SEL_CL3;
				uid_data_index = 6;     // Look at the table, if use 3 level, the first index of the data only on the 2nd level is 6.
				is_use_cascade_tag = false;						// Never used in CL3.
				break;
			
			default:
				return STATUS_INTERNAL_ERROR;
				break;
		}
		
		// How many UID bits are known in this Cascade Level?
		current_level_uid_bit_count = uid_bit_count - (8 * uid_data_index);
		if (current_level_uid_bit_count < 0)
		{
			current_level_uid_bit_count = 0;
		}
		// Copy the known bits from uid->uidByte[] to card_command_array[]
		card_command_index = 2; // destination index in card_command_array[]
		if (is_use_cascade_tag) 
		{
			card_command_array[card_command_index++] = PICC_CMD_CT;
		}
		uint8_t uid_bytes_to_copy = (current_level_uid_bit_count / 8) + (current_level_uid_bit_count % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
		if (uid_bytes_to_copy)
		{
			uint8_t max_bytes_to_copy = is_use_cascade_tag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
			if (uid_bytes_to_copy > max_bytes_to_copy)
			{
				uid_bytes_to_copy = max_bytes_to_copy;
			}
			for (count = 0; count < uid_bytes_to_copy; count++)
			{
				card_command_array[card_command_index++] = uid->data_array[uid_data_index + count];
			}
		}
		// Now that the data has been copied we need to include the 8 bits in CT in current_level_uid_bit_count
		if (is_use_cascade_tag)
		{
			current_level_uid_bit_count += 8;
		}
		
		// Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
		select_is_done = false;
		while (select_is_done == false)
		{
			// FIRST, when we first communicate with the PICC, we will send a
			// ANTICOLLISION command to the PICC. If there is no collision
			// the PICC will send back the complete UID_data, if there is,
			// We will go to ANTICOLLISION loop ie this while (select_is_done == false) loop
			// untill the collision is resolve then we can only go forward. If the collision is not
			// resolve, this algorithim can not pick one card out but will go to the eternal loop due to
			// it post-anticollison process that you will see down there.
			if (current_level_uid_bit_count < 32)
			{
				txLastBits_value			= current_level_uid_bit_count % 8; //4
				count						= current_level_uid_bit_count / 8;	// Number of whole bytes in the UID part. //1
				card_command_index			= 2 + count;						// Number of whole bytes: SEL + NVB + UIDs //3
				card_command_array[1]		= (card_command_index << 4) + txLastBits_value;	// NVB - Number of Valid Bits //3B + 4b
				card_command_byte_count		= card_command_index + (txLastBits_value ? 1 : 0);

				// Store response in the unused part of card_command_array
				card_response_ptr			= &card_command_array[card_command_index];
				card_response_byte_count	= sizeof(card_command_array) - card_command_index;
			}
			else //current_level_uid_bit_count >= 32
			{	
				// Find out how many bits and bytes to send and receive.
				// All UID bits in this Cascade Level are known. This is a SELECT.

				// SECCOND, after the PICC send back the uid, PCD will resent the
				// UID data with SEL and NVB before it, the PICC then will send back the
				// SAK if the uid_data resent is the same with the uid_data the PICC have.
				// In SAK, the PICC will let you know if there is a cascade level so that the
				// program can continue untill all the uid_data is extracted from the PICC.

				// NVB - Number of Valid Bits: Seven whole bytes, do not count 2bytes from crc_A
				card_command_array[1] = 0x70;
				
				// Calculate BCC - Block Check Character
				//card_command_array[6] = card_command_array[2] ^ card_command_array[3] ^ card_command_array[4] ^ card_command_array[5];
				
				// Calculate CRC_A
				result_status = rfid_rc522_calculate_crc(card_command_array, 7, &card_command_array[7]);
				if (result_status != STATUS_OK)
				{
					return result_status;
				}

				// 0 => All 8 bits are valid.
				txLastBits_value			= 0;
				card_command_byte_count		= 9;

				// Store response in the last 3 bytes of card_command_array (BCC and CRC_A - not needed after tx)
				// The response is not the uid_data in this case but the SAK of the PICC.
				// Remind: SAK only be sent back from PICC if the PCD sent the full uid_data in that
				// cascade level.
				card_response_ptr			= &card_command_array[6];
				card_response_byte_count	= 3;
			}
			
			// Setting up the rfid_rc522 to send the card_command_array that
			// we prepare above.
			// Set bit adjustments
			// Having a separate variable is overkill. But it makes the next line easier to read.
			rxAlign_value = txLastBits_value;
			// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
			rfid_rc522_write_one_data(BitFramingReg, (rxAlign_value << 4) + txLastBits_value);
			
			// Transmit the card_command_array and receive the response.
			result_status = rfid_rc522_transceive_data(card_command_array, card_command_byte_count, card_response_ptr, &card_response_byte_count, &txLastBits_value, rxAlign_value, false);
			
			// If there is more than one PICC in the field => collision.
			// This is our post-collision procedure.
			if (result_status == STATUS_COLLISION)
			{ 
				// CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
				uint8_t CollReg_data = rfid_rc522_read_one_data(CollReg);
				// CollPosNotValid
				if (CollReg_data & 0x20)
				{ 
					// Without a valid collision position we cannot continue
					return STATUS_COLLISION; 
				}
				// Values 0-31, 0 means bit 32.
				uint8_t CollPos_value = CollReg_data & 0x1F;
				if (CollPos_value == 0)
				{
					CollPos_value = 32;
				}
				// No progress - should not happen 
				if (CollPos_value <= current_level_uid_bit_count)
				{ 
					return STATUS_INTERNAL_ERROR;
				}

				// After able to dettect collision, we choose the PICC.
				// FIRST, we set up the card_command_buffer so that we
				// continue send the already reapeted byte and bit.
				//Ex = 12b -> 1B + 4b 
				current_level_uid_bit_count				= CollPos_value;
				// The bit to modify
				count									= current_level_uid_bit_count % 8; //4
				check_bit								= (current_level_uid_bit_count - 1) % 8; //3
				// First byte is card_command_index 0.
				// If = 3B => 1B SEL + 1B NVB + 1B uid_data + 1B uid_data collision
				card_command_index						= 1 + (current_level_uid_bit_count / 8) + (count ? 1 : 0);

				// SECCOND, since there only 2 option in that collision bit position
				// either 0 or 1, so in this case we always default the CollPos bit is 1.
				// So any PICC that at CollPos bit is 0 will be never chosen by our algorithm.
				// The only way for the CollPos bit 0 to be chosen is in the case where there is
				// no collision at all!
				// If the collision is still happen, as you can see the no PICC will be able to select
				// since at the collision bit position, we default it = 1. So the only way for any PICC
				// to be selected is collision is resolved.
				// If CollPoss = 4 in that byte, check_bit = 3, xxxx x[repeated bit] | 0000 1000
				card_command_array[card_command_index]	|= (1 << check_bit);
			}
			else if (result_status != STATUS_OK)
			{
				return result_status;
			}
			else // STATUS_OK
			{ 
				if (current_level_uid_bit_count >= 32) // This was a SELECT.
				{ 
					select_is_done = true;
					// No more anticollision 
					// We continue below outside the while.
				}
				else
				{	// After send ANTICOLLISION command and there is no collision dettected
					// we can proudly say the current_level_uid_bit_count = 32 and proceed.
					current_level_uid_bit_count = 32;
					// Run loop again to do the SELECT.
				}
			}
		} // End of while (!select_is_done)
		
		// We do not check the CBB - it was constructed by us above.
		
		// Copy the found UID bytes from card_command_array[] to uid->uidByte[]
		// 
		card_command_index	= (card_command_array[2] == PICC_CMD_CT) ? 3 : 2; // source index in card_command_array[]
		uid_bytes_to_copy	= (card_command_array[2] == PICC_CMD_CT) ? 3 : 4;
		for (count = 0; count < uid_bytes_to_copy; count++)
		{
			uid->data_array[uid_data_index + count] = card_command_array[card_command_index++];
		}
		
		// Check response SAK (Select Acknowledge)
		if (card_response_byte_count != 3 || txLastBits_value != 0) // SAK must be exactly 24 bits (1 byte + CRC_A).
		{
			return STATUS_ERROR;
		}

		// Verify CRC_A - do our own calculation and store the control in card_command_array[2..3] - those bytes are not needed anymore.
		result_status = rfid_rc522_calculate_crc(card_response_ptr, 1, &card_command_array[2]);
		if (result_status != STATUS_OK)
		{
			return result_status;
		}
		if ((card_command_array[2] != card_response_ptr[1]) || (card_command_array[3] != card_response_ptr[2]))
		{
			return STATUS_CRC_WRONG;
		}
		if (card_response_ptr[0] & 0x04) // Cascade bit set - UID not complete yes
		{
			cascade_level++;
		}
		else
		{
			uid_is_complete = true;
			uid->select_acknowledge_data = card_response_ptr[0];
		}
	} // End of while (!uid_is_complete)
	
	// Set correct uid->size
	uid->byte_count = 3 * cascade_level + 1;

	return STATUS_OK;
}*/

/**
 * @brief 	Transmits SELECT/ANTICOLLISION commands to select a single PICC.
 * 			Before calling this function the PICCs must be placed in the READY(*) state by calling PICC_RequestA() or PICC_WakeupA().
 * 			On success:
 * 			- The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
 * 			- The UID size and value of the chosen PICC is returned in *uid along with the SAK.
 * 
 * 			A PICC UID consists of 4, 7 or 10 bytes.
 * 			Only 4 bytes can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
 * 			UID size	Number of UID bytes		Cascade levels		Example of PICC
 * 			========	===================		==============		===============
 * 			single				 4						1				MIFARE Classic
 * 			double				 7						2				MIFARE Ultralight
 * 			triple				10						3				Not currently in use?
 * @param	uid	            Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
 *                          If set you must also supply uid->size. You must set the uid_bit_count, if give it 0 the command will run in loop.
 * @return 	STATUS_OK on success, STATUS_??? otherwise.
 */
enum rfid_rc522_status_code rfid_rc522_read_serial(struct UID *uid)
{
	bool uid_is_complete;
	bool select_is_done;
	uint8_t uid_bit_count = 0;
	uint8_t cascade_level = 1;
	enum rfid_rc522_status_code result_status;
	uint8_t count;
	uint8_t check_bit;
	uint8_t card_command_index;
	uint8_t uid_data_index;				    // The first index in uid->data_aray[] that is used in the current Cascade Level.
	int8_t current_level_uid_bit_count;		// The number of only known UID bits in the current Cascade Level, no counting command.
	uint8_t card_command_array[9];			// The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
	uint8_t card_command_byte_count;		// The number of bytes used in the card_command_array, ie the number of bytes to transfer to the FIFO.
	uint8_t rxAlign_value;					// Used in BitFramingReg. Defines the bit position for the first bit received.
	uint8_t txLastBits_value;				// Used in BitFramingReg. The number of valid bits in the last transmitted byte. 
	uint8_t *card_response_ptr;
	uint8_t card_response_byte_count;
	
	// Description of card_command_array structure:
	//		Byte 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
	//		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits. 
	//		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
	//		Byte 3: UID-data
	//		Byte 4: UID-data
	//		Byte 5: UID-data
	//		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
	//		Byte 7: CRC_A
	//		Byte 8: CRC_A
	// The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
	//
	// Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
	//		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
	//		========	=============	=====	=====	=====	=====
	//		 4 bytes		1			uid0	uid1	uid2	uid3
	//		 7 bytes		1			CT		uid0	uid1	uid2
	//						2			uid3	uid4	uid5	uid6
	//		10 bytes		1			CT		uid0	uid1	uid2
	//						2			CT		uid3	uid4	uid5
	//						3			uid6	uid7	uid8	uid9
	
// Prepare MFRC522
	rfid_rc522_clear_register_bit_mask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	
	// Repeat Cascade Level loop until we have a complete UID.
	uid_is_complete = false;
	while (uid_is_complete == false)
	{
		card_command_array[0] = PICC_CMD_SEL_CL1;
		uid_data_index = 0;
		
		// How many UID bits are known in this Cascade Level?
		current_level_uid_bit_count = uid_bit_count - (8 * uid_data_index);
		
		// Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
		select_is_done = false;
		while (select_is_done == false)
		{
			// FIRST, when we first communicate with the PICC, we will send a
			// ANTICOLLISION command to the PICC. If there is no collision
			// the PICC will send back the complete UID_data, if there is,
			// We will go to ANTICOLLISION loop ie this while (select_is_done == false) loop
			// untill the collision is resolve then we can only go forward. If the collision is not
			// resolve, this algorithim can not pick one card out but will go to the eternal loop due to
			// it post-anticollison process that you will see down there.
			txLastBits_value			= current_level_uid_bit_count % 8; //4
			count						= current_level_uid_bit_count / 8;	// Number of whole bytes in the UID part. //1
			card_command_index			= 2 + count;						// Number of whole bytes: SEL + NVB + UIDs //3
			card_command_array[1]		= (card_command_index << 4) + txLastBits_value;	// NVB - Number of Valid Bits //3B + 4b
			card_command_byte_count		= card_command_index + (txLastBits_value ? 1 : 0);

			// Store response in the unused part of card_command_array
			card_response_ptr			= &card_command_array[card_command_index];
			card_response_byte_count	= sizeof(card_command_array) - card_command_index;
			
			// Setting up the rfid_rc522 to send the card_command_array that
			// we prepare above.
			// Set bit adjustments
			// Having a separate variable is overkill. But it makes the next line easier to read.
			rxAlign_value = txLastBits_value;
			// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
			rfid_rc522_write_one_data(BitFramingReg, (rxAlign_value << 4) + txLastBits_value);
			
			// Transmit the card_command_array and receive the response.
			result_status = rfid_rc522_transceive_data(card_command_array, card_command_byte_count, card_response_ptr, &card_response_byte_count, &txLastBits_value, rxAlign_value, false);
			
			// If there is more than one PICC in the field => collision.
			// This is our post-collision procedure.
			if (result_status == STATUS_COLLISION)
			{ 
				// CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
				uint8_t CollReg_data = rfid_rc522_read_one_data(CollReg);
				// CollPosNotValid
				if (CollReg_data & 0x20)
				{ 
					// Without a valid collision position we cannot continue
					return STATUS_COLLISION; 
				}
				// Values 0-31, 0 means bit 32.
				uint8_t CollPos_value = CollReg_data & 0x1F;
				if (CollPos_value == 0)
				{
					CollPos_value = 32;
				}
				// No progress - should not happen 
				if (CollPos_value <= current_level_uid_bit_count)
				{ 
					return STATUS_INTERNAL_ERROR;
				}

				// After able to dettect collision, we choose the PICC.
				// FIRST, we set up the card_command_buffer so that we
				// continue send the already reapeted byte and bit.
				//Ex = 12b -> 1B + 4b 
				current_level_uid_bit_count				= CollPos_value;
				// The bit to modify
				count									= current_level_uid_bit_count % 8; //4
				check_bit								= (current_level_uid_bit_count - 1) % 8; //3
				// First byte is card_command_index 0.
				// If = 3B => 1B SEL + 1B NVB + 1B uid_data + 1B uid_data collision
				card_command_index						= 1 + (current_level_uid_bit_count / 8) + (count ? 1 : 0);

				// SECCOND, since there only 2 option in that collision bit position
				// either 0 or 1, so in this case we always default the CollPos bit is 1.
				// So any PICC that at CollPos bit is 0 will be never chosen by our algorithm.
				// The only way for the CollPos bit 0 to be chosen is in the case where there is
				// no collision at all!
				// If the collision is still happen, as you can see the no PICC will be able to select
				// since at the collision bit position, we default it = 1. So the only way for any PICC
				// to be selected is collision is resolved.
				// If CollPoss = 4 in that byte, check_bit = 3, xxxx x[repeated bit] | 0000 1000
				card_command_array[card_command_index]	|= (1 << check_bit);
			}
			else if (result_status != STATUS_OK)
			{
				return result_status;
			}
			else // STATUS_OK
			{ 
				// This was a SELECT. 
				select_is_done = true;
				// No more anticollision 
				// We continue below outside the while.
			}
		} // End of while (!select_is_done)
		
		// We do not check the CBB - it was constructed by us above.
		
		// Copy the found UID bytes from card_command_array[] to uid->uidByte[]
		// 
		uint8_t uid_bytes_to_copy;
		card_command_index	= (card_command_array[2] == PICC_CMD_CT) ? 3 : 2; // source index in card_command_array[]
		uid_bytes_to_copy	= (card_command_array[2] == PICC_CMD_CT) ? 3 : 4;
		for (count = 0; count < uid_bytes_to_copy; count++)
		{
			uid->data_array[uid_data_index + count] = card_command_array[card_command_index++];
		}

		// Verify CRC_A - do our own calculation and store the control in card_command_array[2..3] - those bytes are not needed anymore.
		uid_is_complete = true;
		uid->select_acknowledge_data = card_response_ptr[0];
	} // End of while (!uid_is_complete)
	
	// Set correct uid->size
	uid->byte_count = 3 * cascade_level + 1;

	return STATUS_OK;
}

/**
 * Instructs a PICC in state ACTIVE(*) to go to state HALT.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */ 
enum rfid_rc522_status_code rfid_card_halt_A()
{
	enum rfid_rc522_status_code result_status;
	uint8_t command_buffer_array[4];
	
	// Build command buffer
	command_buffer_array[0] = PICC_CMD_HLTA;
	command_buffer_array[1] = 0;
	// Calculate CRC_A
	result_status = rfid_rc522_calculate_crc(command_buffer_array, 2, &command_buffer_array[2]);
	if (result_status != STATUS_OK)
	{
		return result_status;
	}
	
	// Send the command.
	// The standard says:
	//		If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
	//		HLTA command, this response shall be interpreted as 'not acknowledge'.
	// We interpret that this way: Only STATUS_TIMEOUT is a success.
	result_status = rfid_rc522_transceive_data(command_buffer_array, sizeof(command_buffer_array), 0, 0, 0, 0, 0);
	if (result_status == STATUS_TIMEOUT)
	{
		return STATUS_OK;
	}
	if (result_status == STATUS_OK) // That is ironically NOT ok in this case ;-)
	{ 
		return STATUS_ERROR;
	}
	return result_status;
}

/////////////////////////////////////////////////////////////////////////////////////
// Convenience functions - does not add extra functionality
/////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief	Wait for card 
*/
enum rfid_rc522_status_code rfid_rc522_wait_for_card()
{
	uint8_t buffer_ATQA[2];
	uint8_t buffer_byte_count = sizeof(buffer_ATQA);

	enum rfid_rc522_status_code result = rfid_card_request_A(buffer_ATQA, &buffer_byte_count);
	return (result == STATUS_OK || result == STATUS_COLLISION );
} // End PICC_IsNewCardPresent()
