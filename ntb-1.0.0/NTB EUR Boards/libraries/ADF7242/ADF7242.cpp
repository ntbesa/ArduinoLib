/***************************************************************************//**
 * @file ADF7242.cpp
 * @version 0.1
 * @author Samuel Kranz
 * @date 11.11.2016
 * @brief This is a .cpp (Arduino) port of the ADF7242 lib (ebreitenstein)
 * @copyright Copyright 2016 NTB Buchs, http://www.ntb.ch/esa. All Rights reserved
 ******************************************************************************/
#include "ADF7242.h"
#include "Arduino.h"

#define MAX_POLL_LOOPS				50
#define SPI_FREQ					  4000000UL

/* All Registers */

#define REG_EXT_CTRL				0x100	/* RW External LNA/PA and internal PA control configuration bits */
#define REG_TX_FSK_TEST				0x101	/* RW TX FSK test mode configuration */
#define REG_FSK_PREAMBLE			0x102	/* RW GFSK/FSK preamble length configuration */
#define REG_CCA1					0x105	/* RW RSSI threshold for CCA */
#define REG_CCA2					0x106	/* RW CCA mode configuration */
#define REG_BUFFERCFG				0x107	/* RW RX_BUFFER overwrite control */
#define REG_PKT_CFG					0x108	/* RW FCS evaluation configuration */
#define REG_DELAYCFG0				0x109	/* RW RC_RX command to SFD or sync word search delay */
#define REG_DELAYCFG1				0x10A	/* RW RC_TX command to TX state */
#define REG_DELAYCFG2				0x10B	/* RW Mac delay extention */
#define REG_SYNC_WORD0				0x10C	/* RW sync word bits [7:0] of [23:0]  */
#define REG_SYNC_WORD1				0x10D	/* RW sync word bits [15:8] of [23:0]  */
#define REG_SYNC_WORD2				0x10E	/* RW sync word bits [23:16] of [23:0]  */
#define REG_SYNC_CONFIG				0x10F	/* RW sync word configuration */
#define REG_FSK_PREAMBLE_CONFIG		0x111
#define REG_RC_CFG					0x13E	/* RW RX / TX packet configuration */
#define REG_RC_VAR44				0x13F	/* RW RESERVED */
#define REG_CH_FREQ0				0x300	/* RW Channel Frequency Settings - Low Byte */
#define REG_CH_FREQ1				0x301	/* RW Channel Frequency Settings - Middle Byte */
#define REG_CH_FREQ2				0x302	/* RW Channel Frequency Settings - 2 MSBs */
#define REG_TX_FD					0x304	/* RW TX Frequency Deviation Register */
#define REG_DM_CFG0					0x305	/* RW RX Discriminator BW Register */
#define REG_TX_M					0x306	/* RW TX Mode Register */
#define REG_RX_M					0x307	/* RW RX Mode Register */
#define REG_RRB						0x30C	/* R RSSI Readback Register */
#define REG_LRB						0x30D	/* R Link Quality Readback Register */
#define REG_DR0						0x30E	/* RW bits [15:8] of [15:0] for data rate setting */
#define REG_DR1						0x30F	/* RW bits [7:0] of [15:0] for data rate setting */
#define REG_PRAMPG					0x313	/* RW RESERVED */
#define REG_TXPB					0x314	/* RW TX Packet Storage Base Address */
#define REG_RXPB					0x315	/* RW RX Packet Storage Base Address */
#define REG_TMR_CFG0				0x316	/* RW Wake up Timer Configuration Register - High Byte */
#define REG_TMR_CFG1				0x317	/* RW Wake up Timer Configuration Register - Low Byte */
#define REG_TMR_RLD0				0x318	/* RW Wake up Timer Value Register - High Byte */
#define REG_TMR_RLD1				0x319	/* RW Wake up Timer Value Register - Low Byte */
#define REG_TMR_CTRL				0x31A	/* RW Wake up Timer Timeout flag */
#define REG_PD_AUX					0x31E	/* RW Battmon enable */
#define REG_GP_CFG					0x32C	/* RW GPIO Configuration */
#define REG_GP_OUT					0x32D	/* RW GPIO Configuration */
#define REG_GP_IN					0x32E	/* R GPIO Configuration */
#define REG_SYNT					0x335	/* RW bandwidth calibration timers */
#define REG_CAL_CFG					0x33D	/* RW Calibration Settings */
#define REG_PA_BIAS					0x36E	/* RW PA BIAS */
#define REG_SYNT_CAL				0x371	/* RW Oscillator and Doubler Configuration */
#define REG_IIRF_CFG				0x389	/* RW BB Filter Decimation Rate */
#define REG_CDR_CFG					0x38A	/* RW CDR kVCO */
#define REG_DM_CFG1					0x38B	/* RW Postdemodulator Filter */
#define REG_AGCSTAT					0x38E	/* R RXBB Ref Osc Calibration Engine Readback */
#define REG_RXCAL0					0x395	/* RW RX BB filter tuning, LSB */
#define REG_RXCAL1					0x396	/* RW RX BB filter tuning, MSB */
#define REG_RXFE_CFG				0x39B	/* RW RXBB Ref Osc & RXFE Calibration */
#define REG_PA_RR					0x3A7	/* RW Set PA ramp rate */
#define REG_PA_CFG					0x3A8	/* RW PA enable */
#define REG_EXTPA_CFG				0x3A9	/* RW External PA BIAS DAC */
#define REG_EXTPA_MSC				0x3AA	/* RW PA Bias Mode */
#define REG_ADC_RBK					0x3AE	/* R Readback temp */
#define REG_AGC_CFG1				0x3B2	/* RW GC Parameters */
#define REG_AGC_MAX					0x3B4	/* RW Slew rate  */
#define REG_AGC_CFG2				0x3B6	/* RW RSSI Parameters */
#define REG_AGC_CFG3				0x3B7	/* RW RSSI Parameters */
#define REG_AGC_CFG4				0x3B8	/* RW RSSI Parameters */
#define REG_AGC_CFG5				0x3B9	/* RW RSSI & NDEC Parameters */
#define REG_AGC_CFG6				0x3BA	/* RW NDEC Parameters */
#define REG_AGC_CFG7				0x3BC
#define REG_OCL_CFG0				0x38F
#define REG_OCL_CFG1				0x3C4	/* RW OCL System Parameters */
#define REG_IRQ1_EN0				0x3C7	/* RW Interrupt Mask set bits  [7:0] of [15:0] for IRQ1 */
#define REG_IRQ1_EN1				0x3C8	/* RW Interrupt Mask set bits  [15:8] of [15:0] for IRQ1 */
#define REG_IRQ2_EN0				0x3C9	/* RW Interrupt Mask set bits  [7:0] of [15:0] for IRQ2 */
#define REG_IRQ2_EN1				0x3CA	/* RW Interrupt Mask set bits  [15:8] of [15:0] for IRQ2 */
#define REG_IRQ1_SRC0				0x3CB	/* RW Interrupt Source  bits  [7:0] of [15:0] for IRQ */
#define REG_IRQ1_SRC1				0x3CC	/* RW Interrupt Source bits  [15:8] of [15:0] for IRQ */
#define REG_OCL_BW0					0x3D2	/* RW OCL System Parameters */
#define REG_OCL_BW1					0x3D3	/* RW OCL System Parameters */
#define REG_OCL_BW2					0x3D4	/* RW OCL System Parameters */
#define REG_OCL_BW3					0x3D5	/* RW OCL System Parameters */
#define REG_OCL_BW4					0x3D6	/* RW OCL System Parameters */
#define REG_OCL_BWS					0x3D7	/* RW OCL System Parameters */
#define REG_OCL_CFG13				0x3E0	/* RW OCL System Parameters */
#define REG_GP_DRV					0x3E3	/* RW I/O pads Configuration and bg trim */
#define REG_BM_CFG					0x3E6	/* RW Battery Monitor Threshold Voltage setting */
#define REG_PREAMBLE_NUM_VALIDATE	0x3F3	/* RW Preabmle validation */
#define REG_SFD_15_4				0x3F4	/* RW Option to set non standard SFD */
#define REG_AFC_CFG					0x3F7	/* RW AFC mode and polarity */
#define REG_AFC_KI_KP				0x3F8	/* RW AFC ki and kp */
#define REG_AFC_RANGE				0x3F9	/* RW AFC range */
#define REG_AFC_READ				0x3FA	/* RW Readback frequency error */

/* REG_EXTPA_MSC */
#define PA_PWR(x)					(((x) & 0xF) << 4)
#define EXTPA_BIAS_SRC				(1 << 3)
#define EXTPA_BIAS_MODE(x)			(((x) & 0x7) << 0)

/* REG_PA_CFG */
#define PA_BRIDGE_DBIAS(x)			(((x) & 0x1F) << 0)

/* REG_PA_BIAS */
#define PA_BIAS_CTRL(x)				(((x) & 0x1F) << 1)
#define REG_PA_BIAS_DFL				(1 << 0)

#define REG_PAN_ID0					0x112
#define REG_PAN_ID1					0x113
#define REG_SHORT_ADDR_0			0x114
#define REG_SHORT_ADDR_1			0x115
#define REG_IEEE_ADDR_0				0x116
#define REG_IEEE_ADDR_1				0x117
#define REG_IEEE_ADDR_2				0x118
#define REG_IEEE_ADDR_3				0x119
#define REG_IEEE_ADDR_4				0x11A
#define REG_IEEE_ADDR_5				0x11B
#define REG_IEEE_ADDR_6				0x11C
#define REG_IEEE_ADDR_7				0x11D
#define REG_FFILT_CFG				0x11E
#define REG_AUTO_CFG				0x11F
#define REG_AUTO_TX1				0x120
#define REG_AUTO_TX2				0x121
#define REG_AUTO_STATUS				0x122

/* REG_FFILT_CFG */
#define ACCEPT_BEACON_FRAMES		(1 << 0)
#define ACCEPT_DATA_FRAMES			(1 << 1)
#define ACCEPT_ACK_FRAMES			(1 << 2)
#define ACCEPT_MACCMD_FRAMES		(1 << 3)
#define ACCEPT_RESERVED_FRAMES		(1 << 4)
#define ACCEPT_ALL_ADDRESS			(1 << 5)

/* REG_AUTO_CFG */
#define AUTO_ACK_FRAMEPEND			(1 << 0)
#define IS_PANCOORD					(1 << 1)
#define RX_AUTO_ACK_EN				(1 << 3)
#define CSMA_CA_RX_TURNAROUND		(1 << 4)

/* REG_AUTO_TX1 */
#define MAX_FRAME_RETRIES(x)		((x) & 0xF)
#define MAX_CCA_RETRIES(x)			(((x) & 0x7) << 4)

/* REG_AUTO_TX2 */
#define CSMA_MAX_BE(x)				((x) & 0xF)
#define CSMA_MIN_BE(x)				(((x) & 0xF) << 4)

#define CMD_SPI_NOP					0xFF				/* No operation. Use for dummy writes */
#define CMD_SPI_PKT_WR				0x10				/* Write telegram to the Packet RAM starting from the TX packet base address pointer tx_packet_base */
#define CMD_SPI_PKT_RD				0x30				/* Read telegram from the Packet RAM starting from RX packet base address pointer rxpb.rx_packet_base */
#define CMD_SPI_MEM_WR(x)			(0x18 + (x >> 8))	/* Write data to MCR or Packet RAM sequentially */
#define CMD_SPI_MEM_RD(x)			(0x38 + (x >> 8))	/* Read data from MCR or Packet RAM sequentially */
#define CMD_SPI_MEMR_WR(x)			(0x08 + (x >> 8))	/* Write data to MCR or Packet RAM as random block */
#define CMD_SPI_MEMR_RD(x)			(0x28 + (x >> 8))	/* Read data from MCR or Packet RAM as random block */
#define CMD_SPI_ADDR_MASK     0b0000011111111111 /* Mask for converting 16 bit addresses to 11 bit */
#define CMD_SPI_PRAM_WR				0x1E				/* Write data sequentially to current PRAM page selected */
#define CMD_SPI_PRAM_RD				0x3E				/* Read data sequentially from current PRAM page selected */
#define CMD_RC_SLEEP				0xB1				/* Invoke transition of radio controller into SLEEP state */
#define CMD_RC_IDLE					0xB2				/* Invoke transition of radio controller into IDLE state */
#define CMD_RC_PHY_RDY				0xB3				/* Invoke transition of radio controller into PHY_RDY state */
#define CMD_RC_RX					0xB4				/* Invoke transition of radio controller into RX state */
#define CMD_RC_TX					0xB5				/* Invoke transition of radio controller into TX state */
#define CMD_RC_MEAS					0xB6				/* Invoke transition of radio controller into MEAS state */
#define CMD_RC_CCA					0xB7				/* Invoke Clear channel assessment */
#define CMD_RC_CSMACA				0xC1				/* initiates CSMA-CA channel access sequence and frame transmission */
#define CMD_RC_PC_RESET				0xC7				/* Program counter reset */
#define CMD_RC_RESET				0xC8				/* Resets the ADF7242 and puts it in the sleep state */


/* AUTO_STATUS */

#define SUCCESS						0
#define SUCCESS_DATPEND				1
#define FAILURE_CSMACA				2
#define FAILURE_NOACK				3
#define AUTO_STATUS_MASK			0x3

#define PRAM_PAGESIZE				256

/* IRQ1 */

#define IRQ_CCA_COMPLETE			(1 << 0)
#define IRQ_SFD_RX					(1 << 1)
#define IRQ_SFD_TX					(1 << 2)
#define IRQ_RX_PKT_RCVD				(1 << 3)
#define IRQ_TX_PKT_SENT				(1 << 4)
#define IRQ_FRAME_VALID				(1 << 5)
#define IRQ_ADDRESS_VALID			(1 << 6)
#define IRQ_CSMA_CA					(1 << 7)

#define AUTO_TX_TURNAROUND			(1 << 3)
#define ADDON_EN					(1 << 4)

#define MODE_IDLE     1
#define MODE_MEAS     2
#define MODE_PHY_RDY  3
#define MODE_RX       4
#define MODE_TX       5
#define MODE_BIT_MASK 0x0F


uint16_t rxBaseAddr = 0;
uint16_t txBaseAddr = 0;

SPISettings spiSettings;

//ADF7242_Pin_TypeDef pinCS;
uint8_t pinCS;
//ADF7242_Pin_TypeDef pinINT;

//uint8_t pinCS;

/**************************************************************************//**
 * @brief
 *   Constructor
 *
 * @param[in] ADF7242_Pin_TypeDef pCS
 *   ChipSelect pin of ADF7242
 *
 * @param[in] ADF7242_Pin_TypeDef pINT
 *   Interrupt pin of ADF7242
 *
 *****************************************************************************/
ADF7242::ADF7242()
{
}

/**************************************************************************//**
 * @brief
 *   SPI release ADF7242
 *
 *****************************************************************************/
void ADF7242::spiRelease()
{
	//*pinCS.PORT |= (1<<*pinCS.PIN);
	digitalWrite(pinCS,HIGH);
}

/**************************************************************************//**
 * @brief
 *   SPI select ADF7242
 *
 *****************************************************************************/
void ADF7242::spiSelect()
{
	digitalWrite(pinCS,LOW);
	//*pinCS.PORT &= ~(1<<*pinCS.PIN);
}

/**************************************************************************//**
 * @brief
 *   Get Status from adf7242
 *
 * @details
 *   TODO
 *
 * @return
 *   uint8_t status
 *****************************************************************************/
uint8_t ADF7242::status()
{
	SPI.beginTransaction(spiSettings);
	ADF7242::spiSelect();
	
	SPI.transfer(CMD_SPI_NOP);
	uint8_t status = SPI.transfer(CMD_SPI_NOP); 
	
	ADF7242::spiRelease();
	SPI.endTransaction();
	
	return status;
}

/**************************************************************************//**
 * @brief
 *   Check if adf7242 is ready for RC changes
 *
 * @details
 *   TODO
 *
 * @return 
 *   uint8_t error: 0 for success, 1 for error
 *****************************************************************************/
uint8_t ADF7242::waitReady()
{
	uint8_t stat;
	uint8_t cnt = 0;

	do {
		stat = ADF7242::status();
		cnt++;
	} while (!(stat & STAT_RC_READY) && (cnt < MAX_POLL_LOOPS));

	return (cnt >= MAX_POLL_LOOPS);
}

/**************************************************************************//**
 * @brief
 *   Read register
 *
 * @details
 *   TODO
 *
 * @param[in] uint16_t addr
 *   Register address
 *
 * @param[out] uint8_t *data
 *   Read data
 *
 * @return uint8_t error 
 *   uint8_t error: 0 for success, 1 for error
 *****************************************************************************/
uint8_t ADF7242::readReg(uint16_t addr,uint8_t *data)
{
	uint8_t buf[4] ={	CMD_SPI_MEM_RD(addr),
						addr,
						CMD_SPI_NOP,
						CMD_SPI_NOP
					};
	
	if(ADF7242::waitReady() == 0)
	{
		SPI.beginTransaction(spiSettings);
		ADF7242::spiSelect();
		
		SPI.transfer(buf,4);
		
		ADF7242::spiRelease();
		SPI.endTransaction();
	
		
		*data = (uint8_t*) buf[3];
		return 0;
	}
	else
	{
		return 1;
	}
}

/**************************************************************************//**
 * @brief
 *   Write register
 *
 * @details
 *   TODO
 *
 * @param[in] uint16_t addr
 *   Register address
 *
 * @param[in] uint8_t data
 *   Write data
 *
 * @return uint8_t error 
 *   0 for success, 1 for error
 *****************************************************************************/
uint8_t ADF7242::writeReg(uint16_t addr, uint8_t data)
{
	uint8_t buf[3]={	CMD_SPI_MEM_WR(addr),
						addr,
						data
					};
  
	if(ADF7242::waitReady() == 0){
		
		SPI.beginTransaction(spiSettings);
		ADF7242::spiSelect();
		
		SPI.transfer(buf,3);
		
		ADF7242::spiRelease();
		SPI.endTransaction();

		return 0;
		
	}else
	{
		return 1;
	}
}

/**************************************************************************//**
 * @brief
 *   Write data into the adf7242 TX buffer
 *
 * @details
 *   TODO
 *
 * @param[in] uint8_t data
 *   Data
 *
 * @param[in] uint8_t len
 *   Data length
 *
 * @return uint8_t error 
 *   0 for success, 1 for error
 *****************************************************************************/
uint8_t ADF7242::writeTxBuf(uint8_t *data, uint8_t len)
{
	uint8_t stat = 0;
	
	for (uint8_t i = 0; i < len; i++)
	{
		stat |= ADF7242::writeReg((txBaseAddr + i) & CMD_SPI_ADDR_MASK, data[i]);
	}
	
	return stat;
}

/**************************************************************************//**
 * @brief
 *   Read data from the adf7242 RX buffer
 *
 * @details
 *   TODO
 *
 * @param[out] uint8_t *data
 *   Data
 *
 * @param[in] uint8_t len
 *   Data length
 *
 * @return uint8_t error 
 *   0 for success, 1 for error
 *****************************************************************************/
uint8_t ADF7242::readRxBuf(uint8_t *data, uint8_t len)
{
	uint8_t stat = 0;
	
	for (uint8_t i = 0; i < len; i++)
	{
		stat |= ADF7242::readReg(rxBaseAddr + i, &data[i]);
	}
	return stat;
}

/**************************************************************************//**
 * @brief
 *   Write Command
 *
 * @details
 *   TODO
 *
 * @param[in] uint8_t cmd
 *   Command
 *
 * @return uint8_t error 
 *   0 for success, 1 for error
 *****************************************************************************/
uint8_t ADF7242::writeCmd(uint8_t cmd)
{
	if(ADF7242::waitReady() == 0){
		
		SPI.beginTransaction(spiSettings);
		ADF7242::spiSelect();
		
		SPI.transfer(cmd);		
		
		ADF7242::spiRelease();
		SPI.endTransaction();
		
		return 0;
	}
	else
	{
		return 1;
	}
}

/**************************************************************************//**
 * @brief
 *   Init GFSK/FSK settings that are common for packet and sport mode
 *
 * @details
 *   TODO
 *
 * @return uint8_t error 
 *   0 for success, 1 for error
 *****************************************************************************/
uint8_t ADF7242::initFskConfig()
{
	uint8_t stat = 0;
	
	// datasheet page 67 table 37
	stat |= ADF7242::writeReg(REG_SYNT,0x28);
	stat |= ADF7242::writeReg(REG_AGC_CFG1,0x34);
	stat |= ADF7242::writeReg(REG_AGC_MAX,0x80);
	stat |= ADF7242::writeReg(REG_AGC_CFG2,0x37);
	stat |= ADF7242::writeReg(REG_AGC_CFG3,0x2A);
	stat |= ADF7242::writeReg(REG_AGC_CFG4,0x1D);
	stat |= ADF7242::writeReg(REG_AGC_CFG6,0x24);
	stat |= ADF7242::writeReg(REG_AGC_CFG7,0x7B);
	stat |= ADF7242::writeReg(REG_OCL_CFG0,0x00);
	stat |= ADF7242::writeReg(REG_OCL_CFG1,0x07);
	
	stat |= ADF7242::writeReg(REG_OCL_BW0,0x1A);
	stat |= ADF7242::writeReg(REG_OCL_BW1,0x19);

	stat |= ADF7242::writeReg(REG_OCL_BW2,0x1E);
	stat |= ADF7242::writeReg(REG_OCL_BW3,0x1E);
	stat |= ADF7242::writeReg(REG_OCL_BW4,0x1E);
	stat |= ADF7242::writeReg(REG_OCL_BWS,0x00);
	stat |= ADF7242::writeReg(REG_OCL_CFG13,0xF0);
	stat |= ADF7242::writeReg(REG_PREAMBLE_NUM_VALIDATE,0x01);
	
	return stat;
}

/**************************************************************************//**
 * @brief
 *   Set datarate for GFSK/FSK mode
 *
 * @details
 *   Datarate in kbps. 
 *   Possible are: 	50kbps
 *					62.5kpbs
 *					100kbps
 *					125 kbps
 *					250kbps
 *					500kbps
 *					1000kbps
 *					2000kbps
 *
 * @param[in] ADF7242_DataRate_TypeDef dataRate
 *   The desired datarate
 *
 * @return uint8_t error 
 *   0 for success, 1 for error
 *****************************************************************************/
uint8_t ADF7242::setFskDataRate(ADF7242_DataRate_TypeDef dataRate)
{
	uint8_t stat = 0;
	// datasheet page 67 table 38
	switch(dataRate)
	{
		case DataRate_50kbps:
      stat |= ADF7242::writeReg(REG_FSK_PREAMBLE,0x04);
			stat |= ADF7242::writeReg(REG_TX_FD,0x03);
			stat |= ADF7242::writeReg(REG_DM_CFG0,0x37);
			stat |= ADF7242::writeReg(REG_TX_M,0x00);
			stat |= ADF7242::writeReg(REG_DR0,0x01);
			stat |= ADF7242::writeReg(REG_DR1,0xF4);
			stat |= ADF7242::writeReg(REG_IIRF_CFG,0x17);
			stat |= ADF7242::writeReg(REG_DM_CFG1,0x08);
			stat |= ADF7242::writeReg(REG_RXFE_CFG,0x06);
			break;
			
		case DataRate_62k5bps:
      stat |= ADF7242::writeReg(REG_FSK_PREAMBLE,0x04);
			stat |= ADF7242::writeReg(REG_TX_FD,0x06);
			stat |= ADF7242::writeReg(REG_DM_CFG0,0x37);
			stat |= ADF7242::writeReg(REG_TX_M,0x00);
			stat |= ADF7242::writeReg(REG_DR0,0x02);
			stat |= ADF7242::writeReg(REG_DR1,0x70);
			stat |= ADF7242::writeReg(REG_IIRF_CFG,0x17);
			stat |= ADF7242::writeReg(REG_DM_CFG1,0x08);
			stat |= ADF7242::writeReg(REG_RXFE_CFG,0x06);
			break;
			
		case DataRate_100kbps:
      stat |= ADF7242::writeReg(REG_FSK_PREAMBLE,0x05);
			stat |= ADF7242::writeReg(REG_TX_FD,0x03);
			stat |= ADF7242::writeReg(REG_DM_CFG0,0x6B);
			stat |= ADF7242::writeReg(REG_TX_M,0x00);
			stat |= ADF7242::writeReg(REG_DR0,0x03);
			stat |= ADF7242::writeReg(REG_DR1,0xE8);
			stat |= ADF7242::writeReg(REG_IIRF_CFG,0x17);
			stat |= ADF7242::writeReg(REG_DM_CFG1,0x0D);
			stat |= ADF7242::writeReg(REG_RXFE_CFG,0x06);
			break;
			
		case DataRate_125kbps:
      stat |= ADF7242::writeReg(REG_FSK_PREAMBLE,0x05);
			stat |= ADF7242::writeReg(REG_TX_FD,0x06);
			stat |= ADF7242::writeReg(REG_DM_CFG0,0x37);
			stat |= ADF7242::writeReg(REG_TX_M,0x00);
			stat |= ADF7242::writeReg(REG_DR0,0x04);
			stat |= ADF7242::writeReg(REG_DR1,0xE2);
			stat |= ADF7242::writeReg(REG_IIRF_CFG,0x17);
			stat |= ADF7242::writeReg(REG_DM_CFG1,0x11);
			stat |= ADF7242::writeReg(REG_RXFE_CFG,0x06);
			break;
	
		case DataRate_250kbps:
      stat |= ADF7242::writeReg(REG_FSK_PREAMBLE,0x05);
			stat |= ADF7242::writeReg(REG_TX_FD,0x0D);
			stat |= ADF7242::writeReg(REG_DM_CFG0,0x19);
			stat |= ADF7242::writeReg(REG_TX_M,0x02);
			stat |= ADF7242::writeReg(REG_DR0,0x09);
			stat |= ADF7242::writeReg(REG_DR1,0xC4);
			stat |= ADF7242::writeReg(REG_IIRF_CFG,0x12);
			stat |= ADF7242::writeReg(REG_DM_CFG1,0x20);
			stat |= ADF7242::writeReg(REG_RXFE_CFG,0x06);
			break;
	
		case DataRate_500kbps:
      stat |= ADF7242::writeReg(REG_FSK_PREAMBLE,0x05);
			stat |= ADF7242::writeReg(REG_TX_FD,0x19);
			stat |= ADF7242::writeReg(REG_DM_CFG0,0x0D);
			stat |= ADF7242::writeReg(REG_TX_M,0x03);
			stat |= ADF7242::writeReg(REG_DR0,0x13);
			stat |= ADF7242::writeReg(REG_DR1,0x88);
			stat |= ADF7242::writeReg(REG_IIRF_CFG,0x0A);
			stat |= ADF7242::writeReg(REG_DM_CFG1,0x3D);
			stat |= ADF7242::writeReg(REG_RXFE_CFG,0x06);
			break;
	
		case DataRate_1000kbps:
     stat |= ADF7242::writeReg(REG_FSK_PREAMBLE,0x07);
			stat |= ADF7242::writeReg(REG_TX_FD,0x32);
			stat |= ADF7242::writeReg(REG_DM_CFG0,0x06);
			stat |= ADF7242::writeReg(REG_TX_M,0x03);
			stat |= ADF7242::writeReg(REG_DR0,0x4E);
			stat |= ADF7242::writeReg(REG_DR1,0x20);
			stat |= ADF7242::writeReg(REG_IIRF_CFG,0x05);
			stat |= ADF7242::writeReg(REG_DM_CFG1,0xAA);
			stat |= ADF7242::writeReg(REG_RXFE_CFG,0x06);
			break;
	
		case DataRate_2000kbps:
     stat |= ADF7242::writeReg(REG_FSK_PREAMBLE,0x09);
			stat |= ADF7242::writeReg(REG_TX_FD,0x32);
			stat |= ADF7242::writeReg(REG_DM_CFG0,0x06);
			stat |= ADF7242::writeReg(REG_TX_M,0x03);
			stat |= ADF7242::writeReg(REG_DR0,0x4E);
			stat |= ADF7242::writeReg(REG_DR1,0x20);
			stat |= ADF7242::writeReg(REG_IIRF_CFG,0x05);
			stat |= ADF7242::writeReg(REG_DM_CFG1,0xAA);
			stat |= ADF7242::writeReg(REG_RXFE_CFG,0x0D);
			break;
			
		default:
			stat = 1;
			break;
	}
		
	return stat;
}

/**************************************************************************//**
 * @brief
 *   Init FSK Packet mode on LNA 1
 *
 * @details
 *   Datarate in kbps. 
 *   Possible are: 	50kbps
 *					62.5kpbs
 *					100kbps
 *					125 kbps
 *					250kbps
 *					500kbps
 *					1000kbps
 *					2000kbps
 *
 * @param[in] ADF7242_DataRate_TypeDef dataRate
 *   The desired datarate
 *
 * @return uint8_t error 
 *   0 for success, 1 for error
 *****************************************************************************/
uint8_t ADF7242::initFskPacketMode(ADF7242_DataRate_TypeDef dataRate)
{
	uint8_t stat = 0;
	
	// clear all interrupts
	stat |= ADF7242::writeReg(REG_IRQ1_SRC0,0xFF);
	stat |= ADF7242::writeReg(REG_IRQ1_SRC1,0xFF);
	
	stat |= ADF7242::initFskConfig();

 stat |= ADF7242::writeReg(REG_FSK_PREAMBLE,0x05);
	
	stat |= ADF7242::setFskDataRate(dataRate);
	
	
	// datasheet page 68 table 39 packet mode
	stat |= ADF7242::writeReg(REG_RC_CFG,0x04);
	
	// SYNC WORD
	// datasheet page 68 table 40 
	stat |= ADF7242::writeReg(REG_SYNC_WORD0,0x31);
	stat |= ADF7242::writeReg(REG_SYNC_WORD1,0x7F);
	stat |= ADF7242::writeReg(REG_SYNC_WORD2,0xAA);
  stat |= ADF7242::writeReg(REG_SYNC_CONFIG,0x10);
	//stat |= ADF7242::writeReg(REG_SYNC_CONFIG,0x00);

	
  //datasheet page 68 table 40 
//	stat |= ADF7242::writeReg(REG_AFC_CFG,0x07);
//	stat |= ADF7242::writeReg(REG_AFC_KI_KP,0x99);
//	stat |= ADF7242::writeReg(REG_AFC_RANGE,80);

	
	// set frequenc
	stat |= ADF7242::writeReg(REG_CH_FREQ0, (2450 * 100L) & 0xFC);
	stat |= ADF7242::writeReg(REG_CH_FREQ0, ((2450 * 100L) >> 8) & 0xFF);
	stat |= ADF7242::writeReg(REG_CH_FREQ0, ((2450 * 100L) >> 16) & 0xFF);
	
	// POWER
	stat |= ADF7242::writeReg(REG_EXTPA_MSC, (15 << 4) | 0x01);	
	stat |= ADF7242::writeReg(REG_PA_CFG, 21 | 0x40);
	stat |= ADF7242::writeReg(REG_PA_BIAS, (63 << 1) | 0x01);	
	stat |= ADF7242::writeReg(REG_PA_RR, 7);
	stat |= ADF7242::writeReg(REG_PKT_CFG, 0 | (2 << 1));
	stat |= ADF7242::writeReg(REG_FSK_PREAMBLE_CONFIG, 0x2A);
  //stat |= ADF7242::writeReg(REG_FSK_PREAMBLE_CONFIG, 0x2C);
	
	return stat;
}

/**************************************************************************//**
 * @brief
 *   Set Sport behavior in RX mode
 *
 * @details
 *   rxConfig( set to 1 for txMode) 	1 = IRQ2 not used, Clock not gated; 
 *										2 = IRQ2 goes high when sync match detected, clock gated with preamble; 
 *										3 = IRQ2 goes high when sync match detected, clock gated with sync match
 *
 * @param[in] uint8_t rxConfig
 *   rxConfig
 *
 * @return uint8_t error 
 *   0 for success, 1 for error
 *****************************************************************************/
uint8_t ADF7242::setSportRxConfig(uint8_t rxConfig)
{
	uint8_t stat = 0;
	
	if ( rxConfig == 1 )
	{
		stat |= ADF7242::writeReg(REG_GP_CFG,0x01);
	}
	if ( rxConfig == 2 )
	{
		stat |= ADF7242::writeReg(REG_GP_CFG,0x02);
	}
	if ( rxConfig == 3 )
	{
		stat |= ADF7242::writeReg(REG_GP_CFG,0x03);
	}
	
	return stat;
}

/**************************************************************************//**
 * @brief
 *   Init FSK SPORT mode on LNA1
 *
 * @details
 *   TODO
 *
 * @param[in] ADF7242_DataRate_TypeDef dataRate
 *   The desired datarate
 *
 * @param[in] uint8_t rxConfig
 *   rxConfig
 *
 * @return uint8_t error 
 *   0 for success, 1 for error
 *****************************************************************************/
uint8_t ADF7242::initFskSportMode(ADF7242_DataRate_TypeDef dataRate, uint8_t rxConfig)
{
	uint8_t stat = 0;
	
	stat |= ADF7242::initFskConfig();
	
	// Preamble Config: Bit5: Lock AGC after preamble; Bit 0-3: Preamble bit pair errors allowed=0 
	stat |= ADF7242::writeReg(REG_FSK_PREAMBLE_CONFIG, 0x2C);
	
	// Sync Word Config: Bit 5-6: Number of mismatches allowed 0-3, Bit 0-4: Sync word length, goes from 0 to 24 Bit
	stat |= ADF7242::writeReg(REG_SYNC_CONFIG,0x00);
			
	stat |= ADF7242::setFskDataRate(dataRate);
	
	// Set to FSK SPORT MODE
	stat |= ADF7242::writeReg(REG_RC_CFG,0x03);
	
	// 
	stat |= ADF7242::setSportRxConfig(rxConfig);

	
	stat |= ADF7242::writeReg(REG_CH_FREQ0, (2450 * 100L) & 0xFC);
	stat |= ADF7242::writeReg(REG_CH_FREQ0, ((2450 * 100L) >> 8) & 0xFF);
	stat |= ADF7242::writeReg(REG_CH_FREQ0, ((2450 * 100L) >> 16) & 0xFF);
	
	return stat;
}

/**************************************************************************//**
 * @brief
 *   Set RX/TX Buffer base address
 *
 * @details
 *   TODO
 *
 * @param[in] uint8_t rxBase
 *   Rx base address
 *
 * @param[in] uint8_t txBase
 *   Tx base address
 *
 * @return uint8_t error 
 *   0 for success, 1 for error
 *****************************************************************************/
uint8_t ADF7242::setRxTxBufBase(uint8_t rxBase, uint8_t txBase)
{
	uint8_t stat=0;
	
	rxBaseAddr = (uint16_t) rxBase;
	txBaseAddr = (uint16_t) txBase;
	stat |= ADF7242::writeReg(REG_TXPB,txBase);
	stat |= ADF7242::writeReg(REG_RXPB,rxBase);
	
	return stat;
}

/**************************************************************************//**
 * @brief
 *   Enable RX Interrupt on Output 1
 *
 * @return uint8_t error 
 *   0 for success, 1 for error
 *****************************************************************************/
uint8_t ADF7242::enableRxCompleteInterrupt()
{
	return ADF7242::writeReg(REG_IRQ1_EN1,IRQ_RX_PKT_RCVD);
}

/**************************************************************************//**
 * @brief
 *   Enable TX Interrupt on Output 1
 *
 * @return uint8_t error 
 *   0 for success, 1 for error
 *****************************************************************************/
uint8_t ADF7242::enableTxCompleteInterrupt()
{
	return ADF7242::writeReg(REG_IRQ1_EN1,IRQ_TX_PKT_SENT);
}

/**************************************************************************//**
 * @brief
 *  Disable all interrupts
 *
 * @return uint8_t error 
 *   0 for success, 1 for error
 *****************************************************************************/
uint8_t ADF7242::disableAllInterrupts()
{
  ADF7242::writeReg(REG_IRQ1_EN0,0x00);
  ADF7242::writeReg(REG_IRQ1_EN1,0x00);
  ADF7242::writeReg(REG_IRQ2_EN0,0x00);
  return ADF7242::writeReg(REG_IRQ2_EN1,0x00);
}

/**************************************************************************//**
 * @brief
 *   Clear RX Interrupt Flag on Output 1
 *
 * @return uint8_t error 
 *   0 for success, 1 for error
 *****************************************************************************/
uint8_t ADF7242::clearRxCompleteInterrupt()
{
	return ADF7242::writeReg(REG_IRQ1_SRC1,IRQ_RX_PKT_RCVD);
}

/**************************************************************************//**
 * @brief
 *   Clear TX Interrupt Flag on Output 1
 *
 * @return uint8_t error 
 *   0 for success, 1 for error
 *****************************************************************************/
uint8_t ADF7242::clearTxCompleteInterrupt()
{
	return ADF7242::writeReg(REG_IRQ1_SRC1,IRQ_TX_PKT_SENT);
}

/**************************************************************************//**
 * @brief
 *   Clear IRQ2_TRFS_GP2 interrupt
 *
 * @return uint8_t error 
 *   0 for success, 1 for error
 *****************************************************************************/
uint8_t ADF7242::clearSportInterrupt()
{
	ADF7242::writeReg(REG_IRQ1_SRC0,0xFF);
	return ADF7242::writeReg(REG_IRQ1_SRC1,0xFF);
}

/**************************************************************************//**
 * @brief
 *   Clear ALL interrupts
 *
 * @return uint8_t error 
 *   0 for success, 1 for error
 *****************************************************************************/
uint8_t ADF7242::clearAllInterrupts(void)
{
  ADF7242::writeReg(REG_IRQ1_SRC0,0xFF);
  return ADF7242::writeReg(REG_IRQ1_SRC1,0xFF);
}

/**************************************************************************//**
 * @brief
 *   Change the adf7242 into TX mode
 *
 * @return uint8_t error 
 *   0 for success, 1 for error
 *****************************************************************************/
uint8_t ADF7242::startTx()
{
  return ADF7242::writeCmd(CMD_RC_TX);
}


/**************************************************************************//**
 * @brief
 *   Change the adf7242 into TX mode and wait until mode is set
 *
 * @return uint8_t error 
 *   0 for success, 1 for error
 *****************************************************************************/
uint8_t ADF7242::startTxWait()
{
  uint8_t cnt = 0;
  uint8_t stat = 0;
  
  do{
    cnt++;
    ADF7242::writeCmd(CMD_RC_TX);
    stat = ADF7242::status();
    Serial.println(stat,HEX);
  }
  while((stat & MODE_BIT_MASK) != MODE_TX && (cnt < MAX_POLL_LOOPS));

  return (cnt >= MAX_POLL_LOOPS);

}

/**************************************************************************//**
 * @brief
 *   Change the adf7242 into RX mode
 *
 * @return uint8_t error 
 *   0 for success, 1 for error
 *****************************************************************************/
uint8_t ADF7242::startRx()
{
	return ADF7242::writeCmd(CMD_RC_RX);
}

/**************************************************************************//**
 * @brief
 *   Change the adf7242 into RX mode and wait until mode is set
 *
 * @return uint8_t error 
 *   0 for success, 1 for error
 *****************************************************************************/
uint8_t ADF7242::startRxWait()
{
  uint8_t cnt = 0;
  uint8_t stat = 0;
  
  do{
    cnt++;
    ADF7242::writeCmd(CMD_RC_RX);
    stat = ADF7242::status();
    Serial.println(stat,HEX);
  }
  while((stat & MODE_BIT_MASK) != MODE_RX && (cnt < MAX_POLL_LOOPS));

  return (cnt >= MAX_POLL_LOOPS);

}

/**************************************************************************//**
 * @brief
 *   Change the adf7242 into IDLE mode
 *
 * @return uint8_t error 
 *   0 for success, 1 for error
 *****************************************************************************/
uint8_t ADF7242::idle()
{
	return ADF7242::writeCmd(CMD_RC_IDLE);
}

/**************************************************************************//**
 * @brief
 *   Change the adf7242 into PHY RDY mode
 *
 * @return uint8_t error 
 *   0 for success, 1 for error
 *****************************************************************************/
uint8_t ADF7242::phyRdy()
{
	return ADF7242::writeCmd(CMD_RC_PHY_RDY);
}

/**************************************************************************//**
 * @brief
 *   Init the adf7242
 *
 * @return uint8_t error 
 *   0 for success, 1 for error
 *****************************************************************************/
//void ADF7242::init(ADF7242_Pin_TypeDef pCS)
void ADF7242::init(uint8_t pCS)
{
	uint16_t i = 0;
  pinCS = pCS;
  
  pinMode(pinCS, OUTPUT);
  digitalWrite(pinCS,HIGH);
  
  SPI.begin();
  
 // *pinCS.DDR |= (1<<*pinCS.PIN); //set cs as output
 // *pinCS.PORT |= (1<<*pinCS.PIN); //set pin to high

	spiSettings = SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0);
	
	SPI.beginTransaction(spiSettings);
	ADF7242::spiSelect();
	
	SPI.transfer(CMD_RC_RESET);	
	
	ADF7242::spiRelease();

	
	for( i = 0 ;i < 4000 ; i++);
	
	ADF7242::spiSelect();
	
	for( i=0;i< 4000 ;i++);
	
	ADF7242::spiRelease();
	SPI.endTransaction();
}


































