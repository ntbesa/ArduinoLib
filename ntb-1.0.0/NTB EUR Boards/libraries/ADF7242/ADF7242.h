/***************************************************************************//**
 * @file ADF7242.h
 * @version 0.1
 * @author Samuel Kranz
 * @date 11.11.2016
 * @brief This is a .cpp (Arduino) port of the ADF7242 lib (ebreitenstein)
 * @copyright Copyright 2016 NTB Buchs, http://www.ntb.ch/esa. All Rights reserved
 ******************************************************************************/

#ifndef ADF7242_H_
#define ADF7242_H_

#include "SPI.h"


enum ADF7242_DataRate_TypeDef{
	DataRate_50kbps,
	DataRate_62k5bps,
	DataRate_100kbps,
	DataRate_125kbps,
	DataRate_250kbps,
	DataRate_500kbps,
	DataRate_1000kbps,
	DataRate_2000kbps
};
    
struct ADF7242_Pin_TypeDef{
	uint8_t *PORT;
	uint8_t *PIN;
  uint8_t *DDR;
};

/* STATUS */

#define STAT_SPI_READY        (1 << 7)
#define STAT_IRQ_STATUS       (1 << 6)
#define STAT_RC_READY       (1 << 5)
#define STAT_CCA_RESULT       (1 << 4)
#define RC_STATUS_IDLE        1
#define RC_STATUS_MEAS        2
#define RC_STATUS_PHY_RDY     3
#define RC_STATUS_RX        4
#define RC_STATUS_TX        5
#define RC_STATUS_MASK        0xF
  
class ADF7242
{
	
	public:

		ADF7242::ADF7242();
  //  ADF7242::ADF7242(uint8_t pCS);
    
		void ADF7242::spiRelease(void);
		void ADF7242::spiSelect(void);
		uint8_t ADF7242::status(void);
		uint8_t ADF7242::waitReady(void);
		uint8_t ADF7242::readReg(uint16_t addr, uint8_t *data);
		uint8_t ADF7242::writeReg(uint16_t addr, uint8_t data);
		uint8_t ADF7242::writeTxBuf(uint8_t *data, uint8_t len);
		uint8_t ADF7242::readRxBuf(uint8_t *data, uint8_t len);
		uint8_t ADF7242::writeCmd(uint8_t cmd);
		uint8_t ADF7242::initFskConfig(void);
		uint8_t ADF7242::setFskDataRate(ADF7242_DataRate_TypeDef dataRate);
		uint8_t ADF7242::initFskPacketMode(ADF7242_DataRate_TypeDef dataRate);
		uint8_t ADF7242::setSportRxConfig(uint8_t rxConfig);
		uint8_t ADF7242::initFskSportMode(ADF7242_DataRate_TypeDef dataRate, uint8_t rxConfig);
		uint8_t ADF7242::setRxTxBufBase(uint8_t rxBase, uint8_t txBase);
    uint8_t ADF7242::clearAllInterrupts(void);
    uint8_t ADF7242::disableAllInterrupts(void);
		uint8_t ADF7242::enableRxCompleteInterrupt(void);
		uint8_t ADF7242::enableTxCompleteInterrupt(void);
		uint8_t ADF7242::clearRxCompleteInterrupt(void);
		uint8_t ADF7242::clearTxCompleteInterrupt(void);
		uint8_t ADF7242::clearSportInterrupt(void);
		uint8_t ADF7242::startTx(void);
    uint8_t ADF7242::startTxWait(void);
		uint8_t ADF7242::startRx(void);
    uint8_t ADF7242::startRxWait(void);
		uint8_t ADF7242::idle(void);
		uint8_t ADF7242::phyRdy(void);
		//void ADF7242::init(ADF7242_Pin_TypeDef pCS);
		void ADF7242::init(uint8_t pCS);

};

#endif /* ADF7242_H_ */


































