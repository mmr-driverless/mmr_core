#include "mmr_bb_manager.h"

CanHandle *_hcan;
AdcHandle *_hadc;
DacHandle *_hdac;

uint16_t ADC_Value[ADC_SIZE] = {};
uint16_t DAC_Value = 0;

bool autonomousFlag = false;  // default FALSE : MANUAL
bool waitFlag = false;
uint8_t clutchSteps = CLUTCH_INIT;

CanRxBuffer buffer = {};
MmrCanMessage message = {
		  .store = buffer
};

/** TESTING **/
extern int tmp[3];
extern HalStatus status;

static int tick = 0, delay = 0;
//static bool flag = 0;
/** END TESTING **/

HalStatus MMR_BB_setup(){
	if (MMR_CAN_BasicSetupAndStart(_hcan) != HAL_OK) {
		return HAL_ERROR;
	}

	if (HAL_ADC_Start_DMA(_hadc, (uint32_t*)&ADC_Value, sizeof(ADC_Value)) != HAL_OK) {
		return HAL_ERROR;
	}

	if (HAL_DAC_Start_DMA(_hdac, DAC_CHANNEL_1, (uint32_t*)&(DAC_Value), 1, DAC_ALIGN_12B_R) != HAL_OK) {
		return HAL_ERROR;
	}

	return HAL_OK;
}

HalStatus MMR_BB_initManager(CanHandle *hcan, AdcHandle *hadc, DacHandle *hdac) {
	_hcan = hcan;
	_hadc = hadc;
	_hdac = hdac;

	return MMR_BB_setup();
}

HalStatus MMR_BB_startManager() {

//	HalStatus status = (autonomousFlag) ?
//			MMR_BB_autonomousMode() :
//			MMR_BB_manualMode();

	status = MMR_BB_autonomousMode();


	if (status != HAL_OK) {
		return HAL_ERROR;
	}

	/** TESTING **/
//	delay = uwTick - tick;
//	if (delay > 100) {
//	  if (MMR_BB_sendNoData(MMR_CAN_MESSAGE_ID_S_CLUTCH) != HAL_OK) {
//		  ;
//	  }
//	  tick = uwTick;
//	  k++;
//	  k%=100;
//	}
	/** END TESTING **/

	uint32_t pendingMessages =
				  HAL_CAN_GetRxFifoFillLevel(_hcan, MMR_CAN_RX_FIFO);

	if (pendingMessages > 0) {
		if (MMR_CAN_Receive(_hcan, &message) != HAL_OK) {
		  return HAL_ERROR;
		}

		if (MMR_BB_receiveManager() != HAL_OK) {
		  return HAL_ERROR;
		}
	}

	return HAL_OK;
}

HalStatus MMR_BB_manualMode() {
	if(!waitFlag) {
		waitFlag = MMR_BB_utilsTimer();
		return HAL_OK;
	}

	/** MONITORING / DEBUG **/
	tmp[0] = ADC_Value[0];
	tmp[1] = ADC_Value[1];
	tmp[2] = DAC_Value;
//	if (flag == 0) {
//		HAL_GPIO_WritePin(S0_GPIO_Port, S0_Pin, SET);
//		HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, RESET);
//	} else {
//		HAL_GPIO_WritePin(S1_GPIO_Port, S0_Pin, SET);
//		HAL_GPIO_WritePin(S0_GPIO_Port, S1_Pin, RESET);
//	}
//
//	delay = uwTick - tick;
//	if (delay > 2000) {
//		tick = uwTick;
//		flag = !flag;
//	}
	/** END MONITORING **/

	/**signal processing using adcValues,
	* such as generating digital signal (DAC)**/

	return HAL_OK;
}

HalStatus MMR_BB_autonomousMode() {
	HalStatus status = HAL_OK;

	if(!waitFlag) {
		waitFlag = MMR_BB_utilsTimer();
		return status;
	}

	tmp[0] = ADC_Value[0];
	tmp[1] = ADC_Value[1];
	tmp[2] = DAC_Value;

	switch (clutchSteps) {
		case CLUTCH_INIT :
			status = MMR_BB_sendNoData(MMR_CAN_MESSAGE_ID_S_CLUTCH); // tira frizione
			// DAC_Value = APPS_MAX;
			//clutchSteps = CLUTCH_WAIT_FIRST; // aspetta conferma tirata frizione
			// tick2 = uwTick;
			break;
		case CLUTCH_WAIT_FIRST :
//			delay = uwTick - tick2;
//			if (delay > 1) {
//				clutchSteps = CLUTCH_INIT;
//			}
			break; // manage the confirm message
		case ECU_WAIT :
			delay = uwTick - tick;
			if (delay > 1000) {
			  status = MMR_BB_sendNoData(MMR_CAN_MESSAGE_ID_S_LV12); // rilascia frizione
			  clutchSteps = CLUTCH_WAIT_SECOND; // aspetta conferma frizione rilasciata
			}
			break; // manage the confirm message (Probably w/LED)
		case RPM_WAIT :
			; break;
		case CLUTCH_WAIT_SECOND :
			; break;

		case CLUTCH_END :
			; break;

		default : status = HAL_ERROR; break;
	}

	return status;
}

HalStatus MMR_BB_receiveManager(){

	HalStatus status = HAL_ERROR;


	switch (message.header.messageId) {
	  /** START **/
	  case MMR_CAN_MESSAGE_ID_AMC_MISSION_FINISHED : // frizione tirata
		  clutchSteps = ECU_WAIT;
		  status = HAL_OK;
		  // status = MMR_BB_sendNoData(MMR_CAN_MESSAGE_ID_ECU_BOSCH_); // Messaggio alla ECU - Launch Control
		  tick = uwTick; // MOCK per messaggi ECU BOSH
		  break;
	  case MMR_CAN_MESSAGE_ID_ECU_BOSCH_ : // se launch control OK
		  clutchSteps = RPM_WAIT;
		  DAC_Value = 30; // necessità di cambiarlo in tensione
		  break;
	  case MMR_CAN_MESSAGE_ID_D_ACCELERATOR_PERCENTAGE : // check RPM
		  if ( (*(int*)buffer) >= RPM ) {
			  status = MMR_BB_sendNoData(MMR_CAN_MESSAGE_ID_S_LV12);
			  clutchSteps = CLUTCH_WAIT_SECOND;
			  DAC_Value = APPS_MIN;
		  }
		  break;
	  case MMR_CAN_MESSAGE_ID_MMR_AS_READY : // frizione rilasciata
		  clutchSteps =CLUTCH_END;
		  DAC_Value = APPS_MIN;
		  status = MMR_BB_sendNoData(MMR_CAN_MESSAGE_ID_AMC_MISSION_FINISHED);
		  break;

	  default:
		  break;
	}

	return status;
}

HalStatus MMR_BB_sendNoData(MmrCanMessageId messageId) {
	MmrCanPacket packet = {
			.header.messageId = messageId,
	};

	MMR_BB_createDefaultPacket(&packet);

	if (MMR_CAN_Send(_hcan, packet) != HAL_OK) {
		;
	}

	return HAL_OK;
}

HalStatus MMR_BB_sendAPPS(MmrCanMessageId messageId, uint32_t data) {
	MmrCanPacket packet = {
			.data = (uint8_t*) &data,
			.length = sizeof(data),
			.header.messageId = messageId,
	};

	if (MMR_CAN_Send(_hcan, packet) != HAL_OK) {
		;
	}

	return HAL_OK;
}

HalStatus MMR_BB_changeDrivingMode() {
	waitFlag = false;
	autonomousFlag = !autonomousFlag;
	clutchSteps = (autonomousFlag) ? CLUTCH_INIT : MANUAL;
	MMR_BB_utilsSetTick();

	return HAL_OK;
}
