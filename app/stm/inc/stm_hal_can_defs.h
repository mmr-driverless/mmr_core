#ifndef APP_STM_INC_STM_HAL_CAN_DEFS_H_
#define APP_STM_INC_STM_HAL_CAN_DEFS_H_

#define CAN_ID_STD (0x00000000U)  /*!< Standard Id */
#define CAN_ID_EXT (0x00000004U)  /*!< Extended Id */
#define CAN_RTR_DATA (0x00000000U)  /*!< Data frame   */
#define CAN_RTR_REMOTE (0x00000002U)  /*!< Remote frame */
#define CAN_RX_FIFO0 (0x00000000U)  /*!< CAN receive FIFO 0 */
#define CAN_RX_FIFO1 (0x00000001U)  /*!< CAN receive FIFO 1 */
#define CAN_FILTER_DISABLE (0x00000000U)  /*!< Disable filter */
#define CAN_FILTER_ENABLE (0x00000001U)  /*!< Enable filter  */
#define CAN_FILTERMODE_IDMASK (0x00000000U)  /*!< Identifier mask mode */
#define CAN_FILTERMODE_IDLIST (0x00000001U)  /*!< Identifier list mode */
#define CAN_FILTER_FIFO0 (0x00000000U)  /*!< Filter FIFO 0 assignment for filter x */
#define CAN_FILTER_FIFO1 (0x00000001U)  /*!< Filter FIFO 1 assignment for filter x */
#define CAN_FILTERSCALE_16BIT (0x00000000U)  /*!< Two 16-bit filters */
#define CAN_FILTERSCALE_32BIT (0x00000001U)  /*!< One 32-bit filter  */

typedef struct CAN_HandleTypeDef CAN_HandleTypeDef;

typedef enum {
  DISABLE = 0U,
  ENABLE = !DISABLE
} FunctionalState;

typedef struct {
  uint32_t StdId;
  uint32_t ExtId;
  uint32_t IDE;
  uint32_t RTR;
  uint32_t DLC;
  FunctionalState TransmitGlobalTime;
} CAN_TxHeaderTypeDef;

typedef struct {
  uint32_t StdId;
  uint32_t ExtId;
  uint32_t IDE;
  uint32_t RTR;
  uint32_t DLC;
  uint32_t Timestamp;
  uint32_t FilterMatchIndex;
} CAN_RxHeaderTypeDef;

typedef enum {
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03
} HAL_StatusTypeDef;

typedef struct {
  uint32_t FilterIdHigh;
  uint32_t FilterIdLow;
  uint32_t FilterMaskIdHigh;
  uint32_t FilterMaskIdLow;
  uint32_t FilterFIFOAssignment;
  uint32_t FilterBank;
  uint32_t FilterMode;
  uint32_t FilterScale;
  uint32_t FilterActivation;
  uint32_t SlaveStartFilterBank;
} CAN_FilterTypeDef;


HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[], uint32_t *pTxMailbox);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan, uint32_t RxFifo, CAN_RxHeaderTypeDef *pHeader, uint8_t aData[]);
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *sFilterConfig);
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *hcan, uint32_t RxFifo);

#endif // !APP_STM_INC_STM_HAL_CAN_DEFS_H_

