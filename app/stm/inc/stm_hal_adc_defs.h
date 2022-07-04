#ifndef APP_STM_INC_STM_HAL_ADC_DEFS_H_
#define APP_STM_INC_STM_HAL_ADC_DEFS_H_

typedef struct {
  uint32_t Direction;
  uint32_t PeriphInc;
  uint32_t MemInc;
  uint32_t PeriphDataAlignment;
  uint32_t MemDataAlignment;    
  uint32_t Mode;
  uint32_t Priority;
} DMA_InitTypeDef;


typedef enum {
  HAL_DMA_STATE_RESET             = 0x00U,  /*!< DMA not yet initialized or disabled */  
  HAL_DMA_STATE_READY             = 0x01U,  /*!< DMA initialized and ready for use   */
  HAL_DMA_STATE_BUSY              = 0x02U,  /*!< DMA process is ongoing              */     
  HAL_DMA_STATE_TIMEOUT           = 0x03   /*!< DMA timeout state                   */  
} HAL_DMA_StateTypeDef;

typedef enum {
  DISABLE = 0U, 
  ENABLE = !DISABLE
} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef struct {
  uint32_t ClockPrescaler;
  uint32_t Resolution;
  uint32_t DataAlign;
  uint32_t ScanConvMode;
  uint32_t EOCSelection;
  FunctionalState LowPowerAutoWait;
  FunctionalState  ContinuousConvMode;
  uint32_t NbrOfConversion;
  FunctionalState DiscontinuousConvMode;
  uint32_t NbrOfDiscConversion;
  uint32_t ExternalTrigConv;
  uint32_t ExternalTrigConvEdge;
  FunctionalState DMAContinuousRequests;
  uint32_t Overrun;
} ADC_InitTypeDef;

typedef struct {
  uint32_t CCR;          /*!< DMA channel x configuration register                                           */
  uint32_t CNDTR;        /*!< DMA channel x number of data register                                          */
  uint32_t CPAR;         /*!< DMA channel x peripheral address register                                      */
  uint32_t CMAR;         /*!< DMA channel x memory address register                                          */
} DMA_Channel_TypeDef;

typedef struct {
  uint32_t ISR;          /*!< DMA interrupt status register,                            Address offset: 0x00 */
  uint32_t IFCR;         /*!< DMA interrupt flag clear register,                        Address offset: 0x04 */
} DMA_TypeDef;

typedef enum {
  HAL_UNLOCKED = 0x00U,
  HAL_LOCKED   = 0x01  
} HAL_LockTypeDef;

typedef struct __DMA_HandleTypeDef {
  DMA_Channel_TypeDef   *Instance;                                                    /*!< Register base address                  */
  DMA_InitTypeDef       Init;                                                         /*!< DMA communication parameters           */ 
  HAL_LockTypeDef       Lock;                                                         /*!< DMA locking object                     */  
  HAL_DMA_StateTypeDef  State;                                                        /*!< DMA transfer state                     */
  void                  *Parent;                                                      /*!< Parent object state                    */  
  void                  (* XferCpltCallback)( struct __DMA_HandleTypeDef * hdma);     /*!< DMA transfer complete callback         */
  void                  (* XferHalfCpltCallback)( struct __DMA_HandleTypeDef * hdma); /*!< DMA Half transfer complete callback    */
  void                  (* XferErrorCallback)( struct __DMA_HandleTypeDef * hdma);    /*!< DMA transfer error callback            */
  void                  (* XferAbortCallback)( struct __DMA_HandleTypeDef * hdma);    /*!< DMA transfer abort callback            */  
   uint32_t         ErrorCode;                                                    /*!< DMA Error code                         */
  DMA_TypeDef          *DmaBaseAddress;                                               /*!< DMA Channel Base Address               */
  uint32_t              ChannelIndex;                                                 /*!< DMA Channel Index                      */
} DMA_HandleTypeDef;

typedef struct {
  uint32_t ContextQueue;                          
  uint32_t ChannelCount;                                        
} ADC_InjectionConfigTypeDef;

typedef struct {
  uint32_t ISR;              /*!< ADC Interrupt and Status Register,                 Address offset: 0x00 */
  uint32_t IER;              /*!< ADC Interrupt Enable Register,                     Address offset: 0x04 */
  uint32_t CR;               /*!< ADC control register,                              Address offset: 0x08 */
  uint32_t CFGR;             /*!< ADC Configuration register,                        Address offset: 0x0C */
  uint32_t RESERVED0;        /*!< Reserved, 0x010                                                         */
  uint32_t SMPR1;            /*!< ADC sample time register 1,                        Address offset: 0x14 */
  uint32_t SMPR2;            /*!< ADC sample time register 2,                        Address offset: 0x18 */
  uint32_t RESERVED1;        /*!< Reserved, 0x01C                                                         */
  uint32_t TR1;              /*!< ADC watchdog threshold register 1,                 Address offset: 0x20 */
  uint32_t TR2;              /*!< ADC watchdog threshold register 2,                 Address offset: 0x24 */
  uint32_t TR3;              /*!< ADC watchdog threshold register 3,                 Address offset: 0x28 */
  uint32_t RESERVED2;        /*!< Reserved, 0x02C                                                         */
  uint32_t SQR1;             /*!< ADC regular sequence register 1,                   Address offset: 0x30 */
  uint32_t SQR2;             /*!< ADC regular sequence register 2,                   Address offset: 0x34 */
  uint32_t SQR3;             /*!< ADC regular sequence register 3,                   Address offset: 0x38 */
  uint32_t SQR4;             /*!< ADC regular sequence register 4,                   Address offset: 0x3C */
  uint32_t DR;               /*!< ADC regular data register,                         Address offset: 0x40 */
  uint32_t RESERVED3;        /*!< Reserved, 0x044                                                         */
  uint32_t RESERVED4;        /*!< Reserved, 0x048                                                         */
  uint32_t JSQR;             /*!< ADC injected sequence register,                    Address offset: 0x4C */
  uint32_t RESERVED5[4];     /*!< Reserved, 0x050 - 0x05C                                                 */
  uint32_t OFR1;             /*!< ADC offset register 1,                             Address offset: 0x60 */
  uint32_t OFR2;             /*!< ADC offset register 2,                             Address offset: 0x64 */
  uint32_t OFR3;             /*!< ADC offset register 3,                             Address offset: 0x68 */
  uint32_t OFR4;             /*!< ADC offset register 4,                             Address offset: 0x6C */
  uint32_t RESERVED6[4];     /*!< Reserved, 0x070 - 0x07C                                                 */
  uint32_t JDR1;             /*!< ADC injected data register 1,                      Address offset: 0x80 */
  uint32_t JDR2;             /*!< ADC injected data register 2,                      Address offset: 0x84 */
  uint32_t JDR3;             /*!< ADC injected data register 3,                      Address offset: 0x88 */
  uint32_t JDR4;             /*!< ADC injected data register 4,                      Address offset: 0x8C */
  uint32_t RESERVED7[4];     /*!< Reserved, 0x090 - 0x09C                                                 */
  uint32_t AWD2CR;           /*!< ADC  Analog Watchdog 2 Configuration Register,     Address offset: 0xA0 */
  uint32_t AWD3CR;           /*!< ADC  Analog Watchdog 3 Configuration Register,     Address offset: 0xA4 */
  uint32_t RESERVED8;        /*!< Reserved, 0x0A8                                                         */
  uint32_t RESERVED9;        /*!< Reserved, 0x0AC                                                         */
  uint32_t DIFSEL;           /*!< ADC  Differential Mode Selection Register,         Address offset: 0xB0 */
  uint32_t CALFACT;          /*!< ADC  Calibration Factors,                          Address offset: 0xB4 */

} ADC_TypeDef;

typedef struct {
  uint32_t CSR;            /*!< ADC Common status register,                  Address offset: ADC1/3 base address + 0x300 */
  uint32_t RESERVED;       /*!< Reserved, ADC1/3 base address + 0x304                                                    */
  uint32_t CCR;            /*!< ADC common control register,                 Address offset: ADC1/3 base address + 0x308 */
  uint32_t CDR;            /*!< ADC common regular data register for dual
                                     AND triple modes,                            Address offset: ADC1/3 base address + 0x30C */
} ADC_Common_TypeDef;

typedef struct __ADC_HandleTypeDef {
  ADC_TypeDef *Instance;              /*!< Register base address */
  ADC_InitTypeDef Init;                   /*!< ADC required parameters */
  DMA_HandleTypeDef *DMA_Handle;            /*!< Pointer DMA Handler */
  HAL_LockTypeDef Lock;                   /*!< ADC locking object */
  uint32_t State;                  /*!< ADC communication state (bitmap of ADC states) */
  uint32_t ErrorCode;              /*!< ADC Error code */
  
#if defined(STM32F302xE) || defined(STM32F303xE) || defined(STM32F398xx) || \
  defined(STM32F302xC) || defined(STM32F303xC) || defined(STM32F358xx) || \
  defined(STM32F303x8) || defined(STM32F334x8) || defined(STM32F328xx) || \
  defined(STM32F301x8) || defined(STM32F302x8) || defined(STM32F318xx)
  ADC_InjectionConfigTypeDef    InjectionConfig ;       /*!< ADC injected channel configuration build-up structure */  
#endif /* STM32F302xE || STM32F303xE || STM32F398xx || */
      /* STM32F302xC || STM32F303xC || STM32F358xx || */
      /* STM32F303x8 || STM32F334x8 || STM32F328xx || */
      /* STM32F301x8 || STM32F302x8 || STM32F318xx    */
} ADC_HandleTypeDef;

#define FLASH_BASE            0x08000000UL /*!< FLASH base address in the alias region */
#define SRAM_BASE             0x20000000UL /*!< SRAM base address in the alias region */
#define PERIPH_BASE           0x40000000UL /*!< Peripheral base address in the alias region */
#define SRAM_BB_BASE          0x22000000UL /*!< SRAM base address in the bit-band region */
#define PERIPH_BB_BASE        0x42000000UL /*!< Peripheral base address in the bit-band region */

#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000UL)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x08000000UL)
#define AHB3PERIPH_BASE       (PERIPH_BASE + 0x10000000UL)

#define ADC2_BASE             (AHB3PERIPH_BASE + 0x00000100UL)
#define ADC1_2_COMMON_BASE    (AHB3PERIPH_BASE + 0x00000300UL)

#define ADC2                ((ADC_TypeDef *) ADC2_BASE)
#define ADC12_COMMON        ((ADC_Common_TypeDef *) ADC1_2_COMMON_BASE)
#define ADC1_2_COMMON       ADC12_COMM

#define ADC_CLOCK_ASYNC_DIV1          (0x00000000U)          /*!< ADC asynchronous clock derived from ADC dedicated PLL */


#define ADC_CFGR_RES_Pos               (3U)                                    
#define ADC_CFGR_RES_Msk               (0x3UL << ADC_CFGR_RES_Pos)              /*!< 0x00000018 */
#define ADC_CFGR_RES                   ADC_CFGR_RES_Msk                        /*!< ADC data resolution */
#define ADC_CFGR_RES_0                 (0x1UL << ADC_CFGR_RES_Pos)              /*!< 0x00000008 */
#define ADC_CFGR_RES_1                 (0x2UL << ADC_CFGR_RES_Pos)  

#define ADC_RESOLUTION_12B      (0x00000000U)          /*!<  ADC 12-bit resolution */
#define ADC_RESOLUTION_10B      ((uint32_t)ADC_CFGR_RES_0)      /*!<  ADC 10-bit resolution */
#define ADC_RESOLUTION_8B       ((uint32_t)ADC_CFGR_RES_1)      /*!<  ADC 8-bit resolution */
#define ADC_RESOLUTION_6B       ((uint32_t)ADC_CFGR_RES)        /*!<  ADC 6-bit resolution */

#define ADC_SCAN_DISABLE         (0x00000000U)
#define ADC_SCAN_ENABLE          (0x00000001U)

#define ADC_CFGR_EXTEN_Pos             (10U)                                   
#define ADC_CFGR_EXTEN_Msk             (0x3UL << ADC_CFGR_EXTEN_Pos)            /*!< 0x00000C00 */
#define ADC_CFGR_EXTEN                 ADC_CFGR_EXTEN_Msk                      /*!< ADC group regular external trigger polarity */
#define ADC_CFGR_EXTEN_0               (0x1UL << ADC_CFGR_EXTEN_Pos)            /*!< 0x00000400 */
#define ADC_CFGR_EXTEN_1               (0x2UL << ADC_CFGR_EXTEN_Pos)

#define ADC_EXTERNALTRIGCONVEDGE_NONE           (0x00000000U)
#define ADC_EXTERNALTRIGCONVEDGE_RISING         ((uint32_t)ADC_CFGR_EXTEN_0)
#define ADC_EXTERNALTRIGCONVEDGE_FALLING        ((uint32_t)ADC_CFGR_EXTEN_1)
#define ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING  ((uint32_t)ADC_CFGR_EXTEN)

#define ADC_SOFTWARE_START                  (0x00000001U)

#define ADC_DATAALIGN_RIGHT      (0x00000000U)
#define ADC_DATAALIGN_LEFT       ((uint32_t)ADC_CFGR_ALIGN)

#define ADC_ISR_EOC_Pos                (2U)                                    
#define ADC_ISR_EOC_Msk                (0x1UL << ADC_ISR_EOC_Pos)               /*!< 0x00000004 */
#define ADC_ISR_EOC                    ADC_ISR_EOC_Msk

#define ADC_ISR_EOS_Pos                (3U)                                    
#define ADC_ISR_EOS_Msk                (0x1UL << ADC_ISR_EOS_Pos)               /*!< 0x00000008 */
#define ADC_ISR_EOS                    ADC_ISR_EOS_Msk                         /*!< ADC group regular end of sequence conversions flag */

#define ADC_EOC_SINGLE_CONV         ((uint32_t) ADC_ISR_EOC)
#define ADC_EOC_SEQ_CONV            ((uint32_t) ADC_ISR_EOS)

#define ADC_OVR_DATA_OVERWRITTEN    (0x00000000U)   /*!< Default setting, to be used for compatibility with other STM32 devices */
#define ADC_OVR_DATA_PRESERVED      (0x00000001U)

typedef enum {
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03
} HAL_StatusTypeDef;

#define ADC_SMPR2_SMP10_Pos            (0U)                                    
#define ADC_SMPR2_SMP10_Msk            (0x7UL << ADC_SMPR2_SMP10_Pos)           /*!< 0x00000007 */
#define ADC_SMPR2_SMP10                ADC_SMPR2_SMP10_Msk                     /*!< ADC channel 10 sampling time selection  */
#define ADC_SMPR2_SMP10_0              (0x1UL << ADC_SMPR2_SMP10_Pos)           /*!< 0x00000001 */
#define ADC_SMPR2_SMP10_1              (0x2UL << ADC_SMPR2_SMP10_Pos)           /*!< 0x00000002 */
#define ADC_SMPR2_SMP10_2              (0x4UL << ADC_SMPR2_SMP10_Pos)           /*!< 0x00000004 */

#define ADC_SAMPLETIME_1CYCLE_5       (0x00000000U)                              /*!< Sampling time 1.5 ADC clock cycle */
#define ADC_SAMPLETIME_2CYCLES_5      ((uint32_t)ADC_SMPR2_SMP10_0)                       /*!< Sampling time 2.5 ADC clock cycles */
#define ADC_SAMPLETIME_4CYCLES_5      ((uint32_t)ADC_SMPR2_SMP10_1)                       /*!< Sampling time 4.5 ADC clock cycles */
#define ADC_SAMPLETIME_7CYCLES_5      ((uint32_t)(ADC_SMPR2_SMP10_1 | ADC_SMPR2_SMP10_0)) /*!< Sampling time 7.5 ADC clock cycles */
#define ADC_SAMPLETIME_19CYCLES_5     ((uint32_t)ADC_SMPR2_SMP10_2)                       /*!< Sampling time 19.5 ADC clock cycles */
#define ADC_SAMPLETIME_61CYCLES_5     ((uint32_t)(ADC_SMPR2_SMP10_2 | ADC_SMPR2_SMP10_0)) /*!< Sampling time 61.5 ADC clock cycles */
#define ADC_SAMPLETIME_181CYCLES_5    ((uint32_t)(ADC_SMPR2_SMP10_2 | ADC_SMPR2_SMP10_1)) /*!< Sampling time 181.5 ADC clock cycles */
#define ADC_SAMPLETIME_601CYCLES_5    ((uint32_t)ADC_SMPR2_SMP10)     

#define ADC_SINGLE_ENDED                (0x00000000U)
#define ADC_DIFFERENTIAL_ENDED          (0x00000001U)

#define ADC_OFFSET_NONE               (0x00U)
#define ADC_OFFSET_1                  (0x01U)
#define ADC_OFFSET_2                  (0x02U)
#define ADC_OFFSET_3                  (0x03U)
#define ADC_OFFSET_4                  (0x04U)

typedef struct {
  uint32_t Channel;
  uint32_t Rank;
  uint32_t SamplingTime;
  uint32_t SingleDiff;
  uint32_t OffsetNumber;
  uint32_t Offset;
} ADC_ChannelConfTypeDef;


#endif /*__STM32F3xx_ADC_H */