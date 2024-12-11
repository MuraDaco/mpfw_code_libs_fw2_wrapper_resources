    


    - uart module management
        - doc:
            - LPC546XX_LPC546xx_Datasheet.pdf
                - an overview of micro
            - LPC546xx User manual:
                - UM10912_LPC546xx_User_Manual.pdf
                    - chapter to see:
                        - Chapter 10: LPC546xx I/O pin configuration (IOCON)
                            - pag 196 - Table 256 (Type D I/O Control registers: FUNC values and pin functions)
                        - Chapter 25: LPC546xx USARTs
            - board schematics:
                - LPC54608_Dev_brd_schematic_Rev-D_170614.pdf
            - UM11035.pdf
                - board description
                    - switches/buttons, jumpers & connectors functions
        - code:
            - determine the uart pin to use
                - board schematichs at 
                    - sheet 3: 
                        - LPC54608FET180
                            - pin P0_29-ISP_FC0_RXD (B12 - P0_29)
                            - pin P0_30-ISP_FC0_TXD (A 2 - P0_30)
                    - sheet 11  
                        - 74AVC4TD245BQ chip
                            - pin P0_29-ISP_FC0_RXD (B2)
                            - pin P0_30-ISP_FC0_TXD (B3)
                            - pin BRIDGE_UART_RXD (A2)
                            - pin BRIDGE_UART_TXD (A3)
                    - sheet 12
                        - LPC4322JET100 micro 
                            - pin BRIDGE_UART_RXD (G10 - P2[0])
                            - pin BRIDGE_UART_TXD (G 7 - P2[1])
                    - sheet 13: 
                        - microB USB connector <-> LPC4322JET100 micro
                            - pin USB_DM_LINK <-> (E1 - USB0_DM)
                            - pin USB_DP_LINK <-> (E2 - USB0_DP)
            - pin configuration
                - LPC546xx User manual at
                    - Chapter 10: LPC546xx I/O pin configuration (IOCON)
                        - pag 196 - Table 256 (Type D I/O Control registers: FUNC values and pin functions)
                - code
                    - see "BOARD_InitDEBUG_UARTPins(.)" function
            - setting uart module
                - 
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
    RESET_PeripheralReset(BOARD_DEBUG_UART_RST);
    NVIC_SetPriority

- funzioni per la gestione della trasmissione rs-485
    - impostare il clock della periferica
    - abilitare il clock della periferica
    - impostare la priorità dell'interrupt
    - abilitare l'interrupt

- peripheral init functions
    - clock source choice
        - CLOCK_AttachClk (fsl_clock.h/c)
            - rsDbgConsole::Init
    - clock enabling
        - CLOCK_EnableClock (fsl_clock.h)
            - FLEXCOMM_Init (fsl_flexcomm.h/c)
                - USART_Init
                    - USART_RTOS_Init
    - module turn-on
        - RESET_PeripheralReset (fsl_reset.h/c)
            - FLEXCOMM_Init (fsl_flexcomm.h/c)
                - USART_Init
                    - USART_RTOS_Init
    - interrupt init management
        - interrupt priority set
            - NVIC_SetPriority (core_cm4.h)
                - rsDbgConsole::InitFreeRTOS
        - interrupt handler set
            - FLEXCOMM_SetIRQHandler (fsl_flexcomm.h/c)
                - USART_TransferCreateHandle
                    - USART_RTOS_Init
        - function handler definition & implementation
            - FLEXCOMM0_DriverIRQHandler (fsl_flexcomm.h/c)
                - FLEXCOMM0_IRQHandler (startup_lpc54608.cpp)
        - interrupt enabling
            - EnableIRQ (fsl_common.h) 
                - USART_TransferCreateHandle
                    - USART_RTOS_Init

- MCU registers & function that call them
    - SYSCON->ASYNCAPBCTRL
        - CLOCK_AttachClk
    - SYSCON->MAINCLKSELA
        - CLOCK_AttachClk
    - SYSCON->AHBCLKCTRLSET
        - CLOCK_EnableClock
    - SYSCON->PRESETCTRL / SYSCON->PRESETCTRLSET / SYSCON->PRESETCTRLCLR
        - RESET_ClearPeripheralReset
        - RESET_SetPeripheralReset
            - RESET_PeripheralReset
    - NVIC->IP
        - __NVIC_SetPriority
            - NVIC_SetPriority
    - NVIC->ISER
        - __NVIC_EnableIRQ
            - NVIC_EnableIRQ
                - EnableIRQ

- vector interrupt array
    - 3.3.2.8 Vectors (**UM10912_LPC546xx_User_Manual.pdf**)
        - tab 9.
            - void (* const g_pfnVectors[])(void) (**startup_lpc54608.cpp**)
                -     // Core Level - CM4
                -     &_vStackTop,                       // The initial stack pointer
                -     ResetISR,                          // The reset handler
                -     NMI_Handler,                       // The NMI handler
                -     HardFault_Handler,                 // The hard fault handler
                -     MemManage_Handler,                 // The MPU fault handler
                -     BusFault_Handler,                  // The bus fault handler
                -     UsageFault_Handler,                // The usage fault handler
                -     __valid_user_code_checksum,        // LPC MCU checksum
    - 7.5.3 NMI source selection register (**UM10912_LPC546xx_User_Manual.pdf**)
        - The NMI source selection register selects a peripheral interrupts as source for the NMI interrupt.o For a list of all peripheral interrupts and their IRQ numbers see Table 88. 
    - 6.3.1 Interrupt sources (**UM10912_LPC546xx_User_Manual.pdf**)
        - Table 88 lists the interrupt sources for each peripheral function. Each peripheral device may have one or more interrupt lines to the Vectored Interrupt Controller. Each line may represent more than one interrupt source. The interrupt number does not imply any interrupt priority.
    - B1.5.2 Exception number definition (**DDI0403E_e_armv7m_arm.pdf** *pg B1-525*)
        - Each exception has an associated exception number as Table B1-4 shows.
            - void (* const g_pfnVectors[])(void) = { (**startup_lpc54608.cpp**)
                -   // Core Level - CM4
                -   &_vStackTop,                       // The initial stack pointer
                -   ResetISR,                          // The reset handler
                -   NMI_Handler,                       // The NMI handler
                -   HardFault_Handler,                 // The hard fault handler
                -   MemManage_Handler,                 // The MPU fault handler
                -   BusFault_Handler,                  // The bus fault handler
                -   UsageFault_Handler,                // The usage fault handler
                -   __valid_user_code_checksum,        // LPC MCU checksum
                -   0,                                 // ECRP
                -   0,                                 // Reserved
                -   0,                                 // Reserved
                -   SVC_Handler,                       // SVCall handler
                -   DebugMon_Handler,                  // Debug monitor handler
                -   0,                                 // Reserved
                -   PendSV_Handler,                    // The PendSV handler
                -   SysTick_Handler,                   // The SysTick handler
                -   
                -   // Chip Level - LPC54608
                -   WDT_BOD_IRQHandler,         // 16: Windowed watchdog timer, Brownout detect
                ...
                -   
            }

- IRQ number definition
    - typedef enum IRQn (**LPC54608.h**)
        - /* Auxiliary constants */
        - NotAvail_IRQn                = -128,             /**< Not available device specific interrupt */
        - 
        - /* Core interrupts */
        - NonMaskableInt_IRQn          = -14,              /**< Non Maskable Interrupt */
        - HardFault_IRQn               = -13,              /**< Cortex-M4 SV Hard Fault Interrupt */
        - MemoryManagement_IRQn        = -12,              /**< Cortex-M4 Memory Management Interrupt */
        - BusFault_IRQn                = -11,              /**< Cortex-M4 Bus Fault Interrupt */
        - UsageFault_IRQn              = -10,              /**< Cortex-M4 Usage Fault Interrupt */
        - SVCall_IRQn                  = -5,               /**< Cortex-M4 SV Call Interrupt */
        - DebugMonitor_IRQn            = -4,               /**< Cortex-M4 Debug Monitor Interrupt */
        - PendSV_IRQn                  = -2,               /**< Cortex-M4 Pend SV Interrupt */
        - SysTick_IRQn                 = -1,               /**< Cortex-M4 System Tick Interrupt */
        - 
        - /* Device specific interrupts */
        - WDT_BOD_IRQn                 = 0,                /**< Windowed watchdog timer, Brownout detect */
        - DMA0_IRQn                    = 1,                /**< DMA controller */
        - ...
        - CTIMER3_IRQn                 = 13,               /**< Standard counter/timer CTIMER3 */
        - FLEXCOMM0_IRQn               = 14,               /**< Flexcomm Interface 0 (USART, SPI, I2C, FLEXCOMM) */
        - FLEXCOMM1_IRQn               = 15,               /**< Flexcomm Interface 1 (USART, SPI, I2C, FLEXCOMM) */
        - FLEXCOMM2_IRQn               = 16,               /**< Flexcomm Interface 2 (USART, SPI, I2C, FLEXCOMM) */
        - FLEXCOMM3_IRQn               = 17,               /**< Flexcomm Interface 3 (USART, SPI, I2C, FLEXCOMM) */
        - FLEXCOMM4_IRQn               = 18,               /**< Flexcomm Interface 4 (USART, SPI, I2C, FLEXCOMM) */
        - FLEXCOMM5_IRQn               = 19,               /**< Flexcomm Interface 5 (USART, SPI, I2C,, FLEXCOMM) */
        - FLEXCOMM6_IRQn               = 20,               /**< Flexcomm Interface 6 (USART, SPI, I2C, I2S,, FLEXCOMM) */
        - FLEXCOMM7_IRQn               = 21,               /**< Flexcomm Interface 7 (USART, SPI, I2C, I2S,, FLEXCOMM) */
        - ADC0_SEQA_IRQn               = 22,               /**< ADC0 sequence A completion. */
        - ADC0_SEQB_IRQn               = 23,               /**< ADC0 sequence B completion. */
        - ...
    }


    - CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
        - typedef enum _clock_attach_id (**fsl_clock.h**)
        - {
        -     ...
        -     kFRO12M_to_FLEXCOMM0    = MUX_A(CM_FXCOMCLKSEL0, 0),
        -     kFRO_HF_to_FLEXCOMM0    = MUX_A(CM_FXCOMCLKSEL0, 1),
        -     kAUDIO_PLL_to_FLEXCOMM0 = MUX_A(CM_FXCOMCLKSEL0, 2),
        -     kMCLK_to_FLEXCOMM0      = MUX_A(CM_FXCOMCLKSEL0, 3),
        -     kFRG_to_FLEXCOMM0       = MUX_A(CM_FXCOMCLKSEL0, 4),
        -     kNONE_to_FLEXCOMM0      = MUX_A(CM_FXCOMCLKSEL0, 7),
        -     ...
        - }


    - NVIC_SetPriority(DEMO_USART_IRQn, USART_NVIC_PRIO);
        - typedef enum IRQn { (**LPC54608.h**)
            - /* Auxiliary constants */
            - NotAvail_IRQn                = -128,             /**< Not available device specific interrupt */
            - 
            - /* Core interrupts */
            - NonMaskableInt_IRQn          = -14,              /**< Non Maskable Interrupt */
            - HardFault_IRQn               = -13,              /**< Cortex-M4 SV Hard Fault Interrupt */
            - MemoryManagement_IRQn        = -12,              /**< Cortex-M4 Memory Management Interrupt */
            - BusFault_IRQn                = -11,              /**< Cortex-M4 Bus Fault Interrupt */
            - UsageFault_IRQn              = -10,              /**< Cortex-M4 Usage Fault Interrupt */
            - SVCall_IRQn                  = -5,               /**< Cortex-M4 SV Call Interrupt */
            - DebugMonitor_IRQn            = -4,               /**< Cortex-M4 Debug Monitor Interrupt */
            - PendSV_IRQn                  = -2,               /**< Cortex-M4 Pend SV Interrupt */
            - SysTick_IRQn                 = -1,               /**< Cortex-M4 System Tick Interrupt */
            - 
            - /* Device specific interrupts */
            - WDT_BOD_IRQn                 = 0,                /**< Windowed watchdog timer, Brownout detect */
            - DMA0_IRQn                    = 1,                /**< DMA controller */
            - ...
            - ...
            - CTIMER3_IRQn                 = 13,               /**< Standard counter/timer CTIMER3 */
            - FLEXCOMM0_IRQn               = 14,               /**< Flexcomm Interface 0 (USART, SPI, I2C, FLEXCOMM) */
            - ...
            }


- Table 429. USART register overview
    /** USART - Register Layout Typedef */
    - typedef struct {
    -   __IO uint32_t CFG;                               /**< USART Configuration register. Basic USART configuration settings that typically are not changed during operation., offset: 0x0 */
    -   __IO uint32_t CTL;                               /**< USART Control register. USART control settings that are more likely to change during operation., offset: 0x4 */
    -   __IO uint32_t STAT;                              /**< USART Status register. The complete status value can be read here. Writing ones clears some bits in the register. Some bits can be cleared by writing a 1 to them., offset: 0x8 */
    - __IO uint32_t INTENSET;                          /**< Interrupt Enable read and Set register for USART (not FIFO) status. Contains individual interrupt enable bits for each potential USART interrupt. A complete value may be read from this register. Writing a 1 to any implemented bit position causes that bit to be set., offset: 0xC */
    - __O  uint32_t INTENCLR;                          /**< Interrupt Enable Clear register. Allows clearing any combination of bits in the INTENSET register. Writing a 1 to any implemented bit position causes the corresponding bit to be cleared., offset: 0x10 */
    -      uint8_t RESERVED_0[12];
    - __IO uint32_t BRG;                               /**< Baud Rate Generator register. 16-bit integer baud rate divisor value., offset: 0x20 */
    - __I  uint32_t INTSTAT;                           /**< Interrupt status register. Reflects interrupts that are currently enabled., offset: 0x24 */
    - __IO uint32_t OSR;                               /**< Oversample selection register for asynchronous communication., offset: 0x28 */
    - __IO uint32_t ADDR;                              /**< Address register for automatic address matching., offset: 0x2C */
    -      uint8_t RESERVED_1[3536];
    - __IO uint32_t FIFOCFG;                           /**< FIFO configuration and enable register., offset: 0xE00 */
    - __IO uint32_t FIFOSTAT;                          /**< FIFO status register., offset: 0xE04 */
    - __IO uint32_t FIFOTRIG;                          /**< FIFO trigger settings for interrupt and DMA request., offset: 0xE08 */
    -      uint8_t RESERVED_2[4];
    - __IO uint32_t FIFOINTENSET;                      /**< FIFO interrupt enable set (enable) and read register., offset: 0xE10 */
    - __IO uint32_t FIFOINTENCLR;                      /**< FIFO interrupt enable clear (disable) and read register., offset: 0xE14 */
    - __I  uint32_t FIFOINTSTAT;                       /**< FIFO interrupt status register., offset: 0xE18 */
    -      uint8_t RESERVED_3[4];
    - __IO uint32_t FIFOWR;                            /**< FIFO write data., offset: 0xE20 */
    -      uint8_t RESERVED_4[12];
    - __I  uint32_t FIFORD;                            /**< FIFO read data., offset: 0xE30 */
    -      uint8_t RESERVED_5[12];
    - __I  uint32_t FIFORDNOPOP;                       /**< FIFO data read with no FIFO pop., offset: 0xE40 */
    -      uint8_t RESERVED_6[440];
    - __I  uint32_t ID;                                /**< Peripheral identification register., offset: 0xFFC */
    } USART_Type;

    - CLOCK_EnableClock(s_flexcommClocks[idx]);
        - FLEXCOMM_Init (reg_pointer_usart)
            - USART_Init
   - RESET_PeripheralReset(s_flexcommResets[idx]);
       - FLEXCOMM_Init (reg_pointer_usart)
            - USART_Init
    - RESET_PeripheralReset(BOARD_DEBUG_UART_RST);
        - typedef enum _SYSCON_RSTn (**fsl_reset.h**)
        - {
        -     kFLASH_RST_SHIFT_RSTn = 0 | 7U,          /**< Flash controller reset control */
        -     kFMC_RST_SHIFT_RSTn = 0 | 8U,            /**< Flash accelerator reset control */
        -     ...
        -     kADC0_RST_SHIFT_RSTn = 0 | 27U,          /**< ADC0 reset control */
        - 
        -     kMRT_RST_SHIFT_RSTn = 65536 | 0U,        /**< Multi-rate timer (MRT) reset control */
        -     ...
        -     kFC0_RST_SHIFT_RSTn = 65536 | 11U,       /**< Flexcomm Interface 0 reset control */
        -     kFC1_RST_SHIFT_RSTn = 65536 | 12U,       /**< Flexcomm Interface 1 reset control */
        -     kFC2_RST_SHIFT_RSTn = 65536 | 13U,       /**< Flexcomm Interface 2 reset control */
        -     kFC3_RST_SHIFT_RSTn = 65536 | 14U,       /**< Flexcomm Interface 3 reset control */
        -     kFC4_RST_SHIFT_RSTn = 65536 | 15U,       /**< Flexcomm Interface 4 reset control */
        -     ...
        - }


    - clock attach
    - clock frequency
    - clock source
    - #define BOARD_DEBUG_UART_CLK_FREQ CLOCK_GetFlexCommClkFreq(0)
        - necessario per impostare il baudrate
    - #define BOARD_DEBUG_UART_CLK_ATTACH kFRO12M_to_FLEXCOMM0
        - #define BOARD_DEBUG_UART_CLKSRC kCLOCK_Flexcomm0


    - CLOCK_GetFro12MFreq
        - SYSCON->FCLKSEL[id] / CLOCK_GetFlexCommClkFreq

/** SYSCON - Register Layout Typedef */ (see **Table118 Registeroverview:Mainsystemconfiguration(baseaddress0x40000000)** )
typedef struct {
    uint8_t RESERVED_0[16];
    
    ...
    
    __IO uint32_t FCLKSEL[10];                       /**< Flexcomm 0 clock source select, array offset: 0x2B0, array step: 0x4 */

    ...
} SYSCON_Type;


#define BOARD_DEBUG_UART_CLK_FREQ CLOCK_GetFlexCommClkFreq(0)
#define BOARD_DEBUG_UART_CLK_ATTACH kFRO12M_to_FLEXCOMM0
#define BOARD_DEBUG_UART_RST kFC0_RST_SHIFT_RSTn
#define BOARD_DEBUG_UART_CLKSRC kCLOCK_Flexcomm0

 