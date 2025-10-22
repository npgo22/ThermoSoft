/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ethernetif.h"
#include "lwip.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/opt.h"
#include "lwip/timeouts.h"
#include "lwip/udp.h"
#include "max31856.h"
#include "netif/etharp.h"

#if LWIP_DHCP
#include "lwip/dhcp.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ETH_TxPacketConfigTypeDef TxConfig;
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

DCACHE_HandleTypeDef hdcache1;

ETH_HandleTypeDef heth;

FDCAN_HandleTypeDef hfdcan1;

FMAC_HandleTypeDef hfmac;

SPI_HandleTypeDef hspi1;

PCD_HandleTypeDef hpcd_USB_DRD_FS;

/* USER CODE BEGIN PV */
extern struct netif gnetif;

// MAX31856 sensor instances
max31856_t therm1, therm2, therm3, therm4;
struct udp_pcb *udp_tx_pcb;
sensor_data_packet_t sensor_packet;
uint8_t batch_index = 0; // Current index in the batch (0 to BATCH_SIZE-1)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_SPI1_Init(void);
static void MX_ICACHE_Init(void);
static void MX_DCACHE1_Init(void);
static void MX_FMAC_Init(void);
/* USER CODE BEGIN PFP */
static void Netif_Config(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void Init_MAX31856_Sensors(void)
{
  // Initialize Sensor 1
  therm1.spi_handle = &hspi1;
  therm1.cs_pin.gpio_port = CS1_Port;
  therm1.cs_pin.gpio_pin = CS1_Pin;
  therm1.fault_pin.gpio_port = NFAULT1_Port;
  therm1.fault_pin.gpio_pin = NFAULT1_Pin;
  therm1.drdy_pin.gpio_port = NDRDY1_Port;
  therm1.drdy_pin.gpio_pin = NDRDY1_Pin;
  max31856_init(&therm1);
  max31856_set_noise_filter(&therm1, MAX31856_NOISE_FILTER);
  max31856_set_cold_junction_enable(&therm1, MAX31856_CJ_ENABLE);
  max31856_set_thermocouple_type(&therm1, MAX31856_TC_TYPE);
  max31856_set_average_samples(&therm1, MAX31856_AVG_SAMPLES);
  max31856_set_open_circuit_fault_detection(&therm1, MAX31856_OC_FAULT_DETECT);
  max31856_set_conversion_mode(&therm1, MAX31856_CONVERSION_MODE);

  // Initialize Sensor 2
  therm2.spi_handle = &hspi1;
  therm2.cs_pin.gpio_port = CS2_Port;
  therm2.cs_pin.gpio_pin = CS2_Pin;
  therm2.fault_pin.gpio_port = NFAULT2_Port;
  therm2.fault_pin.gpio_pin = NFAULT2_Pin;
  therm2.drdy_pin.gpio_port = NDRDY2_Port;
  therm2.drdy_pin.gpio_pin = NDRDY2_Pin;
  max31856_init(&therm2);
  max31856_set_noise_filter(&therm2, MAX31856_NOISE_FILTER);
  max31856_set_cold_junction_enable(&therm2, MAX31856_CJ_ENABLE);
  max31856_set_thermocouple_type(&therm2, MAX31856_TC_TYPE);
  max31856_set_average_samples(&therm2, MAX31856_AVG_SAMPLES);
  max31856_set_open_circuit_fault_detection(&therm2, MAX31856_OC_FAULT_DETECT);
  max31856_set_conversion_mode(&therm2, MAX31856_CONVERSION_MODE);

  // Initialize Sensor 3
  therm3.spi_handle = &hspi1;
  therm3.cs_pin.gpio_port = CS3_Port;
  therm3.cs_pin.gpio_pin = CS3_Pin;
  therm3.fault_pin.gpio_port = NFAULT3_Port;
  therm3.fault_pin.gpio_pin = NFAULT3_Pin;
  therm3.drdy_pin.gpio_port = NDRDY3_Port;
  therm3.drdy_pin.gpio_pin = NDRDY3_Pin;
  max31856_init(&therm3);
  max31856_set_noise_filter(&therm3, MAX31856_NOISE_FILTER);
  max31856_set_cold_junction_enable(&therm3, MAX31856_CJ_ENABLE);
  max31856_set_thermocouple_type(&therm3, MAX31856_TC_TYPE);
  max31856_set_average_samples(&therm3, MAX31856_AVG_SAMPLES);
  max31856_set_open_circuit_fault_detection(&therm3, MAX31856_OC_FAULT_DETECT);
  max31856_set_conversion_mode(&therm3, MAX31856_CONVERSION_MODE);

  // Initialize Sensor 4
  therm4.spi_handle = &hspi1;
  therm4.cs_pin.gpio_port = CS4_Port;
  therm4.cs_pin.gpio_pin = CS4_Pin;
  therm4.fault_pin.gpio_port = NFAULT4_Port;
  therm4.fault_pin.gpio_pin = NFAULT4_Pin;
  therm4.drdy_pin.gpio_port = NDRDY4_Port;
  therm4.drdy_pin.gpio_pin = NDRDY4_Pin;
  max31856_init(&therm4);
  max31856_set_noise_filter(&therm4, MAX31856_NOISE_FILTER);
  max31856_set_cold_junction_enable(&therm4, MAX31856_CJ_ENABLE);
  max31856_set_thermocouple_type(&therm4, MAX31856_TC_TYPE);
  max31856_set_average_samples(&therm4, MAX31856_AVG_SAMPLES);
  max31856_set_open_circuit_fault_detection(&therm4, MAX31856_OC_FAULT_DETECT);
  max31856_set_conversion_mode(&therm4, MAX31856_CONVERSION_MODE);
}

static void Init_UDP_Socket(void)
{
  ip_addr_t dest_addr;

  // Create new UDP PCB
  udp_tx_pcb = udp_new();
  if (udp_tx_pcb == NULL) {
    // Handle error
    return;
  }

  // Bind to any local address and port
  err_t err = udp_bind(udp_tx_pcb, IP_ADDR_ANY, 0);
  if (err != ERR_OK) {
    udp_remove(udp_tx_pcb);
    return;
  }

  // Set destination address and port
  IP4_ADDR(&dest_addr, UDP_DEST_IP_ADDR0, UDP_DEST_IP_ADDR1, UDP_DEST_IP_ADDR2, UDP_DEST_IP_ADDR3);
  err = udp_connect(udp_tx_pcb, &dest_addr, UDP_DEST_PORT);
  if (err != ERR_OK) {
    udp_remove(udp_tx_pcb);
    return;
  }
}

static void Read_MAX31856_Sensor(max31856_t *sensor, float *temp_value)
{
  // Check for faults
  max31856_read_fault(sensor);

  if (sensor->sr.val != 0) {
    // Fault detected - send 0.0
    *temp_value = 0.0f;

    // Clear fault for next reading
    max31856_clear_fault_status(sensor);
    return;
  }

  // No fault - read thermocouple temperature
  *temp_value = max31856_read_TC_temp(sensor);
}

static void Send_Sensor_Data_UDP(void)
{
  struct pbuf *p;

  // Set packet tag and timestamp
  sensor_packet.packetTag = PACKET_ID;
  sensor_packet.packetTime = HAL_GetTick();

  // Allocate pbuf for data
  p = pbuf_alloc(PBUF_TRANSPORT, sizeof(sensor_data_packet_t), PBUF_RAM);
  if (p == NULL) {
    return;
  }

  // Copy data to pbuf
  memcpy(p->payload, &sensor_packet, sizeof(sensor_data_packet_t));

  // Send the data
  err_t err = udp_send(udp_tx_pcb, p);

  // Free the pbuf
  pbuf_free(p);

  if (err != ERR_OK) {
    // Handle send error
  }

  // Reset batch index for next packet
  batch_index = 0;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_USB_PCD_Init();
  MX_SPI1_Init();
  MX_ICACHE_Init();
  MX_DCACHE1_Init();
  MX_FMAC_Init();
  /* USER CODE BEGIN 2 */

  lwip_init();
  Netif_Config();
  // Initialize MAX31856 sensors
  Init_MAX31856_Sensors();

  ethernetif_input(&gnetif);
  sys_check_timeouts();

  // Initialize UDP socket
  Init_UDP_Socket();

  uint32_t last_send_time = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    //     /* Read a received packet from the Ethernet buffers and send it
    //      to the lwIP for handling */
    //     ethernetif_input(&gnetif);
    //     /* Handle timeouts */
    //     sys_check_timeouts();
    // #if LWIP_NETIF_LINK_CALLBACK
    //     Ethernet_Link_Periodic_Handle(&gnetif);
    // #endif
    // #if LWIP_DHCP
    //     DHCP_Periodic_Handle(&gnetif);
    // #endif
    /* Read a received packet from the Ethernet buffers and send it
     to the lwIP for handling */
    ethernetif_input(&gnetif);
    /* Handle timeouts */
    sys_check_timeouts();
#if LWIP_NETIF_LINK_CALLBACK
    Ethernet_Link_Periodic_Handle(&gnetif);
#endif
#if LWIP_DHCP
    DHCP_Periodic_Handle(&gnetif);
#endif

    // Collect sensor readings at configured interval
    if ((HAL_GetTick() - last_send_time) >= UDP_SEND_INTERVAL_MS) {
      if (netif_is_up(&gnetif)) {
        // Read all sensors and store in current batch position
        Read_MAX31856_Sensor(&therm1, &sensor_packet.tc1_temps[batch_index]);
        Read_MAX31856_Sensor(&therm2, &sensor_packet.tc2_temps[batch_index]);
        Read_MAX31856_Sensor(&therm3, &sensor_packet.tc3_temps[batch_index]);
        Read_MAX31856_Sensor(&therm4, &sensor_packet.tc4_temps[batch_index]);

        batch_index++;

        // When batch is full, send the packet
        if (batch_index >= BATCH_SIZE) {
          Send_Sensor_Data_UDP();
        }
      }
      last_send_time = HAL_GetTick();
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 10;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }

  /** Configure the programming delay
   */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
   */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN | RCC_PERIPHCLK_SPI1;
  PeriphClkInitStruct.PLL2.PLL2Source = RCC_PLL2_SOURCE_HSE;
  PeriphClkInitStruct.PLL2.PLL2M = 5;
  PeriphClkInitStruct.PLL2.PLL2N = 80;
  PeriphClkInitStruct.PLL2.PLL2P = 80;
  PeriphClkInitStruct.PLL2.PLL2Q = 5;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2_VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2_VCORANGE_WIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.PLL2.PLL2ClockOut = RCC_PLL2_DIVP | RCC_PLL2_DIVQ;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2Q;
  PeriphClkInitStruct.Spi1ClockSelection = RCC_SPI1CLKSOURCE_PLL2P;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief DCACHE1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DCACHE1_Init(void)
{
  /* USER CODE BEGIN DCACHE1_Init 0 */

  /* USER CODE END DCACHE1_Init 0 */

  /* USER CODE BEGIN DCACHE1_Init 1 */

  /* USER CODE END DCACHE1_Init 1 */
  hdcache1.Instance = DCACHE1;
  hdcache1.Init.ReadBurstType = DCACHE_READ_BURST_WRAP;
  if (HAL_DCACHE_Init(&hdcache1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN DCACHE1_Init 2 */

  /* USER CODE END DCACHE1_Init 2 */
}

/**
 * @brief ETH Initialization Function
 * @param None
 * @retval None
 */
void MX_ETH_Init(void)
{
  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

  static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK) {
    Error_Handler();
  }

  memset(&TxConfig, 0, sizeof(ETH_TxPacketConfigTypeDef));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */
}

/**
 * @brief FDCAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_FDCAN1_Init(void)
{
  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 16;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 1;
  hfdcan1.Init.NominalTimeSeg2 = 1;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */
}

/**
 * @brief FMAC Initialization Function
 * @param None
 * @retval None
 */
static void MX_FMAC_Init(void)
{
  /* USER CODE BEGIN FMAC_Init 0 */

  /* USER CODE END FMAC_Init 0 */

  /* USER CODE BEGIN FMAC_Init 1 */

  /* USER CODE END FMAC_Init 1 */
  hfmac.Instance = FMAC;
  if (HAL_FMAC_Init(&hfmac) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN FMAC_Init 2 */

  /* USER CODE END FMAC_Init 2 */
}

/**
 * @brief ICACHE Initialization Function
 * @param None
 * @retval None
 */
static void MX_ICACHE_Init(void)
{
  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
   */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{
  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x7;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi1.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi1.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief USB Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_PCD_Init(void)
{
  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_DRD_FS.Instance = USB_DRD_FS;
  hpcd_USB_DRD_FS.Init.dev_endpoints = 8;
  hpcd_USB_DRD_FS.Init.speed = USBD_FS_SPEED;
  hpcd_USB_DRD_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_DRD_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.bulk_doublebuffer_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.iso_singlebuffer_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_DRD_FS) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC,
                    GPIO_PIN_14 | GPIO_PIN_2 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_12,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_8 | GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC15 PC0 PC3
                           PC10 PC11 */
  GPIO_InitStruct.Pin =
      GPIO_PIN_13 | GPIO_PIN_15 | GPIO_PIN_0 | GPIO_PIN_3 | GPIO_PIN_10 | GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC2 PC7 PC8
                           PC9 PC12 */
  GPIO_InitStruct.Pin =
      GPIO_PIN_14 | GPIO_PIN_2 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA8 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_8 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief  Setup the network interface
 *   None
 * @retval None
 */
static void Netif_Config(void)
{
  ip_addr_t ipaddr;
  ip_addr_t netmask;
  ip_addr_t gw;
#if LWIP_DHCP
  ip_addr_set_zero_ip4(&ipaddr);
  ip_addr_set_zero_ip4(&netmask);
  ip_addr_set_zero_ip4(&gw);
#else
  /* IP address default setting */
  IP4_ADDR(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
  IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
  IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
#endif
  /* add the network interface */
  netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &ethernet_input);
  /*  Registers the default network interface */
  netif_set_default(&gnetif);
#if LWIP_NETIF_LINK_CALLBACK
  netif_set_link_callback(&gnetif, ethernet_link_status_updated);
#if LWIP_DHCP
  dhcp_start(&gnetif);
#endif
  ethernet_link_status_updated(&gnetif);
#endif
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
