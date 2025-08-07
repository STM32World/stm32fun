/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ux_device_dfu_media.c
  * @author  MCD Application Team
  * @brief   USBX Device applicative file
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

#include "ux_device_dfu_media.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

#define DFU_MEDIA_ERASE_TIME    (uint16_t)5U
#define DFU_MEDIA_PROGRAM_TIME  (uint16_t)5U

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  USBD_DFU_Activate
  *         This function is called when insertion of a DFU device.
  * @param  dfu_instance: Pointer to the dfu class instance.
  * @retval none
  */
VOID USBD_DFU_Activate(VOID *dfu_instance)
{
  /* USER CODE BEGIN USBD_DFU_Activate */
  UX_PARAMETER_NOT_USED(dfu_instance);
  /* USER CODE END USBD_DFU_Activate */

  return;
}

/**
  * @brief  USBD_DFU_Deactivate
  *         This function is called when extraction of a DFU device.
  * @param  dfu_instance: Pointer to the dfu class instance.
  * @retval none
  */
VOID USBD_DFU_Deactivate(VOID *dfu_instance)
{
  /* USER CODE BEGIN USBD_DFU_Deactivate */
  UX_PARAMETER_NOT_USED(dfu_instance);
  /* USER CODE END USBD_DFU_Deactivate */

  return;
}

/**
  * @brief  USBD_DFU_GetStatus
  *         This function is invoked to get media status.
  * @param  dfu_instance: Pointer to the dfu class instance.
  * @param  media_status: dfu media status.
  * @retval status
  */
UINT USBD_DFU_GetStatus(VOID *dfu_instance, ULONG *media_status)
{
  UINT status = UX_SUCCESS;

  /* USER CODE BEGIN USBD_DFU_GetStatus */
  UX_PARAMETER_NOT_USED(dfu_instance);
  UX_PARAMETER_NOT_USED(media_status);
  /* USER CODE END USBD_DFU_GetStatus */

  return status;
}

/**
  * @brief  USBD_DFU_Read
  *         This function is invoked when host is requesting to read from media.
  * @param  dfu_instance: Pointer to the dfu class instance.
  * @param  block_number: block number.
  * @param  data_pointer: Pointer to the Source buffer.
  * @param  length: Number of data to be read (in bytes).
  * @param  actual_length: length of data to be written.
  * @retval status
  */
UINT USBD_DFU_Read(VOID *dfu_instance, ULONG block_number, UCHAR *data_pointer,
                   ULONG length, ULONG *actual_length)
{
  UINT status = UX_SUCCESS;

  /* USER CODE BEGIN USBD_DFU_Read */
  UX_PARAMETER_NOT_USED(dfu_instance);
  UX_PARAMETER_NOT_USED(block_number);
  UX_PARAMETER_NOT_USED(data_pointer);
  UX_PARAMETER_NOT_USED(length);
  UX_PARAMETER_NOT_USED(actual_length);
  /* USER CODE END USBD_DFU_Read */

  return status;
}

/**
  * @brief  USBD_DFU_Write
  *         This function is invoked when host is requesting to write in media.
  * @param  dfu_instance: Pointer to the dfu class instance.
  * @param  block_number: block number.
  * @param  data_pointer: Pointer to the Source buffer.
  * @param  length: Number of data to be read (in bytes).
  * @param  media_status: dfu media status.
  * @retval status
  */
UINT USBD_DFU_Write(VOID *dfu_instance, ULONG block_number, UCHAR *data_pointer,
                    ULONG length, ULONG *media_status)
{
  UINT status = UX_SUCCESS;

  /* USER CODE BEGIN USBD_DFU_Write */
  UX_PARAMETER_NOT_USED(dfu_instance);
  UX_PARAMETER_NOT_USED(block_number);
  UX_PARAMETER_NOT_USED(data_pointer);
  UX_PARAMETER_NOT_USED(length);
  UX_PARAMETER_NOT_USED(media_status);
  /* USER CODE END USBD_DFU_Write */

  return status;
}

/**
  * @brief  USBD_DFU_Notify
  *         This function is invoked to application when a begin and end
  *         of transfer of the firmware occur.
  * @param  dfu_instance: Pointer to the dfu class instance.
  * @param  notification: unused.
  * @retval status
  */
UINT USBD_DFU_Notify(VOID *dfu_instance, ULONG notification)
{
  UINT status = UX_SUCCESS;

  /* USER CODE BEGIN USBD_DFU_Notify */
  UX_PARAMETER_NOT_USED(dfu_instance);
  UX_PARAMETER_NOT_USED(notification);
  /* USER CODE END USBD_DFU_Notify */

  return status;
}

#ifdef UX_DEVICE_CLASS_DFU_CUSTOM_REQUEST_ENABLE

/**
  * @brief  USBD_DFU_CustomRequest
  *         This function is invoked to Handles DFU sub-protocol request.
  * @param  dfu_instance: Pointer to the dfu class instance.
  * @param  transfer: transfer request.
  * @retval status
  */
UINT USBD_DFU_CustomRequest(VOID *dfu_instance, UX_SLAVE_TRANSFER *transfer)
{
  UINT status = UX_SUCCESS;

  /* USER CODE BEGIN USBD_DFU_CustomRequest */
  UX_PARAMETER_NOT_USED(dfu_instance);
  UX_PARAMETER_NOT_USED(transfer);
  /* USER CODE END USBD_DFU_CustomRequest */

  return status;
}

#endif /* UX_DEVICE_CLASS_DFU_CUSTOM_REQUEST_ENABLE */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
