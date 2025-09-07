/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ux_device_customhid.c
  * @author  MCD Application Team
  * @brief   USBX Device Custom HID applicative source file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "ux_device_customhid.h"

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
  * @brief  USBD_Custom_HID_Activate
  *         This function is called when insertion of a Custom HID device.
  * @param  hid_instance: Pointer to the hid class instance.
  * @retval none
  */
VOID USBD_Custom_HID_Activate(VOID *hid_instance)
{
  /* USER CODE BEGIN USBD_Custom_HID_Activate */
  UX_PARAMETER_NOT_USED(hid_instance);
  /* USER CODE END USBD_Custom_HID_Activate */

  return;
}

/**
  * @brief  USBD_Custom_HID_Deactivate
  *         This function is called when extraction of a Custom HID device.
  * @param  hid_instance: Pointer to the hid class instance.
  * @retval none
  */
VOID USBD_Custom_HID_Deactivate(VOID *hid_instance)
{
  /* USER CODE BEGIN USBD_Custom_HID_Deactivate */
  UX_PARAMETER_NOT_USED(hid_instance);
  /* USER CODE END USBD_Custom_HID_Deactivate */

  return;
}

/**
  * @brief  USBD_Custom_HID_SetFeature
  *         This function is invoked when the host sends a HID SET_REPORT
  *         to the application over Endpoint 0 (Set Feature).
  * @param  hid_instance: Pointer to the hid class instance.
  * @param  hid_event: Pointer to structure of the hid event.
  * @retval status
  */
UINT USBD_Custom_HID_SetFeature(UX_SLAVE_CLASS_HID *hid_instance,
                                UX_SLAVE_CLASS_HID_EVENT *hid_event)
{
  UINT status = UX_SUCCESS;

  /* USER CODE BEGIN USBD_Custom_HID_SetFeature */
  UX_PARAMETER_NOT_USED(hid_instance);
  UX_PARAMETER_NOT_USED(hid_event);
  /* USER CODE END USBD_Custom_HID_SetFeature */

  return status;
}

/**
  * @brief  USBD_Custom_HID_GetReport
  *         This function is invoked when host is requesting event through
  *         control GET_REPORT request.
  * @param  hid_instance: Pointer to the hid class instance.
  * @param  hid_event: Pointer to structure of the hid event.
  * @retval status
  */
UINT USBD_Custom_HID_GetReport(UX_SLAVE_CLASS_HID *hid_instance,
                               UX_SLAVE_CLASS_HID_EVENT *hid_event)
{
  UINT status = UX_SUCCESS;

  /* USER CODE BEGIN USBD_Custom_HID_GetReport */
  UX_PARAMETER_NOT_USED(hid_instance);
  UX_PARAMETER_NOT_USED(hid_event);
  /* USER CODE END USBD_Custom_HID_GetReport */

  return status;
}

#ifdef UX_DEVICE_CLASS_HID_INTERRUPT_OUT_SUPPORT

/**
  * @brief  USBD_Custom_HID_SetReport
  *         This function is invoked when the host sends a HID SET_REPORT
  *         to the application over Endpoint OUT (Set Report).
  * @param  hid_instance: Pointer to the hid class instance.
  * @retval none
  */
VOID USBD_Custom_HID_SetReport(struct UX_SLAVE_CLASS_HID_STRUCT *hid_instance)
{
  /* USER CODE BEGIN USBD_Custom_HID_SetReport */
  UX_PARAMETER_NOT_USED(hid_instance);
  /* USER CODE END USBD_Custom_HID_SetReport */

  return;
}

/**
  * @brief  USBD_Custom_HID_EventMaxNumber
  *         This function to set receiver event max number parameter.
  * @param  none
  * @retval receiver event max number
  */
ULONG USBD_Custom_HID_EventMaxNumber(VOID)
{
  ULONG max_number = 0U;

  /* USER CODE BEGIN USBD_Custom_HID_EventMaxNumber */

  /* USER CODE END USBD_Custom_HID_EventMaxNumber */

  return max_number;
}

/**
  * @brief  USBD_Custom_HID_EventMaxLength
  *         This function to set receiver event max length parameter.
  * @param  none
  * @retval receiver event max length
  */
ULONG USBD_Custom_HID_EventMaxLength(VOID)
{
  ULONG max_length = 0U;

  /* USER CODE BEGIN USBD_Custom_HID_EventMaxLength */

  /* USER CODE END USBD_Custom_HID_EventMaxLength */

  return max_length;
}

#endif /* UX_DEVICE_CLASS_HID_INTERRUPT_OUT_SUPPORT */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
