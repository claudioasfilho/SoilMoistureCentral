/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
#include "em_common.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app.h"

// The advertising set handle allocated from Bluetooth stack.




typedef struct {
  bd_addr address;
  uint8_t address_type;
  uint8_t connection;
}CONN_DEVICES;

static CONN_DEVICES Connectable_Devices_Array[40];
static uint8_t Connectable_Devices_Counter = 0;
static uint8_t Connected_Devices_Counter = 0;
static uint8_t Connecting_Devices_Counter = 0;
static uint8_t ConInnProcessTimeCounter = 0;

#define MAX_NUMBER_OF_CONNECTABLE_DEVICES 40
#define MAX_NUMBER_OF_CONNECTIONS 32

#define CENTRAL_SOFTTIMER_HANDLER 0xFE

typedef enum CentralStages
{
      Disabled,
      Scanning,
      Scanning_Completed,
      Connecting_Devices,
      Connection_in_Process,
      Connections_Completed,
      Collecting_Data
}CENTRALSTAGES;

typedef union {

        uint32_t data;
        uint8_t array[4];
}_32BArray_Union_t;

CENTRALSTAGES Central_State = Disabled;
typedef struct {
  uint16_t connection_handle;   //This is used for connection handle for connection oriented, and for sync handle for connection less mode
  bd_addr address;
  uint8_t address_type;
  uint32_t cte_service_handle;
  uint16_t cte_enable_char_handle;
  //connection_state_t connection_state;
  //aoa_libitems_t aoa_states;
} conn_properties_t;


//#define AD_FIELD_I 0x06
//#define AD_FIELD_C 0x07

#define SERVICE_UUID_LEN 2
//static const uint8_t cte_service[SERVICE_UUID_LEN] = { 0x9, 0x18 };

const char DEVICE_NAME_STRING[] = "MoistSensor";//"Silabsxx:xx";

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////
}

void Change_Central_State(CENTRALSTAGES Temp_Central_State)
{
  switch (Temp_Central_State)
  {
    case Disabled:
      Central_State = Disabled;
      app_log("Central_State = Disabled\n\r");
    break;

    case Scanning:
      Central_State = Scanning;
      app_log("Central_State = Scanning\n\r");
    break;

    case Scanning_Completed:
      Central_State = Scanning_Completed;
      app_log("Central_State = Scanning_Completed\n\r");
    break;

    case Connecting_Devices:
      Central_State = Connecting_Devices;
      app_log("Central_State = Connecting_Devices\n\r");
    break;

    case Connection_in_Process:
      Central_State = Connection_in_Process;
      app_log("Central_State = Connection_in_Process\n\r");
    break;

    case Connections_Completed:
      Central_State = Connections_Completed;
      app_log("Central_State = Connections_Completed\n\r");
    break;

    case Collecting_Data:
      Central_State = Collecting_Data;
      app_log("Central_State = Collecting_Data\n\r");

    break;

  }

}

//sl_bt_gatt_set_characteristic_notification(1, 24, 2)
static int process_scan_response(struct sl_bt_evt_scanner_scan_report_s *pResp) {
  int i = 0;
  int adMatchFound = 0;
  int adLen;
  int adType;

  app_log("Processing Scanning report \n\r");
  while (i < (pResp->data.len - 1)) {
    adLen = pResp->data.data[i];
    adType = pResp->data.data[i + 1];

    // Type 0x09 = Complete Local Name, 0x08 Shortened Name
    if (adType == 0x08) {

        printf("%s \n\r", pResp->data.data + i + 2);
      // Check if device name is Throughput Tester
      if (memcmp(pResp->data.data + i + 2, DEVICE_NAME_STRING, 6) == 0) {
        adMatchFound = 1;
        app_log(" Match found \n\r");
        app_log("Connectable devices counter %d",Connectable_Devices_Counter);
      //  if (Connectable_Devices_Counter < 32)
        {
          Connectable_Devices_Array[Connectable_Devices_Counter].address = pResp->address;
          Connectable_Devices_Array[Connectable_Devices_Counter].address_type = pResp->address_type;
          //app_log("Connectable devices found %d",Connectable_Devices_Counter);
          Connectable_Devices_Counter++;
        }
        break;
      }
      else app_log(" NO Match \n\r");
    }
    // Jump to next AD record
    i = i + adLen + 1;
  }

  return (adMatchFound);
}



void printDeviceAddress(bd_addr address)
{
  app_log("%2x:%2x:%2x:%2x:%2x:%2x \n\r",address.addr[5],address.addr[4],address.addr[3],address.addr[2],address.addr[1],address.addr[0]);
}
/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  //sl_status_t sc;
  //uint8_t conn_handle;
  //conn_properties_t* conn;
  _32BArray_Union_t Sensor_data;
  static uint8_t MinuteCounter = 0;


  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
      // Print boot message.
      app_log("Bluetooth stack booted: v%d.%d.%d-b%d\n",
              evt->data.evt_system_boot.major,
              evt->data.evt_system_boot.minor,
              evt->data.evt_system_boot.patch,
              evt->data.evt_system_boot.build);

      Connectable_Devices_Counter = 0;
      Connected_Devices_Counter = 0;
      Central_State = Disabled;

      //Set a continuous Timer to drive the Central App State Machine
      sl_bt_system_set_soft_timer(1*32768, CENTRAL_SOFTTIMER_HANDLER, 0);
      sl_bt_scanner_start(1, 1);

      Change_Central_State(Scanning);


      break;

      case sl_bt_evt_scanner_scan_report_id:
        // Check if the tag is whitelisted
      {

        if (process_scan_response(&(evt->data.evt_scanner_scan_report)))
        {
          sl_bt_scanner_stop();
          app_log("Final count on Connectable devices %d\n\r",Connectable_Devices_Counter+1);
          app_log("List of Connectable devices\n\r");

          for (uint8_t i =0 ; i<Connectable_Devices_Counter; i++)
          {
            printDeviceAddress(Connectable_Devices_Array[i].address);
            //app_log("%2x:%2x:%2x:%2x:%2x:%2x \n\r",Connectable_Devices_Array[i].address.addr[5],Connectable_Devices_Array[i].address.addr[4],Connectable_Devices_Array[i].address.addr[3],Connectable_Devices_Array[i].address.addr[2],Connectable_Devices_Array[i].address.addr[1],Connectable_Devices_Array[i].address.addr[0]);
          }

          Change_Central_State(Scanning_Completed);
        }


      }
        break;

    case sl_bt_evt_system_soft_timer_id:

    if (evt->data.evt_system_soft_timer.handle == CENTRAL_SOFTTIMER_HANDLER)
           {
             if (Central_State == Scanning_Completed)
             {
               Change_Central_State(Connecting_Devices);
               Connected_Devices_Counter = 0;
               Connecting_Devices_Counter = 0;

               sl_bt_connection_open(Connectable_Devices_Array[Connecting_Devices_Counter].address, Connectable_Devices_Array[Connecting_Devices_Counter].address_type, 1,&Connectable_Devices_Array[Connecting_Devices_Counter].connection);
               Change_Central_State(Connection_in_Process);
               ConInnProcessTimeCounter = 0;
             }

             if (Central_State == Collecting_Data)

               {
                 if(MinuteCounter++ >59)
                   {
                     sl_bt_gatt_read_characteristic_value(1, 21);
                     MinuteCounter = 0;
                   }


               }



           }


      break;




    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:

      app_log("Connection opened %d\n", Connected_Devices_Counter++);
      app_log("%2x:%2x:%2x:%2x:%2x:%2x \n\r",
              evt->data.evt_connection_opened.address.addr[5],
              evt->data.evt_connection_opened.address.addr[4],
              evt->data.evt_connection_opened.address.addr[3],
              evt->data.evt_connection_opened.address.addr[2],
              evt->data.evt_connection_opened.address.addr[1],
              evt->data.evt_connection_opened.address.addr[0]);
      Change_Central_State(Connections_Completed);


      Change_Central_State(Collecting_Data);
      MinuteCounter = 0;



      break;

    case sl_bt_evt_gatt_characteristic_value_id:

      Sensor_data.array[0] = evt->data.evt_gatt_characteristic_value.value.data[0];
      Sensor_data.array[1] = evt->data.evt_gatt_characteristic_value.value.data[1];
      Sensor_data.array[2] = evt->data.evt_gatt_characteristic_value.value.data[2];
      Sensor_data.array[3] = evt->data.evt_gatt_characteristic_value.value.data[3];

      app_log("Moisture Sensor data = %d \n\r", Sensor_data.data );

      break;
    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      app_log("Connection closed\n");
      Central_State = Scanning_Completed;

      // Restart advertising after client has disconnected.

      break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}
