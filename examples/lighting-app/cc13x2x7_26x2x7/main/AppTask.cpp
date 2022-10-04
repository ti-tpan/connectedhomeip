/*
 *
 *    Copyright (c) 2020 Project CHIP Authors
 *    Copyright (c) 2020 Texas Instruments Incorporated
 *    All rights reserved.
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *        http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

#include "AppTask.h"
#include "AppConfig.h"
#include "AppEvent.h"
#include <app/server/Server.h>

#include "FreeRTOS.h"

#include <credentials/DeviceAttestationCredsProvider.h>
#include <credentials/examples/DeviceAttestationCredsExample.h>

#include <platform/CHIPDeviceLayer.h>

#include <app/clusters/ota-requestor/BDXDownloader.h>
#include <app/clusters/ota-requestor/DefaultOTARequestor.h>
#include <app/clusters/ota-requestor/DefaultOTARequestorDriver.h>
#include <app/clusters/ota-requestor/DefaultOTARequestorStorage.h>
#include <lib/support/CHIPMem.h>
#include <lib/support/CHIPPlatformMemory.h>
#include <platform/cc13x2_26x2/OTAImageProcessorImpl.h>

#include <app/server/OnboardingCodesUtil.h>

#include <app-common/zap-generated/attribute-id.h>
#include <app-common/zap-generated/attribute-type.h>
#include <app-common/zap-generated/cluster-id.h>
#include <app/util/attribute-storage.h>

#include <ti/drivers/apps/Button.h>
#include <ti/drivers/apps/LED.h>

/* syscfg */
#include <ti_drivers_config.h>

#define APP_TASK_STACK_SIZE (4096)
#define APP_TASK_PRIORITY 4
#define APP_EVENT_QUEUE_SIZE 10

// local data structures
typedef struct
{
    uint8_t isCarConnected;
    uint8_t chargeLevel; // 254 means 100%
} EVChargeData;
typedef enum
{
    TLV_Type_bool = 'b',
    TLV_Type_uint8 = 'c',
    TLV_Type_uint16 = 'd',
} TLV_Type;
typedef struct
{
    TLV_Type type;
    uint8_t length;
    uint32_t value;
} TLV;

using namespace ::chip;
using namespace ::chip::Credentials;
using namespace ::chip::DeviceLayer;

static TaskHandle_t sAppTaskHandle;
static QueueHandle_t sAppEventQueue;

static LED_Handle sAppRedHandle;
static LED_Handle sAppGreenHandle;
static Button_Handle sAppLeftHandle;
static Button_Handle sAppRightHandle;
static UART_Handle sUartHandle;
static uint8_t sDataBuf[11] = {0}; // we know what the packet structure will look like
static EVChargeData sEvChargerData =
{
    .isCarConnected = 0,
    .chargeLevel = 0,
};


AppTask AppTask::sAppTask;

static DefaultOTARequestor sRequestorCore;
static DefaultOTARequestorStorage sRequestorStorage;
static DefaultOTARequestorDriver sRequestorUser;
static BDXDownloader sDownloader;
static OTAImageProcessorImpl sImageProcessor;

void InitializeOTARequestor(void)
{
    // Initialize and interconnect the Requestor and Image Processor objects
    SetRequestorInstance(&sRequestorCore);

    sRequestorStorage.Init(Server::GetInstance().GetPersistentStorage());
    sRequestorCore.Init(Server::GetInstance(), sRequestorStorage, sRequestorUser, sDownloader);
    sImageProcessor.SetOTADownloader(&sDownloader);
    sDownloader.SetImageProcessorDelegate(&sImageProcessor);
    sRequestorUser.Init(&sRequestorCore, &sImageProcessor);
}

static void LocalErrorSpin()
{

    // error...
    if (sAppGreenHandle != 0)
    {
        LED_setOn(sAppGreenHandle, LED_BRIGHTNESS_MAX);
        LED_startBlinking(sAppGreenHandle, 500, LED_BLINK_FOREVER);
    }
    if (sAppRedHandle != 0)
    {
        LED_setOff(sAppRedHandle);
        LED_startBlinking(sAppRedHandle, 500, LED_BLINK_FOREVER);
    }
    while (1);
}
void AppTask::AppUartReadCallback(UART_Handle handle, void * buf, size_t count)
{
    const uint8_t magicHeader[] = {'B', 'E', 'B', 'E'};
    uint8_t* pData = (uint8_t*) buf;
    if (!memcmp(pData, magicHeader, sizeof(magicHeader)))
    {
        pData += sizeof(magicHeader);
        // uint8_t numOfElements = *pData; // commented here because it's unused for now
        pData += 3;
        sEvChargerData.isCarConnected = *pData;
        pData += 3;
        sEvChargerData.chargeLevel = *pData;
    
        AppTask::sAppTask.mSyncClusterToLocalAction = true; // in case there is state change
        // if no car is connected and data says car is connected (state change)
        if (BoltLockMgr().IsUnlocked() && sEvChargerData.isCarConnected)
        {
            AppEvent event;
            event.Type                  = AppEvent::kEventType_AppEvent;
            event.Handler = CarConnectedHandler;
            if (xQueueSendFromISR(sAppEventQueue, &event, NULL) != pdPASS)
            {
                /* Failed to post the message */
            }

            if (sEvChargerData.chargeLevel == 0)
                sEvChargerData.chargeLevel = 1; // minimum
        }

        // if car is connected and data says car is disconnected (state change)
        else if (!BoltLockMgr().IsUnlocked() && !sEvChargerData.isCarConnected)
        {
            sEvChargerData.chargeLevel = 0; // fulfill the "off" relationship between Level Control and On/Off
            
            AppEvent event;
            event.Type                  = AppEvent::kEventType_AppEvent;
            event.Handler = CarDisconnectedHandler;
            if (xQueueSendFromISR(sAppEventQueue, &event, NULL) != pdPASS)
            {
                /* Failed to post the message */
            }
        }

        // no state change, car is connected and charging
        else if (sEvChargerData.isCarConnected)
        {
            // no local state change/action of on/off to sync
            AppTask::sAppTask.mSyncClusterToLocalAction = false; 
            
            // schedule to update cluster state
            //chip::DeviceLayer::PlatformMgr().ScheduleWork(UpdateClusterState, reinterpret_cast<intptr_t>(nullptr));
            AppEvent event;
            event.Type                  = AppEvent::kEventType_AppEvent;
            event.Handler = NewChargeData;
            if (xQueueSendFromISR(sAppEventQueue, &event, NULL) != pdPASS)
            {
                /* Failed to post the message */
            }
            
            // fully charged
            if (sEvChargerData.chargeLevel >= 254)
            {
                sEvChargerData.chargeLevel = 254; // maximum
                LED_stopBlinking(sAppRedHandle);
                LED_setOn(sAppRedHandle, LED_BRIGHTNESS_MAX);
            }
        }
    }

    // get ready for next reading
    int32_t status = UART_read(sUartHandle, sDataBuf, sizeof(sDataBuf));
    if (status != UART_STATUS_SUCCESS)
        LocalErrorSpin();
}

int AppTask::StartAppTask()
{
    int ret = 0;

    sAppEventQueue = xQueueCreate(APP_EVENT_QUEUE_SIZE, sizeof(AppEvent));
    if (sAppEventQueue == NULL)
    {
        PLAT_LOG("Failed to allocate app event queue");
        while (1)
            ;
    }

    // Start App task.
    if (xTaskCreate(AppTaskMain, "APP", APP_TASK_STACK_SIZE / sizeof(StackType_t), NULL, APP_TASK_PRIORITY, &sAppTaskHandle) !=
        pdPASS)
    {
        PLAT_LOG("Failed to create app task");
        while (1)
            ;
    }
    return ret;
}

int AppTask::Init()
{
    LED_Params ledParams;
    Button_Params buttonParams;

    // cc13x2_26x2LogInit();

    // Init Chip memory management before the stack
    Platform::MemoryInit();

    CHIP_ERROR ret = PlatformMgr().InitChipStack();
    if (ret != CHIP_NO_ERROR)
    {
        PLAT_LOG("PlatformMgr().InitChipStack() failed");
        while (1)
            ;
    }

    ret = ThreadStackMgr().InitThreadStack();
    if (ret != CHIP_NO_ERROR)
    {
        PLAT_LOG("ThreadStackMgr().InitThreadStack() failed");
        while (1)
            ;
    }

    ret = ConnectivityMgr().SetThreadDeviceType(ConnectivityManager::kThreadDeviceType_Router);
    if (ret != CHIP_NO_ERROR)
    {
        PLAT_LOG("ConnectivityMgr().SetThreadDeviceType() failed");
        while (1)
            ;
    }

    ret = PlatformMgr().StartEventLoopTask();
    if (ret != CHIP_NO_ERROR)
    {
        PLAT_LOG("PlatformMgr().StartEventLoopTask() failed");
        while (1)
            ;
    }

    ret = ThreadStackMgrImpl().StartThreadTask();
    if (ret != CHIP_NO_ERROR)
    {
        PLAT_LOG("ThreadStackMgr().StartThreadTask() failed");
        while (1)
            ;
    }

    // Init ZCL Data Model and start server
    PLAT_LOG("Initialize Server");
    static chip::CommonCaseDeviceServerInitParams initParams;
    (void) initParams.InitializeStaticResourcesBeforeServerInit();
    chip::Server::GetInstance().Init(initParams);

    // Initialize device attestation config
    SetDeviceAttestationCredentialsProvider(Examples::GetExampleDACProvider());

    // Initialize LEDs
    PLAT_LOG("Initialize LEDs");
    LED_init();

    LED_Params_init(&ledParams); // default PWM LED
    sAppRedHandle = LED_open(CONFIG_LED_RED, &ledParams);
    LED_setOff(sAppRedHandle);

    LED_Params_init(&ledParams); // default PWM LED
    sAppGreenHandle = LED_open(CONFIG_LED_GREEN, &ledParams);
    LED_setOff(sAppGreenHandle);

    // Initialize buttons
    PLAT_LOG("Initialize buttons");
    Button_init();

    Button_Params_init(&buttonParams);
    buttonParams.buttonEventMask   = Button_EV_CLICKED | Button_EV_LONGCLICKED;
    buttonParams.longPressDuration = 1000U; // ms
    sAppLeftHandle                 = Button_open(CONFIG_BTN_LEFT, &buttonParams);
    Button_setCallback(sAppLeftHandle, ButtonLeftEventHandler);

    Button_Params_init(&buttonParams);
    buttonParams.buttonEventMask   = Button_EV_CLICKED | Button_EV_LONGCLICKED;
    buttonParams.longPressDuration = 1000U; // ms
    sAppRightHandle                = Button_open(CONFIG_BTN_RIGHT, &buttonParams);
    Button_setCallback(sAppRightHandle, ButtonRightEventHandler);

    // Initialize UART for cluster updates from host (e.g. charging status and charge level)
    UART_init();
    UART_Params uartParams;
    UART_Params_init(&uartParams);
    uartParams.readMode = UART_MODE_CALLBACK;
    uartParams.readCallback = AppUartReadCallback;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;
    sUartHandle = UART_open(CONFIG_UART_DEBUG, &uartParams);
    if (sUartHandle == 0)
    {
        LocalErrorSpin();
    }
    else
    {
#if 1
        int32_t status = UART_read(sUartHandle, sDataBuf, sizeof(sDataBuf));
        if (status != UART_STATUS_SUCCESS)
            LocalErrorSpin();
#endif
    }

    // Initialize BoltLock module
    PLAT_LOG("Initialize BoltLock");
    BoltLockMgr().Init();
    chip::DeviceLayer::PlatformMgr().ScheduleWork(UpdateClusterState, reinterpret_cast<intptr_t>(nullptr));

    BoltLockMgr().SetCallbacks(ActionInitiated, ActionCompleted);

    ConfigurationMgr().LogDeviceConfig();

    InitializeOTARequestor();

    // QR code will be used with CHIP Tool
    PrintOnboardingCodes(RendezvousInformationFlags(RendezvousInformationFlag::kBLE));

    return 0;
}

void AppTask::AppTaskMain(void * pvParameter)
{
    AppEvent event;

    sAppTask.Init();

    while (1)
    {
        /* Task pend until we have stuff to do */
        if (xQueueReceive(sAppEventQueue, &event, portMAX_DELAY) == pdTRUE)
        {
            sAppTask.DispatchEvent(&event);
        }
    }
}

void AppTask::PostEvent(const AppEvent * aEvent)
{
    if (xQueueSend(sAppEventQueue, aEvent, 0) != pdPASS)
    {
        /* Failed to post the message */
    }
}

void AppTask::ButtonLeftEventHandler(Button_Handle handle, Button_EventMask events)
{
    AppEvent event;
    event.Type = AppEvent::kEventType_ButtonLeft;

    if (events & Button_EV_CLICKED)
    {
        event.ButtonEvent.Type = AppEvent::kAppEventButtonType_Clicked;
    }
    else if (events & Button_EV_LONGCLICKED)
    {
        event.ButtonEvent.Type = AppEvent::kAppEventButtonType_LongClicked;
    }
    // button callbacks are in ISR context
    if (xQueueSendFromISR(sAppEventQueue, &event, NULL) != pdPASS)
    {
        /* Failed to post the message */
    }
}

void AppTask::ButtonRightEventHandler(Button_Handle handle, Button_EventMask events)
{
    AppEvent event;
    event.Type = AppEvent::kEventType_ButtonRight;

    if (events & Button_EV_CLICKED)
    {
        event.ButtonEvent.Type = AppEvent::kAppEventButtonType_Clicked;
    }
    else if (events & Button_EV_LONGCLICKED)
    {
        event.ButtonEvent.Type = AppEvent::kAppEventButtonType_LongClicked;
    }
    // button callbacks are in ISR context
    if (xQueueSendFromISR(sAppEventQueue, &event, NULL) != pdPASS)
    {
        /* Failed to post the message */
    }
}

void AppTask::ActionInitiated(BoltLockManager::Action_t aAction, BoltLockManager::Actor_t aActor)
{
    // If the action has been initiated by the lock, update the bolt lock trait
    // and start flashing the LEDs rapidly to indicate action initiation.
    if (aAction == BoltLockManager::LOCK_ACTION)
    {
        // PLAT_LOG("Lock initiated");     
        PLAT_LOG("Car is connecting");     
    }
    else if (aAction == BoltLockManager::UNLOCK_ACTION)
    {
        PLAT_LOG("Car is disconnecting");
    }

    if (BoltLockManager::ACTOR_APP == aActor)
    {
        sAppTask.mSyncClusterToLocalAction = true;
    }

}

void AppTask::ActionCompleted(BoltLockManager::Action_t aAction)
{
    // if the action has been completed by the lock, update the bolt lock trait.
    // Turn on the lock LED if in a LOCKED state OR
    // Turn off the lock LED if in an UNLOCKED state.
    if (aAction == BoltLockManager::LOCK_ACTION)
    {
        PLAT_LOG("Charging Device connected");
        LED_startBlinking(sAppRedHandle, 250 /* ms */, LED_BLINK_FOREVER);
    }
    else if (aAction == BoltLockManager::UNLOCK_ACTION)
    {
        PLAT_LOG("Charging Device disconnected");
        LED_stopBlinking(sAppRedHandle);
        LED_setOff(sAppRedHandle);
    }
    
    if (sAppTask.mSyncClusterToLocalAction)
    {
        chip::DeviceLayer::PlatformMgr().ScheduleWork(UpdateClusterState, reinterpret_cast<intptr_t>(nullptr));
        sAppTask.mSyncClusterToLocalAction = false;
    }
}

void AppTask::DispatchEvent(AppEvent * aEvent)
{
    switch (aEvent->Type)
    {
    case AppEvent::kEventType_ButtonLeft:
        if (AppEvent::kAppEventButtonType_Clicked == aEvent->ButtonEvent.Type)
        {
            if (!BoltLockMgr().IsUnlocked())
            {
                BoltLockMgr().InitiateAction(BoltLockManager::ACTOR_APP, BoltLockManager::UNLOCK_ACTION);
            }
        }
        else if (AppEvent::kAppEventButtonType_LongClicked == aEvent->ButtonEvent.Type)
        {
            // Disable BLE advertisements
            if (ConnectivityMgr().IsBLEAdvertisingEnabled())
            {
                chip::Server::GetInstance().ScheduleFactoryReset();
            }
        }
        break;

    case AppEvent::kEventType_ButtonRight:
        if (AppEvent::kAppEventButtonType_Clicked == aEvent->ButtonEvent.Type)
        {
            if (BoltLockMgr().IsUnlocked())
            {
                BoltLockMgr().InitiateAction(BoltLockManager::ACTOR_APP, BoltLockManager::LOCK_ACTION);
            }
        }
        else if (AppEvent::kAppEventButtonType_LongClicked == aEvent->ButtonEvent.Type)
        {
            // Enable BLE advertisements
            if (!ConnectivityMgr().IsBLEAdvertisingEnabled())
            {
                if (Server::GetInstance().GetCommissioningWindowManager().OpenBasicCommissioningWindow() == CHIP_NO_ERROR)
                {
                    PLAT_LOG("Enabled BLE Advertisement");
                    LED_startBlinking(sAppGreenHandle, 500, LED_BLINK_FOREVER);                 
    	            chip::DeviceLayer::SystemLayer().StartTimer(chip::System::Clock::Seconds16(1), CheckCommissioningStatus, reinterpret_cast<intptr_t>(nullptr));	
                }
                else
                {
                    PLAT_LOG("OpenBasicCommissioningWindow() failed");
                }
            }
        }
        break;

    case AppEvent::kEventType_AppEvent:
        if (NULL != aEvent->Handler)
        {
            aEvent->Handler(aEvent);
        }
        break;

    case AppEvent::kEventType_None:
    default:
        break;
    }
}

void AppTask::RoutineLevelAdjust(chip::System::Layer * systemLayer, void * appState)
{
    // toby: note: this is optimized for quick code-writing.
    // if it is useful in the future, will consider re-factoring it for better coding convention.

    static bool isActive = false;

    if (isActive) return;
    
    // check that unit is "off". if "off", exit.
    uint8_t isOn;
    EmberAfStatus status = emberAfReadAttribute(1, ZCL_ON_OFF_CLUSTER_ID, ZCL_ON_OFF_ATTRIBUTE_ID, CLUSTER_MASK_SERVER,
                                                 (uint8_t *) &isOn, 1, NULL);
    if (status != EMBER_ZCL_STATUS_SUCCESS)
    {
        PLAT_LOG("ERR: reading on/off %x", status);
        return;
    }
    if (!isOn)
    {
        isActive = false;
	return;
    }
    
    // read Level Control value
    uint8_t curLevel;
    status = emberAfReadAttribute(1, ZCL_LEVEL_CONTROL_CLUSTER_ID, ZCL_CURRENT_LEVEL_ATTRIBUTE_ID, CLUSTER_MASK_SERVER,
                                                 (uint8_t *) &curLevel, 1, NULL);
    if (status != EMBER_ZCL_STATUS_SUCCESS)
    {
        PLAT_LOG("ERR: reading level control %x", status);
        return;
    }
    // if Level is not already maxed out
    if (curLevel < 232)
    {
        // adjust Level Control
        // note: writing the attribute will trigger the report
        uint8_t newLevel = curLevel + 2;
        status = emberAfWriteAttribute(1, ZCL_LEVEL_CONTROL_CLUSTER_ID, ZCL_CURRENT_LEVEL_ATTRIBUTE_ID, CLUSTER_MASK_SERVER,
                                 (uint8_t *) &newLevel, ZCL_INT8U_ATTRIBUTE_TYPE);
        if (status != EMBER_ZCL_STATUS_SUCCESS)
        {
            PLAT_LOG("ERR: updating Level Control %x", status);
            return;
        }
        
        // start countdown for next call of this function
        chip::DeviceLayer::SystemLayer().StartTimer(chip::System::Clock::Seconds16(2), RoutineLevelAdjust, reinterpret_cast<intptr_t>(nullptr));	
    }
    else
    {
        isActive = false;
        LED_stopBlinking(sAppRedHandle);
        LED_setOn(sAppRedHandle, 50);
    }
    
    
}

void AppTask::UpdateClusterState(intptr_t context)
{
    PLAT_LOG("AppTask::UpdateClusterState");
    
    // write the new on/off value
    EmberAfStatus status = emberAfWriteAttribute(1, ZCL_ON_OFF_CLUSTER_ID, ZCL_ON_OFF_ATTRIBUTE_ID, CLUSTER_MASK_SERVER,
                                                 (uint8_t *) &sEvChargerData.isCarConnected, ZCL_BOOLEAN_ATTRIBUTE_TYPE);
    if (status != EMBER_ZCL_STATUS_SUCCESS)
    {
        PLAT_LOG("ERR: updating on/off %x", status);
	    return;
    }
    
    status = emberAfWriteAttribute(1, ZCL_LEVEL_CONTROL_CLUSTER_ID, ZCL_CURRENT_LEVEL_ATTRIBUTE_ID, CLUSTER_MASK_SERVER,
                                                 (uint8_t *) &sEvChargerData.chargeLevel, ZCL_INT8U_ATTRIBUTE_TYPE);
    if (status != EMBER_ZCL_STATUS_SUCCESS)
    {
        PLAT_LOG("ERR: updating Level Control %x", status);
        return;
    }

    /*
    uint8_t newValue = !BoltLockMgr().IsUnlocked();

    // write the new on/off value
    EmberAfStatus status = emberAfWriteAttribute(1, ZCL_ON_OFF_CLUSTER_ID, ZCL_ON_OFF_ATTRIBUTE_ID, CLUSTER_MASK_SERVER,
                                                 (uint8_t *) &newValue, ZCL_BOOLEAN_ATTRIBUTE_TYPE);
    if (status != EMBER_ZCL_STATUS_SUCCESS)
    {
        PLAT_LOG("ERR: updating on/off %x", status);
	    return;
    }
    
    uint8_t newLevel = 64; // ~25%
    status = emberAfWriteAttribute(1, ZCL_LEVEL_CONTROL_CLUSTER_ID, ZCL_CURRENT_LEVEL_ATTRIBUTE_ID, CLUSTER_MASK_SERVER,
                                                 (uint8_t *) &newLevel, ZCL_INT8U_ATTRIBUTE_TYPE);
    if (status != EMBER_ZCL_STATUS_SUCCESS)
    {
        PLAT_LOG("ERR: updating Level Control %x", status);
        return;
    }
    
    
    if (newValue)
    {
        chip::DeviceLayer::SystemLayer().StartTimer(chip::System::Clock::Seconds16(2), RoutineLevelAdjust, reinterpret_cast<intptr_t>(nullptr));
    }
    */
    
}

void AppTask::CheckCommissioningStatus(chip::System::Layer * systemLayer, void * appState)
{
    // toby: after BLE adv starts for commissioning, this function periodically checks the commissioning status (BLE, Thread, etc)
    // and updates the associated LED status accordingly
    
    // assume this usecase always:
    // button starts BLE adv
    // before BLE stops adv, commissioner will initiate the connection
    // now, device is waiting for Thread commissioning
    // once Thread commissioning is done, keep LED on

    if (ConnectivityMgr().IsBLEAdvertising())
    {
        // blink LED at 500 msec interval
        LED_startBlinking(sAppGreenHandle, 250, LED_BLINK_FOREVER);
    }
    else if (!(ConnectivityMgr().IsThreadAttached() && ConnectivityMgr().IsThreadProvisioned()))
    {
        LED_startBlinking(sAppGreenHandle, 500, LED_BLINK_FOREVER);
    }
    else
    {
        LED_stopBlinking(sAppGreenHandle);
        LED_setOff(sAppGreenHandle);
        return;
    }

   	// start countdown for next checkup
	chip::DeviceLayer::SystemLayer().StartTimer(chip::System::Clock::Seconds16(2), CheckCommissioningStatus, reinterpret_cast<intptr_t>(nullptr));	

}

void AppTask::CarConnectedHandler(AppEvent * appEvent)
{
    BoltLockMgr().InitiateAction(BoltLockManager::ACTOR_APP, BoltLockManager::LOCK_ACTION);
}
void AppTask::CarDisconnectedHandler(AppEvent * appEvent)
{
    BoltLockMgr().InitiateAction(BoltLockManager::ACTOR_APP, BoltLockManager::UNLOCK_ACTION);
}
void AppTask::NewChargeData(AppEvent * appEvent)
{    
    chip::DeviceLayer::PlatformMgr().ScheduleWork(UpdateClusterState, reinterpret_cast<intptr_t>(nullptr));
}
