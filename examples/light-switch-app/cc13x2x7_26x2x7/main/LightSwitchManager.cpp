/*
 *
 *    Copyright (c) 2020 Project CHIP Authors
 *    Copyright (c) 2019 Google LLC.
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

#include "LightSwitchManager.h"
#include "AppEvent.h"
#include "BindingHandler.h"

#include <app/server/Server.h>
#include <app/util/binding-table.h>
#include <controller/InvokeInteraction.h>

using namespace chip;
using namespace chip::app;

void LightSwitchManager::Init(chip::EndpointId aLightSwitchEndpoint)
{
    BindingHandler::GetInstance().Init();
    mLightSwitchEndpoint = aLightSwitchEndpoint;
}

void LightSwitchManager::InitiateActionSwitch(Action mAction)
{
    BindingHandler::BindingData * data = Platform::New<BindingHandler::BindingData>();
    if (data)
    {
        data->EndpointId = mLightSwitchEndpoint;
        data->ClusterId  = Clusters::OnOff::Id;
        switch (mAction)
        {
        case Action::OFF_ACTION:
            data->CommandId = Clusters::OnOff::Commands::Off::Id;
            break;
        case Action::ON_ACTION:
            data->CommandId = Clusters::OnOff::Commands::On::Id;
            break;
        case Action::TOGGLE_ACTION:
            data->CommandId = Clusters::OnOff::Commands::Toggle::Id;
            break;
        default:
            Platform::Delete(data);
            return;
        }
        data->IsGroup = BindingHandler::GetInstance().IsGroupBound();
        DeviceLayer::PlatformMgr().ScheduleWork(BindingHandler::SwitchWorkerHandler, reinterpret_cast<intptr_t>(data));
    }
}

void LightSwitchManager::DimmerChangeBrightness()
{
    static uint16_t sBrightness;
    BindingHandler::BindingData * data = Platform::New<BindingHandler::BindingData>();
    if (data)
    {
        data->EndpointId = mLightSwitchEndpoint;
        data->CommandId  = Clusters::LevelControl::Commands::MoveToLevel::Id;
        data->ClusterId  = Clusters::LevelControl::Id;
        // add to brightness 3 to approximate 1% step of brightness after each call dimmer change.
        sBrightness += kOnePercentBrightnessApproximation;
        if (sBrightness > kMaximumBrightness)
        {
            sBrightness = 0;
        }
        data->Value   = (uint8_t) sBrightness;
        data->IsGroup = BindingHandler::GetInstance().IsGroupBound();
        DeviceLayer::PlatformMgr().ScheduleWork(BindingHandler::SwitchWorkerHandler, reinterpret_cast<intptr_t>(data));
    }
}

