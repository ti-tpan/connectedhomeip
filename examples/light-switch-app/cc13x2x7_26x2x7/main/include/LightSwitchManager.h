/*
 *
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

#ifndef LIGHT_SWITCH_MANAGER_H
#define LIGHT_SWITCH_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#include "AppEvent.h"

#include <FreeRTOS.h>
#include <timers.h>

#include <app/util/basic-types.h>
#include <lib/core/CHIPError.h>

class LightSwitchManager;
//static LightSwitchManager & GetInstance();

class LightSwitchManager
{
public:
    enum Action
    {
	OFF_ACTION=0,
	ON_ACTION,
        TOGGLE_ACTION,

        INVALID_ACTION
    } Action_e;

    void Init(chip::EndpointId aLightSwitchEndpoint);
    void InitiateActionSwitch(Action aAction);
    void DimmerChangeBrightness();
    chip::EndpointId GetLightSwitchEndpointId() { return mLightSwitchEndpoint; }

    static LightSwitchManager & GetInstance()
    {
        static LightSwitchManager sLightSwitch;
        return sLightSwitch;
    }

    /*
    static LightSwitchManager & GetInstance();
    friend LightSwitchManager & GetInstance();
    */

    /*
    friend LightSwitchManager & GetInstance()
    {
        return LightSwitchManager::sLightSwitch;
    }
    */

private:
    constexpr static auto kOnePercentBrightnessApproximation = 3;
    constexpr static auto kMaximumBrightness                 = 254;

    chip::EndpointId mLightSwitchEndpoint;

    // friend LightSwitchManager & LightSwitchMgr(void);

    static LightSwitchManager sLightSwitch;
};

/*
inline LightSwitchManager & GetInstance()
{
    return LightSwitchManager::sLightSwitch;
}
*/


#endif // LOCK_MANAGER_H

