/*
# Copyright  2021 Andrea Scipone
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
*/

#include <Windows.h>
#include <stdlib.h>

#include <iostream>
#include <string>

#include <vesc_driver/vesc_device_uuid_lookup.hpp>

int main(int argc, char ** argv)
{
  std::string devicePort = (argc > 1 ? argv[1] : "COM3");
  std::string maxRetry_ = (argc > 2 ? argv[2] : "50");
  std::string VESC_UUID_ENV = "VESC_UUID_ENV=";

  vesc_driver::VescDeviceLookup lookup(devicePort);
  int maxRetry = stoi(maxRetry_);
  for (int i = 0; i < maxRetry; i++) {
    Sleep(20);
    if (lookup.isReady()) {break;}
  }

  if (lookup.isReady()) {
    VESC_UUID_ENV += lookup.deviceUUID();

    std::cout << lookup.deviceUUID() << std::endl;
    _putenv(VESC_UUID_ENV.c_str());
    return 0;
  } else {
    return -1;
  }
}
