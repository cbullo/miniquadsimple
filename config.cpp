#include "config.h"

#include "EEPROM.h"

Config config;

#define CONFIG_MAGIC 0xDEADBEEF
#define CONFIG_VERSION 1

void ReadConfig() {
  Config temp_config;
  EEPROM.get(0, temp_config);

  if (temp_config.magic == CONFIG_MAGIC &&
      temp_config.version == CONFIG_VERSION) {
    config = temp_config;
  }
}

void StoreConfig() {
  config.magic = CONFIG_MAGIC;
  config.version = CONFIG_VERSION;
  EEPROM.put(0, config);
}

