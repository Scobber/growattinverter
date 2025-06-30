#include <ModbusMaster.h>
#include <ArduinoJson.h>
#include <Arduino.h>

#include "GrowattTypes.h"
#include "Growatt.h"
#include "Config.h"
#ifndef __CONFIG_H__
#error Please rename Config.h.example to Config.h
#endif
#include <time.h>

#if GROWATT_MODBUS_VERSION == 120
  #include "Growatt120.h"
#elif GROWATT_MODBUS_VERSION == 124
  #include "Growatt124.h"
#elif GROWATT_MODBUS_VERSION == 305
  #include "Growatt305.h"
#else
  #error "Unsupported Growatt Modbus version"
#endif

ModbusMaster Modbus;

// Constructor
Growatt::Growatt() {
  _eDevice = Undef_stick;
  _PacketCnt = 0;
  _prevTotalEnergy = 0;
  _prevEnergyValid = false;
  _accEnergyL1 = 0;
  _accEnergyL2 = 0;
  _accEnergyL3 = 0;
}

void Growatt::InitProtocol() {
  /**
   * @brief Initialize the protocol struct
   * @param version The version of the modbus protocol to use
   */
  #if GROWATT_MODBUS_VERSION == 120
    init_growatt120(_Protocol); 
  #elif GROWATT_MODBUS_VERSION == 124
    init_growatt124(_Protocol);
  #elif GROWATT_MODBUS_VERSION == 305
    init_growatt305(_Protocol);
  #else
    #error "Unsupported Growatt Modbus version"
  #endif
}

void Growatt::begin(Stream &serial) {
  /**
   * @brief Set up communication with the inverter
   * @param serial The serial interface
   */
  uint8_t res;

  #if SIMULATE_INVERTER == 1
    _eDevice = SIMULATE_DEVICE;
  #else
    // init communication with the inverter
    Serial.begin(9600);
    Modbus.begin(1, serial);
    res = Modbus.readInputRegisters(0, 1);
    if(res == Modbus.ku8MBSuccess) {
      _eDevice = ShineWiFi_S; // Serial
    } else {
      delay(1000);
      Serial.begin(115200);
      Modbus.begin(1, serial);
      res = Modbus.readInputRegisters(0, 1);
      if(res == Modbus.ku8MBSuccess) {
        _eDevice = ShineWiFi_X; // USB
      }
      delay(1000);
    }
  #endif
}

eDevice_t Growatt::GetWiFiStickType() {
  /**
   * @brief After initialisation the type of the wifi stick is known
   * @returns eDevice_t type of the wifi stick
   */

  return _eDevice;
}

bool Growatt::ReadInputRegisters() {
  /**
   * @brief Read the input registers from the inverter
   * @returns true if data was read successfully, false otherwise
   */
  uint16_t registerAddress;
  uint8_t res;

  // read each fragment separately
  for (int i = 0; i < _Protocol.InputFragmentCount; i++) {
    res = Modbus.readInputRegisters(
      _Protocol.InputReadFragments[i].StartAddress,
      _Protocol.InputReadFragments[i].FragmentSize
    );
    if(res == Modbus.ku8MBSuccess) {
      for (int j = 0; j < _Protocol.InputRegisterCount; j++) {
        // make sure the register we try to read is in the fragment
        if (_Protocol.InputRegisters[j].address >= _Protocol.InputReadFragments[i].StartAddress) {
          // when we exceed the fragment size, skip to new fragment
          if (_Protocol.InputRegisters[j].address >= _Protocol.InputReadFragments[i].StartAddress + _Protocol.InputReadFragments[i].FragmentSize)
            break;
          // let's say the register address is 1013 and read window is 1000-1050
          // that means the response in the buffer is on position 1013 - 1000 = 13
          registerAddress = _Protocol.InputRegisters[j].address - _Protocol.InputReadFragments[i].StartAddress;
          if (_Protocol.InputRegisters[j].size == SIZE_16BIT) {
            _Protocol.InputRegisters[j].value = Modbus.getResponseBuffer(registerAddress);
          } else {
            _Protocol.InputRegisters[j].value = (Modbus.getResponseBuffer(registerAddress) << 16) + Modbus.getResponseBuffer(registerAddress + 1);
          }
        }
      }
    } else {
      return false;
    }
  }
  return true;
}

bool Growatt::ReadHoldingRegisters() {
  /**
   * @brief Read the holding registers from the inverter
   * @returns true if data was read successfully, false otherwise
   */
  uint16_t registerAddress;
  uint8_t res;

  // read each fragment separately
  for (int i = 0; i < _Protocol.HoldingFragmentCount; i++) {
    res = Modbus.readHoldingRegisters(
      _Protocol.HoldingReadFragments[i].StartAddress,
      _Protocol.HoldingReadFragments[i].FragmentSize
    );
    if(res == Modbus.ku8MBSuccess) {
      for (int j = 0; j < _Protocol.HoldingRegisterCount; j++) {
        if (_Protocol.HoldingRegisters[j].address >= _Protocol.HoldingReadFragments[i].StartAddress) {
          if (_Protocol.HoldingRegisters[j].address >= _Protocol.HoldingReadFragments[i].StartAddress + _Protocol.HoldingReadFragments[i].FragmentSize)
            break;
          registerAddress = _Protocol.HoldingRegisters[j].address - _Protocol.HoldingReadFragments[i].StartAddress;
          if (_Protocol.HoldingRegisters[j].size == SIZE_16BIT) {
            _Protocol.HoldingRegisters[j].value = Modbus.getResponseBuffer(registerAddress);
          } else {
            _Protocol.HoldingRegisters[j].value = (Modbus.getResponseBuffer(registerAddress) << 16) + Modbus.getResponseBuffer(registerAddress + 1);
          }
        }
      }
    } else {
      return false;
    }
  }
  return true;
}

bool Growatt::ReadData() {
  /**
   * @brief Reads the data from the inverter and updates the internal data structures
   * @returns true if data was read successfully, false otherwise
   */

  _PacketCnt++;
  _GotData = ReadInputRegisters() && ReadHoldingRegisters();
  if (_GotData) {
    _UpdateEnergyAccumulation();
  }
  return _GotData;
}

sGrowattModbusReg_t Growatt::GetInputRegister(uint16_t reg) {
  /**
   * @brief get the internal representation of the input register
   * @param reg the register to get
   * @returns the register value
   */
    if (_GotData == false) {
      ReadData();
    }
    return _Protocol.InputRegisters[reg];
}

sGrowattModbusReg_t Growatt::GetHoldingRegister(uint16_t reg) {
  /**
   * @brief get the internal representation of the holding register
   * @param reg the register to get
   * @returns the register value
   */
    if (_GotData == false) {
      ReadData();
    }
    return _Protocol.HoldingRegisters[reg];
}

bool Growatt::ReadHoldingReg(uint16_t adr, uint16_t* result) {
  /**
   * @brief read 16b holding register
   * @param adr address of the register
   * @param result pointer to the result
   * @returns true if successful
   */
    uint8_t res = Modbus.readHoldingRegisters(adr, 1);
    if (res == Modbus.ku8MBSuccess) {
        *result = Modbus.getResponseBuffer(0);
        return true;
    }
    return false;
}

bool Growatt::ReadHoldingReg(uint16_t adr, uint32_t* result) {
  /**
   * @brief read 32b holding register
   * @param adr address of the register
   * @param result pointer to the result
   * @returns true if successful
   */
    uint8_t res = Modbus.readHoldingRegisters(adr, 2);
    if (res == Modbus.ku8MBSuccess) {
        *result = (Modbus.getResponseBuffer(0) << 16) + Modbus.getResponseBuffer(1);
        return true;
    }
    return false;
}

bool Growatt::WriteHoldingReg(uint16_t adr, uint16_t value) {
  /**
   * @brief write 16b holding register
   * @param adr address of the register
   * @param value value to write to the register
   * @returns true if successful
   */
    uint8_t res = Modbus.writeSingleRegister(adr, value);
    if (res == Modbus.ku8MBSuccess) {
        return true;
    }
    return false;
}


bool Growatt::ReadInputReg(uint16_t adr, uint16_t* result) {
  /**
   * @brief read 16b input register
   * @param adr address of the register
   * @param result pointer to the result
   * @returns true if successful
   */
    uint8_t res = Modbus.readInputRegisters(adr, 1);
    if (res == Modbus.ku8MBSuccess) {
        *result = Modbus.getResponseBuffer(0);
        return true;
    }
    return false;
}

bool Growatt::ReadInputReg(uint16_t adr, uint32_t* result) {
  /**
   * @brief read 32b input register
   * @param adr address of the register
   * @param result pointer to the result
   * @returns true if successful
   */
    uint8_t res = Modbus.readInputRegisters(adr, 2);
    if (res == Modbus.ku8MBSuccess) {
        *result = (Modbus.getResponseBuffer(0) << 16) + Modbus.getResponseBuffer(1);
        return true;
    }
    return false;
}

double Growatt::_round2(double value) {
   return (int)(value * 100 + 0.5) / 100.0;
}

void Growatt::_UpdateEnergyAccumulation() {
#if GROWATT_MODBUS_VERSION == 305
  double totE = _Protocol.InputRegisters[P305_ENERGY_TOTAL].value *
                _Protocol.InputRegisters[P305_ENERGY_TOTAL].multiplier * 1000.0;
  double pac_l1 = _Protocol.InputRegisters[P305_AC_POWER].value *
                  _Protocol.InputRegisters[P305_AC_POWER].multiplier;
  double pac_l2 = 0;
  double pac_l3 = 0;
#elif GROWATT_MODBUS_VERSION == 120
  double totE = _Protocol.InputRegisters[P120_ENERGY_TOTAL].value *
                _Protocol.InputRegisters[P120_ENERGY_TOTAL].multiplier * 1000.0;
  double pac_l1 = _Protocol.InputRegisters[P120_GRID_L1_OUTPUT_POWER].value *
                  _Protocol.InputRegisters[P120_GRID_L1_OUTPUT_POWER].multiplier;
  double pac_l2 = _Protocol.InputRegisters[P120_GRID_L2_OUTPUT_POWER].value *
                  _Protocol.InputRegisters[P120_GRID_L2_OUTPUT_POWER].multiplier;
  double pac_l3 = _Protocol.InputRegisters[P120_GRID_L3_OUTPUT_POWER].value *
                  _Protocol.InputRegisters[P120_GRID_L3_OUTPUT_POWER].multiplier;
#elif GROWATT_MODBUS_VERSION == 124
  double totE = _Protocol.InputRegisters[P124_EAC_TOTAL].value *
                _Protocol.InputRegisters[P124_EAC_TOTAL].multiplier * 1000.0;
  double pac_l1 = _Protocol.InputRegisters[P124_PAC1].value *
                  _Protocol.InputRegisters[P124_PAC1].multiplier;
  double pac_l2 = _Protocol.InputRegisters[P124_PAC2].value *
                  _Protocol.InputRegisters[P124_PAC2].multiplier;
  double pac_l3 = _Protocol.InputRegisters[P124_PAC3].value *
                  _Protocol.InputRegisters[P124_PAC3].multiplier;
#else
  double totE = 0;
  double pac_l1 = 0, pac_l2 = 0, pac_l3 = 0;
#endif

  double sumPacInstant = pac_l1 + pac_l2 + pac_l3;
  if (_prevEnergyValid) {
    double deltaE = totE - _prevTotalEnergy;
    if (deltaE < 0)
      deltaE = 0;
    if (sumPacInstant > 0) {
      _accEnergyL1 += deltaE * pac_l1 / sumPacInstant;
      _accEnergyL2 += deltaE * pac_l2 / sumPacInstant;
      _accEnergyL3 += deltaE * pac_l3 / sumPacInstant;
    }
  }
  _prevTotalEnergy = totE;
  _prevEnergyValid = true;
}

uint8_t Growatt::MapStatusToFronius(uint32_t status) {
  switch (status) {
    case GwStatusWaiting:
      return 1;  // Waiting
    case GwStatusNormal:
      return 7;  // Running
    case GwStatusFault:
      return 16; // Fault
    default:
      return 0;  // Unknown
  }
}

void Growatt::CreateJson(char *Buffer, const char *MacAddress) {
  StaticJsonDocument<2048> doc;

#if SIMULATE_INVERTER != 1
  for (int i = 0; i < _Protocol.InputRegisterCount; i++) {
    if (_Protocol.InputRegisters[i].multiplier  == (int)_Protocol.InputRegisters[i].multiplier) {
      doc[_Protocol.InputRegisters[i].name] = _Protocol.InputRegisters[i].value * _Protocol.InputRegisters[i].multiplier;
    } else {
      doc[_Protocol.InputRegisters[i].name] = _round2(_Protocol.InputRegisters[i].value * _Protocol.InputRegisters[i].multiplier);
    }
  }
  for (int i = 0; i < _Protocol.HoldingRegisterCount; i++) {
    if (_Protocol.HoldingRegisters[i].multiplier  == (int)_Protocol.HoldingRegisters[i].multiplier) {
      doc[_Protocol.HoldingRegisters[i].name] = _Protocol.HoldingRegisters[i].value * _Protocol.HoldingRegisters[i].multiplier;
    } else {
      doc[_Protocol.HoldingRegisters[i].name] = _round2(_Protocol.HoldingRegisters[i].value * _Protocol.HoldingRegisters[i].multiplier);
    }
  }
#else
  #warning simulating the inverter
  doc["Status"] = 1;
  doc["DcPower"] = 230;
  doc["DcVoltage"] = 70.5;
  doc["DcInputCurrent"] = 8.5;
  doc["AcFreq"] = 50.00;
  doc["AcVoltage"] = 230.0;
  doc["AcPower"] = 0.00;
  doc["EnergyToday"] = 0.3;
  doc["EnergyTotal"] = 49.1;
  doc["OperatingTime"] = 123456;
  doc["Temperature"] = 21.12;
  doc["AccumulatedEnergy"] = 320;
  doc["EnergyToday"] = 0.3;
  doc["EnergyToday"] = 0.3;
#endif // SIMULATE_INVERTER
  doc["Mac"] = MacAddress;
  doc["Cnt"] = _PacketCnt;
  serializeJson(doc, Buffer, MQTT_MAX_PACKET_SIZE);
}

void Growatt::CreateDeviceInfoJson(char *Buffer) {
  StaticJsonDocument<512> doc;

  JsonObject head = doc.createNestedObject("Head");
  head.createNestedObject("RequestArguments");
  JsonObject status = head.createNestedObject("Status");
  status["Code"] = 0;
  time_t now = time(nullptr);
  struct tm *tm_info = localtime(&now);
  char ts[30];
  snprintf(ts, sizeof(ts), "%04d-%02d-%02dT%02d:%02d:%02d+00:00",
           tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
           tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
  head["Timestamp"] = ts;

  JsonObject body = doc.createNestedObject("Body");
  JsonObject data = body.createNestedObject("Data");

#if GROWATT_MODBUS_VERSION == 305
  uint32_t gwStatus = _Protocol.InputRegisters[P305_I_STATUS].value;
#elif GROWATT_MODBUS_VERSION == 120
  uint32_t gwStatus = _Protocol.InputRegisters[P120_I_STATUS].value;
#elif GROWATT_MODBUS_VERSION == 124
  uint32_t gwStatus = _Protocol.InputRegisters[P124_I_STATUS].value;
#else
  uint32_t gwStatus = 0;
#endif
  uint8_t froniusStatus = MapStatusToFronius(gwStatus);
  data["DeviceType"] = FRONIUS_DEVICE_TYPE;
  data["Serial"] = FRONIUS_SERIAL;

  serializeJson(doc, Buffer, MQTT_MAX_PACKET_SIZE);
}

void Growatt::CreateUIJson(char *Buffer) {
  StaticJsonDocument<2048> doc;
  const char* unitStr[] = {"", "W", "kWh", "V", "A", "s", "%", "Hz", "C", "VA"};

#if SIMULATE_INVERTER != 1
  for (int i = 0; i < _Protocol.InputRegisterCount; i++) {
    if (_Protocol.InputRegisters[i].frontend == true || _Protocol.InputRegisters[i].plot == true) {
      JsonArray arr = doc.createNestedArray(_Protocol.InputRegisters[i].name);

       // value
      if (_Protocol.InputRegisters[i].multiplier  == (int)_Protocol.InputRegisters[i].multiplier) {
        arr.add(_Protocol.InputRegisters[i].value * _Protocol.InputRegisters[i].multiplier);
      } else {
        arr.add(_round2(_Protocol.InputRegisters[i].value * _Protocol.InputRegisters[i].multiplier));
      }
      arr.add(unitStr[_Protocol.InputRegisters[i].unit]); //unit
      arr.add(_Protocol.InputRegisters[i].plot); //should be plotted
    }
  }
  for (int i = 0; i < _Protocol.HoldingRegisterCount; i++) {
    if (_Protocol.HoldingRegisters[i].frontend == true || _Protocol.HoldingRegisters[i].plot == true) {
      JsonArray arr = doc.createNestedArray(_Protocol.HoldingRegisters[i].name);

      //value
      if (_Protocol.HoldingRegisters[i].multiplier  == (int)_Protocol.HoldingRegisters[i].multiplier) {
        arr.add(_Protocol.HoldingRegisters[i].value * _Protocol.HoldingRegisters[i].multiplier);
      } else {
        arr.add(_round2(_Protocol.HoldingRegisters[i].value * _Protocol.HoldingRegisters[i].multiplier));
      }
      arr.add(unitStr[_Protocol.HoldingRegisters[i].unit]);
      arr.add(_Protocol.HoldingRegisters[i].plot); //should be plotted
    }
  }

  // compute additional aggregated statistics
#if GROWATT_MODBUS_VERSION == 305
  double uac_l1 = _Protocol.InputRegisters[P305_AC_VOLTAGE].value *
                   _Protocol.InputRegisters[P305_AC_VOLTAGE].multiplier;
  double uac_l2 = 0, uac_l3 = 0;
  double iac_l1 = _Protocol.InputRegisters[P305_AC_OUTPUT_CURRENT].value *
                   _Protocol.InputRegisters[P305_AC_OUTPUT_CURRENT].multiplier;
  double iac_l2 = 0, iac_l3 = 0;
  double pac_l1 = _Protocol.InputRegisters[P305_AC_POWER].value *
                   _Protocol.InputRegisters[P305_AC_POWER].multiplier;
  double pac_l2 = 0, pac_l3 = 0;
  double dayE = _Protocol.InputRegisters[P305_ENERGY_TODAY].value *
                _Protocol.InputRegisters[P305_ENERGY_TODAY].multiplier * 1000.0;
#elif GROWATT_MODBUS_VERSION == 120
  double uac_l1 = _Protocol.InputRegisters[P120_GRID_L1_VOLTAGE].value *
                   _Protocol.InputRegisters[P120_GRID_L1_VOLTAGE].multiplier;
  double uac_l2 = _Protocol.InputRegisters[P120_GRID_L2_VOLTAGE].value *
                   _Protocol.InputRegisters[P120_GRID_L2_VOLTAGE].multiplier;
  double uac_l3 = _Protocol.InputRegisters[P120_GRID_L3_VOLTAGE].value *
                   _Protocol.InputRegisters[P120_GRID_L3_VOLTAGE].multiplier;
  double iac_l1 = _Protocol.InputRegisters[P120_GRID_L1_OUTPUT_CURRENT].value *
                   _Protocol.InputRegisters[P120_GRID_L1_OUTPUT_CURRENT].multiplier;
  double iac_l2 = _Protocol.InputRegisters[P120_GRID_L2_OUTPUT_CURRENT].value *
                   _Protocol.InputRegisters[P120_GRID_L2_OUTPUT_CURRENT].multiplier;
  double iac_l3 = _Protocol.InputRegisters[P120_GRID_L3_OUTPUT_CURRENT].value *
                   _Protocol.InputRegisters[P120_GRID_L3_OUTPUT_CURRENT].multiplier;
  double pac_l1 = _Protocol.InputRegisters[P120_GRID_L1_OUTPUT_POWER].value *
                   _Protocol.InputRegisters[P120_GRID_L1_OUTPUT_POWER].multiplier;
  double pac_l2 = _Protocol.InputRegisters[P120_GRID_L2_OUTPUT_POWER].value *
                   _Protocol.InputRegisters[P120_GRID_L2_OUTPUT_POWER].multiplier;
  double pac_l3 = _Protocol.InputRegisters[P120_GRID_L3_OUTPUT_POWER].value *
                   _Protocol.InputRegisters[P120_GRID_L3_OUTPUT_POWER].multiplier;
  double dayE = _Protocol.InputRegisters[P120_ENERGY_TODAY].value *
                _Protocol.InputRegisters[P120_ENERGY_TODAY].multiplier * 1000.0;
#elif GROWATT_MODBUS_VERSION == 124
  double uac_l1 = _Protocol.InputRegisters[P124_VAC1].value *
                   _Protocol.InputRegisters[P124_VAC1].multiplier;
  double uac_l2 = _Protocol.InputRegisters[P124_VAC2].value *
                   _Protocol.InputRegisters[P124_VAC2].multiplier;
  double uac_l3 = _Protocol.InputRegisters[P124_VAC3].value *
                   _Protocol.InputRegisters[P124_VAC3].multiplier;
  double iac_l1 = _Protocol.InputRegisters[P124_IAC1].value *
                   _Protocol.InputRegisters[P124_IAC1].multiplier;
  double iac_l2 = _Protocol.InputRegisters[P124_IAC2].value *
                   _Protocol.InputRegisters[P124_IAC2].multiplier;
  double iac_l3 = _Protocol.InputRegisters[P124_IAC3].value *
                   _Protocol.InputRegisters[P124_IAC3].multiplier;
  double pac_l1 = _Protocol.InputRegisters[P124_PAC1].value *
                   _Protocol.InputRegisters[P124_PAC1].multiplier;
  double pac_l2 = _Protocol.InputRegisters[P124_PAC2].value *
                   _Protocol.InputRegisters[P124_PAC2].multiplier;
  double pac_l3 = _Protocol.InputRegisters[P124_PAC3].value *
                   _Protocol.InputRegisters[P124_PAC3].multiplier;
  double dayE = _Protocol.InputRegisters[P124_EAC_TODAY].value *
                _Protocol.InputRegisters[P124_EAC_TODAY].multiplier * 1000.0;
#else
  double uac_l1 = 0, uac_l2 = 0, uac_l3 = 0;
  double iac_l1 = 0, iac_l2 = 0, iac_l3 = 0;
  double pac_l1 = 0, pac_l2 = 0, pac_l3 = 0;
  double dayE = 0;
#endif

  double uac_avg = (uac_l1 + uac_l2 + uac_l3) / 3.0;
  double line_avg = uac_avg * 1.7320508; // sqrt(3)
  double iac_avg = (iac_l1 + iac_l2 + iac_l3) / 3.0;
  double pac_sum = pac_l1 + pac_l2 + pac_l3;

  double sumPac = pac_sum;
  double dayE_l1 = 0, dayE_l2 = 0, dayE_l3 = 0;
  if (sumPac != 0) {
    dayE_l1 = dayE * pac_l1 / sumPac;
    dayE_l2 = dayE * pac_l2 / sumPac;
    dayE_l3 = dayE * pac_l3 / sumPac;
  }

  JsonArray arr = doc.createNestedArray("VoltageAvg");
  arr.add(_round2(uac_avg));
  arr.add("V");
  arr.add(false);

  arr = doc.createNestedArray("LineVoltageAvg");
  arr.add(_round2(line_avg));
  arr.add("V");
  arr.add(false);

  arr = doc.createNestedArray("CurrentAvg");
  arr.add(_round2(iac_avg));
  arr.add("A");
  arr.add(false);

  arr = doc.createNestedArray("PowerSum");
  arr.add(_round2(pac_sum));
  arr.add("W");
  arr.add(false);

  arr = doc.createNestedArray("DayEnergyL1");
  arr.add(_round2(dayE_l1 / 1000.0));
  arr.add("kWh");
  arr.add(false);
  arr = doc.createNestedArray("DayEnergyL2");
  arr.add(_round2(dayE_l2 / 1000.0));
  arr.add("kWh");
  arr.add(false);
  arr = doc.createNestedArray("DayEnergyL3");
  arr.add(_round2(dayE_l3 / 1000.0));
  arr.add("kWh");
  arr.add(false);

  arr = doc.createNestedArray("TotalEnergyL1");
  arr.add(_round2(_accEnergyL1 / 1000.0));
  arr.add("kWh");
  arr.add(false);
  arr = doc.createNestedArray("TotalEnergyL2");
  arr.add(_round2(_accEnergyL2 / 1000.0));
  arr.add("kWh");
  arr.add(false);
  arr = doc.createNestedArray("TotalEnergyL3");
  arr.add(_round2(_accEnergyL3 / 1000.0));
  arr.add("kWh");
  arr.add(false);
#else
  #warning simulating the inverter
  JsonArray arr = doc.createNestedArray("Status");
  arr.add(1);arr.add("");arr.add(false);
  arr = doc.createNestedArray("DcPower");
  arr.add(230);arr.add("W");arr.add(true);
  arr = doc.createNestedArray("DcVoltage");
  arr.add(70.5);arr.add("V");arr.add(false);
  arr = doc.createNestedArray("DcInputCurrent");
  arr.add(8.5);arr.add("A");arr.add(false);
  arr = doc.createNestedArray("AcFreq");
  arr.add(50);arr.add("Hz");arr.add(false);
  arr = doc.createNestedArray("AcVoltage");
  arr.add(230);arr.add("V");arr.add(false);
  arr = doc.createNestedArray("AcPower");
  arr.add(0.00);arr.add("W");arr.add(false);
  arr = doc.createNestedArray("EnergyToday");
  arr.add(0.3);arr.add("kWh");arr.add(false);
  arr = doc.createNestedArray("EnergyTotal");
  arr.add(49.1);arr.add("kWh");arr.add(false);
  arr = doc.createNestedArray("OperatingTime");
  arr.add(123456);arr.add("s");arr.add(false);
  arr = doc.createNestedArray("Temperature");
  arr.add(21.12);arr.add("C");arr.add(false);
  arr = doc.createNestedArray("AccumulatedEnergy");
  arr.add(320);arr.add("kWh");arr.add(false);
  arr = doc.createNestedArray("EnergyToday");
  arr.add(0.3);arr.add("kWh");arr.add(false);
#endif // SIMULATE_INVERTER

  serializeJson(doc, Buffer, 4096);
}

void Growatt::CreateFroniusJson(char *Buffer) {
  StaticJsonDocument<2048> doc;

  JsonObject head = doc.createNestedObject("Head");
  JsonObject req = head.createNestedObject("RequestArguments");
  req["DeviceId"] = 1;
  req["Scope"] = "Device";
  req["DataCollection"] = "CommonInverterData";
  JsonObject status = head.createNestedObject("Status");
  status["Code"] = 0;
  status["Reason"] = "";
  status["UserMessage"] = "";
  time_t now = time(nullptr);
  struct tm *tm_info = localtime(&now);
  char ts[30];
  snprintf(ts, sizeof(ts), "%04d-%02d-%02dT%02d:%02d:%02d+00:00",
           tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
           tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
  head["Timestamp"] = ts;

  JsonObject body = doc.createNestedObject("Body");
  JsonObject data = body.createNestedObject("Data");

#if GROWATT_MODBUS_VERSION == 305
  double pac  = _Protocol.InputRegisters[P305_AC_POWER].value * _Protocol.InputRegisters[P305_AC_POWER].multiplier;
  double fac  = _Protocol.InputRegisters[P305_AC_FREQUENCY].value * _Protocol.InputRegisters[P305_AC_FREQUENCY].multiplier;
  double uac  = _Protocol.InputRegisters[P305_AC_VOLTAGE].value * _Protocol.InputRegisters[P305_AC_VOLTAGE].multiplier;
  double iac  = _Protocol.InputRegisters[P305_AC_OUTPUT_CURRENT].value * _Protocol.InputRegisters[P305_AC_OUTPUT_CURRENT].multiplier;
  double pdc  = _Protocol.InputRegisters[P305_DC_POWER].value * _Protocol.InputRegisters[P305_DC_POWER].multiplier;
  double udc  = _Protocol.InputRegisters[P305_DC_VOLTAGE].value * _Protocol.InputRegisters[P305_DC_VOLTAGE].multiplier;
  double idc  = _Protocol.InputRegisters[P305_DC_INPUT_CURRENT].value * _Protocol.InputRegisters[P305_DC_INPUT_CURRENT].multiplier;
  double dayE = _Protocol.InputRegisters[P305_ENERGY_TODAY].value * _Protocol.InputRegisters[P305_ENERGY_TODAY].multiplier * 1000.0;
  double totE = _Protocol.InputRegisters[P305_ENERGY_TOTAL].value * _Protocol.InputRegisters[P305_ENERGY_TOTAL].multiplier * 1000.0;
  double uac_l1 = uac, uac_l2 = 0, uac_l3 = 0;
  double iac_l1 = iac, iac_l2 = 0, iac_l3 = 0;
  double pac_l1 = pac, pac_l2 = 0, pac_l3 = 0;
#elif GROWATT_MODBUS_VERSION == 120
  double pac  = _Protocol.InputRegisters[P120_OUTPUT_POWER].value * _Protocol.InputRegisters[P120_OUTPUT_POWER].multiplier;
  double fac  = _Protocol.InputRegisters[P120_GRID_FREQUENCY].value * _Protocol.InputRegisters[P120_GRID_FREQUENCY].multiplier;
  double uac  = _Protocol.InputRegisters[P120_GRID_L1_VOLTAGE].value * _Protocol.InputRegisters[P120_GRID_L1_VOLTAGE].multiplier;
  double iac  = _Protocol.InputRegisters[P120_GRID_L1_OUTPUT_CURRENT].value * _Protocol.InputRegisters[P120_GRID_L1_OUTPUT_CURRENT].multiplier;
  double pdc  = _Protocol.InputRegisters[P120_INPUT_POWER].value * _Protocol.InputRegisters[P120_INPUT_POWER].multiplier;
  double udc  = _Protocol.InputRegisters[P120_PV1_VOLTAGE].value * _Protocol.InputRegisters[P120_PV1_VOLTAGE].multiplier;
  double idc  = (_Protocol.InputRegisters[P120_PV1_INPUT_CURRENT].value * _Protocol.InputRegisters[P120_PV1_INPUT_CURRENT].multiplier) +
                 (_Protocol.InputRegisters[P120_PV2_INPUT_CURRENT].value * _Protocol.InputRegisters[P120_PV2_INPUT_CURRENT].multiplier);
  double dayE = _Protocol.InputRegisters[P120_ENERGY_TODAY].value * _Protocol.InputRegisters[P120_ENERGY_TODAY].multiplier * 1000.0;
  double totE = _Protocol.InputRegisters[P120_ENERGY_TOTAL].value * _Protocol.InputRegisters[P120_ENERGY_TOTAL].multiplier * 1000.0;
  double uac_l1 = _Protocol.InputRegisters[P120_GRID_L1_VOLTAGE].value * _Protocol.InputRegisters[P120_GRID_L1_VOLTAGE].multiplier;
  double uac_l2 = _Protocol.InputRegisters[P120_GRID_L2_VOLTAGE].value * _Protocol.InputRegisters[P120_GRID_L2_VOLTAGE].multiplier;
  double uac_l3 = _Protocol.InputRegisters[P120_GRID_L3_VOLTAGE].value * _Protocol.InputRegisters[P120_GRID_L3_VOLTAGE].multiplier;
  double iac_l1 = _Protocol.InputRegisters[P120_GRID_L1_OUTPUT_CURRENT].value * _Protocol.InputRegisters[P120_GRID_L1_OUTPUT_CURRENT].multiplier;
  double iac_l2 = _Protocol.InputRegisters[P120_GRID_L2_OUTPUT_CURRENT].value * _Protocol.InputRegisters[P120_GRID_L2_OUTPUT_CURRENT].multiplier;
  double iac_l3 = _Protocol.InputRegisters[P120_GRID_L3_OUTPUT_CURRENT].value * _Protocol.InputRegisters[P120_GRID_L3_OUTPUT_CURRENT].multiplier;
  double pac_l1 = _Protocol.InputRegisters[P120_GRID_L1_OUTPUT_POWER].value * _Protocol.InputRegisters[P120_GRID_L1_OUTPUT_POWER].multiplier;
  double pac_l2 = _Protocol.InputRegisters[P120_GRID_L2_OUTPUT_POWER].value * _Protocol.InputRegisters[P120_GRID_L2_OUTPUT_POWER].multiplier;
  double pac_l3 = _Protocol.InputRegisters[P120_GRID_L3_OUTPUT_POWER].value * _Protocol.InputRegisters[P120_GRID_L3_OUTPUT_POWER].multiplier;
#elif GROWATT_MODBUS_VERSION == 124
  double pac  = _Protocol.InputRegisters[P124_PAC].value * _Protocol.InputRegisters[P124_PAC].multiplier;
  double fac  = _Protocol.InputRegisters[P124_FAC].value * _Protocol.InputRegisters[P124_FAC].multiplier;
  double uac  = _Protocol.InputRegisters[P124_VAC1].value * _Protocol.InputRegisters[P124_VAC1].multiplier;
  double iac  = _Protocol.InputRegisters[P124_IAC1].value * _Protocol.InputRegisters[P124_IAC1].multiplier;
  double pdc  = _Protocol.InputRegisters[P124_INPUT_POWER].value * _Protocol.InputRegisters[P124_INPUT_POWER].multiplier;
  double udc  = _Protocol.InputRegisters[P124_PV1_VOLTAGE].value * _Protocol.InputRegisters[P124_PV1_VOLTAGE].multiplier;
  double idc  = (_Protocol.InputRegisters[P124_PV1_CURRENT].value * _Protocol.InputRegisters[P124_PV1_CURRENT].multiplier) +
                 (_Protocol.InputRegisters[P124_PV2_CURRENT].value * _Protocol.InputRegisters[P124_PV2_CURRENT].multiplier);
  double dayE = _Protocol.InputRegisters[P124_EAC_TODAY].value * _Protocol.InputRegisters[P124_EAC_TODAY].multiplier * 1000.0;
  double totE = _Protocol.InputRegisters[P124_EAC_TOTAL].value * _Protocol.InputRegisters[P124_EAC_TOTAL].multiplier * 1000.0;
  double uac_l1 = _Protocol.InputRegisters[P124_VAC1].value * _Protocol.InputRegisters[P124_VAC1].multiplier;
  double uac_l2 = _Protocol.InputRegisters[P124_VAC2].value * _Protocol.InputRegisters[P124_VAC2].multiplier;
  double uac_l3 = _Protocol.InputRegisters[P124_VAC3].value * _Protocol.InputRegisters[P124_VAC3].multiplier;
  double iac_l1 = _Protocol.InputRegisters[P124_IAC1].value * _Protocol.InputRegisters[P124_IAC1].multiplier;
  double iac_l2 = _Protocol.InputRegisters[P124_IAC2].value * _Protocol.InputRegisters[P124_IAC2].multiplier;
  double iac_l3 = _Protocol.InputRegisters[P124_IAC3].value * _Protocol.InputRegisters[P124_IAC3].multiplier;
  double pac_l1 = _Protocol.InputRegisters[P124_PAC1].value * _Protocol.InputRegisters[P124_PAC1].multiplier;
  double pac_l2 = _Protocol.InputRegisters[P124_PAC2].value * _Protocol.InputRegisters[P124_PAC2].multiplier;
  double pac_l3 = _Protocol.InputRegisters[P124_PAC3].value * _Protocol.InputRegisters[P124_PAC3].multiplier;
#else
  double pac = 0, fac = 0, uac = 0, iac = 0, pdc = 0, udc = 0, idc = 0, dayE = 0, totE = 0;
  double uac_l1 = 0, uac_l2 = 0, uac_l3 = 0;
  double iac_l1 = 0, iac_l2 = 0, iac_l3 = 0;
  double pac_l1 = 0, pac_l2 = 0, pac_l3 = 0;
#endif

  // accumulate per phase energy based on total energy increments
  double sumPacInstant = pac_l1 + pac_l2 + pac_l3;
  if (_prevEnergyValid) {
    double deltaE = totE - _prevTotalEnergy;
    if (deltaE < 0) deltaE = 0;
    if (sumPacInstant > 0) {
      _accEnergyL1 += deltaE * pac_l1 / sumPacInstant;
      _accEnergyL2 += deltaE * pac_l2 / sumPacInstant;
      _accEnergyL3 += deltaE * pac_l3 / sumPacInstant;
    }
  }
  _prevTotalEnergy = totE;
  _prevEnergyValid = true;

  double dayE_l1 = 0, dayE_l2 = 0, dayE_l3 = 0;
  double totE_l1 = _accEnergyL1, totE_l2 = _accEnergyL2, totE_l3 = _accEnergyL3;
  double sumPac = pac_l1 + pac_l2 + pac_l3;
  if (sumPac != 0) {
    dayE_l1 = dayE * pac_l1 / sumPac;
    dayE_l2 = dayE * pac_l2 / sumPac;
    dayE_l3 = dayE * pac_l3 / sumPac;
  }

#if GROWATT_MODBUS_VERSION == 305
  uint32_t gwStatus = _Protocol.InputRegisters[P305_I_STATUS].value;
#elif GROWATT_MODBUS_VERSION == 120
  uint32_t gwStatus = _Protocol.InputRegisters[P120_I_STATUS].value;
#elif GROWATT_MODBUS_VERSION == 124
  uint32_t gwStatus = _Protocol.InputRegisters[P124_I_STATUS].value;
#else
  uint32_t gwStatus = 0;
#endif
  uint8_t froniusStatus = MapStatusToFronius(gwStatus);

  JsonObject pacObj = data.createNestedObject("PAC");
  pacObj["Value"] = pac;
  pacObj["Unit"] = "W";
  JsonObject pdcObj = data.createNestedObject("PDC");
  pdcObj["Value"] = pdc;
  pdcObj["Unit"] = "W";
  JsonObject facObj = data.createNestedObject("FAC");
  facObj["Value"] = fac;
  facObj["Unit"] = "Hz";
  JsonObject uacObj = data.createNestedObject("UAC");
  uacObj["Value"] = uac;
  uacObj["Unit"] = "V";
  JsonObject iacObj = data.createNestedObject("IAC");
  iacObj["Value"] = iac;
  iacObj["Unit"] = "A";
  JsonObject uacL1Obj = data.createNestedObject("UAC_L1");
  uacL1Obj["Value"] = uac_l1;
  uacL1Obj["Unit"] = "V";
  JsonObject uacL2Obj = data.createNestedObject("UAC_L2");
  uacL2Obj["Value"] = uac_l2;
  uacL2Obj["Unit"] = "V";
  JsonObject uacL3Obj = data.createNestedObject("UAC_L3");
  uacL3Obj["Value"] = uac_l3;
  uacL3Obj["Unit"] = "V";
  JsonObject iacL1Obj = data.createNestedObject("IAC_L1");
  iacL1Obj["Value"] = iac_l1;
  iacL1Obj["Unit"] = "A";
  JsonObject iacL2Obj = data.createNestedObject("IAC_L2");
  iacL2Obj["Value"] = iac_l2;
  iacL2Obj["Unit"] = "A";
  JsonObject iacL3Obj = data.createNestedObject("IAC_L3");
  iacL3Obj["Value"] = iac_l3;
  iacL3Obj["Unit"] = "A";
  JsonObject devStat = data.createNestedObject("DeviceStatus");
  devStat["ErrorCode"] = 0;
  devStat["StatusCode"] = froniusStatus;
  JsonObject pacL1Obj = data.createNestedObject("PAC_L1");
  pacL1Obj["Value"] = pac_l1;
  pacL1Obj["Unit"] = "W";
  JsonObject pacL2Obj = data.createNestedObject("PAC_L2");
  pacL2Obj["Value"] = pac_l2;
  pacL2Obj["Unit"] = "W";
  JsonObject pacL3Obj = data.createNestedObject("PAC_L3");
  pacL3Obj["Value"] = pac_l3;
  pacL3Obj["Unit"] = "W";
  JsonObject udcObj = data.createNestedObject("UDC");
  udcObj["Value"] = udc;
  udcObj["Unit"] = "V";
  JsonObject idcObj = data.createNestedObject("IDC");
  idcObj["Value"] = idc;
  idcObj["Unit"] = "A";
  JsonObject dayObj = data.createNestedObject("DAY_ENERGY");
  dayObj["Value"] = dayE;
  dayObj["Unit"] = "Wh";
  JsonObject dayL1Obj = data.createNestedObject("DAY_ENERGY_L1");
  dayL1Obj["Value"] = dayE_l1;
  dayL1Obj["Unit"] = "Wh";
  JsonObject dayL2Obj = data.createNestedObject("DAY_ENERGY_L2");
  dayL2Obj["Value"] = dayE_l2;
  dayL2Obj["Unit"] = "Wh";
  JsonObject dayL3Obj = data.createNestedObject("DAY_ENERGY_L3");
  dayL3Obj["Value"] = dayE_l3;
  dayL3Obj["Unit"] = "Wh";
  JsonObject totObj = data.createNestedObject("TOTAL_ENERGY");
  totObj["Value"] = totE;
  totObj["Unit"] = "Wh";
  JsonObject totL1Obj = data.createNestedObject("TOTAL_ENERGY_L1");
  totL1Obj["Value"] = totE_l1;
  totL1Obj["Unit"] = "Wh";
  JsonObject totL2Obj = data.createNestedObject("TOTAL_ENERGY_L2");
  totL2Obj["Value"] = totE_l2;
  totL2Obj["Unit"] = "Wh";
  JsonObject totL3Obj = data.createNestedObject("TOTAL_ENERGY_L3");
  totL3Obj["Value"] = totE_l3;
  totL3Obj["Unit"] = "Wh";

  serializeJson(doc, Buffer, MQTT_MAX_PACKET_SIZE);
}

void Growatt::CreatePowerFlowJson(char *Buffer) {
  StaticJsonDocument<2048> doc;

  JsonObject head = doc.createNestedObject("Head");
  head.createNestedObject("RequestArguments");
  JsonObject status = head.createNestedObject("Status");
  status["Code"] = 0;
  status["Reason"] = "";
  status["UserMessage"] = "";
  time_t now = time(nullptr);
  struct tm *tm_info = localtime(&now);
  char ts[30];
  snprintf(ts, sizeof(ts), "%04d-%02d-%02dT%02d:%02d:%02d+00:00",
           tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
           tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
  head["Timestamp"] = ts;

  JsonObject body = doc.createNestedObject("Body");
  JsonObject data = body.createNestedObject("Data");
  JsonObject site = data.createNestedObject("Site");

#if GROWATT_MODBUS_VERSION == 305
  double pac = _Protocol.InputRegisters[P305_AC_POWER].value * _Protocol.InputRegisters[P305_AC_POWER].multiplier;
  double pdc = _Protocol.InputRegisters[P305_DC_POWER].value * _Protocol.InputRegisters[P305_DC_POWER].multiplier;
  double dayE = _Protocol.InputRegisters[P305_ENERGY_TODAY].value * _Protocol.InputRegisters[P305_ENERGY_TODAY].multiplier * 1000.0;
  double totE = _Protocol.InputRegisters[P305_ENERGY_TOTAL].value * _Protocol.InputRegisters[P305_ENERGY_TOTAL].multiplier * 1000.0;
#elif GROWATT_MODBUS_VERSION == 120
  double pac = _Protocol.InputRegisters[P120_OUTPUT_POWER].value * _Protocol.InputRegisters[P120_OUTPUT_POWER].multiplier;
  double pdc = _Protocol.InputRegisters[P120_INPUT_POWER].value * _Protocol.InputRegisters[P120_INPUT_POWER].multiplier;
  double dayE = _Protocol.InputRegisters[P120_ENERGY_TODAY].value * _Protocol.InputRegisters[P120_ENERGY_TODAY].multiplier * 1000.0;
  double totE = _Protocol.InputRegisters[P120_ENERGY_TOTAL].value * _Protocol.InputRegisters[P120_ENERGY_TOTAL].multiplier * 1000.0;
#elif GROWATT_MODBUS_VERSION == 124
  double pac = _Protocol.InputRegisters[P124_PAC].value * _Protocol.InputRegisters[P124_PAC].multiplier;
  double pdc = _Protocol.InputRegisters[P124_INPUT_POWER].value * _Protocol.InputRegisters[P124_INPUT_POWER].multiplier;
  double dayE = _Protocol.InputRegisters[P124_EAC_TODAY].value * _Protocol.InputRegisters[P124_EAC_TODAY].multiplier * 1000.0;
  double totE = _Protocol.InputRegisters[P124_EAC_TOTAL].value * _Protocol.InputRegisters[P124_EAC_TOTAL].multiplier * 1000.0;
#else
  double pac = 0, pdc = 0, dayE = 0, totE = 0;
#endif

  site["P_PV"] = pdc;
  site["P_Load"] = pac;
  site["E_DAY"] = dayE;
  site["E_TOTAL"] = totE;

  JsonObject invs = data.createNestedObject("Inverters");
  JsonObject inv = invs.createNestedObject("1");
  inv["DT"] = FRONIUS_DEVICE_TYPE;
  inv["P"] = pac;

  serializeJson(doc, Buffer, MQTT_MAX_PACKET_SIZE);
}

void Growatt::CreateInverterInfoJson(char *Buffer) {
  StaticJsonDocument<512> doc;

  JsonObject head = doc.createNestedObject("Head");
  head.createNestedObject("RequestArguments");
  JsonObject status = head.createNestedObject("Status");
  status["Code"] = 0;
  status["Reason"] = "";
  status["UserMessage"] = "";
  time_t now = time(nullptr);
  struct tm *tm_info = localtime(&now);
  char ts[30];
  snprintf(ts, sizeof(ts), "%04d-%02d-%02dT%02d:%02d:%02d+00:00",
           tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
           tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
  head["Timestamp"] = ts;

  JsonObject body = doc.createNestedObject("Body");
  JsonObject data = body.createNestedObject("Data");
  JsonObject inv = data.createNestedObject("1");

#if GROWATT_MODBUS_VERSION == 305
  double pdc = _Protocol.InputRegisters[P305_DC_POWER].value * _Protocol.InputRegisters[P305_DC_POWER].multiplier * 1000.0;
#elif GROWATT_MODBUS_VERSION == 120
  double pdc = _Protocol.InputRegisters[P120_INPUT_POWER].value * _Protocol.InputRegisters[P120_INPUT_POWER].multiplier * 1000.0;
#elif GROWATT_MODBUS_VERSION == 124
  double pdc = _Protocol.InputRegisters[P124_INPUT_POWER].value * _Protocol.InputRegisters[P124_INPUT_POWER].multiplier * 1000.0;
#else
  double pdc = 0;
#endif

#if GROWATT_MODBUS_VERSION == 305
  uint32_t gwStatus = _Protocol.InputRegisters[P305_I_STATUS].value;
#elif GROWATT_MODBUS_VERSION == 120
  uint32_t gwStatus = _Protocol.InputRegisters[P120_I_STATUS].value;
#elif GROWATT_MODBUS_VERSION == 124
  uint32_t gwStatus = _Protocol.InputRegisters[P124_I_STATUS].value;
#else
  uint32_t gwStatus = 0;
#endif
  uint8_t froniusStatus = MapStatusToFronius(gwStatus);

  inv["CustomName"] = "Growatt Inverter";
  inv["DT"] = FRONIUS_DEVICE_TYPE;
  inv["ErrorCode"] = 0;
  inv["PVPower"] = (uint32_t)pdc;
  inv["Show"] = 1;
  inv["StatusCode"] = froniusStatus;
  inv["UniqueID"] = FRONIUS_SERIAL;

  serializeJson(doc, Buffer, MQTT_MAX_PACKET_SIZE);
}

void Growatt::CreateLoggerInfoJson(char *Buffer) {
  StaticJsonDocument<512> doc;

  JsonObject head = doc.createNestedObject("Head");
  head.createNestedObject("RequestArguments");
  JsonObject status = head.createNestedObject("Status");
  status["Code"] = 0;
  status["Reason"] = "";
  status["UserMessage"] = "";
  time_t now = time(nullptr);
  struct tm *tm_info = localtime(&now);
  char ts[30];
  snprintf(ts, sizeof(ts), "%04d-%02d-%02dT%02d:%02d:%02d+00:00",
           tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
           tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
  head["Timestamp"] = ts;

  JsonObject body = doc.createNestedObject("Body");
  JsonObject data = body.createNestedObject("LoggerInfo");
  data["SWVersion"] = "1.0";
  data["HWVersion"] = "1.0";
  data["TimezoneLocation"] = "UTC";

  serializeJson(doc, Buffer, MQTT_MAX_PACKET_SIZE);
}

void Growatt::CreateActiveDeviceInfoJson(char *Buffer) {
  StaticJsonDocument<256> doc;

  JsonObject head = doc.createNestedObject("Head");
  JsonObject args = head.createNestedObject("RequestArguments");
  args["DeviceClass"] = "SensorCard";
  JsonObject status = head.createNestedObject("Status");
  status["Code"] = 0;
  status["Reason"] = "";
  status["UserMessage"] = "";
  time_t now = time(nullptr);
  struct tm *tm_info = localtime(&now);
  char ts[30];
  snprintf(ts, sizeof(ts), "%04d-%02d-%02dT%02d:%02d:%02d+00:00",
           tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
           tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
  head["Timestamp"] = ts;

  JsonObject body = doc.createNestedObject("Body");
  body.createNestedObject("Data");

  serializeJson(doc, Buffer, MQTT_MAX_PACKET_SIZE);
}
