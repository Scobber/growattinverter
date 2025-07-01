#include "Arduino.h"
#include "Growatt125.h"

void init_growatt125(sProtocolDefinition_t &Protocol) {
    // Set count explicitly to match defined registers (auto-deduced)
    Protocol.InputRegisterCount = P125_REGISTER_COUNT;

    // Read fragments (respect 64-register limit)
    Protocol.InputFragmentCount = 4;
    Protocol.InputReadFragments[0] = sGrowattReadFragment_t{0, 64};
    Protocol.InputReadFragments[1] = sGrowattReadFragment_t{64, 64};
    Protocol.InputReadFragments[2] = sGrowattReadFragment_t{128, 64};
    Protocol.InputReadFragments[3] = sGrowattReadFragment_t{192, 64};

    Protocol.HoldingRegisterCount = 0;
    Protocol.HoldingFragmentCount = 0;

    Protocol.InputRegisters[P125_I_STATUS] = {0, 0, SIZE_16BIT, "InverterStatus", 1, NONE, true, false};
    Protocol.InputRegisters[P125_INPUT_POWER] = {1, 0, SIZE_32BIT, "InputPower", 0.1, POWER_W, true, true};
    Protocol.InputRegisters[P125_PV1_VOLTAGE] = {3, 0, SIZE_16BIT, "PV1Voltage", 0.1, VOLTAGE, true, false};
    Protocol.InputRegisters[P125_PV1_CURRENT] = {4, 0, SIZE_16BIT, "PV1Current", 0.1, CURRENT, false, false};
    Protocol.InputRegisters[P125_PV1_POWER] = {5, 0, SIZE_32BIT, "PV1Power", 0.1, POWER_W, false, false};
    Protocol.InputRegisters[P125_PV2_VOLTAGE] = {7, 0, SIZE_16BIT, "PV2Voltage", 0.1, VOLTAGE, true, false};
    Protocol.InputRegisters[P125_PV2_CURRENT] = {8, 0, SIZE_16BIT, "PV2Current", 0.1, CURRENT, false, false};
    Protocol.InputRegisters[P125_PV2_POWER] = {9, 0, SIZE_32BIT, "PV2Power", 0.1, POWER_W, false, false};

    Protocol.InputRegisters[P125_PAC] = {35, 0, SIZE_32BIT, "OutputPower", 0.1, POWER_W, true, true};
    Protocol.InputRegisters[P125_FAC] = {37, 0, SIZE_16BIT, "GridFrequency", 0.01, FREQUENCY, true, false};

    Protocol.InputRegisters[P125_VAC1] = {38, 0, SIZE_16BIT, "L1Voltage", 0.1, VOLTAGE, true, false};
    Protocol.InputRegisters[P125_IAC1] = {39, 0, SIZE_16BIT, "L1Current", 0.1, CURRENT, true, false};
    Protocol.InputRegisters[P125_PAC1] = {40, 0, SIZE_32BIT, "L1Power", 0.1, POWER_W, true, false};
    Protocol.InputRegisters[P125_VAC2] = {42, 0, SIZE_16BIT, "L2Voltage", 0.1, VOLTAGE, true, false};
    Protocol.InputRegisters[P125_IAC2] = {43, 0, SIZE_16BIT, "L2Current", 0.1, CURRENT, true, false};
    Protocol.InputRegisters[P125_PAC2] = {44, 0, SIZE_32BIT, "L2Power", 0.1, POWER_W, true, false};
    Protocol.InputRegisters[P125_VAC3] = {46, 0, SIZE_16BIT, "L3Voltage", 0.1, VOLTAGE, true, false};
    Protocol.InputRegisters[P125_IAC3] = {47, 0, SIZE_16BIT, "L3Current", 0.1, CURRENT, true, false};
    Protocol.InputRegisters[P125_PAC3] = {48, 0, SIZE_32BIT, "L3Power", 0.1, POWER_W, true, false};

    Protocol.InputRegisters[P125_VAC_RS] = {50, 0, SIZE_16BIT, "VoltageRS", 0.1, VOLTAGE, false, false};
    Protocol.InputRegisters[P125_VAC_ST] = {51, 0, SIZE_16BIT, "VoltageST", 0.1, VOLTAGE, false, false};
    Protocol.InputRegisters[P125_VAC_TR] = {52, 0, SIZE_16BIT, "VoltageTR", 0.1, VOLTAGE, false, false};

    Protocol.InputRegisters[P125_EAC_TODAY] = {53, 0, SIZE_32BIT, "EnergyToday", 0.1, POWER_KWH, true, false};
    Protocol.InputRegisters[P125_EAC_TOTAL] = {55, 0, SIZE_32BIT, "EnergyTotal", 0.1, POWER_KWH, true, false};
    Protocol.InputRegisters[P125_TIME_TOTAL] = {57, 0, SIZE_32BIT, "WorkTimeTotal", 0.5, SECONDS, false, false};

    Protocol.InputRegisters[P125_TEMP1] = {93, 0, SIZE_16BIT, "Temp1", 0.1, TEMPERATURE, false, false};
    Protocol.InputRegisters[P125_TEMP2] = {94, 0, SIZE_16BIT, "Temp2", 0.1, TEMPERATURE, false, false};
    Protocol.InputRegisters[P125_TEMP3] = {95, 0, SIZE_16BIT, "Temp3", 0.1, TEMPERATURE, false, false};

    Protocol.InputRegisters[P125_DERATE_REASON] = {1123, 0, SIZE_16BIT, "DerateReason", 1, NONE, true, false};
    Protocol.InputRegisters[P125_EXPORT_LIMIT_ENABLED] = {1148, 0, SIZE_16BIT, "ExportLimitEnabled", 1, NONE, true, false};
    Protocol.InputRegisters[P125_EXPORT_LIMIT_PERCENT] = {1149, 0, SIZE_16BIT, "ExportLimitPercent", 0.1, PRECENTAGE, true, false};

    Protocol.InputRegisters[P125_FAULT_CODE] = {1185, 0, SIZE_16BIT, "FaultCode", 1, NONE, true, false};
    Protocol.InputRegisters[P125_FAULT_MASK_HIGH] = {1186, 0, SIZE_16BIT, "FaultMaskHigh", 1, NONE, false, false};
    Protocol.InputRegisters[P125_FAULT_MASK_LOW] = {1187, 0, SIZE_16BIT, "FaultMaskLow", 1, NONE, false, false};
    Protocol.InputRegisters[P125_WARNING_MASK_HIGH] = {1188, 0, SIZE_16BIT, "WarningMaskHigh", 1, NONE, false, false};
    Protocol.InputRegisters[P125_WARNING_MASK_LOW] = {1189, 0, SIZE_16BIT, "WarningMaskLow", 1, NONE, false, false};

    Protocol.InputRegisters[P125_PDISCHARGE] = {1009, 0, SIZE_32BIT, "DischargePower", 0.1, POWER_W, true, true};
    Protocol.InputRegisters[P125_PCHARGE] = {1011, 0, SIZE_32BIT, "ChargePower", 0.1, POWER_W, true, true};
    Protocol.InputRegisters[P125_VBAT] = {1013, 0, SIZE_16BIT, "BatteryVoltage", 0.1, VOLTAGE, true, false};
    Protocol.InputRegisters[P125_SOC] = {1014, 0, SIZE_16BIT, "BatterySOC", 1, PRECENTAGE, true, true};

    Protocol.InputRegisters[P125_PAC_TO_USER] = {1015, 0, SIZE_32BIT, "PowerToUser", 0.1, POWER_W, true, true};
    Protocol.InputRegisters[P125_PAC_TO_USER_TOTAL] = {1021, 0, SIZE_32BIT, "PowerToUserTotal", 0.1, POWER_KWH, false, false};
    Protocol.InputRegisters[P125_PAC_TO_GRID] = {1023, 0, SIZE_32BIT, "PowerToGrid", 0.1, POWER_W, true, true};
    Protocol.InputRegisters[P125_PAC_TO_GRID_TOTAL] = {1029, 0, SIZE_32BIT, "PowerToGridTotal", 0.1, POWER_KWH, false, false};
    Protocol.InputRegisters[P125_PLOCAL_LOAD] = {1031, 0, SIZE_32BIT, "PowerToLocalLoad", 0.1, POWER_W, true, false};
    Protocol.InputRegisters[P125_PLOCAL_LOAD_TOTAL] = {1037, 0, SIZE_32BIT, "PowerToLocalLoadTotal", 0.1, POWER_KWH, true, false};

    Protocol.InputRegisters[P125_BATTERY_TEMPERATURE] = {1040, 0, SIZE_16BIT, "BatteryTemp", 0.1, TEMPERATURE, false, false};
    Protocol.InputRegisters[P125_BATTERY_STATE] = {1041, 0, SIZE_16BIT, "BatteryState", 1, NONE, false, false};

    Protocol.InputRegisters[P125_ETOUSER_TODAY] = {1044, 0, SIZE_32BIT, "EnergyToUserToday", 0.1, POWER_KWH, false, false};
    Protocol.InputRegisters[P125_ETOUSER_TOTAL] = {1046, 0, SIZE_32BIT, "EnergyToUserTotal", 0.1, POWER_KWH, false, false};
    Protocol.InputRegisters[P125_ETOGRID_TODAY] = {1048, 0, SIZE_32BIT, "EnergyToGridToday", 0.1, POWER_KWH, false, false};
    Protocol.InputRegisters[P125_ETOGRID_TOTAL] = {1050, 0, SIZE_32BIT, "EnergyToGridTotal", 0.1, POWER_KWH, false, false};
    Protocol.InputRegisters[P125_EDISCHARGE_TODAY] = {1052, 0, SIZE_32BIT, "DischargeEnergyToday", 0.1, POWER_KWH, false, false};
    Protocol.InputRegisters[P125_EDISCHARGE_TOTAL] = {1054, 0, SIZE_32BIT, "DischargeEnergyTotal", 0.1, POWER_KWH, false, false};
    Protocol.InputRegisters[P125_ECHARGE_TODAY] = {1056, 0, SIZE_32BIT, "ChargeEnergyToday", 0.1, POWER_KWH, false, false};
    Protocol.InputRegisters[P125_ECHARGE_TOTAL] = {1058, 0, SIZE_32BIT, "ChargeEnergyTotal", 0.1, POWER_KWH, false, false};
    Protocol.InputRegisters[P125_ETOLOCALLOAD_TODAY] = {1060, 0, SIZE_32BIT, "LocalLoadEnergyToday", 0.1, POWER_KWH, false, false};
    Protocol.InputRegisters[P125_ETOLOCALLOAD_TOTAL] = {1062, 0, SIZE_32BIT, "LocalLoadEnergyTotal", 0.1, POWER_KWH, false, false};

    Protocol.InputRegisters[P125_OUTPUT_PERCENT] = {1100, 0, SIZE_16BIT, "OutputPercent", 0.1, PRECENTAGE, false, false};
    Protocol.InputRegisters[P125_PF] = {1101, 0, SIZE_16BIT, "PowerFactor", 0.01, NONE, false, false};
    Protocol.InputRegisters[P125_REACTIVE_POWER_MODE] = {1120, 0, SIZE_16BIT, "ReactivePowerMode", 1, NONE, false, false};
    Protocol.InputRegisters[P125_PF_COMMAND] = {1121, 0, SIZE_16BIT, "PowerFactorCommand", 0.01, NONE, false, false};

    Protocol.InputRegisters[P125_VOLTAGE_TRIP_OV] = {1130, 0, SIZE_16BIT, "VoltageTripOV", 0.1, VOLTAGE, false, false};
    Protocol.InputRegisters[P125_VOLTAGE_TRIP_UV] = {1131, 0, SIZE_16BIT, "VoltageTripUV", 0.1, VOLTAGE, false, false};
    Protocol.InputRegisters[P125_FREQ_TRIP_OF] = {1132, 0, SIZE_16BIT, "FreqTripOF", 0.01, FREQUENCY, false, false};
    Protocol.InputRegisters[P125_FREQ_TRIP_UF] = {1133, 0, SIZE_16BIT, "FreqTripUF", 0.01, FREQUENCY, false, false};
    Protocol.InputRegisters[P125_VOLTAGE_RECONNECT] = {1134, 0, SIZE_16BIT, "VoltageReconnect", 0.1, VOLTAGE, false, false};
    Protocol.InputRegisters[P125_FREQ_RECONNECT] = {1135, 0, SIZE_16BIT, "FreqReconnect", 0.01, FREQUENCY, false, false};
    Protocol.InputRegisters[P125_START_DELAY] = {1136, 0, SIZE_16BIT, "StartDelay", 1, SECONDS, false, false};
    Protocol.InputRegisters[P125_RECONNECT_DELAY] = {1137, 0, SIZE_16BIT, "ReconnectDelay", 1, SECONDS, false, false};
    Protocol.InputRegisters[P125_RAMP_UP_RATE] = {1138, 0, SIZE_16BIT, "RampUpRate", 0.1, NONE, false, false};
    Protocol.InputRegisters[P125_RAMP_DOWN_RATE] = {1139, 0, SIZE_16BIT, "RampDownRate", 0.1, NONE, false, false};
}
