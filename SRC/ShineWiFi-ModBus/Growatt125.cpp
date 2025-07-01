#include "Arduino.h"
#include "Growatt125.h"

// NOTE: my inverter (SPH4-10KTL3 BH-UP) only manages to read 64 registers in one read!
void init_growatt125(sProtocolDefinition_t &Protocol) {
    // definition of input registers
    Protocol.InputRegisterCount = P125_REGISTER_COUNT;

    // address, value, size, name, multiplier, unit, frontend, plot
    Protocol.InputFragmentCount = 4;
    Protocol.InputReadFragments[0] = sGrowattReadFragment_t{0, 64};
    Protocol.InputReadFragments[1] = sGrowattReadFragment_t{64, 64};
    Protocol.InputReadFragments[2] = sGrowattReadFragment_t{128, 64};
    Protocol.InputReadFragments[3] = sGrowattReadFragment_t{192, 64};

    Protocol.HoldingRegisterCount = 0;
    Protocol.HoldingFragmentCount = 0;

    // FEAGMENT 1: BEGIN
    Protocol.InputRegisters[P125_I_STATUS] = sGrowattModbusReg_t{0, 0, SIZE_16BIT, "InverterStatus", 1, NONE, true, false}; // #1
    Protocol.InputRegisters[P125_INPUT_POWER] = sGrowattModbusReg_t{1, 0, SIZE_32BIT, "InputPower", 0.1, POWER_W, true, true}; // #2
    Protocol.InputRegisters[P125_PV1_VOLTAGE] = sGrowattModbusReg_t{3, 0, SIZE_16BIT, "PV1Voltage", 0.1, VOLTAGE, true, false}; // #3
    Protocol.InputRegisters[P125_PV1_CURRENT] = sGrowattModbusReg_t{4, 0, SIZE_16BIT, "PV1Current", 0.1, CURRENT, false, false}; // #4
    Protocol.InputRegisters[P125_PV1_POWER] = sGrowattModbusReg_t{5, 0, SIZE_32BIT, "PV1Power", 0.1, POWER_W, false, false}; // #5
    Protocol.InputRegisters[P125_PV2_VOLTAGE] = sGrowattModbusReg_t{7, 0, SIZE_16BIT, "PV2Voltage", 0.1, VOLTAGE, true, false}; // #6
    Protocol.InputRegisters[P125_PV2_CURRENT] = sGrowattModbusReg_t{8, 0, SIZE_16BIT, "PV2Current", 0.1, CURRENT, false, false}; // #7
    Protocol.InputRegisters[P125_PV2_POWER] = sGrowattModbusReg_t{9, 0, SIZE_32BIT, "PV2Power", 0.1, POWER_W, false, false}; // #8

    Protocol.InputRegisters[P125_PAC] = sGrowattModbusReg_t{35, 0, SIZE_32BIT, "OutputPower", 0.1, POWER_W, true, true}; // #9
    Protocol.InputRegisters[P125_FAC] = sGrowattModbusReg_t{37, 0, SIZE_16BIT, "GridFrequency", 0.01, FREQUENCY, true, false}; // #10

    Protocol.InputRegisters[P125_VAC1] = sGrowattModbusReg_t{38, 0, SIZE_16BIT, "L1Voltage", 0.1, VOLTAGE, true, false}; // #11
    Protocol.InputRegisters[P125_IAC1] = sGrowattModbusReg_t{39, 0, SIZE_16BIT, "L1Current", 0.1, CURRENT, true, false}; // #12
    Protocol.InputRegisters[P125_PAC1] = sGrowattModbusReg_t{40, 0, SIZE_32BIT, "L1Power", 0.1, POWER_W, true, false}; // #13
    Protocol.InputRegisters[P125_VAC2] = sGrowattModbusReg_t{42, 0, SIZE_16BIT, "L2Voltage", 0.1, VOLTAGE, true, false}; // #14
    Protocol.InputRegisters[P125_IAC2] = sGrowattModbusReg_t{43, 0, SIZE_16BIT, "L2Current", 0.1, CURRENT, true, false}; // #15
    Protocol.InputRegisters[P125_PAC2] = sGrowattModbusReg_t{44, 0, SIZE_32BIT, "L2Power", 0.1, POWER_W, true, false}; // #16
    Protocol.InputRegisters[P125_VAC3] = sGrowattModbusReg_t{46, 0, SIZE_16BIT, "L3Voltage", 0.1, VOLTAGE, true, false}; // #17
    Protocol.InputRegisters[P125_IAC3] = sGrowattModbusReg_t{47, 0, SIZE_16BIT, "L3Current", 0.1, CURRENT, true, false}; // #18
    Protocol.InputRegisters[P125_PAC3] = sGrowattModbusReg_t{48, 0, SIZE_32BIT, "L3Power", 0.1, POWER_W, true, false}; // #19

    Protocol.InputRegisters[P125_VAC_RS] = sGrowattModbusReg_t{50, 0, SIZE_16BIT, "VoltageRS", 0.1, VOLTAGE, false, false}; // #20
    Protocol.InputRegisters[P125_VAC_ST] = sGrowattModbusReg_t{51, 0, SIZE_16BIT, "VoltageST", 0.1, VOLTAGE, false, false}; // #21
    Protocol.InputRegisters[P125_VAC_TR] = sGrowattModbusReg_t{52, 0, SIZE_16BIT, "VoltageTR", 0.1, VOLTAGE, false, false}; // #22

    Protocol.InputRegisters[P125_EAC_TODAY] = sGrowattModbusReg_t{53, 0, SIZE_32BIT, "EnergyToday", 0.1, POWER_KWH, true, false}; // #23
    Protocol.InputRegisters[P125_EAC_TOTAL] = sGrowattModbusReg_t{55, 0, SIZE_32BIT, "EnergyTotal", 0.1, POWER_KWH, true, false}; // #24
    Protocol.InputRegisters[P125_TIME_TOTAL] = sGrowattModbusReg_t{57, 0, SIZE_32BIT, "WorkTimeTotal", 0.5, SECONDS, false, false}; // #25

    Protocol.InputRegisters[P125_TEMP1] = sGrowattModbusReg_t{93, 0, SIZE_16BIT, "Temp1", 0.1, TEMPERATURE, false, false}; // #26
    Protocol.InputRegisters[P125_TEMP2] = sGrowattModbusReg_t{94, 0, SIZE_16BIT, "Temp2", 0.1, TEMPERATURE, false, false}; // #27
    Protocol.InputRegisters[P125_TEMP3] = sGrowattModbusReg_t{95, 0, SIZE_16BIT, "Temp3", 0.1, TEMPERATURE, false, false}; // #28

    Protocol.InputRegisters[P125_DERATE_REASON] = sGrowattModbusReg_t{1123, 0, SIZE_16BIT, "DerateReason", 1, NONE, true, false}; // #29
    Protocol.InputRegisters[P125_EXPORT_LIMIT_ENABLED] = sGrowattModbusReg_t{1148, 0, SIZE_16BIT, "ExportLimitEnabled", 1, NONE, true, false}; // #30
    Protocol.InputRegisters[P125_EXPORT_LIMIT_PERCENT] = sGrowattModbusReg_t{1149, 0, SIZE_16BIT, "ExportLimitPercent", 0.1, PRECENTAGE, true, false}; // #31

    Protocol.InputRegisters[P125_FAULT_CODE] = sGrowattModbusReg_t{1185, 0, SIZE_16BIT, "FaultCode", 1, NONE, true, false}; // #32
    Protocol.InputRegisters[P125_FAULT_MASK_HIGH] = sGrowattModbusReg_t{1186, 0, SIZE_16BIT, "FaultMaskHigh", 1, NONE, false, false}; // #33
    Protocol.InputRegisters[P125_FAULT_MASK_LOW] = sGrowattModbusReg_t{1187, 0, SIZE_16BIT, "FaultMaskLow", 1, NONE, false, false}; // #34
    Protocol.InputRegisters[P125_WARNING_MASK_HIGH] = sGrowattModbusReg_t{1188, 0, SIZE_16BIT, "WarningMaskHigh", 1, NONE, false, false}; // #35
    Protocol.InputRegisters[P125_WARNING_MASK_LOW] = sGrowattModbusReg_t{1189, 0, SIZE_16BIT, "WarningMaskLow", 1, NONE, false, false}; // #36

    Protocol.InputRegisters[P125_PDISCHARGE] = sGrowattModbusReg_t{1009, 0, SIZE_32BIT, "DischargePower", 0.1, POWER_W, true, true}; // #37
    Protocol.InputRegisters[P125_PCHARGE] = sGrowattModbusReg_t{1011, 0, SIZE_32BIT, "ChargePower", 0.1, POWER_W, true, true}; // #38
    Protocol.InputRegisters[P125_VBAT] = sGrowattModbusReg_t{1013, 0, SIZE_16BIT, "BatteryVoltage", 0.1, VOLTAGE, true, false}; // #39
    Protocol.InputRegisters[P125_SOC] = sGrowattModbusReg_t{1014, 0, SIZE_16BIT, "BatterySOC", 1, PRECENTAGE, true, true}; // #40

    Protocol.InputRegisters[P125_PAC_TO_USER] = sGrowattModbusReg_t{1015, 0, SIZE_32BIT, "PowerToUser", 0.1, POWER_W, true, true}; // #41
    Protocol.InputRegisters[P125_PAC_TO_USER_TOTAL] = sGrowattModbusReg_t{1021, 0, SIZE_32BIT, "PowerToUserTotal", 0.1, POWER_KWH, false, false}; // #42
    Protocol.InputRegisters[P125_PAC_TO_GRID] = sGrowattModbusReg_t{1023, 0, SIZE_32BIT, "PowerToGrid", 0.1, POWER_W, true, true}; // #43
    Protocol.InputRegisters[P125_PAC_TO_GRID_TOTAL] = sGrowattModbusReg_t{1029, 0, SIZE_32BIT, "PowerToGridTotal", 0.1, POWER_KWH, false, false}; // #44
    Protocol.InputRegisters[P125_PLOCAL_LOAD] = sGrowattModbusReg_t{1031, 0, SIZE_32BIT, "PowerToLocalLoad", 0.1, POWER_W, true, false}; // #45
    Protocol.InputRegisters[P125_PLOCAL_LOAD_TOTAL] = sGrowattModbusReg_t{1037, 0, SIZE_32BIT, "PowerToLocalLoadTotal", 0.1, POWER_KWH, true, false}; // #46

    Protocol.InputRegisters[P125_BATTERY_TEMPERATURE] = sGrowattModbusReg_t{1040, 0, SIZE_16BIT, "BatteryTemp", 0.1, TEMPERATURE, false, false}; // #47
    Protocol.InputRegisters[P125_BATTERY_STATE] = sGrowattModbusReg_t{1041, 0, SIZE_16BIT, "BatteryState", 1, NONE, false, false}; // #48

    Protocol.InputRegisters[P125_ETOUSER_TODAY] = sGrowattModbusReg_t{1044, 0, SIZE_32BIT, "EnergyToUserToday", 0.1, POWER_KWH, false, false}; // #49
    Protocol.InputRegisters[P125_ETOUSER_TOTAL] = sGrowattModbusReg_t{1046, 0, SIZE_32BIT, "EnergyToUserTotal", 0.1, POWER_KWH, false, false}; // #50
    Protocol.InputRegisters[P125_ETOGRID_TODAY] = sGrowattModbusReg_t{1048, 0, SIZE_32BIT, "EnergyToGridToday", 0.1, POWER_KWH, false, false}; // #51
    Protocol.InputRegisters[P125_ETOGRID_TOTAL] = sGrowattModbusReg_t{1050, 0, SIZE_32BIT, "EnergyToGridTotal", 0.1, POWER_KWH, false, false}; // #52
    Protocol.InputRegisters[P125_EDISCHARGE_TODAY] = sGrowattModbusReg_t{1052, 0, SIZE_32BIT, "DischargeEnergyToday", 0.1, POWER_KWH, false, false}; // #53
    Protocol.InputRegisters[P125_EDISCHARGE_TOTAL] = sGrowattModbusReg_t{1054, 0, SIZE_32BIT, "DischargeEnergyTotal", 0.1, POWER_KWH, false, false}; // #54
    Protocol.InputRegisters[P125_ECHARGE_TODAY] = sGrowattModbusReg_t{1056, 0, SIZE_32BIT, "ChargeEnergyToday", 0.1, POWER_KWH, false, false}; // #55
    Protocol.InputRegisters[P125_ECHARGE_TOTAL] = sGrowattModbusReg_t{1058, 0, SIZE_32BIT, "ChargeEnergyTotal", 0.1, POWER_KWH, false, false}; // #56
    Protocol.InputRegisters[P125_ETOLOCALLOAD_TODAY] = sGrowattModbusReg_t{1060, 0, SIZE_32BIT, "LocalLoadEnergyToday", 0.1, POWER_KWH, false, false}; // #57
    Protocol.InputRegisters[P125_ETOLOCALLOAD_TOTAL] = sGrowattModbusReg_t{1062, 0, SIZE_32BIT, "LocalLoadEnergyTotal", 0.1, POWER_KWH, false, false}; // #58

    Protocol.InputRegisters[P125_OUTPUT_PERCENT] = sGrowattModbusReg_t{1100, 0, SIZE_16BIT, "OutputPercent", 0.1, PRECENTAGE, false, false}; // #59
    Protocol.InputRegisters[P125_PF] = sGrowattModbusReg_t{1101, 0, SIZE_16BIT, "PowerFactor", 0.01, NONE, false, false}; // #60
    Protocol.InputRegisters[P125_REACTIVE_POWER_MODE] = sGrowattModbusReg_t{1120, 0, SIZE_16BIT, "ReactivePowerMode", 1, NONE, false, false}; // #61
    Protocol.InputRegisters[P125_PF_COMMAND] = sGrowattModbusReg_t{1121, 0, SIZE_16BIT, "PowerFactorCommand", 0.01, NONE, false, false}; // #62

    Protocol.InputRegisters[P125_VOLTAGE_TRIP_OV] = sGrowattModbusReg_t{1130, 0, SIZE_16BIT, "VoltageTripOV", 0.1, VOLTAGE, false, false}; // #63
    Protocol.InputRegisters[P125_VOLTAGE_TRIP_UV] = sGrowattModbusReg_t{1131, 0, SIZE_16BIT, "VoltageTripUV", 0.1, VOLTAGE, false, false}; // #64
    Protocol.InputRegisters[P125_FREQ_TRIP_OF] = sGrowattModbusReg_t{1132, 0, SIZE_16BIT, "FreqTripOF", 0.01, FREQUENCY, false, false}; // #65
    Protocol.InputRegisters[P125_FREQ_TRIP_UF] = sGrowattModbusReg_t{1133, 0, SIZE_16BIT, "FreqTripUF", 0.01, FREQUENCY, false, false}; // #66
    Protocol.InputRegisters[P125_VOLTAGE_RECONNECT] = sGrowattModbusReg_t{1134, 0, SIZE_16BIT, "VoltageReconnect", 0.1, VOLTAGE, false, false}; // #67
    Protocol.InputRegisters[P125_FREQ_RECONNECT] = sGrowattModbusReg_t{1135, 0, SIZE_16BIT, "FreqReconnect", 0.01, FREQUENCY, false, false}; // #68
    Protocol.InputRegisters[P125_START_DELAY] = sGrowattModbusReg_t{1136, 0, SIZE_16BIT, "StartDelay", 1, SECONDS, false, false}; // #69
    Protocol.InputRegisters[P125_RECONNECT_DELAY] = sGrowattModbusReg_t{1137, 0, SIZE_16BIT, "ReconnectDelay", 1, SECONDS, false, false}; // #70
    Protocol.InputRegisters[P125_RAMP_UP_RATE] = sGrowattModbusReg_t{1138, 0, SIZE_16BIT, "RampUpRate", 0.1, NONE, false, false}; // #71
    Protocol.InputRegisters[P125_RAMP_DOWN_RATE] = sGrowattModbusReg_t{1139, 0, SIZE_16BIT, "RampDownRate", 0.1, NONE, false, false}; // #72
}
