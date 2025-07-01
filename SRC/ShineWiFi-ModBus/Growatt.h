#ifndef _GROWATT_H_
#define _GROWATT_H_

#include "GrowattTypes.h"

class Growatt {
  public:
    Growatt();
    sProtocolDefinition_t _Protocol;

    void begin(Stream &serial);
    void InitProtocol();

    bool ReadInputRegisters();
    bool ReadHoldingRegisters();
    bool ReadInputRegistersFast();
    bool ReadHoldingRegistersFast();
    bool ReadData(bool fullRead = true);
    eDevice_t GetWiFiStickType();
    sGrowattModbusReg_t GetInputRegister(uint16_t reg);
    sGrowattModbusReg_t GetHoldingRegister(uint16_t reg);
    bool ReadInputReg(uint16_t adr, uint32_t* result);
    bool ReadInputReg(uint16_t adr, uint16_t* result);
    bool ReadHoldingReg(uint16_t adr, uint32_t* result);
    bool ReadHoldingReg(uint16_t adr, uint16_t* result);
    bool WriteHoldingReg(uint16_t adr, uint16_t value);
    bool ConfigureExportLimit(uint16_t percent);
    void CreateJson(char *Buffer, const char *MacAddress);
    void CreateUIJson(char *Buffer);
    void CreateFroniusJson(char *Buffer);
    void CreatePowerFlowJson(char *Buffer);
    void CreateDeviceInfoJson(char *Buffer);
    void CreateInverterInfoJson(char *Buffer);
    void CreateLoggerInfoJson(char *Buffer);
    void CreateActiveDeviceInfoJson(char *Buffer);
    static uint8_t MapStatusToFronius(uint32_t status);
  private:
    eDevice_t _eDevice;
    bool _GotData;
    uint32_t _PacketCnt;
    // previous total energy reading to compute increments
    double _prevTotalEnergy;
    bool _prevEnergyValid;
    // accumulated phase energies in Wh since startup
    double _accEnergyL1;
    double _accEnergyL2;
    double _accEnergyL3;

    eDevice_t _InitModbusCommunication();
    static double _round2(double value);
    void _UpdateEnergyAccumulation();

};

#endif // _GROWATT_H_