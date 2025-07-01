#ifndef _GROWATT_TYPES_H_
#define _GROWATT_TYPES_H_

#include <cstring>

typedef enum {
  Undef_stick  = 0,
  ShineWiFi_S  = 1, // Serial DB9-Connector, 9600Bd, Protocol v3.05 (2013)
  ShineWiFi_X  = 2, // USB Type A with Bajonet locking , 115200Bd, Protocol v1.05 (2018)
  ShineWiFi_F  = 3  // USB Type A DB9-style screws, (Baudrate and protocol unclear; likely 115200Bd / v1.05)
} eDevice_t;

typedef enum {
  GwStatusWaiting,
  GwStatusNormal,
  GwStatusFault
} eGrowattStatus_t;

// Protocol v3.05 also knows Autostart, v1.05 does not:
typedef enum {
  GwOnOffOFF    = 0x0000,
  GwOnOffON     = 0x0001,   // default for v1.05
  GwOnOffOFFAuto = 0x0100,
  GwOnOffONAuto = 0x0101    // default for v3.05
} eGrowattOnOff_t;

typedef enum {
    H_STATUS = 0,
} SupportedModbusHoldingRegisters_t;

typedef enum {
    NONE,
    POWER_W,
    POWER_KWH,
    VOLTAGE,
    CURRENT,
    SECONDS,
    PRECENTAGE,
    FREQUENCY,
    TEMPERATURE,
    VA,
} RegisterUnit_t;

typedef enum {
    SIZE_16BIT,
    SIZE_32BIT,
} RegisterSize_t;

typedef struct {
  uint16_t address;
  uint32_t value;
  RegisterSize_t size;
  char name[64];
  float multiplier;
  RegisterUnit_t unit;
  bool frontend;
  bool plot;

  sGrowattModbusReg_t() : address(0), value(0), size(SIZE_16BIT), multiplier(1), unit(NONE), frontend(false), plot(false) {
    name[0] = '\0';
  }

  sGrowattModbusReg_t(uint16_t a, uint32_t v, RegisterSize_t s, const char *n,
                       float m, RegisterUnit_t u, bool f, bool p)
      : address(a), value(v), size(s), multiplier(m), unit(u), frontend(f), plot(p) {
    strncpy(name, n, sizeof(name));
    name[sizeof(name) - 1] = '\0';
  }
} sGrowattModbusReg_t;

// Growatt limits maximal number of registers that can be polled
// with a single read. Define reading frames using this. Can be nicer..
typedef struct {
    uint16_t StartAddress;
    uint8_t FragmentSize;
} sGrowattReadFragment_t;

typedef struct {
    uint16_t InputRegisterCount;
    uint8_t InputFragmentCount;
    uint16_t HoldingRegisterCount;
    uint8_t HoldingFragmentCount;
    sGrowattModbusReg_t InputRegisters[125];
    sGrowattModbusReg_t HoldingRegisters[125];
    sGrowattReadFragment_t InputReadFragments[20];
    sGrowattReadFragment_t HoldingReadFragments[20];
} sProtocolDefinition_t;


#endif // _GROWATT_TYPES_H_