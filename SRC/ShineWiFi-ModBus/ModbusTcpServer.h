#ifndef _MODBUS_TCP_SERVER_H_
#define _MODBUS_TCP_SERVER_H_

#include <Arduino.h>
#ifdef ESP8266
#include <ESP8266WiFi.h>
#elif ESP32
#include <WiFi.h>
#endif

#include "Config.h"
#include "Growatt.h"

#ifndef MODBUS_TCP_MAX_CLIENTS
#define MODBUS_TCP_MAX_CLIENTS 2
#endif
#ifndef MODBUS_TCP_UNIT_ID
#define MODBUS_TCP_UNIT_ID 1
#endif

class ModbusTcpServer {
  public:
    ModbusTcpServer();
    void begin(uint16_t port, Growatt *inverter);
    void loop();

  private:
    WiFiServer* _server;
    WiFiClient _clients[MODBUS_TCP_MAX_CLIENTS];
    Growatt *_inverter;

    void _handleClient(WiFiClient &client);
    bool _handleReadHoldingRegisters(uint16_t startAddr, uint16_t qty, uint8_t *pdu, uint16_t *pduLen, uint8_t *exceptionCode);
    bool _handleReadInputRegisters(uint16_t startAddr, uint16_t qty, uint8_t *pdu, uint16_t *pduLen, uint8_t *exceptionCode);
    bool _handleWriteSingleRegister(uint16_t address, uint16_t value, uint8_t *pdu, uint16_t *pduLen, uint8_t *exceptionCode);
    bool _handleWriteMultipleRegisters(uint16_t address, uint16_t qty, const uint8_t *data, uint8_t byteCount, uint8_t *pdu, uint16_t *pduLen, uint8_t *exceptionCode);
    void _sendException(WiFiClient &client, uint16_t transactionId, uint8_t unitId, uint8_t functionCode, uint8_t exceptionCode);
    void _sendResponse(WiFiClient &client, uint16_t transactionId, uint8_t unitId, const uint8_t *pdu, uint16_t pduLen);
};

#endif // _MODBUS_TCP_SERVER_H_
