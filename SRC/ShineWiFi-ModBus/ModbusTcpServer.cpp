#include "ModbusTcpServer.h"

ModbusTcpServer::ModbusTcpServer() : _server(nullptr), _inverter(nullptr) {}

void ModbusTcpServer::begin(uint16_t port, Growatt *inverter) {
  _inverter = inverter;
  _server = new WiFiServer(port);
  _server->begin();
  _server->setNoDelay(true);
}

void ModbusTcpServer::loop() {
  if (_server == nullptr || _inverter == nullptr) {
    return;
  }

  WiFiClient incoming = _server->available();
  if (incoming) {
    for (uint8_t i = 0; i < MODBUS_TCP_MAX_CLIENTS; i++) {
      if (!_clients[i] || !_clients[i].connected()) {
        if (_clients[i]) {
          _clients[i].stop();
        }
        _clients[i] = incoming;
        break;
      }
    }
  }

  for (uint8_t i = 0; i < MODBUS_TCP_MAX_CLIENTS; i++) {
    if (_clients[i] && _clients[i].connected()) {
      _handleClient(_clients[i]);
    } else if (_clients[i]) {
      _clients[i].stop();
    }
  }
}

void ModbusTcpServer::_handleClient(WiFiClient &client) {
  if (client.available() < 7) {
    return;
  }

  uint8_t mbap[7];
  if (client.readBytes((char*)mbap, sizeof(mbap)) != sizeof(mbap)) {
    return;
  }

  const uint16_t transactionId = (mbap[0] << 8) | mbap[1];
  const uint16_t protocolId = (mbap[2] << 8) | mbap[3];
  const uint16_t length = (mbap[4] << 8) | mbap[5];
  const uint8_t unitId = mbap[6];

  if (protocolId != 0 || length < 2 || length > 253) {
    return;
  }
  if (unitId != MODBUS_TCP_UNIT_ID && unitId != 0xFF) {
    return;
  }

  const uint16_t pduInputLen = length - 1;
  uint8_t requestPdu[253];
  if (client.readBytes((char*)requestPdu, pduInputLen) != pduInputLen) {
    return;
  }

  const uint8_t functionCode = requestPdu[0];
  uint8_t responsePdu[253];
  uint16_t responsePduLen = 0;
  uint8_t exceptionCode = 0;

  bool ok = false;
  switch (functionCode) {
    case 0x03: {
      if (pduInputLen != 5) {
        exceptionCode = 0x03;
        break;
      }
      const uint16_t startAddr = (requestPdu[1] << 8) | requestPdu[2];
      const uint16_t qty = (requestPdu[3] << 8) | requestPdu[4];
      ok = _handleReadHoldingRegisters(startAddr, qty, responsePdu, &responsePduLen, &exceptionCode);
      break;
    }
    case 0x04: {
      if (pduInputLen != 5) {
        exceptionCode = 0x03;
        break;
      }
      const uint16_t startAddr = (requestPdu[1] << 8) | requestPdu[2];
      const uint16_t qty = (requestPdu[3] << 8) | requestPdu[4];
      ok = _handleReadInputRegisters(startAddr, qty, responsePdu, &responsePduLen, &exceptionCode);
      break;
    }
    case 0x06: {
      if (pduInputLen != 5) {
        exceptionCode = 0x03;
        break;
      }
      const uint16_t address = (requestPdu[1] << 8) | requestPdu[2];
      const uint16_t value = (requestPdu[3] << 8) | requestPdu[4];
      ok = _handleWriteSingleRegister(address, value, responsePdu, &responsePduLen, &exceptionCode);
      break;
    }
    case 0x10: {
      if (pduInputLen < 6) {
        exceptionCode = 0x03;
        break;
      }
      const uint16_t address = (requestPdu[1] << 8) | requestPdu[2];
      const uint16_t qty = (requestPdu[3] << 8) | requestPdu[4];
      const uint8_t byteCount = requestPdu[5];
      if (pduInputLen != (uint16_t)(6 + byteCount)) {
        exceptionCode = 0x03;
        break;
      }
      ok = _handleWriteMultipleRegisters(address, qty, &requestPdu[6], byteCount, responsePdu, &responsePduLen, &exceptionCode);
      break;
    }
    default:
      exceptionCode = 0x01;
      break;
  }

  if (!ok) {
    _sendException(client, transactionId, unitId, functionCode, exceptionCode == 0 ? 0x04 : exceptionCode);
    return;
  }

  _sendResponse(client, transactionId, unitId, responsePdu, responsePduLen);
}

bool ModbusTcpServer::_handleReadHoldingRegisters(uint16_t startAddr, uint16_t qty, uint8_t *pdu, uint16_t *pduLen, uint8_t *exceptionCode) {
  if (qty == 0 || qty > 125) {
    *exceptionCode = 0x03;
    return false;
  }
  if ((uint32_t)startAddr + qty > 0x10000UL) {
    *exceptionCode = 0x02;
    return false;
  }
  pdu[0] = 0x03;
  pdu[1] = qty * 2;
  for (uint16_t i = 0; i < qty; i++) {
    uint16_t value = 0;
    _inverter->GetHoldingWordByAddress(startAddr + i, &value);
    pdu[2 + (i * 2)] = (value >> 8) & 0xFF;
    pdu[3 + (i * 2)] = value & 0xFF;
  }
  *pduLen = 2 + (qty * 2);
  return true;
}

bool ModbusTcpServer::_handleReadInputRegisters(uint16_t startAddr, uint16_t qty, uint8_t *pdu, uint16_t *pduLen, uint8_t *exceptionCode) {
  if (qty == 0 || qty > 125) {
    *exceptionCode = 0x03;
    return false;
  }
  if ((uint32_t)startAddr + qty > 0x10000UL) {
    *exceptionCode = 0x02;
    return false;
  }
  pdu[0] = 0x04;
  pdu[1] = qty * 2;
  for (uint16_t i = 0; i < qty; i++) {
    uint16_t value = 0;
    _inverter->GetInputWordByAddress(startAddr + i, &value);
    pdu[2 + (i * 2)] = (value >> 8) & 0xFF;
    pdu[3 + (i * 2)] = value & 0xFF;
  }
  *pduLen = 2 + (qty * 2);
  return true;
}

bool ModbusTcpServer::_handleWriteSingleRegister(uint16_t address, uint16_t value, uint8_t *pdu, uint16_t *pduLen, uint8_t *exceptionCode) {
  if (!_inverter->IsHoldingRegisterWriteAddress(address)) {
    *exceptionCode = 0x02;
    return false;
  }
  if (!_inverter->WriteHoldingReg(address, value)) {
    *exceptionCode = 0x04;
    return false;
  }
  pdu[0] = 0x06;
  pdu[1] = (address >> 8) & 0xFF;
  pdu[2] = address & 0xFF;
  pdu[3] = (value >> 8) & 0xFF;
  pdu[4] = value & 0xFF;
  *pduLen = 5;
  return true;
}

bool ModbusTcpServer::_handleWriteMultipleRegisters(uint16_t address, uint16_t qty, const uint8_t *data, uint8_t byteCount, uint8_t *pdu, uint16_t *pduLen, uint8_t *exceptionCode) {
  if (qty == 0 || qty > 123 || byteCount != qty * 2) {
    *exceptionCode = 0x03;
    return false;
  }
  if ((uint32_t)address + qty > 0x10000UL) {
    *exceptionCode = 0x02;
    return false;
  }

  for (uint16_t i = 0; i < qty; i++) {
    if (!_inverter->IsHoldingRegisterWriteAddress(address + i)) {
      *exceptionCode = 0x02;
      return false;
    }
  }

  for (uint16_t i = 0; i < qty; i++) {
    const uint16_t value = (data[i * 2] << 8) | data[(i * 2) + 1];
    if (!_inverter->WriteHoldingReg(address + i, value)) {
      *exceptionCode = 0x04;
      return false;
    }
  }

  pdu[0] = 0x10;
  pdu[1] = (address >> 8) & 0xFF;
  pdu[2] = address & 0xFF;
  pdu[3] = (qty >> 8) & 0xFF;
  pdu[4] = qty & 0xFF;
  *pduLen = 5;
  return true;
}

void ModbusTcpServer::_sendException(WiFiClient &client, uint16_t transactionId, uint8_t unitId, uint8_t functionCode, uint8_t exceptionCode) {
  uint8_t pdu[2];
  pdu[0] = functionCode | 0x80;
  pdu[1] = exceptionCode;
  _sendResponse(client, transactionId, unitId, pdu, sizeof(pdu));
}

void ModbusTcpServer::_sendResponse(WiFiClient &client, uint16_t transactionId, uint8_t unitId, const uint8_t *pdu, uint16_t pduLen) {
  uint8_t mbap[7];
  mbap[0] = (transactionId >> 8) & 0xFF;
  mbap[1] = transactionId & 0xFF;
  mbap[2] = 0;
  mbap[3] = 0;
  const uint16_t length = pduLen + 1;
  mbap[4] = (length >> 8) & 0xFF;
  mbap[5] = length & 0xFF;
  mbap[6] = unitId;
  client.write(mbap, sizeof(mbap));
  client.write(pdu, pduLen);
}
