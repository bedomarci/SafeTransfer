#ifndef SAFETRANSFER_H
#define SAFETRANSFER_H
#include "Arduino.h"
#include <Wire.h>

#include "FastCRC.h"

#define CRC_SIZE 2
#define TYPE_SIZE 1

template <typename T>
class SafeTransfer
{
private:
    typedef void (*ST_RECEIVE_CB_SIG)(T);
    typedef T (*ST_REQUEST_CB_SIG)();
    ST_RECEIVE_CB_SIG _receiveCallback;
    ST_REQUEST_CB_SIG _requestCallback;
    enum PacketType {
        DATA = 0,
        ACK = 1,
        ERROR = 2,
        RETRY = 3,
    };
    TwoWire *_wire = nullptr;
    FastCRC16 CRC16;

    uint8_t _address;
    const uint16_t dataSize = sizeof(T);
    const uint16_t packetSize = TYPE_SIZE + dataSize + CRC_SIZE;
    uint8_t *_buffer;
    void bufferToData(uint8_t **buffer, T *data);
    void dataToBuffer(T *data, uint8_t **buffer);
    void appendCrc(uint8_t ** buffer);
    void appendPacketType(uint8_t ** buffer, PacketType type);
    void receive(uint8_t ** buffer);
    void poll();
    bool isCrcValid(uint8_t ** buffer);



public:
    SafeTransfer();
    void begin(TwoWire *wire);
    void setAddress(uint8_t address);
    void loop();
    void onReceive(ST_RECEIVE_CB_SIG cb);
    void onRequest(ST_REQUEST_CB_SIG cb);
    void request();
    void sendToMaster(T data);
    void sendToSlave(T data);
    void sendToSlave(uint8_t address, T data);
    ~SafeTransfer();
};


template <typename T>
SafeTransfer<T>::SafeTransfer(){}

template <typename T>
SafeTransfer<T>::~SafeTransfer() {}

template <typename T>
void SafeTransfer<T>::bufferToData(uint8_t **buffer, T *data) {
    memcpy(data, *buffer + TYPE_SIZE, dataSize);
}

template <typename T>
void SafeTransfer<T>::dataToBuffer(T *data, uint8_t **buffer) {
    *buffer = new uint8_t[packetSize];
    memcpy(*buffer + TYPE_SIZE, data, packetSize);
}

template <typename T>
void SafeTransfer<T>::appendCrc(uint8_t ** buffer){
    uint16_t crc = CRC16.xmodem(*buffer, dataSize + TYPE_SIZE);
    memcpy(*buffer + dataSize + TYPE_SIZE, &crc, CRC_SIZE);
}

template <typename T>
void SafeTransfer<T>::appendPacketType(uint8_t ** buffer, PacketType type){
    *buffer[0] = (uint8_t) type;
}

template <typename T>
bool SafeTransfer<T>::isCrcValid(uint8_t ** buffer) {
    uint16_t expectedCrc = CRC16.xmodem(*buffer, dataSize + TYPE_SIZE);
    uint16_t receivedCrc;
    memcpy(&receivedCrc, *buffer + dataSize + TYPE_SIZE, CRC_SIZE);
    return expectedCrc == receivedCrc;
}

template <typename T>
void SafeTransfer<T>::begin(TwoWire *wire) {
    this->_wire = wire;
#ifndef ESP32
    this->_wire->onReceive([=](int){});
#endif
}

template <typename T>
void SafeTransfer<T>::setAddress(uint8_t address) {
    this->_address = address;
}

template <typename T>
void SafeTransfer<T>::loop() {
    if (!_wire) return;
    if (Wire.available()) {
        uint8_t bufferIndex = 0;
        _buffer = new uint8_t[packetSize];
        while (0 < Wire.available()) {
            _buffer[bufferIndex++] =  Wire.read();
        }
        receive(&_buffer);
    }
}

template <typename T>
void SafeTransfer<T>::receive(uint8_t **buffer) {
    PacketType type = (PacketType)*buffer[0];
    Serial.println(uint8_t(type));
    switch (type)
    {
    case DATA:
        if (!_receiveCallback) return;
        T data;
        bufferToData(buffer, &data);
        Serial.println(data);
        if (isCrcValid(buffer)) {
            _receiveCallback(data);
        } else {
            //send error message
        }
        break;
    
    default:
        break;
    }

}

template <typename T>
void SafeTransfer<T>::onReceive(ST_RECEIVE_CB_SIG cb) {
    _receiveCallback = cb;
}
template <typename T>
void SafeTransfer<T>::onRequest(ST_REQUEST_CB_SIG cb) {
    _requestCallback = cb;
}
template <typename T>
void SafeTransfer<T>::request() {
    if (!_requestCallback) return;
    T data = _requestCallback();
    this->write(data);
}

//WRITER FOR SLAVE (SENDING DATA TO MASTER)
template <typename T>
void SafeTransfer<T>::sendToMaster(T data) {
    if (!_wire) return;
    this->dataToBuffer(&data, &_buffer);
    this->appendPacketType(&_buffer, DATA);
    this->appendCrc(&_buffer);
    _wire->write(_buffer, packetSize);    
}

//WRITER FOR MASTER (SENDING DATA TO SLAVE)
template <typename T>
void SafeTransfer<T>::sendToSlave(uint8_t address, T data) {
    if (!_wire) return;
    _wire->beginTransmission(address);
    this->sendToMaster(data);
    _wire->endTransmission();
}

//WRITER FOR MASTER (SENDING DATA TO SLAVE)
template <typename T>
void SafeTransfer<T>::sendToSlave(T data) {
    this->sendToSlave(_address, data);
}

#endif //SAFETRANSFER_H