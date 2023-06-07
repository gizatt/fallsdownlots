#pragma once

#include <string>
#include "bluefruit.h"
#include <PacketSerial.h>

// BLE Service
BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart * bleuart; // uart over ble

PacketSerial_<COBS> blueart_packet_serial;

const int BLE_MTU = 64;

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(*bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds 
}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);

  // Give connection time to stabilize
  delay(1000);

  // Note: as far as I can tell right now, MTU exchange requests
  // aren't well handled on the PC / Windows Bluetooth driver / Python BLeak
  // side somewhere -- connections with an MTU over 20 will just hang. That's
  // a shame, as an MTU of 20 seems to restrict my total effective bandwidth
  // to be extremely small...
  connection->requestPHY();
  delay(1000);
  connection->requestMtuExchange(BLE_MTU);
  delay(1000);
  connection->requestDataLengthUpdate();
  delay(1000);

  Bluefruit.printInfo();
  
  // TODO(gizatt) See https://github.com/adafruit/Adafruit_nRF52_Arduino/blob/master/libraries/Bluefruit52Lib/examples/Peripheral/throughput/throughput.ino
  // for some potential connection configuration options for possible speed-up.
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);

  bleuart->flush();
  bleuart->flushTXD();
}

bool begin_ble(const std::string& name){
  
    Bluefruit.configCentralConn(247, 6, 10, 10); // bigger-than-normal queue sizes
    Bluefruit.configPrphConn(247, 6, 10, 10);
    //Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
    //Bluefruit.configCentralBandwidth(BANDWIDTH_MAX);
    Bluefruit.begin();

    Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
    Bluefruit.Periph.setConnectCallback(connect_callback);
    Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
    //Bluefruit.configPrphConn(BLE_MTU, BLE_GAP_EVENT_LENGTH_DEFAULT, 10, 10);
    //Bluefruit.configCentralConn(BLE_MTU, BLE_GAP_EVENT_LENGTH_DEFAULT, 10, 10);

    // To be consistent OTA DFU should be added first if it exists
    bledfu.begin();

    // Configure and Start Device Information Service
    bledis.setManufacturer("Adafruit Industries");
    bledis.setModel(name.c_str());
    bledis.begin();

    // Configure and Start BLE Uart Service
     
    bleuart = new BLEUart(BLE_MTU * 2);
    //bleuart->bufferTXD(true);
    bleuart->begin();
    // Set up and start advertising
    startAdv();

    Serial.println("BLE setup done.");

    // Set up PacketSerial uart wrapper.
    blueart_packet_serial.setStream(bleuart);

    return true;
}


// Updates the PacketSerial object only when we
// have an active connection. This will dispatch any callbacks
// you have registered to `blueart_package_serial` if packets
// are received.
void update_ble_uart(){
  // Echo received data
  if (Bluefruit.connected() && bleuart->notifyEnabled())
  {
    blueart_packet_serial.update();
  }
}

// Returns true if data was sent.
bool maybe_send_ble_uart(const uint8_t * buf, int len){
  if (Bluefruit.connected() && bleuart->notifyEnabled()){
    blueart_packet_serial.send(buf, len);
    //bleuart->flushTXD();
    return true;
  } else {
    return false;
  }
}
