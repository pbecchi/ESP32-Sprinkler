
#include <Arduino.h>
#include "BLEDevice.h"
//#include "BLEScan.h"
#include "ble.h"
// The remote service we wish to connect to.
static BLEUUID serviceUUID("6e400001-b5a3-f393-e0a9-e50e24dcca9e");//91bad492-b950-4226-aa2b-4ede9fa42f59
																   // The characteristic of the remote service we are interested in.
static BLEUUID    charUUIDTX("6e400002-b5a3-f393-e0a9-e50e24dcca9e");//BLUEART_UUID_CHR_RXD// 0d563a58 - 196a - 48ce - ace2 - dfec78acc814");
static BLEUUID    charUUIDRX("6e400003-b5a3-f393-e0a9-e50e24dcca9e");//BLUEART_UUID_CHR_RXD// 0d563a58 - 196a - 48ce - ace2 - dfec78acc814");

static BLEAddress *pServerAddress;
static boolean doConnect = false;
static boolean connected = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLERemoteCharacteristic* pRemoteCharacteristicRX;

const uint8_t x[] = { 0,0 };
const uint8_t v[] = { 1,0 };
bool onoff = false;

static void notifyCallback
(BLERemoteCharacteristic* pBLERemoteCharacteristic,
	uint8_t* pData,
	size_t length,
	bool isNotify) {

	pRemoteCharacteristicRX->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)x, 2, true);

	//Serial.print("Notify callback for characteristic RX ");
	// Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
	// Serial.print(" of data length ");
	// Serial.println(length);
	//std::string value = pRemoteCharacteristicRX->readValue();
	Serial.print("RX: ");
	Serial.print((char *)pData);
	Serial.println();
	pRemoteCharacteristicRX->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)v, 2, true);


}

bool connectToServer(BLEAddress pAddress) {
	Serial.print("Forming a connection to ");
	Serial.println(pAddress.toString().c_str());

	BLEClient*  pClient = BLEDevice::createClient();
	Serial.println(" - Created client");

	// Connect to the remove BLE Server.
	pClient->connect(pAddress);
	Serial.println(" - Connected to server");

	// Obtain a reference to the service we are after in the remote BLE server.
	BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
	if (pRemoteService == nullptr) {
		Serial.print("Failed to find our service UUID: ");
		Serial.println(serviceUUID.toString().c_str());
		return false;
	}
	Serial.println(" - Found our service");


	// Obtain a reference to the characteristic in the service of the remote BLE server.
	pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUIDTX);
	if (pRemoteCharacteristic == nullptr) {
		Serial.print("Failed to find our characteristic TX UUID: ");
		Serial.println(charUUIDTX.toString().c_str());
		return false;
	}
	pRemoteCharacteristicRX = pRemoteService->getCharacteristic(charUUIDRX);
	if (pRemoteCharacteristicRX == nullptr) {
		Serial.print("Failed to find our characteristic RX UUID: ");
		Serial.println(charUUIDRX.toString().c_str());
		return false;
	}

	Serial.println(" - Found our characteristicS");

	// Read the value of the characteristic.
	//   std::string value = pRemoteCharacteristicRX->readValue();
	//  Serial.print("The RX characteristic value was: ");
	// Serial.println(value.c_str());
	//pRemoteCharacteristicRX->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)x, 2, true);


	pRemoteCharacteristicRX->registerForNotify(notifyCallback);

}
/**
* Scan for BLE servers and find the first one that advertises the service we are looking for.
*/
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
	/**
	* Called for each advertising BLE server.
	*/
	void onResult(BLEAdvertisedDevice advertisedDevice) {
		Serial.print("BLE Advertised Device found: ");
		Serial.println(advertisedDevice.toString().c_str());

		// We have found a device, let us now see if it contains the service we are looking for.
		if (advertisedDevice.haveServiceUUID() && advertisedDevice.getServiceUUID().equals(serviceUUID)) {

			// 
			Serial.print("Found our device!  address: ");
			advertisedDevice.getScan()->stop();

			pServerAddress = new BLEAddress(advertisedDevice.getAddress());
			doConnect = true;

		} // Found our server
	} // onResult
}; // MyAdvertisedDeviceCallbacks


void BLE::setup() {
	Serial.println("Starting Arduino BLE Client application...");
	BLEDevice::init("");

	// Retrieve a Scanner and set the callback we want to use to be informed when we
	// have detected a new device.  Specify that we want active scanning and start the
	// scan to run for 30 seconds.
	BLEScan* pBLEScan = BLEDevice::getScan();
	pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
	pBLEScan->setActiveScan(true);
	pBLEScan->start(30);
} // End of setup.

  // This is the Arduino main loop function.
void BLE::send(String newValue) {

	// If the flag "doConnect" is true then we have scanned for and found the desired
	// BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
	// connected we set the connected flag to be true.
	if (doConnect == true) {
		if (connectToServer(*pServerAddress)) {
			Serial.println("We are now connected to the BLE Server.");
			connected = true;
		} else {
			Serial.println("We have failed to connect to the server; there is nothin more we will do.");
		}
		doConnect = false;
	}

	// If we are connected to a peer BLE Server, update the characteristic each time we are reached
	// with the current time since boot.
	if (connected) {
		pRemoteCharacteristicRX->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)v, 2, true);

		//		String newValue = "Time since boot: " + String(millis() / 10);
		Serial.println("Setting new characteristic value to \"" + newValue + "\"");

		// Set the characteristic's value to be the array of bytes that is actually a string.
		for (byte i = 0; i < newValue.length(); i += 19) {
			Serial.println(newValue.c_str() + i);
			pRemoteCharacteristic->writeValue(newValue.c_str() + i, (newValue.length() - i > 19) ? 19 : newValue.length() - i);
		}
		//	if (onoff)
		//		Serial.println(pRemoteCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->readValue().c_str());
		//	else
		//  pRemoteCharacteristicRX->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)x, 2, true);
		//	delay(500);
		/*	if (pRemoteCharacteristicRX->canRead()) {
		std::string value = pRemoteCharacteristicRX->readValue();
		Serial.print("The  characteristic value was: ");
		for (byte i = 0; i < value.length(); i++) Serial.println(value[i], HEX);
		*/
		//}
	}
}
BLE::BLE() {
}


BLE::~BLE() {
}
