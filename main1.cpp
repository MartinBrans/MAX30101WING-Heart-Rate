#include <events/mbed_events.h>
#include <mbed.h>
#include "ble/BLE.h"
#include "ble/Gap.h"
#include "ble/services/HeartRateService.h"
#include "max32630fthr.h"
#include "USBSerial.h"
// #include "pretty_printer.h"

// ADC Vref volts (this is fixed for MAX32630)
// #define VREF 1.2f
// #define VMAX 4.200
// #define VMIN 3.000

MAX32630FTHR pegasus(MAX32630FTHR::VIO_3V3);

//Setup I2C bus
I2C i2cBus(P5_7, P6_0);     //Actually i2c2

//Setup PMIC
MAX14690 pmic(&i2cBus);

DigitalOut led1(LED1, 1);

USBSerial pc1; // Direct connection to PC via USB, not MAXDAP 


const static char     DEVICE_NAME[] = "MAX32630FTHR";
static const uint16_t uuid16_list[] = {GattService::UUID_HEART_RATE_SERVICE};

static HeartRateService* heartRateServicePtr;
uint16_t hrValue = 100;

static EventQueue eventQueue(/* event count */ 16 * EVENTS_EVENT_SIZE);

void disconnectionCallback(const Gap::DisconnectionCallbackParams_t *params)
{
    BLE::Instance().gap().startAdvertising();
}

void updateSensorValue() {
    // Do blocking calls or whatever is necessary for sensor polling.
    // In our case, we simply update the HRM measurement.
    hrValue++;

    //  100 <= HRM bps <=175
    if (hrValue == 175) {
        hrValue = 100;
    }

    heartRateServicePtr->updateHeartRate(hrValue);
        
}

void blinkCallback(void)
{
    led1 = !led1; /* Do blinky on LED1 while we're waiting for BLE events */

    BLE &ble = BLE::Instance();
    if (ble.gap().getState().connected) {
        eventQueue.call(updateSensorValue);
    }
}

/**
 * This function is called when the ble initialization process has failled
 */
void onBleInitError(BLE &ble, ble_error_t error)
{
    /* Initialization error handling should go here */
    pc1.printf("BLE INIT ERROR\r\n");
    switch(error) {
        case BLE_ERROR_NONE:
            pc1.printf("BLE_ERROR_NONE: No error");
            break;
        case BLE_ERROR_BUFFER_OVERFLOW:
            pc1.printf("BLE_ERROR_BUFFER_OVERFLOW: The requested action would cause a buffer overflow and has been aborted");
            break;
        case BLE_ERROR_NOT_IMPLEMENTED:
            pc1.printf("BLE_ERROR_NOT_IMPLEMENTED: Requested a feature that isn't yet implement or isn't supported by the target HW");
            break;
        case BLE_ERROR_PARAM_OUT_OF_RANGE:
            pc1.printf("BLE_ERROR_PARAM_OUT_OF_RANGE: One of the supplied parameters is outside the valid range");
            break;
        case BLE_ERROR_INVALID_PARAM:
            pc1.printf("BLE_ERROR_INVALID_PARAM: One of the supplied parameters is invalid");
            break;
        case BLE_STACK_BUSY:
            pc1.printf("BLE_STACK_BUSY: The stack is busy");
            break;
        case BLE_ERROR_INVALID_STATE:
            pc1.printf("BLE_ERROR_INVALID_STATE: Invalid state");
            break;
        case BLE_ERROR_NO_MEM:
            pc1.printf("BLE_ERROR_NO_MEM: Out of Memory");
            break;
        case BLE_ERROR_OPERATION_NOT_PERMITTED:
            pc1.printf("BLE_ERROR_OPERATION_NOT_PERMITTED");
            break;
        case BLE_ERROR_INITIALIZATION_INCOMPLETE:
            pc1.printf("BLE_ERROR_INITIALIZATION_INCOMPLETE");
            break;
        case BLE_ERROR_ALREADY_INITIALIZED:
            pc1.printf("BLE_ERROR_ALREADY_INITIALIZED");
            break;
        case BLE_ERROR_UNSPECIFIED:
            pc1.printf("BLE_ERROR_UNSPECIFIED: Unknown error");
            break;
        case BLE_ERROR_INTERNAL_STACK_FAILURE:
            pc1.printf("BLE_ERROR_INTERNAL_STACK_FAILURE: internal stack faillure");
            break;
        // case BLE_ERROR_NOT_FOUND:
            // pc1.printf("BLE_ERROR_NOT_FOUND");
            // break;
    }

}

/**
 * Callback triggered when the ble initialization process has finished
 */
void bleInitComplete(BLE::InitializationCompleteCallbackContext *params)
{
    BLE&        ble   = params->ble;
    ble_error_t error = params->error;

    if (error != BLE_ERROR_NONE) {
        /* In case of error, forward the error handling to onBleInitError */
        onBleInitError(ble, error);
        return;
    }

    /* Ensure that it is the default instance of BLE */
    if(ble.getInstanceID() != BLE::DEFAULT_INSTANCE) {
        return;
    }

    ble.gap().onDisconnection(disconnectionCallback);

    /* Setup primary service */
    heartRateServicePtr = new HeartRateService(ble, hrValue, HeartRateService::LOCATION_WRIST);

    /* Setup advertising */
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_16BIT_SERVICE_IDS, (uint8_t *) uuid16_list, sizeof(uuid16_list));
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LOCAL_NAME, (uint8_t *) DEVICE_NAME, sizeof(DEVICE_NAME));
    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.gap().setAdvertisingInterval(1000); /* 1000ms */
    ble.gap().startAdvertising();

    pc1.printf("Started Advertising...\r\n");
}

void scheduleBleEventsProcessing(BLE::OnEventsToProcessCallbackContext* context) {
    BLE &ble = BLE::Instance();
    eventQueue.call(Callback<void()>(&ble, &BLE::processEvents));
}

int main()
{
    // pegasus.max14690.monSet(pegasus.max14690.MON_BAT, pegasus.max14690.MON_DIV4);
    // pmic.monCfg = MAX14690::MON_BAT;

    eventQueue.call_every(500, blinkCallback);

    BLE &ble = BLE::Instance();
    ble.onEventsToProcess(scheduleBleEventsProcessing);
    ble.init(bleInitComplete);

    eventQueue.dispatch_forever();

    return 0;
}