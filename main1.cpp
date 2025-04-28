#include <events/mbed_events.h>
#include <mbed.h>
#include "ble/BLE.h"
#include "ble/Gap.h"
#include "ble/services/HeartRateService.h"
// #include "pretty_printer.h"
#include "algo_HR.h"
#include "hardware_config.h"
#include "USBSerial.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define MY_BUF_SIZE (1024) // Size of the data buffers to compute HR
#define MY_WIN_SIZE (800) // Number of samples required to compute HR
#define MY_DEL_SIZE (200) // Number of samples thrown when buffers are almost full


//variable for the algorithm
uint16_t sampleRate =100;
uint16_t compSpO2=1;
int16_t ir_ac_comp =0;
int16_t red_ac_comp=0;
int16_t green_ac_comp=0;
int16_t ir_ac_mag=0;
int16_t red_ac_mag=0;
int16_t green_ac_mag=0;
uint16_t HRbpm2=0;
uint16_t SpO2B=0;
uint16_t DRdy=0;

//declare large variables outside of main
uint32_t redData[MY_BUF_SIZE];//set array to max fifo size
uint32_t irData[MY_BUF_SIZE];//set array to max fifo size
uint32_t greenData[MY_BUF_SIZE];//set array to max fifo size

uint8_t fifoData[MAX30101::MAX_FIFO_BYTES];
uint16_t idx, readBytes;
uint16_t HRTemp;
uint16_t spo2Temp;

int r=0; //counter for redData position
int ir=0; //counter for irData position
int g =0; //counter for greenData position
int c=0; //counter to print values

int numFails = 0; // After 5 failure to detect a HR, assume its gone (show 9999)


// Setup board, select 3.3V logic
MAX32630FTHR pegasus(MAX32630FTHR::VIO_3V3);

// Setup I2C bus
I2C i2cBusSensor(I2C1_SDA, I2C1_SCL);         // I2C bus, P3_4 = SDA, P3_5 = SCL

// Setup sensor
MAX30101 op_sensor(i2cBusSensor);

DigitalOut rLED(LED1, 0); // Debug led (red)
// DigitalOut gLED(LED2, 0); // Debug led (green)
DigitalOut bLED(LED3, 0); // Debug led (blue)

USBSerial pc1; // Direct connection to PC via USB, not MAXDAP 


const static char     DEVICE_NAME[] = "MAX32630FTHR";
static const uint16_t uuid16_list[] = {GattService::UUID_HEART_RATE_SERVICE};

static HeartRateService* heartRateServicePtr;

static EventQueue eventQueue(/* event count */ 16 * EVENTS_EVENT_SIZE);

void disconnectionCallback(const Gap::DisconnectionCallbackParams_t *params)
{
    BLE::Instance().gap().startAdvertising();
}


void updateBLEValue() {
    //If the above algorithm returns a valid heart rate on the last sample, it is printed
    if(DRdy==1) {
        // pc.printf("Heart Rate = %i\r\n",HRbpm2);
        // pc.printf("SPO2 = %i\r\n",SpO2B);
        // pc1.printf("\r\n");
        pc1.printf("Heart Rate = %i\r\n",HRbpm2);
        pc1.printf("SPO2 = %i\r\n",SpO2B);
        heartRateServicePtr->updateHeartRate(HRbpm2);
        numFails = 0;
    }
    else if (HRTemp!=0)//if a valid heart was calculated at all, it is printed
    {
        // pc.printf("Heart Rate = %i\r\n",HRTemp);
        // pc.printf("SPO2 = %i\r\n",spo2Temp);
        // pc1.printf("\r\n");
        pc1.printf("Heart Rate = %i\r\n",HRTemp);
        pc1.printf("SPO2 = %i\r\n",spo2Temp);
        heartRateServicePtr->updateHeartRate(HRTemp);
        numFails = 0;
    }
    else
    {
        // pc.printf("Calculation failed...waiting for more samples...\r\n");
        // pc.printf("Please keep your finger on the MAX30101 sensor with minimal movement.\r\n"); 
        // pc1.printf("\r\n");
        pc1.printf("Calculation failed...waiting for more samples...\r\n");
        pc1.printf("Please keep your finger on the MAX30101 sensor with minimal movement.\r\n"); 
        numFails++;
        if (numFails >= 5) heartRateServicePtr->updateHeartRate(9999);
    }
    
}


void updateValues() {

    bLED = !bLED; /* Do blinky on LED3 while we're waiting for BLE events */
    if (r>=MY_WIN_SIZE && ir>=MY_WIN_SIZE && g>=MY_WIN_SIZE) {

        BLE &ble = BLE::Instance();
        if (ble.gap().getState().connected) {

            //runs the heart rate and SpO2 algorithm
            for(c=0, HRTemp = 0; c<r; c++) {

                HRSpO2Func(irData[c], redData[c],greenData[c], c,sampleRate, compSpO2,
                            &ir_ac_comp,&red_ac_comp, &green_ac_comp, &ir_ac_mag,&red_ac_mag,
                            &green_ac_mag, &HRbpm2,&SpO2B,&DRdy);
                if(DRdy)
                {
                    HRTemp = HRbpm2;
                    spo2Temp = SpO2B;    
                }

            }
        }

        //dump the first MY_DEL_SIZE samples after calculation
        for(c=MY_DEL_SIZE; c<MY_BUF_SIZE; c++){
            redData[c-MY_DEL_SIZE]=redData[c];
            irData[c-MY_DEL_SIZE]=irData[c];
            greenData[c-MY_DEL_SIZE] = greenData[c];
        }
        //reset counters
        r-=MY_DEL_SIZE;
        ir-=MY_DEL_SIZE;
        g-=MY_DEL_SIZE;
    }


}

void updateValuesCallback() {
    // Always Compute HR, otherwise buffers overflow
    eventQueue.call(updateValues);

    // Update the transmitted value if BLE connected
    BLE &ble = BLE::Instance();
    if (ble.gap().getState().connected) {
        eventQueue.call(updateBLEValue);
    }
}


void sensorPoll()
{
    int rc = op_sensor.readFIFO(MAX30101::ThreeLedChannels, fifoData, readBytes);

    if(rc == 0) {
        // pc1.printf("%i ", readBytes);
        // Convert read bytes into samples
        for (idx = 0; idx < readBytes; idx+=9) {
            if (r >= MY_BUF_SIZE || ir >= MY_BUF_SIZE || g >= MY_BUF_SIZE) {
                // pc.printf("Overflow!");
                pc1.printf("Overflow!\r\n");
                rLED.write(1);
                break;
            }
            redData[r++] = ((fifoData[idx] << 16) | (fifoData[idx + 1] << 8) | (fifoData[idx + 2])) & 0x03FFFF;

            irData[ir++] = ((fifoData[idx + 3] << 16) | (fifoData[idx + 4] << 8) | (fifoData[idx + 5])) & 0x03FFFF;

            greenData[g++] = ((fifoData[idx + 6] << 16) | (fifoData[idx + 7] << 8) | (fifoData[idx + 8])) & 0x03FFFF;
            // pc.printf("%i,%i,%i\r\n",redData[r-1],irData[ir-1],greenData[g-1]);
        }
    } else {
        pc1.printf("Sensor read error\r\n");
        rLED.write(1);
    }
}

void sensorPollCallback(void) {
    eventQueue.call(sensorPoll);
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
        // case BLE_ERROR_NOT_FOUND: // Later Mbed OS versions
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
    heartRateServicePtr = new HeartRateService(ble, 9999, HeartRateService::LOCATION_WRIST);

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
    Thread::wait(1.0); // Initial delay to give USBSerial time to detect (otherwise first messages get lost)

    // Serial pc(USBTX, USBRX);            // Use USB debug probe for serial link (DAPLINK, through MAXDAP)
    // pc.baud(115200);                    // Baud rate = 115200

    // Setup PMIC on sensor board (for VLED supply)
    DigitalOut VLED_EN(P3_3,0);                //Enable for VLEDs
    pmic_config(i2cBusSensor, VLED_EN);

    rLED.write(0);
    
    int rc = op_sensor_config(op_sensor);   // Config sensor, return 0 on success
    if (rc) {
        while(1) {
            rLED = !rLED;
            pc1.printf("Something went wrong when configuring the sensor\r\n");
            Thread::wait(2.0);
        }
    }

    // // Monitor battery (from battery level example code, not used here anymore)
    // // Setup I2C bus
    // I2C i2cBusBattery(P5_7, P6_0);     //Actually i2c2
    // // Setup PMIC on FTHR board
    // MAX14690 pmic(&i2cBusBattery);
    // pegasus.max14690.monSet(pegasus.max14690.MON_BAT, pegasus.max14690.MON_DIV4);
    // pmic.monCfg = MAX14690::MON_BAT;

    // pc.printf("Starting Program...Please wait a few seconds while data is being collected.\r\n");
    pc1.printf("Starting Program...Please wait a few seconds while data is being collected.\r\n");

    eventQueue.call_every(180, sensorPollCallback); // Poll the sensor every 0.18s to empty sensor FIFO queue
    eventQueue.call_every(1800, updateValuesCallback); // Compute new value for HR and update on BLE (if connected) every ~2s

    BLE &ble = BLE::Instance();
    ble.onEventsToProcess(scheduleBleEventsProcessing);
    ble.init(bleInitComplete);

    eventQueue.dispatch_forever();

    return 0;
}
