#include "hardware_config.h"

bool op_sensor_config(MAX30101 &op_sensor)
{

    //Reset Device
    MAX30101::ModeConfiguration_u modeConfig;
    modeConfig.all = 0;
    modeConfig.bits.reset = 1;
    modeConfig.bits.mode = MAX30101::MultiLedMode;     // Sets SPO2 Mode
    int32_t rc = op_sensor.setModeConfiguration(modeConfig);


    //disable MAX30101 interrupts
    MAX30101::InterruptBitField_u ints;
    if(rc == 0) {
        ints.all = 0;
        ints.bits.a_full = 1;       // Enable FIFO almost full interrupt
        ints.bits.ppg_rdy =1;       //Enables an interrupt when a new sample is ready
        rc = op_sensor.enableInterrupts(ints);
    }

    //configure FIFO
    MAX30101::FIFO_Configuration_u fifoConfig;
    if(rc == 0) {
        fifoConfig.all = 0;
        fifoConfig.bits.fifo_a_full = 10;                            // Max level of 17 samples
        fifoConfig.bits.sample_average = MAX30101::AveragedSamples_0;// Average 0 samples
        fifoConfig.bits.fifo_roll_over_en = 1;
        rc = op_sensor.setFIFOConfiguration(fifoConfig);
    }

    MAX30101::SpO2Configuration_u spo2Config;
    if(rc == 0) {
        spo2Config.all = 0;                                 // clears register
        spo2Config.bits.spo2_adc_range = 1;                 //sets resolution to 4096 nAfs
        spo2Config.bits.spo2_sr = MAX30101::SR_100_Hz;     // SpO2 SR = 100Hz
        spo2Config.bits.led_pw = MAX30101::PW_3;            // 18-bit ADC resolution ~400us
        rc = op_sensor.setSpO2Configuration(spo2Config);
    }

    //Set time slots for LEDS
    MAX30101::ModeControlReg_u multiLED;
    if(rc==0) {
        //sets timing for control register 1
        multiLED.bits.lo_slot=1;
        multiLED.bits.hi_slot=2;
        rc = op_sensor.setMultiLEDModeControl(MAX30101::ModeControlReg1, multiLED);
        if(rc==0) {
            multiLED.bits.lo_slot=3;
            multiLED.bits.hi_slot=0;
            rc = op_sensor.setMultiLEDModeControl(MAX30101::ModeControlReg2, multiLED);
        }
    }

    //Set LED drive currents
    if(rc == 0) {
        // Heart Rate only, 1 LED channel, Pulse amp. = ~7mA
        rc = op_sensor.setLEDPulseAmplitude(MAX30101::LED1_PA, 0x34);
        //To include SPO2, 2 LED channel, Pulse amp. ~7mA
        if(rc==0) {
            rc = op_sensor.setLEDPulseAmplitude(MAX30101::LED2_PA, 0x34);
        }
        if(rc==0) {
            rc = op_sensor.setLEDPulseAmplitude(MAX30101::LED3_PA, 0xFF);
        }

    }

    //Set operating mode
    modeConfig.all = 0;
    if(rc == 0) {
        modeConfig.bits.mode = MAX30101::MultiLedMode;     // Sets multiLED mode
        rc = op_sensor.setModeConfiguration(modeConfig);
    }


    return rc;
}

void pmic_config(I2C & i2c_bus, DigitalOut & pmic_en)
{

    const uint8_t PMIC_ADRS = 0x54;
    const uint8_t BBB_EXTRA_ADRS = 0x1C;
    const uint8_t BOOST_VOLTAGE = 0x05;

    char data_buff[] = {BBB_EXTRA_ADRS, 0x40};    //BBBExtra register address
    //and data to enable passive
    //pull down.
    i2c_bus.write(PMIC_ADRS, data_buff,2);        //write to BBBExtra register

    data_buff[0] = BOOST_VOLTAGE;
    data_buff[1] = 0x08;                          //Boost voltage configuration
    //register followed by data
    //to set voltage to 4.5V 1f
    pmic_en = 0;                                  //disables VLED 08
    i2c_bus.write(PMIC_ADRS, data_buff,2);        //write to BBBExtra register
    pmic_en = 1;                                  //enables VLED
}
