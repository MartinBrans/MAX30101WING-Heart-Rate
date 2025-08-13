// Inspired from https://github.com/paulvangentcom/heartrate_analysis_Arduino/tree/master
// Major change: adapted the ADC resolution from Arduino's 10 bits to MAX30101's 18 bits
/*
 * Arduino Heart Rate Analysis Toolbox - Peak Finder ARM
 *      Copyright (C) 2018 Paul van Gent
 *      
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License V3 as published by
 * the Free Software Foundation. The program is free for any commercial and
 * non-commercial usage and adaptation, granted you give the recipients 
 * of your code the same open-source rights and license.
 * 
 * You can read the full license granted to you here:
 * https://www.gnu.org/licenses/gpl-3.0.en.html
 * 
 * Please add the following citation to any work utilising one or more of the
 * implementation from this project:
 * 
 * <Add JORS paper reference once published>
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "algo_HR.h"
#include <cstdint>

// Uncomment definitions corresponding to ADC resolution
 // 10 bits (Arduino)
// #define ADC_MAX (1023)
// #define CLIP_LIM (3)

// 18 bits (MAX30101)
#define ADC_MAX (262143)
#define CLIP_LIM (1000)

// -------------------- User Settable Variables --------------------
const int16_t sample_rate = 100;
double max_bpm = 200; //The max BPM to be expected, used in error detection (default 180)
double min_bpm = 40; //The min BPM to be expected, used in error detection (default 45)

// -------------------- Non-Settable Variables --------------------
// Seriously, don't touch
int32_t largestVal = 0;
int16_t largestValPos = 0;
int clippingcount = 0;
int8_t clipFlag = 0;
int clipStart = 0;
int8_t clipEnd = 0;
int32_t lastVal = 0;
int16_t max_RR = (60.0 / min_bpm) * 1000.0;
int16_t min_RR = (60.0 / max_bpm) * 1000.0;
const int16_t ROIRange = sample_rate * 0.75;
int16_t RR_multiplier = 1000 / sample_rate;
int8_t firstCall = 1;

int32_t cbuf[32];

// -------------------- Define Data Structs --------------------
struct workingDataContainer
{
    long absoluteCount;// = 0;
    
    //buffers
    int32_t curVal;// = 0;
    int16_t datalen;// = sample_rate
    int32_t hrData[sample_rate];// = {0};
    int16_t buffPos;// = 0;
    
    //movavg variables
    int16_t windowSize;// = sample_rate * 0.6; //windowSize in samples
    int32_t hrMovAvg[sample_rate];// = {0};
    int16_t oldestValuePos;// = 1;
    long movAvgSum;// = 0;
    int32_t rangeLow;// = 0;
    int32_t rangeLowNext;// = ADC_MAX+1;
    int32_t rangeHigh;// = ADC_MAX;
    int32_t rangeHighNext;// = 1;
    int16_t rangeCounter;// = 0;
    int16_t rangeRange;// = 2 * sample_rate;

    //peak variables
    int32_t ROI[ROIRange];// = {0};
    //int16_t ROI_interp[40] = {0};
    int16_t ROIPos;// = 0;
    int16_t peakFlag;// = 0;
    int8_t ROI_overflow;// = 0;
    long curPeak;// = 0;
    long curPeakEnd;// = 0;
    long lastPeak;// = 0;

    //peak validation variables
    int8_t initFlag;// = 0; //use for initialisation
    int16_t lastRR;// = 0;
    int16_t curRR;// = 0;
    int16_t recent_RR[20];// = {0};
    int16_t RR_mean;// = 0;
    int16_t RR_sum;// = 0;
    int16_t RR_pos;// = 0;
    int16_t lower_threshold;// = 0;
    int16_t upper_threshold;// = 1;
};

static const int32_t b[3] = {19718,0,-19718};
static const int32_t a[3] = {262144,-482455,222707};
static int32_t x1=0,x2=0;
static int32_t y1=0,y2=0;
static int32_t z1=0,z2=0;

int64_t mul32(int32_t a, int32_t b){
    return ((int64_t)a)*((int64_t) b);
}

//  Bandpass 2nd order FIR Filter
int32_t bp_fir_filter(int32_t din)
{
    int64_t y = mul32(b[1],x1) - mul32(a[1],y1) + mul32(b[0],din)
              + mul32(b[2],x2) - mul32(a[2],y2);
    int32_t dinter = (int32_t) (y >> 18); // Equivalent to dividing by a[0] (= 2^18)
    // Apply a second time to double the order
    y = mul32(b[1],y1) - mul32(a[1],z1) + mul32(b[0],dinter)
      + mul32(b[2],y2) - mul32(a[2],z2);
    int32_t dout = (int32_t) (y >> 18); // Equivalent to dividing by a[0] (= 2^18)
    x2 = x1; y2 = y1; z2 = z1;
    x1 = din; y1 = dinter; z1 = dout;
    return dout; 
}


struct workingDataContainer workingData;

// -------------------- Define Helper Functions --------------------
int findMax(int32_t arr[], int16_t arrLen, struct workingDataContainer &workingData)
{
    largestVal = 0;
    largestValPos = 0;
    clippingcount = 0;
    clipFlag = 0;
    clipStart = 0;
    clipEnd = 0;
    lastVal = 0;
    
    for(int i = 0; i<arrLen; i++)
    {
        if((abs((double)(lastVal - arr[i])) <= CLIP_LIM) && (arr[i] > ADC_MAX-CLIP_LIM))
        {
            if(clipFlag == 0)
            {
                clipFlag = 1;
                clipStart = i;
            } else {
                clippingcount++;
            }
        } else {
            if(clipFlag == 1)
            {
                clipEnd = i;
            }
        }
        
        lastVal = arr[i];        
        
        if(arr[i] > largestVal) 
        {
            largestVal = arr[i];
            largestValPos = i;
        }

        /*if(clippingcount > 3) 
        //preliminary clipping correction
        //disabled: doesn't work properly
        {
            largestValPos = (clipStart + (clipEnd - clipStart)) / 2;
        }*/
    }
        
    return workingData.curPeakEnd - (arrLen - largestValPos);
}

void getMeanRR(struct workingDataContainer &workingData)
{ //returns the mean of the RR_list array.
    workingData.RR_sum = 0;
    for(int i = 0; i<20; i++)
    {
        workingData.RR_sum += workingData.recent_RR[i];
    }
    workingData.RR_mean = workingData.RR_sum / 20;
}

long mapl(long x, long in_min, long in_max)
{
    return (x - in_min) * ADC_MAX / (in_max - in_min) + 1;
}

void establish_range(struct workingDataContainer &workingData)
{
    if(workingData.rangeCounter <= workingData.rangeRange)
    {
        //update upcoming ranges
        if(workingData.rangeLowNext > workingData.curVal) workingData.rangeLowNext = workingData.curVal;
        if(workingData.rangeHighNext < workingData.curVal) workingData.rangeHighNext = workingData.curVal;
        workingData.rangeCounter++;
    } else {
        //set range, minimum range should be bigger than 50
        //otherwise set to default of (0, 1024)
        if((workingData.rangeHighNext - workingData.rangeLowNext) > 50)
        {
            //update range
            workingData.rangeLow = workingData.rangeLowNext;
            workingData.rangeHigh = workingData.rangeHighNext;
            workingData.rangeLowNext = ADC_MAX+1;
            workingData.rangeHighNext = 1;
        } else {
            //reset range to default
            workingData.rangeLow = 0;
            workingData.rangeHigh = ADC_MAX+1;
        }
        workingData.rangeCounter = 0;
    }
}

// -------------------- Define Main Functions --------------------
void movingAvg(struct workingDataContainer &workingData)
{

    establish_range(workingData);
    workingData.curVal = mapl(workingData.curVal, workingData.rangeLow, workingData.rangeHigh);
    if(workingData.curVal < 0) workingData.curVal = 0;
    //if(workingData.curVal > ADC_MAX) workingData.curVal = ADC_MAX;
    
    workingData.movAvgSum += workingData.curVal; //update total sum by adding recent value
    workingData.movAvgSum -= workingData.hrData[workingData.oldestValuePos]; //as well as subtracting oldest value
    workingData.hrMovAvg[workingData.buffPos] = workingData.movAvgSum / workingData.windowSize; //compute moving average
    workingData.hrData[workingData.buffPos] = workingData.curVal; //store sensor value
    int32_t val;
}

void updatePeak(struct workingDataContainer &workingData)
{
    //updates peak positions, adds RR-interval to recent intervals
    workingData.recent_RR[workingData.RR_pos] = workingData.curRR;
    workingData.RR_pos++;
    
    if(workingData.RR_pos >= 20)
    {
        workingData.RR_pos = 0;
        workingData.initFlag = 1;
    }

//     if(!report_hr)
//     {
//         Serial.print(workingData.curRR);
//         Serial.print(",");
//         Serial.println(workingData.curPeak);
//     } else {
//         Serial.print(workingData.curRR);
//         Serial.print(",");
//         Serial.print(workingData.curPeak);
//     }
}

void validatePeak(struct workingDataContainer &workingData)
{
    //validate peaks by thresholding, only update if within thresholded band
    if(workingData.initFlag != 0)
    {
        getMeanRR(workingData);
        // if((workingData.RR_mean* 0.3) <= 300){
        //     workingData.lower_threshold = workingData.RR_mean - 300;
        //     workingData.upper_threshold = workingData.RR_mean + 300;
        // } else{
        //     workingData.lower_threshold = workingData.RR_mean - (0.3 * workingData.RR_mean);
        //     workingData.upper_threshold = workingData.RR_mean + (0.3 * workingData.RR_mean);
        // }
        
        if(//workingData.curRR < workingData.upper_threshold &&
        //workingData.curRR > workingData.lower_threshold &&
        abs((double)(workingData.curRR - workingData.lastRR)) < 500)
        {
            updatePeak(workingData);
        // }    else {
        //     if(report_hr) Serial.print(",");
        }
    }
}

void checkForPeak(struct workingDataContainer &workingData)
{
    if(workingData.hrData[workingData.buffPos] >= workingData.hrMovAvg[workingData.buffPos])
    {
        if(workingData.ROIPos >= ROIRange){
            workingData.ROI_overflow = 1;
        //     if(report_hr) Serial.print(",");
            return;
        } else {
            workingData.peakFlag = 1;
            workingData.ROI[workingData.ROIPos] = workingData.curVal;
            workingData.ROIPos++;
            workingData.ROI_overflow = 0;
        //     if(report_hr) Serial.print(",");
            return;
        }
    }
    
    if((workingData.hrData[workingData.buffPos] <= workingData.hrMovAvg[workingData.buffPos])
    && (workingData.peakFlag == 1))
    {
        if(workingData.ROI_overflow == 1)
        {
            workingData.ROI_overflow = 0;
        } else {
            //solve for peak
            workingData.lastRR = workingData.curRR;
            workingData.curPeakEnd = workingData.absoluteCount;
            workingData.lastPeak = workingData.curPeak;
            workingData.curPeak = findMax(workingData.ROI, workingData.ROIPos, workingData);
            workingData.curRR = (workingData.curPeak - workingData.lastPeak) * RR_multiplier;
            //Serial.println(workingData.curPeak);
            //add peak to struct
        }
        workingData.peakFlag = 0;
        workingData.ROIPos = 0;

        //error detection run, timed at ????????????????????

        if(workingData.curRR > max_RR || workingData.curRR < min_RR)
        {
        //     if(report_hr) Serial.print(",");
            return; //break if outside of BPM bounds anyway
        } else if(workingData.initFlag != 0)
        {
            validatePeak(workingData);
        } else {
            updatePeak(workingData);
        }
    } //else if (report_hr) Serial.print(",");
}


int16_t HRFunc(int32_t sample)
{ 
    // 

    //report the absolute count
//     if(report_hr)
//     {
//         Serial.print(workingData.absoluteCount);
//         Serial.print(",");
//     }

    if (firstCall) {
        firstCall = 0;
        // Reset all

        for (int i = 0; i < 32; i++){cbuf[i] = 0;}
        
        workingData.absoluteCount = 0;
    
        //buffers
        workingData.curVal = 0;
        workingData.datalen = sample_rate;
        for (int i = 0; i < sample_rate; i++){workingData.hrData[i] = 0;}
        workingData.buffPos = 0;
        
        //movavg variables
        workingData.windowSize = sample_rate * 0.6; //windowSize in samples
        for (int i = 0; i < sample_rate; i++){workingData.hrMovAvg[i] = 0;}
        workingData.oldestValuePos = 1;
        workingData.movAvgSum = 0;
        workingData.rangeLow = 0;
        workingData.rangeLowNext = ADC_MAX+1;
        workingData.rangeHigh = ADC_MAX;
        workingData.rangeHighNext = 1;
        workingData.rangeCounter = 0;
        workingData.rangeRange = 2 * sample_rate;

        //peak variables
        for (int i = 0; i < ROIRange; i++){workingData.ROI[i] = 0;}
        //int16_t ROI_interp[40] = {0};
        workingData.ROIPos = 0;
        workingData.peakFlag = 0;
        workingData.ROI_overflow = 0;
        workingData.curPeak = 0;
        workingData.curPeakEnd = 0;
        workingData.lastPeak = 0;

        //peak validation variables
        workingData.initFlag = 1; //use for initialisation
        workingData.lastRR = 0;
        workingData.curRR = 0;
        for (int i = 0; i < 20; i++) {workingData.recent_RR[i] = 0;} // Reset only once
        workingData.RR_mean = 0;
        workingData.RR_sum = 0;
        workingData.RR_pos = 0;
        workingData.lower_threshold = 0;
        workingData.upper_threshold = 1;
    }

    // workingData.curVal = sample;
    workingData.curVal = bp_fir_filter(sample);

    //read the sensor value
    movingAvg(workingData);

    //check if peak is present, update variables if so
    checkForPeak(workingData);
    
    //report raw signal if requested
//     if(report_hr) 
//     {
//         Serial.print(",");
//         Serial.print(workingData.hrMovAvg[workingData.buffPos]);
//         Serial.print(",");
//         Serial.println(workingData.curVal);
//     }
    //update buffer position pointers
    workingData.buffPos++;
    workingData.oldestValuePos++;

    //reset buffer pointers if at end of buffer
    if(workingData.buffPos >= sample_rate) workingData.buffPos = 0;
    if(workingData.oldestValuePos >= sample_rate) workingData.oldestValuePos = 0;

    //increment total sample counter (used for RR determination)
    workingData.absoluteCount++;

    return (uint16_t) (60000/workingData.RR_mean); // seconds per minute * milliseconds per second / milliseconds per beat

}
