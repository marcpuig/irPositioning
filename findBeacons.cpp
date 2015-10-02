#include <stdio.h>

#include "beacon.h"
#include "utils.h"

extern float roll, pitch;
extern unsigned char imageData[PIXELS_X * PIXELS_Y];
extern Container * beaconContainer;
extern pthread_mutex_t mutexImageData;
extern uint32_t photogram;


inline bool insideImage(unsigned int x, unsigned int y) {
    return (x >= 0 && y >= 0 && x < PIXELS_X && y < PIXELS_Y);
}

inline void checkBeacon(unsigned int x, unsigned int y) {
    imageData[y*PIXELS_X + x] = 0;
}

inline bool thresholdTest(unsigned int x, unsigned int y) {
    if (insideImage(x, y)) {
        bool test = imageData[y*PIXELS_X + x] > THRESHOLD;
        checkBeacon(x, y);
        return test;
    }
    return false;
}

void fillBeacon(int x, int y, int &numPixels, int &xSum, int &ySum) {
    xSum += x;
    ySum += y;
    numPixels++;
    
    if (numPixels > MAX_BEACON_PIXELS)
        throw 1;

    if (thresholdTest(x-1, y)) fillBeacon(x-1, y, numPixels, xSum, ySum);
    if (thresholdTest(x-1, y+1)) fillBeacon(x-1, y+1, numPixels, xSum, ySum);
    if (thresholdTest(x, y+1)) fillBeacon(x, y+1, numPixels, xSum, ySum);
    if (thresholdTest(x+1, y+1)) fillBeacon(x+1, y+1, numPixels, xSum, ySum);
    if (thresholdTest(x+1, y)) fillBeacon(x+1, y, numPixels, xSum, ySum);
    if (thresholdTest(x+1, y-1)) fillBeacon(x+1, y-1, numPixels, xSum, ySum);
    if (thresholdTest(x, y-1)) fillBeacon(x, y-1, numPixels, xSum, ySum);
    if (thresholdTest(x-1, y-1)) fillBeacon(x-1, y-1, numPixels, xSum, ySum);
}

void findBeacons() {
    TRACET(DEBUG, DFB, "\nfindBeacons()----------\n");
    
    int numPixels = 0;
    int xSum = 0;
    int ySum = 0;
    
    struct tPossibleBeacon { unsigned int x, y, size; } possibleBeacons[MAX_POSSIBLE_BEACONS];
    int numPossibleBeacons = 0;
    int totalPixels = 0;
    
    pthread_mutex_lock(&mutexImageData);
    STARTCHRONO
#ifdef THUMBNAILS
    write_png_file(photogram);
#endif
    for (int y=0; y < PIXELS_Y; y+=2) {
        for (int x=0; x < PIXELS_X; x+=2) {
            if (thresholdTest(x, y)) {
                if (numPossibleBeacons >= MAX_POSSIBLE_BEACONS) {
                    TRACET(DEBUG, DFB, "WARNING! Too many possible beacons found.\n");
                    pthread_mutex_unlock(&mutexImageData);
                    return;
                }

                try {
                    fillBeacon(x, y, numPixels, xSum, ySum);
                }
                catch (int e) {
                    TRACET(DEBUG, DFB, "Found a beacon too big to process.\n");
                    pthread_mutex_unlock(&mutexImageData);
                    return;
                }

                if (numPixels >= MIN_BEACON_PIXELS && numPixels <= MAX_BEACON_PIXELS) {                        
                    // Get point centre coordinates
                    int x = xSum/numPixels;
                    int y = ySum/numPixels;
                    
                    totalPixels += numPixels;
                    
                    // Create possible beacon
                    tPossibleBeacon &possibleBeacon = possibleBeacons[numPossibleBeacons++];
                    
                    possibleBeacon.x = x;
                    possibleBeacon.y = y;
                    possibleBeacon.size = numPixels; 
                }
                else
                    TRACET(DEBUG, DFB, "Noise found! %d (%d,%d)\n", numPixels, xSum/numPixels, ySum/numPixels);

                numPixels = xSum = ySum = 0;
            }
        }
    }
    pthread_mutex_unlock(&mutexImageData);
    
    // beacon size check
    int meanSize = totalPixels / numPossibleBeacons;
    int numBeacons = 0;
    for (int i=0; i < numPossibleBeacons; i++) {
        tPossibleBeacon &possibleBeacon = possibleBeacons[i];
        
        float factor; 
        if (possibleBeacon.size >= meanSize)
            factor = possibleBeacon.size / meanSize;
        else
            factor = meanSize / possibleBeacon.size;
        
        
        if (factor <= MAX_BEACON_DIFF_FACTOR) {
            if (++numBeacons <= MAX_BEACONS) {
                Beacon & beacon = beaconContainer->getCleanBeacon();
                beacon.loadBeacon(possibleBeacon.x, possibleBeacon.y, roll / RAD2DEG, pitch / RAD2DEG, possibleBeacon.size);
                TRACET(DEBUG, DFB, "Beacon found! id: %d, size: %d, position: (%d, %d) (%f, %f)\n", beacon.getId(), possibleBeacon.size, possibleBeacon.x, possibleBeacon.y, beacon.x, beacon.y);
            }
            else {
                TRACET(DEBUG, DFB, "WARNING! Too many beacons found!\n");
                break;
            }
        }
        else
            TRACET(DEBUG, DFB, "Beacon not used, size: %d, position: (%d,%d)\n", possibleBeacon.size, possibleBeacon.x, possibleBeacon.y); 
    }
    
    //return beacons;
    TRACET(DEBUG, DFB, "-%d us---------------------------------------\n", GETCLICK);
}