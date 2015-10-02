#include <stdio.h>

#include "defines.h"
#include "beacon.h"
#include "utils.h"

extern Container * beaconContainer;

void groupBeacons() {
    STARTCHRONO
    unsigned int num_beacons = beaconContainer->getNumBeacons();
    unsigned int usedBeacons = 0;
    std::vector<Beacon *> beacons;
    bool first;

    TRACET(DEBUG, DGB, "\ngroupBeacons()-----------\n");

    // For each beacon found
    for (unsigned int currentBeaconIndex = 0; currentBeaconIndex < num_beacons; currentBeaconIndex++) {
        Beacon& currentBeacon = beaconContainer->getBeacon(currentBeaconIndex);

        // If the beacon is already grouped skip it!
        if (currentBeacon.used())
            continue;
        
        currentBeacon.used(true);

        TRACET(DEBUG, DGB, "Using beacon %d, closest beacons: ", currentBeacon.getId());

        // Clear the indexed beacon group
        beacons.clear();

        // Put the first beacon index in the indexed beacon group
        beacons.push_back(&currentBeacon);


        // Unset test flag for the beacons
        for (unsigned int i = currentBeaconIndex + 1; i < num_beacons; i++)
            beaconContainer->getBeacon(i).tested(false);

        // Search closest beacons
        unsigned int notUsedBeacons = beaconContainer->getNumBeacons() - usedBeacons;
        unsigned int numToTest =    notUsedBeacons < MAX_WAYPOINT_BEACONS ? notUsedBeacons : MAX_WAYPOINT_BEACONS;
        
        for (unsigned int i = 1; i < numToTest; i++) {
            float minDistance = INFINITY;
            Beacon * closestBeacon = NULL;

            // Search closest beacon
            for (unsigned int j = currentBeaconIndex + 1; j < num_beacons; j++) {
                Beacon & beacon = beaconContainer->getBeacon(j);
                
                if (beacon.tested() || beacon.used())
                    continue;
                
                float distance = (currentBeacon - beacon).abs();

                if (distance < minDistance) {
                    minDistance = distance;
                    closestBeacon = &beacon;
                }
            }
            if (closestBeacon == NULL) {
                TRACET(DEBUG, DGB, "CRITICAL ERROR: Closest Point not found!\n");
                return;
            }

            // Set the closest beacon as tested and put it into the beacon group
            closestBeacon->tested(true);
            beacons.push_back(closestBeacon);            
            
            TRACET(DEBUG, DGB, "%d ", closestBeacon->getId());
        }
     
        // Search minimum beacon distance
        float minDistance = INFINITY;
        for (unsigned int i = 0; i < numToTest; i++) {
            Beacon & iBeacon = *(beacons[i]);
            for (unsigned int j = i + 1; j < numToTest; j++) {
                Beacon & jBeacon = *(beacons[j]);
                float distance = (iBeacon - jBeacon).abs();
                if (distance < minDistance)
                    minDistance = distance;
            }
        }

        bool discardCurrentBeacon = false;
        for (unsigned int i = 1; i < numToTest; i++) {
            float distance = (*(beacons[0]) - *(beacons[i])).abs();
            TRACET(DEBUG, DGB, "\n Distance %d -> %d: %f (min: %f) (%f, %f, %f)", beacons[0]->getId(), beacons[i]->getId(), distance, minDistance,\
                beacons[i]->x, beacons[i]->y, beacons[i]->z);
            if (distance / minDistance > DISTANCE_FACTOR) {
                if (i == 1)
                    discardCurrentBeacon = true;
                
                TRACET(DEBUG, DGB, " Beacon %d is too far\n", beacons[i]->getId());
                // Clean far beacons
                beacons.resize(i);
                break;
            }
        }
        
        if (discardCurrentBeacon || beacons.size() < 3) {
            TRACET(DEBUG, DGB, " Discarding current beacon\n");
            beacons[0]->discard();
            beacons[0]->used(true);
            usedBeacons++;
        }
        else {
            // EUREKA!!! beacon group found, generate beaconGroup and set the beacons in the group as used
            TRACET(DEBUG, DGB, " Beacon group found (%d beacons)\n", int(beacons.size()));
            BeaconGroup & beaconGroup = beaconContainer->getCleanBeaconGroup();
            beaconGroup.appendBeacons(beacons);
            beaconGroup.computeBeaconGroup();
            usedBeacons += beacons.size(); 
        }
    }
 
    TRACET(DEBUG, DGB, "-%d us---------------------------------------\n", GETCLICK);
}