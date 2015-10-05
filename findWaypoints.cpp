#include <stdio.h>

#include "beacon.h"
#include "utils.h"

extern Container nominalContainer;
extern Container * beaconContainer;

// Search waypoints that fits each beacon group and if so obtain the position and orientation
void findWaypoints() {
    STARTCHRONO
    TRACET(DEBUG, DFW, "\nfindWaypoints()------------------------\n");
    
    // Unset positioned flags in the waypoints
    for (unsigned int waypointIndex = 0; waypointIndex < nominalContainer.getNumBeaconGroups(); waypointIndex++) {
        BeaconGroup & waypoint = nominalContainer.getBeaconGroup(waypointIndex);
        waypoint.setPositioned(false);
    }
    
    // For each beacon group found
    for (int beaconGroupIndex = 0; beaconGroupIndex < beaconContainer->getNumBeaconGroups(); beaconGroupIndex++) {
        BeaconGroup & beaconGroup = beaconContainer->getBeaconGroup(beaconGroupIndex);
        unsigned int beaconNumber = beaconGroup.getNumBeacons();

        unsigned int angleScore = 101;
        unsigned int distanceScore = 101;
        unsigned int bestScore = 201;
        unsigned int bestFirstBeaconIndex;
        BeaconGroup *bestWaypoint = NULL;
        
        // For each waypoint
        for (unsigned int waypointIndex = 0; waypointIndex < nominalContainer.getNumBeaconGroups(); waypointIndex++) {
            BeaconGroup & waypoint = nominalContainer.getBeaconGroup(waypointIndex);
            
            TRACET(DEBUG, DFW, " Comparing beaconGroup %d to waypoint %d\n", beaconGroup.getId(), waypoint.getId());
            
            // Check beacon number
            if (beaconNumber != waypoint.getNumBeacons()) {
                TRACET(DEBUG, DFW, "    Beacon number missmatch, skipping waypoint\n");
                continue;
            }

            // Obtain scale
            float scale = beaconGroup.getMeanDistance() / waypoint.getMeanDistance();

            Beacon& nominalBeacon = waypoint.getBeacon(0);
            
            // For each beacon found
            for (unsigned int firstBeaconIndex=0; firstBeaconIndex < beaconNumber; firstBeaconIndex++) {
                Beacon& beacon = beaconGroup.getBeacon(firstBeaconIndex);
                float distance = beacon.distance() / scale;
                TRACET(DEBUG, DFW, "     Using beacon %d as first beacon\n", beacon.getId());
                // Check distance
                float error = fabs(nominalBeacon.distance() - distance);
                if (error < DISTANCE_MARGIN) {
                    distanceScore = error * 100 / DISTANCE_MARGIN;
                    TRACET(DEBUG, DFW, "        Distance test passed, error: %f, score: %d\n", error, distanceScore);
                    TRACET(DEBUG, DFW, "        Angle test in progress\n");
                    // Check angles
                    float maxAngleError = 0;
                    bool found = true;
                    for (unsigned int i=0; i < beaconNumber; i++) {
                        Beacon& cBeacon = beaconGroup.getBeacon((firstBeaconIndex + i) % beaconNumber);
                        Beacon& cNominalBeacon = waypoint.getBeacon(i);
                        
                        TRACET(DEBUG, DFW, "         Testing beacon %d ", cBeacon.getId());
                        float nominalAngle = cNominalBeacon.theta();
                        float angle = cBeacon.theta();
                     
                        float angleError = fabs(nominalAngle - angle);
                        if (angleError > ANGLE_MARGIN) {
                            // Angle test negative, search next beacon that fits the nominal distance
                            found = false;
                            TRACET(DEBUG, DFW, "Angle missmatch (nominal: %f, actual: %f)\n", nominalAngle, angle);
                            break;
                        }
                        else {
                            if (angleError > maxAngleError)
                                maxAngleError = angleError;
                            
                            TRACET(DEBUG, DFW, "Angle matched (nominal: %f, actual: %f)\n", nominalAngle, angle);
                        }
                    }
                    if (found) {
                        // EUREKA!!! the beacon group fits this waypoint!
                        
                        angleScore = maxAngleError * 100 / ANGLE_MARGIN;
                        unsigned int score = angleScore + distanceScore;
                        if (score < bestScore) {
                            bestScore = score;
                            bestWaypoint = &waypoint;
                            bestFirstBeaconIndex = firstBeaconIndex;
                        }
                        
                        TRACET(DEBUG, DFW, "        All angles matched, the beacon group %d fits to the waypoint id %d, score: %d, angleScore: %d, distanceScore: %d\n",\
                            beaconGroup.getId(), waypoint.getId(), score, angleScore, distanceScore);
                        break;
                    }
                    else {
                        TRACET(DEBUG, DFW, "        At least one angle missmatch has been found, choosing another first beacon if available\n\n");
                    }
                }
                else {
                    TRACET(DEBUG, DFW, "        Distance test failed, error: %f\n", error);
                }
            }
        }
        
        if (bestWaypoint != NULL) {
            // Compute position and heading using the best waypoint
            Beacon& nominalBeacon = bestWaypoint->getBeacon(0);
            Beacon& beacon = beaconGroup.getBeacon(bestFirstBeaconIndex);

            // Obtain the heading
            float heading = beacon.phi() - nominalBeacon.phi();
            if (heading < 0)
                heading += 360;

            // Obtain the position
            Vector3<float> position;
            Vector3<float> beaconPosition = beacon;
            
            // Height
            position.z = bestWaypoint->getMeanDistance() / beaconGroup.getMeanDistance();
            
            beaconPosition.rotate(-heading / RAD2DEG);
            // Project p to the ground
            beaconPosition.x = beaconPosition.x * position.z;
            beaconPosition.y = beaconPosition.y * position.z;
            
            position.x = nominalBeacon.x - beaconPosition.x;
            position.y = nominalBeacon.y - beaconPosition.y;
            
            // Load data to the beacon group
            beaconGroup.setWaypoint(*bestWaypoint);
            beaconGroup.setPosition(position);
            beaconGroup.setHeading(heading);
            
            // Update position of the waypoint in the image
            bestWaypoint->setImageCentroid(beaconGroup.getImageCentroid());
            
            bestWaypoint->setPosition(position);
            bestWaypoint->setHeading(heading);
            
            TRACET(DEBUG, DFW, " POSITION FOUND: position: (%f,\t%f,\t%f)\tyaw: %f\n\n", position.x, position.y, position.z, heading);
        }
    }
    TRACET(DEBUG, DFW, "-%d us---------------------------------------\n", GETCLICK);
}