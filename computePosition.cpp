#include <stdio.h>

#include "defines.h"
#include "algebra.h"
#include "beacon.h"
#include "output.h"
#include "utils.h"
#include "IPC/IPC.h"

extern unsigned int photogram;
extern float pitch_offset, yawIncrement;
extern Container nominalContainer;
extern Container * beaconContainer, * oldBeaconContainer;
extern Output output;
extern uint8_t servoSpeed;
extern short desiredx;
extern short desiredy;
extern int64_t photogramMicros;
extern long int lastPositionTime;
extern int32_t lastPhotogramSpentTime;
extern long int lastPhotogramMicros;
extern long int lastPositionedMicros;
extern bool yawIncrementInitialised;
extern bool lost;
extern bool servosEnabled;
extern float roll;
extern float pitch;
extern float droneRoll;
extern float dronePitch;
extern float droneYaw;
extern IPC ipc;
extern Servo rollStatusServo;
extern Servo pitchStatusServo;

unsigned long int lostTime = HOME_SERVOS_ON_LOST_DELAY;
BeaconGroup * trackedWaypoint = NULL;   // Waypoint being tracked by the camera
SignalVector<Vector3<float>> delayedPositionSignal(20, 100000);
Vector3<float> filteredPosition;
long int oldMicros = 0;
locationT lastKnownLocation = { 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0, 0.f, 0.f };


void updateServos(float heading, Vector3<float> position) {
    static short positionReadings = 0;
    Vector3<float> position2D;
    position2D.x = position.x;
    position2D.y = position.y;
    position2D.z = 0;
    
    
    // Search closest waypoint
    BeaconGroup * closestWaypoint;
    float minDistance = INFINITY;
    for (unsigned int waypointIndex = 0; \
        waypointIndex < nominalContainer.getNumBeaconGroups(); waypointIndex++) {
                
        BeaconGroup * waypoint = &nominalContainer.getBeaconGroup(waypointIndex);
    
        if (!waypoint->isLocked())
            continue;
                
        Vector3<float> waypointPosition = waypoint->getCentroid();
        waypointPosition.z = 0;
            
        float distance = (waypointPosition - position2D).abs();
                
        if (distance < minDistance) {
            minDistance = distance;
            closestWaypoint = waypoint;
        }
    }
    Vector3<float> trackedCentroid = trackedWaypoint->getCentroid();
    trackedCentroid.z = 0;
    float oldMinDistance = (trackedCentroid - position2D).abs();
    
    // Need to change the waypoint tracked?
    if (minDistance + SWITCH_WAYPOINT_MARGIN < oldMinDistance) {
        printf("Changing from W%d to W%d\t%f, %f\n", trackedWaypoint->getId(), closestWaypoint->getId(), minDistance, oldMinDistance);
        trackedWaypoint = closestWaypoint;
    }
        
    float newServoPitch;
    float newServoRoll;
    
    if (trackedWaypoint->isPositioned()) {
        
        // Get waypoint position in the image
        Vector2<int> imageCentroid = vImg2air(trackedWaypoint->getImageCentroid());

        // Get estimated servo position
        float estimatedServoRoll = rollStatusServo.getEstimatedAngle();
        float estimatedServoPitch = pitchStatusServo.getEstimatedAngle();
        
        float desiredServoRoll = rollStatusServo.getDesiredAngle();
        float desiredServoPitch = pitchStatusServo.getDesiredAngle();
        
        // Get desired servo angles
        newServoRoll = -FOV_ROLL_D / PIXELS_X * imageCentroid.x + estimatedServoRoll;
        newServoPitch = FOV_PITCH_D / PIXELS_Y * imageCentroid.y + estimatedServoPitch;
 
        ////printf("%d;%d \t E: %f \t D: %f \t N: %f\n", imageCentroid.x, imageCentroid.y, \
            estimatedServoPitch, desiredServoPitch, newServoPitch);
        
    }
    else {
        
        // Roll && Pitch
        float headingRollAxe = heading / RAD2DEG;
        float headingPitchAxe = fmod(heading + 90, 360) / RAD2DEG;

        Vector2<float> e1Roll(sin(headingRollAxe), cos(headingRollAxe));
        Vector2<float> e1Pitch(sin(headingPitchAxe), cos(headingPitchAxe));
        
        Vector3<float> closestWaypointPosition = trackedWaypoint->getCentroid();
        
        // 2D Vector from closest waypoint to the drone
        Vector2<float> e2(position.x - closestWaypointPosition.x, \
            position.y - closestWaypointPosition.y);
        
        double valRoll = e1Roll.dot(e2);
        double valPitch = e1Pitch.dot(e2);
        
        Vector3<float> pRoll(valRoll * e1Roll.x, valRoll * e1Roll.y, 0);
        Vector3<float> pPitch(valPitch * e1Pitch.x, valPitch * e1Pitch.y, 0);
        
        Vector3<float> e2GroundProj(e2.x, e2.y, 0);
        
        newServoRoll = atan2((pRoll - e2GroundProj).abs(), position.z) * RAD2DEG;
        newServoPitch = -atan2((pPitch - e2GroundProj).abs(), position.z) * RAD2DEG;
        
        // sign
        if (e2.dot(e1Roll) < 0) newServoPitch = -newServoPitch;
        if (e2.dot(e1Pitch) < 0) newServoRoll = -newServoRoll;
        
        newServoRoll -= droneRoll;
        newServoPitch -= dronePitch;
        printf("(big jump)\n");
    }
    
    rollStatusServo.setDesiredAngle(newServoRoll);
    pitchStatusServo.setDesiredAngle(newServoPitch);
}

void computePosition() {
    STARTCHRONO
    TRACET(DEBUG, DCP, "\ncomputePosition()------------------------\n");
    Vector3<float> position;
    Vector3<float> velocityVector;
    int num_waypoints = 0;
    float xHeading = 0;
    float yHeading = 0;
    float heading;
    locationT location;
    
    // Count waypoints
    for (int beaconGroupIndex = 0; beaconGroupIndex < \
        beaconContainer->getNumBeaconGroups(); beaconGroupIndex++) {
        
        BeaconGroup& beaconGroup = beaconContainer->getBeaconGroup(beaconGroupIndex);
        if (beaconGroup.isPositioned())
            num_waypoints++;
    }
    TRACET(DEBUG, DCP, " waypoints: %d\n", num_waypoints);
    
    std::ostringstream stringStream, beaconString;
    
    if (num_waypoints > 0) {
        BeaconGroup * lockedWaypoint;    // Waypoint used for the positioning
        
        if (num_waypoints > 1) {
            
            // Get fixed waypoint closer to the master waypoint in number of jumps
            BeaconGroup* reliableBeaconGroup = NULL;
            unsigned int minJumps = MAX_WAYPOINT_BEACONS;
            for (int beaconGroupIndex = 0; beaconGroupIndex < \
                beaconContainer->getNumBeaconGroups(); beaconGroupIndex++) {
                
                BeaconGroup& beaconGroup = \
                    beaconContainer->getBeaconGroup(beaconGroupIndex);
                if (beaconGroup.isPositioned()) {
                    BeaconGroup& waypoint = beaconGroup.getWaypoint();
                    TRACET(DEBUG, DCP, " Id: %d, waypoint id: %d, Locked: %d, masterJumps: %d, minJumps: %d\n", beaconGroup.getId(), waypoint.getId(), waypoint.isLocked() ? 1 : 0, waypoint.getMasterJumps(), minJumps);
                    if (waypoint.isLocked() && waypoint.getMasterJumps() < minJumps) {
                        minJumps = waypoint.getMasterJumps();
                        reliableBeaconGroup = &beaconGroup;
                    }
                }
            }
            
            // Update nominals to non locked waypoints using the most reliable waypoint
            if (reliableBeaconGroup != NULL) {
                TRACET(DEBUG, DCP, " ReliableBeaconGroup: %d, waypoint id: %d, jumps: %d\n",\
                    reliableBeaconGroup->getId(), reliableBeaconGroup->getWaypoint().getId(), minJumps);
                
                int reliableBeaconGroupJumps = reliableBeaconGroup->getWaypoint().getMasterJumps();
                for (int beaconGroupIndex = 0; beaconGroupIndex < \
                    beaconContainer->getNumBeaconGroups(); beaconGroupIndex++) {
                    
                    BeaconGroup& beaconGroup = \
                        beaconContainer->getBeaconGroup(beaconGroupIndex);
                    BeaconGroup& waypoint = beaconGroup.getWaypoint();
                
                    if (beaconGroup.isPositioned() \
                        && waypoint.getMasterJumps() > reliableBeaconGroupJumps) {
                        
                        //TRACE(NOTICE, " Updating nominals of waypoint %d\n", \
                            waypoint.getId());
                        
                        waypoint.updateNominals(beaconGroup.getPosition(), \
                            reliableBeaconGroup->getPosition(), \
                            beaconGroup.getHeading(), \
                            reliableBeaconGroup->getHeading());
                        
                        waypoint.setMasterJumps(reliableBeaconGroupJumps + 1);
                        
                        beaconGroup.setPosition(reliableBeaconGroup->getPosition());
                        beaconGroup.setHeading(reliableBeaconGroup->getHeading());
                        waypoint.setPosition(beaconGroup.getPosition());
                        waypoint.setHeading(beaconGroup.getHeading());
                        waypoint.setImageCentroid(beaconGroup.getImageCentroid());
                    }
                }
            }
            else {
                TRACET(DEBUG, DCP, "ReliableBeaconGroup not found!\n");
                return;
            }

            if (trackedWaypoint == NULL || !trackedWaypoint->isPositioned())
                lockedWaypoint = &reliableBeaconGroup->getWaypoint();
            else
                lockedWaypoint = trackedWaypoint;
            
            position = lockedWaypoint->getPosition();
            heading = lockedWaypoint->getHeading();
        
        }
        else {
            // If only one waypoint found use its determined position and heading
            for (int beaconGroupIndex = 0; beaconGroupIndex < \
                beaconContainer->getNumBeaconGroups(); beaconGroupIndex++) {
                
                BeaconGroup& beaconGroup = \
                    beaconContainer->getBeaconGroup(beaconGroupIndex);
                if (beaconGroup.isPositioned()) {
                    heading = beaconGroup.getHeading();
                    position = beaconGroup.getPosition();
                    lockedWaypoint = &beaconGroup.getWaypoint();
                    break;
                }
            }
        }
        
        if (trackedWaypoint == NULL)
            trackedWaypoint = lockedWaypoint;
        
        float cyaw = correctAngle(droneYaw);
        
        if (!yawIncrementInitialised) {
            yawIncrement = correctAngleOffset(heading - cyaw);
            yawIncrementInitialised = true;
        }
        else {
            float yawOffset = correctAngleOffset(heading - \
                correctAngle(cyaw + yawIncrement));
            yawIncrement = correctAngleOffset(yawIncrement + \
                yawOffset / YAW_OFFSET_DIVIDER);
        }
        
        // Obtain velocity vector
        filteredPosition = lostTime < 20000 ? filteredPosition * 0.85 + position * 0.15 : position;
        long int actualMicros = getMicroSec();
        long int spent = getSpent(actualMicros - oldMicros);
        delayedPositionSignal.push(filteredPosition, spent);
        oldMicros = actualMicros;
        Vector3<float> delayedPosition;
        long int delayedMicros;
        delayedPositionSignal.getSignal(delayedPosition, delayedMicros);
        velocityVector = (position - delayedPosition) * 1000000.f / delayedMicros;
        Vector2<float> groundSpeedVector;
        groundSpeedVector.x = velocityVector.x;
        groundSpeedVector.y = velocityVector.y;
        
        //printf("positionx: %f, filteredPositionx: %f, delayedPositionx: %f, velx: %f, delay: %d\n",\
            position.x, filteredPosition.x, delayedPosition.x, velocityVector.x, delayedMicros);
        
        TRACET(DEBUG, DCP, " yaw: %f, raw heading: %f, yawIncrement: %f, heading: ", \
            cyaw, heading, yawIncrement);
        heading = correctAngle(cyaw + yawIncrement);
        TRACET(DEBUG, DCP, "%f\n", heading);
        
        roll = droneRoll + rollStatusServo.getEstimatedAngle();
        pitch = dronePitch + pitchStatusServo.getEstimatedAngle();
        
        if (servosEnabled)
            updateServos(heading, position);
        else {
            rollStatusServo.setDesiredAngle(0);
            pitchStatusServo.setDesiredAngle(0);
        }
        
        std::ostringstream beaconString, beaconGroupString;
        beaconContainer->getBeaconString(beaconString);
        beaconContainer->getBeaconGroupString(beaconGroupString);
        
        stringStream << "{ \"type\": \"ph\", \"numwaypoints\": " << num_waypoints \
            << ",\"photogram\": " << photogram \
            << ", \"x\": " << position.x \
            << ", \"y\": " << position.y \
            << ", \"z\": " << position.z \
            << ", \"speed\": " << velocityVector.abs() \
            << ", \"groundspeed\": " << groundSpeedVector.abs() \
            << ", \"xspeed\": " << velocityVector.x \
            << ", \"yspeed\": " << velocityVector.y \
            << ", \"zspeed\": " << velocityVector.z \
            << ", \"heading\": " << heading \
            << ", \"waypoints\": " << beaconGroupString.str() \
            << ", \"beacons\": " << beaconString.str() \
            << ", \"cameraPitch\":" << pitch \
            << ", \"cameraRoll\":" << roll \
            << ", \"servoRoll\":" << rollStatusServo.getEstimatedAngle() \
            << ", \"servoPitch\":" << pitchStatusServo.getEstimatedAngle() \
            << ", \"droneYaw\": " << droneYaw \
            << ", \"dronePitch\":" << dronePitch \
            << ", \"droneRoll\": " << droneRoll << " }";
            
        TRACET(DEBUG, DCP, " Position: %f, %f, %f, Heading: %f\n", \
            position.x, position.y, position.z, heading);
        
        TRACETS(DEBUG, DCP, "%f %f %f %f\n", \
            position.x, position.y, position.z, heading);
        
        lost = false;
        lostTime = 0;
        
        location.x = position.x;
        location.y = position.y;
        location.z = position.z;
        location.velx = velocityVector.x;
        location.vely = velocityVector.y;
        location.velz = velocityVector.z;
        location.heading = heading;
        location.num_waypoints = num_waypoints;
        location.waypoint = lockedWaypoint->getId();
        location.servoRoll = rollStatusServo.getPWM();
        location.servoPitch = pitchStatusServo.getPWM();        
        
        lastKnownLocation = location;
    }
    // No waypoints found
    else {
        TRACET(DEBUG, DCP, " No waypoints found\n");
        std::ostringstream beaconString, beaconGroupString;
        beaconContainer->getBeaconString(beaconString);
        beaconContainer->getBeaconGroupString(beaconGroupString);
        
        lost = true;
        lostTime += lastPhotogramSpentTime;
        
        roll = droneRoll + rollStatusServo.getEstimatedAngle();
        pitch = dronePitch + pitchStatusServo.getEstimatedAngle();
        
        if (lostTime >= HOME_SERVOS_ON_LOST_DELAY) {
            rollStatusServo.setDesiredAngle(0);
            pitchStatusServo.setDesiredAngle(0);
            trackedWaypoint = NULL;
        }
        else {
            rollStatusServo.setDesiredAngle(rollStatusServo.getDesiredAngle());
            pitchStatusServo.setDesiredAngle(pitchStatusServo.getDesiredAngle());
        }
        
        stringStream << "{ \"type\": \"nf\", \"numwaypoints\": " << num_waypoints \
            << ",\"photogram\": " << photogram \
            << ", \"waypoints\": " << beaconGroupString.str() \
            << ", \"beacons\": " << beaconString.str() \
            << ", \"cameraPitch\":" << pitch \
            << ", \"cameraRoll\":" << roll \
            << ", \"droneYaw\": " << droneYaw \
            << ", \"servoRoll\":" << rollStatusServo.getEstimatedAngle() \
            << ", \"servoPitch\":" << pitchStatusServo.getEstimatedAngle() \
            << ", \"dronePitch\":" << dronePitch \
            << ", \"droneRoll\": " << droneRoll << " }";
        
        location = lastKnownLocation;
        location.num_waypoints = num_waypoints;
        location.waypoint = 0;
        location.servoRoll = rollStatusServo.getPWM();
        location.servoPitch = pitchStatusServo.getPWM();        
    }
    
    location.desiredx = desiredx;
    location.desiredy = desiredy;
    ipc.setLocation(location);
    
    std::string message = stringStream.str();
    output.Send(message, LOC_TYPE);
    TRACET(DEBUG, DCP, "-%d us---------------------------------------\n", GETCLICK);
} 
