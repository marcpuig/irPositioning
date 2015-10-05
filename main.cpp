#include <stdio.h>
#include <stdint.h>

#include "defines.h"
#include "algebra.h"
#include "utils.h"
#include "beacon.h"
#include "parsewaypoints.h"
#include "output.h"
#include "findBeacons.h"
#include "groupBeacons.h"
#include "findWaypoints.h"
#include "computePosition.h"
#include "picam/camera.h"
#include "picam/graphics.h"
#include "picam/cameracontrol.h"
#include "IPC/IPC.h"

unsigned char imageData[PIXELS_X * PIXELS_Y];
#ifdef THUMBNAIL
unsigned char imageDataThumbnail[PIXELS_X/8 * PIXELS_Y/8];
#endif
bool yawIncrementInitialised = false;
bool lost = true;
bool glitch = true;
bool servosEnabled = true;
uint32_t photogram = 1;
int rollServoSpeed = ROLL_SERVO_SPEED;
int pitchServoSpeed = PITCH_SERVO_SPEED;
int rollServoDelay = ROLL_SERVO_DELAY;
int pitchServoDelay = PITCH_SERVO_DELAY;
short desiredx = 0;
short desiredy = 0;

int32_t lastPhotogramSpentTime;
int64_t lastPositionTime;
int64_t microsSinceStart = 0;
int64_t photogramMicros = 0;
int64_t lastPhotogramMicros = 0;
int64_t lastPositionedMicros = -1;
int64_t startSec = getSec();
float droneRoll = 0;
float dronePitch = 0;
float droneYaw = 0;
float roll = 0; // Camera roll
float pitch = 0; // Camera pitch
float yawIncrement = 0;
Container beaconContainerA;
Container beaconContainerB;
Container nominalContainer;
Container * beaconContainer;
Container * oldBeaconContainer;
Output output(8080);
Vector3<float> glitchOffset;
IPC ipc;
sem_t semCam;
pthread_mutex_t mutexImageData;
Servo rollStatusServo(rollServoSpeed, rollServoDelay, ROLL_PWM_RANGE, ROLL_RANGE, ROLL_OFFSET);
Servo pitchStatusServo(pitchServoSpeed, pitchServoDelay, PITCH_PWM_RANGE, PITCH_RANGE, PITCH_OFFSET);
SignalVector<attitudeT> attitudeSignal(10, ATTITUDE_DELAY);

void *camThread(void* tid) {
    const void* buffer;
    int buffer_len;
    int sval;
    int i = 0;
    
    // Initialize and configure the camera
    CCamera* cam = StartCamera(PIXELS_X, PIXELS_Y, FPS, 1, false);
    TRACE(DEBUG, "Camera opened\n");
    
    for(;;i++) {
        //TRACE(DEBUG, "\n(CAMERA PHOTOGRAM %d, milliseconds: %ld)\n", i, getMillisSinceStart());
        // Get image data
        int ret;
        int delaySteps = 0;
        //STARTCHRONO
        do {
            ret = cam->BeginReadFrame(0, buffer, buffer_len);
            if (!ret) {
                usleep(100);
                delaySteps++;
            }
        } while (!ret);
        
        pthread_mutex_lock(&mutexImageData);
            memcpy(imageData, buffer, PIXELS_X * PIXELS_Y);
        pthread_mutex_unlock(&mutexImageData);
        
        cam->EndReadFrame(0);
        
        // Let the main process continue
        sem_getvalue(&semCam, &sval);
        if (sval < 1)
            sem_post(&semCam);
        //TRACE(DEBUG, "(delaySteps: %d, time: %d, sval: %d)\n", delaySteps, GETCLICK, sval);
    }
}

int main(int argc, char* argv[]) {
    // Initialize shared memory
    if (ipc.init(false))
        exit(1);

    // Initialize nominal beaconContainer
    parseWaypoints();
    
    // Create camera semaphore/ mutex
    sem_init(&semCam, 0, 0);
    if (pthread_mutex_init(&mutexImageData, NULL) != 0) {
        printf("\n Camera mutex init failed\n");
        exit(1);
    }
    
    // Start camera thread
    pthread_t tid;
    int err = pthread_create(&tid, NULL, &camThread, NULL);
    if (err != 0) {
        printf("\nCan't create camera thread :[%s]", strerror(err));
        exit(1);
    }
    
    lastPhotogramMicros = getMicroSec();
    
    // Main loop
    for(;;photogram++) {        
        TRACE(DEBUG, "\nPHOTOGRAM %d, milliseconds: %ld\n", photogram, getMillisSinceStart());
        
        // Update time variables
        photogramMicros = getMicroSec();
        lastPhotogramSpentTime = getSpent(photogramMicros - lastPhotogramMicros);
        microsSinceStart += lastPhotogramSpentTime;
        lastPhotogramMicros = photogramMicros;
        
        // Get drone roll, pitch and yaw
        attitudeT attitude, delayedAttitude;
        ipc.getAttitude(attitude);
        
        attitudeSignal.push(attitude, lastPhotogramSpentTime);
        delayedAttitude = attitudeSignal.getSignal();
        
        droneRoll = delayedAttitude.roll * 0.01;
        dronePitch = delayedAttitude.pitch * 0.01;
        droneYaw = delayedAttitude.yaw * 0.01;
        
        // Update estimated servo values
        rollStatusServo.update(lastPhotogramSpentTime);
        pitchStatusServo.update(lastPhotogramSpentTime);
        
        if (rollStatusServo.inBigStep() || pitchStatusServo.inBigStep()) {
            sem_wait(&semCam);
            continue;
        }
       
        // Update estimated camera roll and pitch
        roll = droneRoll + rollStatusServo.getEstimatedAngle();
        pitch = dronePitch + pitchStatusServo.getEstimatedAngle();
        
        // Switch beacon containers to preserve one history record
        if (beaconContainer == &beaconContainerA) {
            beaconContainer = &beaconContainerB;
            oldBeaconContainer = &beaconContainerA;
        }
        else {
            beaconContainer = &beaconContainerA;
            oldBeaconContainer = &beaconContainerB;
        }
        beaconContainer->clean();
  
        // Wait until new frame is received
        sem_wait(&semCam);
        
        // Where the magic happens
        findBeacons();
        groupBeacons();
        findWaypoints();
        computePosition();
    }
    StopCamera();
    printf("Finished.\n");
    
    return 0;
}
