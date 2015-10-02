/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __IPC_H__
#define __IPC_H__

#include <sys/types.h>
#include <stdint.h>

#define FTOK_FILE "/home/pi/projecte/main.cpp"
#define FTOK_INT 42

struct v3f { float x; float y; float z; };

struct attitudeT {
    int roll;
    int pitch;
    int yaw;
};

struct locationT {
    float x;
    float y;
    float z;
    
    float velx;
    float vely;
    float velz;
    
    float heading;
    uint8_t num_waypoints;
    
    float servoRoll;
    float servoPitch;
    short waypoint;
    
    short desiredx;
    short desiredy;
};

struct packT {
    attitudeT attitude;
    locationT location;
};

class IPC
{
public:
    // Constructor
    IPC();
    ~IPC();
    int init(bool server);
    bool getAttitude(attitudeT &attitude);
    bool setAttitude(attitudeT attitude);
    
    bool getLocation(locationT &location);
    bool setLocation(locationT location);
    
    bool getAltitude(int32_t &altitude);
    
private:
    key_t key;
    int sem_id;
    int shm_id;
    void *shm;
    bool initialized;
    bool server;
    pid_t pID;
};

#endif // __IPC_H__
