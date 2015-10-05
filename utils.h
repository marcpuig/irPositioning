#pragma once

#include "algebra.h"
#include "IPC/IPC.h" // DELETE AFTER DEBUGGING

float correctAngle(float angle);
float correctAngleOffset(float angle);
void pushHistoryUnit(Vector3<float> p, long int t);
Vector3<float> getSpeed();
void resetSpeed();
int64_t getSec();
long int getMicroSec();
long int getMillisSinceStart();
long int getSpent(long int dif);
#ifdef THUMBNAILS
void write_png_file(int photogram);
#endif

inline void img2air(int &x, int &y) {
    int imgX = x;
    //x = y - PIXELS_Y / 2;
    //y = imgX - PIXELS_X / 2;
    x = x - PIXELS_X / 2;
    y = PIXELS_Y / 2 - y;
}

inline Vector2<int> vImg2air(Vector2<unsigned int> imgPoint) {
    Vector2<int> airPoint;
    
    airPoint.x = imgPoint.x - PIXELS_X / 2;
    airPoint.y = PIXELS_Y / 2 - imgPoint.y;
    
    return airPoint;
}

class Chrono {
public:
    Chrono();
    long int click();
    struct timespec t1, t2;
};

struct historyUnitT {
    Vector3<float> p;
    long int t;
};

template <class T> class SignalVector {
public:
    SignalVector(int size, long int responseDelay) \
        : size(size), next(0), cur(size - 1),\
        free(size), used(0), responseDelay(responseDelay) {
        
        data = new unit[size];
    }
    
    void updateResponseDelay(long int delay) { responseDelay = delay; }
    
    T getLastValue() {
        if (used > 0)
            return data[cur].value;
        else {
            TRACE(ERROR, "ERROR: signal vector empty, unable to get last value.\n");
            exit(1);
            //return 0;
        }
    }
    
    void push(T value, long int timeSpent) {
        //printf("(push) (%d, %d), free: %d, used: %d, next: %d, cur: %d\n", ((attitudeT*)&value)->roll, timeSpent, free, used, next, cur);
        data[next].value = value;
        if (used == 0)
            data[next].timeSpent = 100000000;
        else
            data[next].timeSpent = timeSpent;
        cur = next;
        next = ++next % size;
        if (free > 0) free--;
        if (used < size) used++;
        //printf("(push) (%d, %d), free: %d, used: %d, next: %d, cur: %d\n", ((attitudeT*)&value)->roll, timeSpent, free, used, next, cur);
    }
    
    T getSignal() {
        long int timeSpent = 0;
        long int i = cur;
        int tested = 0;
        T value;
        
        while (timeSpent < responseDelay && tested++ < used) {
            timeSpent += data[i].timeSpent;
            value = data[i].value;
            ///printf("  %d -> (%d, %d) ", i, ((attitudeT*)&value)->roll, timeSpent);
            
            if (--i < 0)
                i = size -1 - free;
        }
        //printf(" [tested: %d, used: %d]\n", tested, used);
        
        if (tested > used)
            TRACE(ERROR, "ERROR: Not enough data for the desired responseDelay\n");
        
        return value;
    }
    
    void getSignal(T &value, long int &timeSpent) {
        long int i = cur;
        int tested = 0;
        timeSpent = 0;
        
        while (timeSpent < responseDelay && tested++ < used) {
            timeSpent += data[i].timeSpent;
            value = data[i].value;
            
            if (--i < 0)
                i = size -1 - free;
        }
        
        if (tested > used)
            TRACE(ERROR, "ERROR: Not enough data for the desired responseDelay\n");
        
    }
private:
    struct unit {
        T value;
        long int timeSpent;
    };
    unit * data;
    int next;
    int cur;
    int free;
    int size;
    int used;
    long int responseDelay;
};

class Servo {
public:
    Servo(int speed, long int delay, int pwm_range, float range, float offset) : \
        speed(speed / 1000000.f),\
        responseDelay(delay),\
        delay(0),\
        estimatedAngle(-offset),\
        desiredAngle(0),\
        pwm_range(pwm_range),\
        range(range),\
        offset(offset), \
        bigStep(false), \
        postBigStep(false) {
            
        oldMicros = getMicroSec();
        signal = new SignalVector<float>(25, delay);
        //printf("Servo init, angle: %f\n\n", angle);
    }
    void setDesiredAngle(float angle) {
        desiredAngle = angle + offset;
        if (desiredAngle > range) 
            angle = range;
        else if (desiredAngle < -range)
            angle = -range;
    }
    float getDesiredAngle() { return signal->getLastValue() - offset; }
    void setSpeed(int speed) { this->speed = speed / 1000000.f; }
    float getEstimatedAngle() { return estimatedAngle - offset; }
    void setResponseDelay(int delay) { signal->updateResponseDelay(delay); }
    void setPWMRange(int value) { pwm_range = value; }
    void setRange(float value) { range = value; }
    void setOffset(float value) { offset = value; }
    bool inBigStep() {
        if (bigStep)
            printf("In big step: D: %f, E: %f\n", desiredAngle, estimatedAngle);
        else if (postBigStep)
            printf("In POST big step: D: %f, E: %f\n", desiredAngle, estimatedAngle);
        
        return bigStep || postBigStep;
        
    }
    
    // Updates the estimated servo position
    void update(long int spentTime) {
        signal->push(desiredAngle, spentTime);
        float receivedServoAngle = signal->getSignal();
        
        // update estimated angle
        if (estimatedAngle < receivedServoAngle) {
            estimatedAngle += spentTime * speed; 
            if (estimatedAngle > receivedServoAngle)
                estimatedAngle = receivedServoAngle;
        }
        else {
            estimatedAngle -= spentTime * speed; 
            if (estimatedAngle < receivedServoAngle)
                estimatedAngle = receivedServoAngle;
        }
        
        bool newStep = abs(desiredAngle - estimatedAngle) > 6.;
        
        // Step start
        if (!bigStep && newStep)
            bigStep = true;
        // Step end switch to post big step status
        else if (bigStep && !newStep) {
            postBigStep = 2;
            bigStep = false;
        }
        // In post big step
        else if (postBigStep > 0)
            postBigStep--;
    }
    int getPWM() {
        // Obtain PWM value to send to the servos
        int16_t pulse = (pwm_range * desiredAngle) / range;
        
        if (pulse > pwm_range)
            pulse = ROLL_PWM_RANGE;
        else if (pulse < -pwm_range)
            pulse = -ROLL_PWM_RANGE;
        pulse += PWM_MID;
        
        return pulse;
    }
private:
    float speed; // micro degrees/second
    float desiredAngle;
    float estimatedAngle;
    float responseDelay;
    float delay;
    long int oldMicros;
    int pwm_range;
    float offset;
    float range;
    SignalVector <float> *signal;
    bool bigStep;
    short postBigStep;
};

