/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <stdio.h>
#include <unistd.h> // fork()
#include <stdlib.h> // exit()
#include <sys/types.h>
#include <signal.h>

#include "IPC.h"

IPC::IPC() : initialized(false) {
    key = ftok(FTOK_FILE, FTOK_INT);   
}

IPC::~IPC() {
    if (!initialized)
        return;
    
    if (server) { 
        if (kill(pID, SIGKILL)) {
            perror("Could not kill process");
            return;
        }
        
        if (semctl(sem_id, 0, IPC_RMID) < 0) {
            perror("Could not delete semaphore");
            return;
        }
        
        if (shmdt(shm) < 0) {
            perror("Could not detach shared memory");
            return;
        }
        
        if (shmctl(shm_id, IPC_RMID, NULL) < 0) {
            perror("Could not remove the shared memory identifier");
            return;
        }
    }
}

int IPC::init(bool _server) {
    server = _server;
    
    if (server) {
        // Create the semaphore
        sem_id = semget(key, 1, IPC_CREAT | 0666);
        if (sem_id < 0) {
            perror("Could not create sem");
            return 1;
        }
    
        if (semctl(sem_id, 0, SETVAL, 1) < 0) {
            perror("Could not set value of semaphore");
            return 2;
        }
        
        // Create the shared memory
        if ((shm_id = shmget(key, sizeof(attitudeT), IPC_CREAT | 0666)) < 0) {
            perror("shmget");
            return 3;
        }
        
        // Execute the process
        pID = fork();
        if (pID == 0) {                // child
            chdir("/home/pi/projecte/build");
            execl("/home/pi/projecte/build/projecte", (char *) 0);
            exit(0);
        }
        else if (pID < 0) {
            perror("Unable to fork");
            return 4;
        }
    }
    else {
        // Now obtain the (hopefully) existing sem
        sem_id = semget(key, 0, 0);
        if (sem_id < 0) {
            perror("Could not obtain semaphore");
            return 5;
        }
        
        if ((shm_id = shmget(key, sizeof(attitudeT), 0666)) < 0) {
            perror("shmget");
            return 6;
        }
    }
    
    // Now we attach the segment to our data space.
    if ((shm = shmat(shm_id, NULL, 0)) == (void *) -1) {
        perror("shmat");
        return 7;
    }
    initialized = true;
    return 0;
}

bool IPC::getAttitude(attitudeT &attitude) {
    if (!initialized) {
        perror("IPC not initialized.");
        return false;
    }
    
    struct sembuf sop;
    packT* pack = (packT *)shm;
    
    sop.sem_num = 0;
    sop.sem_op = -1;
    sop.sem_flg = SEM_UNDO;
    
    if (semop(sem_id, &sop, 1) < 0) {
        perror("Unable to aquire a semaphore unit.");
        return false;
    }
    
    attitude = pack->attitude;
    
    sop.sem_op = 1;
    if (semop(sem_id, &sop, 1) < 0) {
        perror("Unable to release a semaphore unit.");   
        return false;
    }
    
    return true;
}

bool IPC::getAltitude(int32_t &altitude) {
    if (!initialized) {
        perror("IPC not initialized.");
        return false;
    }
    
    bool positioned;
    struct sembuf sop;
    packT* pack = (packT *)shm;
    
    sop.sem_num = 0;
    sop.sem_op = -1;
    sop.sem_flg = SEM_UNDO;
    
    if (semop(sem_id, &sop, 1) < 0) {
        perror("Unable to aquire a semaphore unit.");
        return false;
    }
    
    altitude = pack->location.z;
    positioned = pack->location.num_waypoints > 0;
    
    sop.sem_op = 1;
    if (semop(sem_id, &sop, 1) < 0) {
        perror("Unable to release a semaphore unit.");   
        return false;
    }
    
    return positioned;
}


bool IPC::setAttitude(attitudeT attitude) {
    if (!initialized) {
        perror("IPC not initialized.");
        return false;
    }
    
    struct sembuf sop;
    packT* pack = (packT *)shm;
    
    sop.sem_num = 0;
    sop.sem_op = -1;
    sop.sem_flg = SEM_UNDO;
    
    if (semop(sem_id, &sop, 1) < 0) {
        perror("Unable to aquire a semaphore unit.");
        return false;
    }
    
    pack->attitude = attitude;
    
    sop.sem_op = 1;
    
    if (semop(sem_id, &sop, 1) < 0) {
        perror("Unable to release a semaphore unit.");
        return false;
    }
    
    return true;
}

bool IPC::getLocation(locationT &location) {
    if (!this->initialized) {
        perror("IPC not initialized.");
        return false;
    }
    
    struct sembuf sop;
    packT* pack = (packT *)shm;
    
    sop.sem_num = 0;
    sop.sem_op = -1;
    sop.sem_flg = SEM_UNDO;
    
    if (semop(sem_id, &sop, 1) < 0) {
        perror("Unable to aquire a semaphore unit.");
        return false;
    }
    
    location = pack->location;
    
    sop.sem_op = 1;
    
    if (semop(sem_id, &sop, 1) < 0) {
        perror("Unable to release a semaphore unit.");    
        return false;
    }
 
    return true;
}

bool IPC::setLocation(locationT location) {
    if (!initialized) {
        perror("IPC not initialized.");
        return false;
    }
    
    struct sembuf sop;
    packT* pack = (packT *)shm;
    
    sop.sem_num = 0;
    sop.sem_op = -1;
    sop.sem_flg = SEM_UNDO;
    
    if (semop(sem_id, &sop, 1) < 0) {
        perror("Unable to aquire a semaphore unit.");
        return false;
    }
    
    pack->location = location;
    
    sop.sem_op = 1;
    
    if (semop(sem_id, &sop, 1) < 0) {
        perror("Unable to release a semaphore unit.");
        return false;
    }
    
    return true;
}
