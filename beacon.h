#pragma once
#include <sstream>
#include <vector>

#include "defines.h"
#include "algebra.h"

class Beacon : public Vector3<float> {
public:
  Beacon() : bUsed(false), bTested(false), dTheta(0), dDistance(0), dPhi(0), Vector3<float>(0, 0, 0) {}
  void loadBeacon(int x, int y, float roll = 0, float pitch = 0, int size = 0);
  inline void loadBeacon(float x, float y, float z, unsigned int id = 0) {
    this->x = x; this->y = y; this->z = z; this->id = id;
    clean();
  }
  inline void clean() { bUsed = bTested = bDiscarded = false;  }
  inline bool discard() { bDiscarded = true; }
  inline bool discarded() { return bDiscarded; }
  inline bool used() { return bUsed; }
  inline bool tested() { return bTested; }
  inline void used(bool status) { bUsed = status; }
  inline void tested(bool status) { bTested = status; }
  inline float theta() { return dTheta; }
  inline float phi() { return dPhi; }
  inline float distance() { return dDistance; }
  inline void theta(float value) { dTheta = value; }
  inline void phi(float value) { dPhi = value; }
  inline void distance(float value) { dDistance = value; }
  inline int getId() { return id; }
  inline void setId(int id) { this->id = id; }
  inline int getSize() { return size; }
  inline Vector2<unsigned int> getImagePoint() { return imagePoint; }
  inline void setImagePoint(Vector2<unsigned int> point) { imagePoint = point; }
  inline Vector3<float>& operator=(const Vector3<float>& other) { 
    if (this != &other)
      this->x = other.x; this->y = other.y;  this->z = other.z; 
    return *this;
  }
  
private:
  bool bUsed;
  bool bTested;
  bool bDiscarded;
  float dTheta; // Relative to other beacons
  float dPhi; // Relative to x axis
  float dDistance;
  int id;
  int size;
  unsigned int index;
  Vector2<unsigned int> imagePoint;
};

class BeaconGroup {
public:
  BeaconGroup() : numBeacons(0), meanDistance(0), appearences(0),\
    positioned(false), locked(false), masterJumps(-1) {}
  
  inline unsigned int getId() { return id; }
  inline void setId(unsigned int id) { this->id = id; }
  inline bool isPositioned() { return positioned; }
  inline bool setPositioned(bool positioned) { this->positioned = positioned; }
  inline bool isMaster() { return master; }
  inline void setMaster(bool master) { this->master = master; if (master) { masterJumps = 0; locked = true; } }
  inline bool isLocked() { return locked; }
  inline void setLocked(bool locked) { this->locked = locked; }
  inline unsigned int getMasterJumps() { return masterJumps; }
  inline void setMasterJumps(unsigned int masterJumps) { this->masterJumps = masterJumps; }
  inline Vector3<float> getCentroid() {  return centroid; }
  inline void setCentroid(Vector3<float> centroid) { this->centroid = centroid; }
  inline float getHeading() { return heading; }
  inline void setHeading(float heading) { this->heading = heading; }
  inline float getMeanDistance() { return meanDistance; }
  inline Beacon & getBeacon(unsigned int pos) { return *(beacons[pos]); }
  inline unsigned int getNumBeacons() { return numBeacons; }
  inline Vector3<float> getPosition() { return position; }
  inline void setPosition(Vector3<float> position) { this->position = position; positioned = true; }
  inline BeaconGroup & getWaypoint() { return *waypoint; }
  inline void setWaypoint(BeaconGroup & waypoint) {this->waypoint = &waypoint; }
  inline void increaseAppearences() { appearences++; }
  inline unsigned int getAppearances() { return appearences; }
  inline void resetAppearances() { appearences = 0; }
  inline void setImageCentroid(Vector2<unsigned int> point) { imageCentroid = point; }
  inline Vector2<unsigned int> getImageCentroid() { return imageCentroid; }
  inline void seImageCentroid(Vector2<unsigned int> centroid) { imageCentroid = centroid; }
  
  inline void clean() { positioned = false; numBeacons = 0; }
  inline void appendBeacon(Beacon & beacon) { beacons[numBeacons++] = &beacon; beacon.used(true); }
  
  void getBeaconString(std::ostringstream &string);
  void appendBeacons(std::vector<Beacon *> & vector);
  void computeBeaconGroup();
  void updateNominals(Vector3<float> wrongPosition, Vector3<float> position, float wrongHeading, float heading);
  
private:
  Beacon * beacons[MAX_BEACONS];
  BeaconGroup * waypoint;
  unsigned int numBeacons, id, masterJumps, appearences;
  float meanDistance, heading;
  bool positioned, master, locked;
  Vector3<float> position, centroid;
  Vector2<unsigned int> imageCentroid;
};

class Container {
public:
  Container() : numBeacons(0), numBeaconGroups(0), maxBeacons(0) { };
  void clean();
  void getBeaconString(std::ostringstream &string);
  void getBeaconGroupString(std::ostringstream &string);
  BeaconGroup & getCleanBeaconGroup();
  Beacon & getCleanBeacon();
  inline unsigned int getNumBeaconGroups() { return numBeaconGroups; };
  inline unsigned int getNumBeacons() { return numBeacons; };
  inline BeaconGroup & getBeaconGroup(unsigned int index) { return beaconGroups[index]; };
  inline Beacon & getBeacon(unsigned int i) { return beacons[i]; };
  inline unsigned int getMaxBeacons() { return maxBeacons; }
  inline void setMaxBeacons(unsigned int maxBeacons) { this->maxBeacons = maxBeacons; }
 
private:    
  unsigned int numBeaconGroups, numBeacons, maxBeacons;
  BeaconGroup beaconGroups[MAX_WAYPOINTS];
  Beacon beacons[MAX_BEACONS];
};