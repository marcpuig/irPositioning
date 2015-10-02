#include "beacon.h"
#include "defines.h"
#include "utils.h"

void Beacon::loadBeacon(int x, int y, float roll, float pitch, int size) {
  const Vector3<float> FLOOR_POINT(0, 0, 0);
  const Vector3<float> POINT_PLANE(0, 0, -1);
  const Vector3<float> UP_VECTOR(0, 0, 1);
  Vector3<float> roll_vector(0, 1, 0);  // Y
  Vector3<float> pitch_vector(1, 0, 0);  // X
  Vector3<float> yaw_vector(0, 0, 1);  // Z
  
  this->size = size;
  
  // Backup image point
  this->imagePoint.x = x;
  this->imagePoint.y = y;

  // Convert point from image RS to air RS
  img2air(x, y);

  // PITCH
  yaw_vector = yaw_vector.rotate(pitch_vector, -pitch);
  roll_vector = roll_vector.rotate(pitch_vector, -pitch);

  // ROLL
  pitch_vector = pitch_vector.rotate(roll_vector, -roll);
  yaw_vector = yaw_vector.rotate(roll_vector, -roll);

  // Vector pointing to the beacon
  Vector3<float> line_vector = yaw_vector * FOCAL_LENGTH + roll_vector * y * D_PER_PIXEL + pitch_vector * x * D_PER_PIXEL;

  // Project point to an horizontal plane
  float d = UP_VECTOR.dot(line_vector);
  if (d < 0.000001)  // Parallel
    this->x = this->y = 0;
  else {
    this->x = line_vector.x / d;
    this->y = line_vector.y / d;
  }
  this->z = 0;
  
  this->used(false);
}


void BeaconGroup::getBeaconString(std::ostringstream& string) {
  string << "[ ";
  for (int beaconIndex = 0; beaconIndex < numBeacons; beaconIndex++) {
    if (beaconIndex != 0)
      string << ", ";
    string << beacons[beaconIndex]->getId();
  }
  string << " ]";
}

void Container::getBeaconString(std::ostringstream& string) {
  string << "[ ";
  for (int beaconIndex = 0; beaconIndex < numBeacons; beaconIndex++) {
    Beacon &b = beacons[beaconIndex];
    if (beaconIndex != 0)
      string << ", ";
    
    string << "{ \"id\": " << b.getId() << ", \"xi\": " << b.getImagePoint().x << ", \"yi\": " << b.getImagePoint().y << \
      ", \"xp\": " << b.x << ", \"yp\": " << b.y << ", \"size\": " << b.getSize() << ", \"discarded\": " << b.discarded() <<  " }";
  }
  string << " ]";
}

void Container::getBeaconGroupString(std::ostringstream &string) {
  string << "[ ";
  bool first = true;
  for (int beaconGroupIndex = 0; beaconGroupIndex < numBeaconGroups; beaconGroupIndex++) {
    BeaconGroup& beaconGroup = getBeaconGroup(beaconGroupIndex);
    std::ostringstream beaconString;
    beaconGroup.getBeaconString(beaconString);
  
    if (first)
      first = false;
    else
      string << ", ";
    
    if (beaconGroup.isPositioned()) {
      string << "{ \"id\":" << beaconGroup.getId() \
        << ", \"beacons\": " << beaconString.str() \
        << ", \"positioned\": 1" \
        << ", \"x\": " << beaconGroup.getPosition().x \
        << ", \"y\": " << beaconGroup.getPosition().y \
        << ", \"z\": " << beaconGroup.getPosition().z \
        << ", \"heading\": " << beaconGroup.getHeading() \
        << ", \"waypoint\": " << beaconGroup.getWaypoint().getId() \
        << " }";
    }
    else {
      string << "{ \"id\":" << beaconGroup.getId() \
        << ", \"beacons\": " << beaconString.str() \
        << ", \"positioned\": 0 }";
    }
  }
  string << " ]";
}


void BeaconGroup::appendBeacons(std::vector< Beacon* >& vector) {
  for (std::vector<Beacon *>::iterator it=vector.begin(); it < vector.end(); it++)
    appendBeacon(*(*it));
}

// Computes polar coordinates of each beacon of the specified beaconGroup
void BeaconGroup::computeBeaconGroup() {
  // Compute centroid
  centroid.x = centroid.y = centroid.z = 0;
  imageCentroid.x = imageCentroid.y = 0;
  for (unsigned int i=0; i < numBeacons; i++) {
    centroid += *(beacons[i]);
    imageCentroid += beacons[i]->getImagePoint();
  }
  centroid /= numBeacons;
  imageCentroid /= numBeacons;
  //printf("-> id: %d, imageCentroid: (%d, %d)\n", id, imageCentroid.x, imageCentroid.y);

  // Set distances to the centroid
  for (unsigned int i=0; i < numBeacons; i++)
    beacons[i]->distance((*(beacons[i]) - centroid).abs());
  
  // Compute angles and distances
  meanDistance = 0;
  
  for (unsigned int i=0; i < numBeacons; i++) {
    Beacon& beacon = *(beacons[i]);
    Vector3<float> p = beacon - centroid;
    
    //TRACE(DEBUG, "%f, %f\t%f, %f\t%f, %f\n", p.x, p.y, centroid.x, centroid.y, beacon.x, beacon.y);
    float phi = atan2(p.y, p.x) * RAD2DEG;
    if (phi >= 0)
      beacon.phi(phi);
    else
      beacon.phi(360 + phi);
    
    float distance = sqrt(p.x * p.x + p.y * p.y);
    meanDistance += distance;
    beacon.distance(distance);
  }
  meanDistance /= numBeacons;
  
  // Order beacon group
  for (unsigned int i=0; i < numBeacons - 1; i++) {
    float minAngle = 360;
    unsigned int minAngleIndex = -1;
    for (unsigned int j=i; j < numBeacons; j++) {
      if (beacons[j]->phi() < minAngle) {
        minAngle = beacons[j]->phi();
        minAngleIndex = j;
      }
    }
    if (minAngleIndex == -1) {
      TRACE(DEBUG, "minAngle not found!");
      return;
    }
    // Swap positions
    Beacon * tempBeacon = beacons[i];
    beacons[i] = beacons[minAngleIndex];
    beacons[minAngleIndex] = tempBeacon;
  }
  
  // Set angles to relative angles between beacons
  for (unsigned int i=0; i < numBeacons - 1; i++)
    getBeacon(i).theta(getBeacon((i + 1) % numBeacons).phi() - getBeacon(i).phi());
  getBeacon(numBeacons - 1).theta(360 - getBeacon(numBeacons - 1).phi() + getBeacon(0).phi()); 
  
  /*for (unsigned int i=0; i < numBeacons; i++) {
    Beacon & beacon = getBeacon(i);
    TRACE(DEBUG, "Beacon id: %d, phi: %f, theta: %f, distance: %f\n", beacon.getId(), beacon.phi(), beacon.theta(), beacon.distance());
  }*/
}

void BeaconGroup::updateNominals(Vector3<float> wrongPosition, Vector3<float> position, float wrongHeading, float heading) {
  Vector3<float> translation(position - wrongPosition);
  translation.z = 0; // Suppose beacons are on the floor plane
  float rotation = wrongHeading - heading;
  TRACE(DEBUG, " Position: %f, %f, %f, heading: %f\n", position.x, position.y, position.z, heading);
  TRACE(DEBUG, " Wrong position: %f, %f, %f, heading: %f\n", wrongPosition.x, wrongPosition.y, wrongPosition.z, wrongHeading);
  TRACE(DEBUG, " Translation: %f, %f, %f, Rotation: %f\n", translation.x, translation.y, translation.z, rotation);
  rotation /= RAD2DEG;
  
  for (unsigned int beaconIndex = 0; beaconIndex < numBeacons; beaconIndex++) {
    Beacon& beacon = getBeacon(beaconIndex);
    
    TRACE(DEBUG, "  Beacon: %d, Position: (%f, %f, %f)\n", beacon.getId(), beacon.x, beacon.y, beacon.z);
    beacon -= wrongPosition;
    beacon.rotate(rotation);
    beacon += wrongPosition + translation;
    TRACE(DEBUG, "  Beacon: %d, Corrected position: (%f, %f, %f)\n", beacon.getId(), beacon.x, beacon.y, beacon.z);
  }
  
  computeBeaconGroup();
  locked = true;
}

void Container::clean() {
  for (unsigned int i=0; i < numBeaconGroups; i++)
    beaconGroups[i].clean();
  
  for (unsigned int i=0; i < numBeacons; i++)
    beacons[i].clean();
    
  numBeaconGroups = numBeacons = 0;
}

// Returns a clean unused beacon group
BeaconGroup & Container::getCleanBeaconGroup() {
  BeaconGroup & beaconGroup = beaconGroups[numBeaconGroups];
  beaconGroup.setId(numBeaconGroups++);
  beaconGroup.clean();
  return beaconGroup;
}
  
// Returns a clean unused beacon
Beacon & Container::getCleanBeacon() {
  Beacon & beacon = beacons[numBeacons];
  beacon.setId(numBeacons++);
  beacon.clean();
  return beacon;
}
