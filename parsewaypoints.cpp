#include "rapidxml/rapidxml.hpp"
#include "rapidxml/rapidxml_utils.hpp"
#include "beacon.h"
#include <string.h>

using namespace rapidxml;

extern Container nominalContainer;
extern Container beaconContainer;

void parseWaypoints() {
  try {
    file<> xmlFile("waypoints.xml");
    xml_document<> doc;
    doc.parse<0>(xmlFile.data());
    xml_node<> *node = doc.first_node("waypoints");
    
    // Iterate for each waypoint
    for (xml_node<> *wchild = node->first_node(); wchild; wchild = wchild->next_sibling()) {
      BeaconGroup &beaconGroup = nominalContainer.getCleanBeaconGroup();
      
      // Set waypoint name
      //xml_attribute<> *attr = wchild->first_attribute("name");
      //strcpy(beaconGroup.name, attr->value());
      
      xml_attribute<> *attr = wchild->first_attribute("id");
      beaconGroup.setId(atoi(attr->value()));
      
      float tx, ty, tz = 0;
      if (attr = wchild->first_attribute("x"))
        tx = atof(attr->value());
      if (attr = wchild->first_attribute("y"))
        ty = atof(attr->value());
      if (attr = wchild->first_attribute("z"))
        tz = atof(attr->value());
      if (attr = wchild->first_attribute("master"))
        beaconGroup.setMaster(atoi(attr->value()) != 0 ? true : false);
      

      // Iterate for each beacon in the current waypoint
      int numBeacons = 0;
      for (xml_node<> *bchild = wchild->first_node(); bchild; bchild = bchild->next_sibling()) {
        Beacon & beacon = nominalContainer.getCleanBeacon();
        
        float x = atof((bchild->first_attribute("x"))->value());
        float y = atof((bchild->first_attribute("y"))->value());
        float z = atof((bchild->first_attribute("z"))->value());
        int id = atoi((bchild->first_attribute("id"))->value());
        
        beacon.loadBeacon(x + tx, y + ty, z + tz, id);
        beaconGroup.appendBeacon(beacon);
        numBeacons++;
      }
      if (numBeacons > nominalContainer.getMaxBeacons())
        nominalContainer.setMaxBeacons(numBeacons);
    
      beaconGroup.computeBeaconGroup();
    }
  }
  
  catch(std::exception &e) {
    printf("Exception parsing xml file: %s. Exiting...\n", e.what());
    exit(1);
  }
}
