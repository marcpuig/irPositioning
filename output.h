#include <string>
#include <math.h>

#define LOC_TYPE 0
#define ORI_TYPE 1
#define NUM_TYPES 2
#define MAX_CLIENTS 10

extern void setOffsets();

class Output {
public:
  Output(int port);
  void Send(std::string message, int type);
};