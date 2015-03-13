#include "Astar.h"
#include "detectionManager.h"
#include <stdint.h>
#include <math.h>

typedef int lidarPoint;


lidarPoint lDist[360] = { 0 };

//test function
setDist(uint16_t angle,uint16_t dist)
{
  lDist[angle] = dist;
}

uint16_t addObs()
{
  for (uint16_t angle = 0; angle < 360; angle++)
  {
    if (lDist[angle] != -1 && lDist[angle] != 0)
    {
      uint16_t coor = get_startCoor();//change to the position of the robot
      uint16_t obsCoor = coor + (int)(lDist[angle] * cos(angle)*G_LENGTH)
        + (int)(lDist[angle] * sin(angle));
      putObstacle(obsCoor);
      
    }
  }
  return 0;
}
