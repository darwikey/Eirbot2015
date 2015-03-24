#ifndef ALGO_H
#define ALGO_H
#include "stdint.h"
#include <math.h>
#include <stdint.h>

//#include "task_manager.h"
//#include "avoidance.h"

#define DIST  10
#define OBSTACLE 1
#define GOAL 2
#define START 3
#define OPENLIST 4
#define CLOSEDLIST 5
#define UNIT 50
#define LENGTH (3000 )
#define G_LENGTH 60
#define WIDTH (2000 )
#define G_WIDTH 40
#define G_SIZE (G_WIDTH*G_LENGTH)
#define OUT 0
#define STACK_SIZE 200
#define DRIVE 1
#define ROTATE 2

typedef struct node node;
struct node
{
  uint16_t parent;//2o
  //uint8_t remainDist;//0o//hcost
  float crossedDist;//1o//gcost
  //int allDist;//0o//fcost
  int8_t type;//1o
  uint16_t coor;//2o
};


typedef struct coordinate
{
  uint16_t x;
  uint16_t y;
} coordinate;


typedef coordinate mvStackElement;



uint8_t aStarLoop(void);
float findDist(node node1, node node2);
int8_t isVoid(int8_t list);
void initNeighbors(uint16_t current, uint16_t *neighbors);
uint16_t findBest(uint8_t openlist);
//int findDistC(node node1, node node2);

coordinate getCoordinate(node n);
coordinate positionMmToCoordinate(float x, float y);
node getNode(uint8_t x, uint8_t y);


void initObstacle(void);
int8_t astarMv(void);
void printGraphe(void);
void putObstacle(uint16_t coor);
void deleteObstacle(uint16_t coor);
void clearGraphe(void);

uint16_t get_startCoor(void);
uint16_t get_goalCoor(void);
void set_startCoor(coordinate coor);
void set_goalCoor(coordinate coor);


void stopAstarMovement(void);

//
#endif
