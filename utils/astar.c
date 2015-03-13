#include "astar.h"
#include "stdio.h"
#include "smooth_traj_manager.h"

//extern uint16_t stack_begin;
//graphe où sera charge la table et ses obstacles
node graphe[G_SIZE] = { 0 };
uint16_t startCoor;
uint16_t goalCoor;
int stopMovement = 0;
/*
unsigned int abs(int val)
{
  if (val<0)return -val;
  else return val;
}*/

uint8_t aStarLoop()
{
  coordinate coor;
  uint16_t neighbors[8];
  graphe[startCoor].type = CLOSEDLIST; //OPENLIST is list of node we have to analysed//list is a type 
  //graphe[startCoor].remainDist = findDist(graphe[startCoor], graphe[goalCoor]);
  uint16_t current = startCoor;


  if (startCoor == goalCoor)
  {
    return 1;
  }

  printf("start algorithme \r\n");
  while (graphe[goalCoor].type != CLOSEDLIST)//closed list is nodes which are already analysed
  {
    printf("current %d \r\n", current);
    //init neighbors
    initNeighbors(current, neighbors);
    for (uint8_t i = 0; i < 8; i++)
    {
      if (graphe[neighbors[i]].type != OBSTACLE && graphe[neighbors[i]].type != CLOSEDLIST && neighbors[i] != OUT)
      {
        if (graphe[neighbors[i]].type == OPENLIST)
        {
          //update the node if it's already in the openlist
          if (graphe[neighbors[i]].crossedDist > (findDist(graphe[neighbors[i]], graphe[current]) + graphe[current].crossedDist))
          {
            graphe[neighbors[i]].parent = graphe[current].coor;
            graphe[neighbors[i]].crossedDist = findDist(graphe[neighbors[i]], graphe[current]) + graphe[current].crossedDist;
          }
        }
        else//add it
        {
          graphe[neighbors[i]].parent = graphe[current].coor;
          //  graphe[neighbors[i]].remainDist = findDist(graphe[neighbors[i]], graphe[goalCoor]);
          graphe[neighbors[i]].crossedDist = findDist(graphe[neighbors[i]], graphe[current]) + graphe[current].crossedDist;
          graphe[neighbors[i]].type = OPENLIST;
        }
      }
    }
    if (isVoid(OPENLIST))
    {

      break;
    }
    current = findBest(OPENLIST);
    coor = getCoor(graphe[current]);
    printf("(%d,%d) \r\n", (int)coor.x, (int)coor.y);
    //put the analysed node in the closedList
    graphe[current].type = CLOSEDLIST;
  }
  if (graphe[goalCoor].parent == 0)
  {
    printf("path not found");
    return 0;
  }
  else
  {

    //reconstruct the path
    printf("path: \n");
    while (current != startCoor)
    {
      coor = getCoor(graphe[current]);
      printf("(%d,%d), ", (int)coor.x, (int)coor.y);
      current = graphe[graphe[current].parent].coor;
    }
    coor = getCoor(graphe[startCoor]);
    printf("(%d,%d), ", (int)coor.x, (int)coor.y);
    return 1;
  }
}

float findDist(node node1, node node2)
{
  coordinate c1 = getCoor(node1);
  coordinate c2 = getCoor(node2);
  //trial with manhattan dist
  //upgrade
  float dist = sqrt((c1.x - c2.x)*(c1.x - c2.x) + (c1.y - c2.y)*(c1.y - c2.y));


  //we can change with defined dist for the neighbors dist
  return dist;
}


int8_t isVoid(int8_t list)
{
  for (uint16_t i = 1; i < G_SIZE; i++)
  {
    if (graphe[i].type == list)
    {
      return 0;
    }
  }
  return 1;
}

void initNeighbors(uint16_t current, uint16_t *neighbors)
{

  neighbors[0] = current - 1 * G_LENGTH - 1;
  neighbors[1] = current - 1 * G_LENGTH;
  neighbors[2] = current - 1 * G_LENGTH + 1;
  neighbors[3] = current - 1;
  neighbors[4] = current + 1;
  neighbors[5] = current + 1 * G_LENGTH - 1;
  neighbors[6] = current + 1 * G_LENGTH;
  neighbors[7] = current + 1 * G_LENGTH + 1;

  if (current - 1 * G_LENGTH - 1 > G_SIZE || current - 1 * G_LENGTH - 1 < 0 || current%G_LENGTH < 1)
  {
    neighbors[0] = OUT;
  }
  if (current - 1 * G_LENGTH > G_SIZE || current - 1 * G_LENGTH < 0)
  {
    neighbors[1] = OUT;
  }
  if (current - 1 * G_LENGTH + 1 > G_SIZE || current - 1 * G_LENGTH + 1 < 0 || current%G_LENGTH >= G_LENGTH - 1)
  {
    neighbors[2] = OUT;
  }
  if (current - 1 > G_SIZE || current - 1 < 0 || current%G_LENGTH < 1)
  {
    neighbors[3] = OUT;
  }
  if (current + 1 > G_SIZE || current + 1 < 0 || current%G_LENGTH >= G_LENGTH - 1)
  {
    neighbors[4] = OUT;
  }
  if (current + 1 * G_LENGTH - 1 > G_SIZE || current + 1 * G_LENGTH - 1 < 0 || current%G_LENGTH < 1)
  {
    neighbors[5] = OUT;
  }
  if (current + 1 * G_LENGTH > G_SIZE || current + 1 * G_LENGTH < 0)
  {
    neighbors[6] = OUT;
  }
  if (current + 1 * G_LENGTH + 1 > G_SIZE || current + 1 * G_LENGTH + 1 < 0 || current%G_LENGTH >= G_LENGTH - 1)
  {
    neighbors[7] = OUT;
  }
}

uint16_t findBest(uint8_t openlist)
{
  uint16_t best = 0;
  uint16_t max = 0xffff;//changer si trop couteux
  uint16_t i = 0;
  while (i < G_SIZE)
  {
    if (graphe[i].type == openlist)
    {
      if (graphe[i].crossedDist + findDist(graphe[i], graphe[goalCoor]) < max)
      {
        max = graphe[i].crossedDist + findDist(graphe[i], graphe[goalCoor]);
        best = graphe[i].coor;
        printf("newBest %d \r\n", (int)graphe[i].coor);
        printf("i = %d \r\n", (int)i);
      }
    }
    i++;
  }

  return best;
}

coordinate getCoor(node n)
{
  coordinate c;
  c.x = n.coor % G_LENGTH;
  c.y = n.coor / G_LENGTH;
  return c;
}

node getNode(uint8_t x,uint8_t y)
{
  return graphe[x + y*G_LENGTH];
}

void initObstacle()
{

  for (int i = 0; i < G_LENGTH; i++)
  {
    graphe[i].type = OBSTACLE;
  }
  for (int j = 0; j < G_WIDTH; j++)
  {
    graphe[j*G_LENGTH].type = OBSTACLE;
  }

  for (int j = 15; j < G_WIDTH; j++)
  {
    graphe[j*G_LENGTH + 30].type = OBSTACLE;
  }
}


// diminue le nombre de point dans la trajectoire
void polishing(mvStack *s)
{
  node current = graphe[goalCoor];
  uint8_t type = 10;
  mvStackElement e;
  e.x = 0;
  e.y = 0;
  push(s, e);
  while (current.coor != startCoor)
  {
    graphe[current.coor].type = 7;//à enlever
    printf("polishing %d %d \n", (int)current.coor, (int)type);
    if (current.parent == current.coor - 1 * G_LENGTH - 1 && type!=0)//0
      //012
      //3x4
      //567
    {
        e.x = current.coor%G_LENGTH;
        e.y = current.coor/G_LENGTH;
        push(s, e);

        type = 0;
    }
    else if (current.parent == current.coor - 1 * G_LENGTH && type != 1)//1
    {
      e.x = current.coor%G_LENGTH;
      e.y = current.coor / G_LENGTH;
      push(s, e);

      type = 1;
    }
    else if (current.parent == current.coor - 1 * G_LENGTH + 1 && type != 2)//2
    {
      e.x = current.coor%G_LENGTH;
      e.y = current.coor / G_LENGTH;
      push(s, e);

      type = 2;
    }
    else if (current.parent == current.coor - 1 && type != 3)//3
    {
      e.x = current.coor%G_LENGTH;
      e.y = current.coor / G_LENGTH;
      push(s, e);

      type = 3;
    }
    else if (current.parent == current.coor + 1 && type != 4)//4
    {
      e.x = current.coor%G_LENGTH;
      e.y = current.coor / G_LENGTH;
      push(s, e);

      type = 4;
    }
    else if (current.parent == current.coor + 1 * G_LENGTH - 1 && type != 5)//5
    {
      e.x = current.coor%G_LENGTH;
      e.y = current.coor / G_LENGTH;
      push(s, e);

      type = 5;
    }
    else if (current.parent == current.coor + 1 * G_LENGTH && type != 6)//6
    {
      e.x = current.coor%G_LENGTH;
      e.y = current.coor / G_LENGTH;
      push(s, e);

      type = 6;
    }
    else if (current.parent == current.coor + 1 * G_LENGTH + 1 && type != 7)//7
    {
      e.x = current.coor%G_LENGTH;
      e.y = current.coor / G_LENGTH;
      push(s, e);

      type = 7;
    }
    else
    {
      printf("Error path not found\n");
    }
    current = graphe[current.parent];
  }
  push(s, e);

}

//stack primitive
mvStackElement pop(mvStack *s)
{
  if (stack_is_empty(s))
  {
    mvStackElement empty = { 0, 0 };
    return empty;
  }
  s->top--;
  return s->items[s->top];
}

void push(mvStack *s, mvStackElement item)
{
  if (stack_full(s))
  {
    printf("stack full \n");
    return;
  }
  s->items[s->top++] = item;
}

uint8_t stack_size(mvStack *s)
{
  return s->top;
}

uint8_t stack_full(mvStack *s)
{
  return s->top == STACK_SIZE;
}

uint8_t stack_is_empty(mvStack *s)
{
  return !s->top;
}
void stack_clear(mvStack *s)
{
  s->top = 0;
  for (int i = 0; i<STACK_SIZE; i++)
  {
    s->items[i].x = 0;
    s->items[i].y = 0;
  }
}

mvStack initStack(void)
{
  mvStack s;
  s.top = 0;
  return s;
}



mvStack stack;//pile d'instructions

int8_t astarMv()
{
  stopMovement = 0;
  printf("A \n");
  //disableAvoidance();
  //set_detection_behaviour(BEHAVIOUR_ASTAR);
  //init graphe
  uint16_t i = 0;
  for (i = 0; i < G_SIZE; i++)
  {
    graphe[i].coor = i;
    graphe[i].parent = 0;
    //graphe[i].remainDist = 0;
    graphe[i].crossedDist = 0;
    if (graphe[i].type != OBSTACLE)
    {
      graphe[i].type = 0;
    }
  }
  printf("astar test \n");



  int8_t bool = aStarLoop();

  //supprimer les obstacle
  //clearGraphe();
  initObstacle();
  //si pas de chemin trouve 
  if (bool == 0)
  {
    printf("no path found \n");
    //actionFailed();
    return 0;
  }


  stack_clear(&stack);//pile d'instructions


  polishing(&stack);
  mvStackElement currentElement;
  currentElement.x = startCoor % G_LENGTH;
  currentElement.y = startCoor / G_LENGTH;
  printf(" begin stack \n");
  
  smooth_traj_goto_xy_mm(UNIT * currentElement.y, UNIT * currentElement.x);
  printf("Add point (%d,%d) \n", UNIT * (startCoor % G_LENGTH),  UNIT * (startCoor / G_LENGTH) );
  //printf("start: %d \n", startCoor%G_LENGTH);

  while (currentElement.x != 0 && currentElement.y != 0 && !stopMovement)
  {
    currentElement = pop(&stack);
    smooth_traj_goto_xy_mm(UNIT * currentElement.y, UNIT * currentElement.x);
    printf("Add point (%d,%d) \n", UNIT * currentElement.x, UNIT * currentElement.y);
    //printf("stackelement:%d,%d \n", currentElement.x, currentElement.y);
    
  }

  //enableAvoidance();
  //while (!trajectory_is_ended());
  printf("end stack \n");

  if (stopMovement)
  {
    return 0;
  }

  printf("succeed \n");
  //set_detection_behaviour(BEHAVIOUR_STOP);


  return 1;
}


uint16_t get_startCoor()
{
  return startCoor;
}

uint16_t get_goalCoor()
{
  return goalCoor;
}

void set_startCoor(uint16_t coor)
{
  startCoor = coor;
}

void set_goalCoor(uint16_t coor)
{
  goalCoor = coor;
}

void stopAstarMovement()
{
  stopMovement = 1;
  stack_clear(&stack);
}

void putObstacle(uint16_t coor)
{
  if (coor < G_SIZE && coor >0)
    graphe[coor].type = OBSTACLE;
}
void deleteObstacle(uint16_t coor)
{
  graphe[coor].type = 0;
}

void printGraphe(void)
{

  for (uint16_t i = 0; i < G_SIZE; ++i)
  {
    if (i%G_LENGTH == 0)
    {
      printf(" \r\n");
      printf(" %d \t", i / G_LENGTH);
    }
    if (i == goalCoor)
    {
      printf("g ");
    }
    else if (i == startCoor)
    {
      printf("# ");
    }
    else
    printf("%d ", (int)graphe[i].type);
  }
}

void clearGraphe(void)
{
  for (uint16_t i = 0; i < G_SIZE; ++i)
  {
    /* code */
    graphe[i].type = 0;
  }
}
