#include <types.h>
#include <lib.h>
#include <synchprobs.h>
#include <synch.h>
#include <opt-A1.h>
#include <array.h>

/* 
 * This simple default synchronization mechanism allows only vehicle at a time
 * into the intersection.   The intersectionSem is used as a a lock.
 * We use a semaphore rather than a lock so that this code will work even
 * before locks are implemented.
 */

/* 
 * Replace this default synchronization mechanism with your own (better) mechanism
 * needed for your solution.   Your mechanism may use any of the available synchronzation
 * primitives, e.g., semaphores, locks, condition variables.   You are also free to 
 * declare other global variables if your solution requires them.
 */

// CV:
static struct cv *intersectionCV;

// Global var for CV (represents no of vehicles in array vehicles/no of vehicles on the roads):
volatile int totalVehicles = 0;

// Lock for CV:
static struct lock *mutex;

typedef struct Vehicle
{
  Direction origin;
  Direction destination;
} Vehicle;

bool checkConditions(Vehicle *v1, Vehicle *v2);       // true if any of the assignment conds. satisfied

bool isVehivleMakingRightTurn(Vehicle *v);            // helper for checkConditions, is v making a right turn

bool perVehicleConditionCheck(Vehicle *v);            // for each vehicle, if conditionCheck is false, cv_wait


// array of vehicles for perVehicleCheck():
struct array *vehicles;

// my helpers:

bool
isVehicleMakingRightTurn(Vehicle *v) {
  //all true conditions for right turn:
  if (((v->origin == north) && (v->destination == west)) ||
      ((v->origin == west) && (v->destination == south)) ||
      ((v->origin == south) && (v->destination == east)) ||
      ((v->origin == east) && (v->destination == north))) {
    return true;
  } else {
    return false;
  }
}

bool
checkConditions(Vehicle *v1, Vehicle *v2) {
  if ((v1->origin == v2->origin) ||
      ((v1->origin == v2->destination) && (v1->destination == v2->origin)) ||
      ((v1->destination != v2->destination) && 
        ((isVehicleMakingRightTurn(v1) || (isVehicleMakingRightTurn(v2)))))) {
        return true;
      } else {
        return false;
      }
}

bool
perVehicleConditionCheck(Vehicle *v) {
  for (unsigned int i=0; i<array_num(vehicles); i++) {
    if (checkConditions(vehicle, array_get(vehicles, i)) == false) {
      cv_wait(intersectionCV, mutex);
      return false;
    }
  }

  // verify curthread is lock owner:
  KASSERT(lock_do_i_hold(mutex));
  totalVehicles++;
  return true;
}


/* 
 * The simulation driver will call this function once before starting
 * the simulation
 *
 * You can use it to initialize synchronization and other variables.
 * 
 */
void
intersection_sync_init(void)
{
  /* replace this default implementation with your own implementation */

  kprintf("intersection_sync_init starting\n");

  // create CV for intersection:
  intersectionCV = cv_create("intersectionCV");
  if (intersectionCV == NULL) {
    panic("couldn't create intersection CV");
  }

  // create lock for above CV:
  mutex = lock_create("mutex");
  if (mutex == NULL) {
    pamic("couldn't create mutex")
  }

  // array for perVehicleCheck():
  vehicles = array_create();
  array_init(vehicles);
  if (vehicles == NULL) {
    pamic("couldn't create vehicles[]")
  }

  return;
}

/* 
 * The simulation driver will call this function once after
 * the simulation has finished
 *
 * You can use it to clean up any synchronization and other variables.
 *
 */
void
intersection_sync_cleanup(void)
{
  KASSERT(intersectionCV != NULL);
  KASSERT(mutex != NULL);
  KASSERT(vehicles != NULL);

  lock_destroy(mutex);
  cv_destroy(intersectionCV);
  array_destroy(vehicles);
}


/*
 * The simulation driver will call this function each time a vehicle
 * tries to enter the intersection, before it enters.
 * This function should cause the calling simulation thread 
 * to block until it is OK for the vehicle to enter the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle is arriving
 *    * destination: the Direction in which the vehicle is trying to go
 *
 * return value: none
 */

void
intersection_before_entry(Direction origin, Direction destination) 
{
  /* replace this default implementation with your own implementation */
  //(void)origin;  /* avoid compiler complaint about unused parameter */
  //(void)destination; /* avoid compiler complaint about unused parameter */

  KASSERT(intersectionCV != NULL);
  KASSERT(mutex != NULL);
  //KASSERT(vehicles != NULL);

  lock_acquire(mutex);

  // make vehicle
  Vehicle *v = kmalloc(sizeof(struct Vehicle));
  KASSERT(v != NULL);
  v->origin = origin;
  v->destination = destination;

  // perVehicleConditionCheck checks conditions of this v with every other existing v:
  while (perVehicleConditionCheck(v) == false) {
    // do nothing, don't increment totalVehicles, just inside here
  }
  // add v to array for future per vehicle checks:
  array_add(vehicles, v, NULL);
  lock_release(mutex);
}


/*
 * The simulation driver will call this function each time a vehicle
 * leaves the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle arrived
 *    * destination: the Direction in which the vehicle is going
 *
 * return value: none
 */

void
intersection_after_exit(Direction origin, Direction destination) 
{
  KASSERT(intersectionCV != NULL);
  KASSERT(mutex != NULL);

  lock_acquire(mutex);

  // chuck out exiting vehicle from array to keep vehicles[] relevant:
  for (unsigned int i=0; i<array_num(vehicles); i++) {
    Vehicle *v = array_get(vehicles, i);
    if ((v->origin = origin) && (v->destination = destination)) {
      array_remove(vehicles, i);
      cv_broadcast(intersectionCV, mutex);
      // 1 vehicle gone from the picture, loop ends since v chucked out
      totalVehicles --;
      break;
    }
  }
  loack_release(mutex);
}