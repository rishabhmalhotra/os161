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

// CV:
static struct cv *intersectionCV;

// Lock for CV:
static struct lock *mutex;

typedef struct Vehicle
{
  Direction origin;
  Direction destination;
} Vehicle;

// bool legalPairs(Vehicle *v1, Vehicle *v2);            // true if any of the assignment conds. satisfied

// bool isVehicleMakingRightTurn(Vehicle *v);            // helper for legalPairs, is v making a right turn

// bool perVehicleConditionCheck(Vehicle *v);            // for each vehicle, if conditionCheck is false, cv_wait


// array of vehicles for perVehicleCheck():
// struct array *vehicles;

// Global var for CV (represents no of vehicles in array vehicles/no of vehicles on the roads):
// volatile int totalVehicles = 0;

volatile int NS = 0;
volatile int NE = 0;
volatile int NW = 0;
volatile int SN = 0;
volatile int SE = 0;
volatile int SW = 0;
volatile int EN = 0;
volatile int ES = 0;
volatile int EW = 0;
volatile int WN = 0;
volatile int WS = 0;
volatile int WE = 0;


// my helpers:

// bool
// isVehicleMakingRightTurn(Vehicle *v) {
//   //all true conditions for right turn:
//   if (((v->origin == north) && (v->destination == west)) ||
//       ((v->origin == west) && (v->destination == south)) ||
//       ((v->origin == south) && (v->destination == east)) ||
//       ((v->origin == east) && (v->destination == north))) {
//     return true;
//   } else {
//     return false;
//   }
// }

// bool
// legalPairs(Vehicle *v1, Vehicle *v2) {
//   if ((v1->origin == v2->origin) ||
//       ((v1->origin == v2->destination) && (v1->destination == v2->origin)) ||
//       ((v1->destination != v2->destination) && 
//         ((isVehicleMakingRightTurn(v1) == true) || (isVehicleMakingRightTurn(v2) == true)))) {
//         kprintf("legalPairs with existing %d vehicle(s), returning true\n", array_num(vehicles));
//         return true;
//       } else {
//         kprintf("not legal Pairs with o:%d, d:%d\n", v2->origin, v2->destination);
//         return false;
//       }
// }

// bool
// perVehicleConditionCheck(Vehicle *v) {
//   lock_acquire(mutex);
//   if (array_num(vehicles) > 0) {
//     for (unsigned int i=0; i<array_num(vehicles); i++) {
//       while (legalPairs(v, array_get(vehicles, i)) == false) {
//         kprintf("Will crash, not legal pair with something so going to cv wait\n");
//         cv_wait(intersectionCV, mutex);
//         return false;
//       }
//     }
//   }

//   // verify curthread is lock owner:
//   KASSERT(lock_do_i_hold(mutex));
//   array_add(vehicles, v, NULL);
//   kprintf("Now array has %d Vehicles, incrementing totalVehicles\n", array_num(vehicles));
//   totalVehicles++;
//   lock_release(mutex);
//   return true;
// }


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
  // create CV for intersection:
  intersectionCV = cv_create("intersectionCV");
  if (intersectionCV == NULL) {
    panic("couldn't create intersection CV");
  }

  // create lock for above CV:
  mutex = lock_create("mutex");
  if (mutex == NULL) {
    panic("couldn't create mutex");
  }

  // array for perVehicleCheck():
  // vehicles = array_create();
  // array_init(vehicles);
  // if (vehicles == NULL) {
  //   panic("couldn't create vehicles[]");
  // }

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
  // KASSERT(vehicles != NULL);

  lock_destroy(mutex);
  cv_destroy(intersectionCV);
  // array_destroy(vehicles);
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
  // KASSERT(vehicles != NULL);
  KASSERT(intersectionCV != NULL);
  KASSERT(mutex != NULL);

  lock_acquire(mutex);

  // make vehicle
  Vehicle *v = kmalloc(sizeof(struct Vehicle));
  KASSERT(v != NULL);
  v->origin = origin;
  v->destination = destination;

  if (v->origin == north) {
    if (v->destination == south) {                                                            // straight line (NS)
      if ((SW == 0) && (EW == 0) && (ES == 0) && (WE == 0) && (WS == 0) && (WN == 0)) {
        NS++;
      } else {
        cv_wait(intersectionCV, mutex);
      }
    } else if (v->destination == east) {                                                     // left turn (NE)
      if ((WN == 0) && (WE == 0) && (SN == 0) && (SW == 0) && (ES == 0) && (EW == 0)) {
        NE++;
      } else {
        cv_wait(intersectionCV, mutex);
      }
    } else {                                                                                // right turn (NW)
      if ((SW == 0) && (EW == 0)) {
        NW++;
      } else {
        cv_wait(intersectionCV, mutex);
      }
    }
  } else if (v->origin == south) {
    if (v->destination == north) {                                                            // straight line (SN)
      if ((EW == 0) && (ES == 0) && (EN == 0) && (WE == 0) && (WN == 0) && (NE == 0)) {
        SN++;
      } else {
        cv_wait(intersectionCV, mutex);
      }
    } else if (v->destination == east) {                                                     // right turn (SE)
      if ((WE == 0) && (NE == 0)) {
        SE++;
      } else {
        cv_wait(intersectionCV, mutex);
      }
    } else {                                                                                 // dest = west
      if ((EW == 0) && (WE == 0) && (ES == 0) && (NS == 0) && (NE == 0) && (WN == 0)) {      // left turn (SW)
        SW++;
      } else {
        cv_wait(intersectionCV, mutex);
      }
    }
  } else if (v->origin == east) {
    if (v->destination == north) {                                                           // right turn (EN)
      if ((SN == 0) && (WN == 0)) {
        EN++;
      } else {
        cv_wait(intersectionCV, mutex);
      }
    } else if (v->destination == south) {                                                    // left turn (ES)
      if ((WS == 0) && (WE == 0) && (WN == 0) && (SN == 0) && (NS == 0) && (NE == 0)) {
        ES++;
      } else {
        cv_wait(intersectionCV, mutex);
      }
    } else {                                                     // straight line (EW)
      if ((SE == 0) && (SN == 0) && (SW == 0) && (NS == 0) && (NE == 0) && (EN == 0)) {
        EW++;
      } else {
        cv_wait(intersectionCV, mutex);
      }
    }
  } else {                                                                                // Origin is West
    if (v->destination == north) {                                                            // left turn (WN)
      if ((SN == 0) && (SW == 0) && (EW == 0) && (ES == 0) && (NS == 0) && (NE == 0)) {
        WN++;
      } else {
        cv_wait(intersectionCV, mutex);
      }
    } else if (v->destination == south) {                                                     // right turn (WS)
      if ((NS == 0) && (ES == 0)) {
        WS++;
      } else {
        cv_wait(intersectionCV, mutex);
      }
    } else {                                                     // straight line (WE)
      if ((ES == 0) && (SW == 0) && (SN == 0) && (NS == 0) && (NE == 0) && (SE == 0)) {
        WE++;
      } else {
        cv_wait(intersectionCV, mutex);
      }
    }
  }

  // perVehicleConditionCheck checks conditions of this v with every other existing v:
  // while (perVehicleConditionCheck(v) == false) {
  //   // do nothing:
  //   totalVehicles += 0;
  // }

  // added v to array for future per vehicle checks
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
  // KASSERT(vehicles != NULL);
  KASSERT(intersectionCV != NULL);
  KASSERT(mutex != NULL);

  lock_acquire(mutex);
  // make vehicle
  Vehicle *v = kmalloc(sizeof(struct Vehicle));
  KASSERT(v != NULL);
  v->origin = origin;
  v->destination = destination;

  if (v->origin == north) {
    if (v->destination == south) {
      NS--;
      cv_broadcast(intersectionCV, mutex);
    } else if (v->destination == east) {
      NE--;
      cv_broadcast(intersectionCV, mutex);
    } else {                                                                            // dest = west
      NW--;
      cv_broadcast(intersectionCV, mutex);
    }
  } else if (v->origin == south) {
    if (v->destination == north) {
      SN--;
      cv_broadcast(intersectionCV, mutex);
    } else if (v->destination == east) {
      SE--;
      cv_broadcast(intersectionCV, mutex);
    } else {                                                                            // dest = west
      SW--;
      cv_broadcast(intersectionCV, mutex);
    }
  } else if (v->origin == east) {
    if (v->destination == north) {
      EN--;
      cv_broadcast(intersectionCV, mutex);
    } else if (v->destination == south) {
      ES--;
      cv_broadcast(intersectionCV, mutex);
    } else {                                                                              // dest = west
      EW--;
      cv_broadcast(intersectionCV, mutex);
    }
  } else {                                                                                // Origin is West
    if (v->destination == north) {
      WN--;
      cv_broadcast(intersectionCV, mutex);
    } else if (v->destination == south) {
      WS--;
      cv_broadcast(intersectionCV, mutex);
    } else {                                                                              // dest = east
      WE--;
      cv_broadcast(intersectionCV, mutex);
    }
  }

  lock_release(mutex);

  // // chuck out exiting vehicle from array to keep vehicles[] relevant:
  // for (unsigned int i=0; i<array_num(vehicles); i++) {
  //   Vehicle *v = array_get(vehicles, i);
  //   if ((v->origin = origin) && (v->destination = destination)) {
  //     kprintf("found matching v inside intersection, chucking it off\n");
  //     lock_acquire(mutex);
  //     array_remove(vehicles, i);
  //     totalVehicles --;
  //     cv_broadcast(intersectionCV, mutex);
  //     lock_release(mutex);
  //     break;
  //   }
  // }
  // kprintf("end of this exit\n");
}
