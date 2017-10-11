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

  lock_destroy(mutex);
  cv_destroy(intersectionCV);
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
  KASSERT(intersectionCV != NULL);
  KASSERT(mutex != NULL);

  lock_acquire(mutex);

  // make vehicle
  Vehicle *v = kmalloc(sizeof(struct Vehicle));
  KASSERT(v != NULL);
  v->origin = origin;
  v->destination = destination;

  if (v->origin == north) {
    if (v->destination == south) {                                                           // straight line (NS) 
        while ((SW != 0) && (EW != 0) && (ES != 0) && (WE != 0) && (WS != 0) && (WN != 0)) {
          cv_wait(intersectionCV, mutex);
        }
        NS++;
    } else if (v->destination == east) {                                                     // left turn (NE)
        while ((WN != 0) && (WE != 0) && (SN != 0) && (SW != 0) && (ES != 0) && (EW != 0)) {
          cv_wait(intersectionCV, mutex);
        }
        NE++;
    } else {                                                                                 // right turn (NW)
        while ((SW != 0) && (EW != 0)) {
          cv_wait(intersectionCV, mutex);
        }
        NW++;
    }
  } else if (v->origin == south) {
    if (v->destination == north) {                                                           // straight line (SN)
        while ((EW != 0) && (ES != 0) && (EN != 0) && (WE != 0) && (WN != 0) && (NE != 0)) {
          cv_wait(intersectionCV, mutex);
        }
        SN++;
    } else if (v->destination == east) {                                                     // right turn (SE)
        while ((WE != 0) && (NE != 0)) {
          cv_wait(intersectionCV, mutex);
        }
        SE++;
    } else {                                                                                 // dest = west, left turn (SW)
        while ((EW != 0) && (WE != 0) && (ES != 0) && (NS != 0) && (NE != 0) && (WN != 0)) {
          cv_wait(intersectionCV, mutex);
        }
        SW++;
    }
  } else if (v->origin == east) {
    if (v->destination == north) {                                                           // right turn (EN)
        while ((SN != 0) && (WN != 0)) {
          cv_wait(intersectionCV, mutex);
        }
        EN++;
    } else if (v->destination == south) {                                                    // left turn (ES)
        while ((WS != 0) && (WE != 0) && (WN != 0) && (SN != 0) && (NS != 0) && (NE != 0)) {
          cv_wait(intersectionCV, mutex);
        }
        ES++;
    } else {                                                                                  // straight line (EW)
        while ((SE != 0) && (SN != 0) && (SW != 0) && (NS != 0) && (NE != 0) && (EN != 0)) {
          cv_wait(intersectionCV, mutex);
        }
        EW++;
    }
  } else {                                                                                    // Origin is West
    if (v->destination == north) {                                                            // left turn (WN)
        while ((SN != 0) && (SW != 0) && (EW != 0) && (ES != 0) && (NS != 0) && (NE != 0)) {
          cv_wait(intersectionCV, mutex);
        }
        WN++;
    } else if (v->destination == south) {                                                     // right turn (WS)
        while ((NS != 0) && (ES != 0)) {
          cv_wait(intersectionCV, mutex);
        }
        WS++;
      }
    } else {                                                                                  // straight line (WE)
        while ((ES != 0) && (SW != 0) && (SN != 0) && (NS != 0) && (NE != 0) && (SE != 0)) {
          cv_wait(intersectionCV, mutex);
        }
        WE++;
    }
    
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
    } else {                                                                            // dest = west
      EW--;
      cv_broadcast(intersectionCV, mutex);
    }
  } else {                                                                              // Origin is West
    if (v->destination == north) {
      WN--;
      cv_broadcast(intersectionCV, mutex);
    } else if (v->destination == south) {
      WS--;
      cv_broadcast(intersectionCV, mutex);
    } else {                                                                            // dest = east
      WE--;
      cv_broadcast(intersectionCV, mutex);
    }
  }

  lock_release(mutex);
}
