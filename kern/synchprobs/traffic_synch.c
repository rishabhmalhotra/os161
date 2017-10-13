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
static struct cv *cv1;
static struct cv *cv2;
static struct cv *cv3;
static struct cv *cv4;
static struct cv *cv5;
static struct cv *cv6;
static struct cv *cv7;
static struct cv *cv8;
static struct cv *cv9;
static struct cv *cv10;
static struct cv *cv11;
static struct cv *cv12;

// Lock for CV:
static struct lock *mutex;

struct array *pqueue;

typedef struct Vehicle
{
  Direction origin;
  Direction destination;
} Vehicle;


                            // Corresponding CVs:
volatile int NS = 0;        //    cv1                                    
volatile int NE = 0;        //    cv2
volatile int NW = 0;        //    cv3
volatile int SN = 0;        //    cv4
volatile int SE = 0;        //    cv5
volatile int SW = 0;        //    cv6
volatile int EN = 0;        //    cv7
volatile int ES = 0;        //    cv8
volatile int EW = 0;        //    cv9
volatile int WN = 0;        //    cv10
volatile int WS = 0;        //    cv11
volatile int WE = 0;        //    cv12

// forward declaration
void checkForCvAndBroadcast(void);


// macros for cv true conditions:
#define cv1trueconditions ((SW == 0) && (EW == 0) && (ES == 0) && (WE == 0) && (WS == 0) && (WN == 0))
#define cv2trueconditions ((WN == 0) && (WE == 0) && (SN == 0) && (SW == 0) && (ES == 0) && (EW == 0) && (SE == 0))
#define cv3trueconditions ((SW == 0) && (EW == 0))
#define cv4trueconditions ((EW == 0) && (ES == 0) && (EN == 0) && (WE == 0) && (WN == 0) && (NE == 0))
#define cv5trueconditions ((WE == 0) && (NE == 0))
#define cv6trueconditions ((EW == 0) && (WE == 0) && (ES == 0) && (NS == 0) && (NE == 0) && (WN == 0) && (NW == 0))
#define cv7trueconditions ((SN == 0) && (WN == 0))
#define cv8trueconditions ((WS == 0) && (WE == 0) && (WN == 0) && (SN == 0) && (NS == 0) && (NE == 0) && (SW == 0)) 
#define cv9trueconditions ((NW == 0) && (SN == 0) && (SW == 0) && (NS == 0) && (NE == 0) && (WN == 0))
#define cv10trueconditions ((SN == 0) && (SW == 0) && (EW == 0) && (ES == 0) && (EN == 0) && (NS == 0) && (NE == 0))
#define cv11trueconditions ((NS == 0) && (ES == 0))
#define cv12trueconditions ((ES == 0) && (SW == 0) && (SN == 0) && (NS == 0) && (NE == 0) && (SE == 0))

// macros for cv false conditions:
#define cv1falseconditions ((SW != 0) || (EW != 0) || (ES != 0) || (WE != 0) || (WS != 0) || (WN != 0))
#define cv2falseconditions ((WN != 0) || (WE != 0) || (SN != 0) || (SW != 0) || (ES != 0) || (EW != 0) || (SE != 0))
#define cv3falseconditions ((SW != 0) || (EW != 0))
#define cv4falseconditions ((EW != 0) || (ES != 0) || (EN != 0) || (WE != 0) || (WN != 0) || (NE != 0))
#define cv5falseconditions ((WE != 0) || (NE != 0))
#define cv6falseconditions ((EW != 0) || (WE != 0) || (ES != 0) || (NS != 0) || (NE != 0) || (WN != 0) || (NW != 0))
#define cv7falseconditions ((SN != 0) || (WN != 0))
#define cv8falseconditions ((WS != 0) || (WE != 0) || (WN != 0) || (SN != 0) || (NS != 0) || (NE != 0) || (SW != 0)) 
#define cv9falseconditions ((NW != 0) || (SN != 0) || (SW != 0) || (NS != 0) || (NE != 0) || (WN != 0))
#define cv10falseconditions ((SN != 0) || (SW != 0) || (EW != 0) || (ES != 0) || (EN != 0) || (NS != 0) || (NE != 0))
#define cv11falseconditions ((NS != 0) || (ES != 0))
#define cv12falseconditions ((ES != 0) || (SW != 0) || (SN != 0) || (NS != 0) || (NE != 0) || (SE != 0))


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
  // create CVs for intersection:
  cv1 = cv_create("cv1");
  if (cv1 == NULL) {
    panic("couldn't create intersection CV");
  }
  cv2 = cv_create("cv2");
  if (cv2 == NULL) {
    panic("couldn't create intersection CV");
  }
  cv3 = cv_create("cv3");
  if (cv3 == NULL) {
    panic("couldn't create intersection CV");
  }
  cv4 = cv_create("cv4");
  if (cv4 == NULL) {
    panic("couldn't create intersection CV");
  }
  cv5 = cv_create("cv5");
  if (cv5 == NULL) {
    panic("couldn't create intersection CV");
  }
  cv6 = cv_create("cv6");
  if (cv6 == NULL) {
    panic("couldn't create intersection CV");
  }
  cv7 = cv_create("cv7");
  if (cv7 == NULL) {
    panic("couldn't create intersection CV");
  }
  cv8 = cv_create("cv8");
  if (cv8 == NULL) {
    panic("couldn't create intersection CV");
  }
  cv9 = cv_create("cv9");
  if (cv9 == NULL) {
    panic("couldn't create intersection CV");
  }
  cv10 = cv_create("cv10");
  if (cv10 == NULL) {
    panic("couldn't create intersection CV");
  }
  cv11 = cv_create("cv11");
  if (cv11 == NULL) {
    panic("couldn't create intersection CV");
  }
  cv12 = cv_create("cv12");
  if (cv12 == NULL) {
    panic("couldn't create intersection CV");
  }

  // init pqueue
  pqueue = array_create();
  array_init(pqueue);

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
  KASSERT(cv1 != NULL);
  KASSERT(cv2 != NULL);
  KASSERT(cv3 != NULL);
  KASSERT(cv4 != NULL);
  KASSERT(cv5 != NULL);
  KASSERT(cv6 != NULL);
  KASSERT(cv7 != NULL);
  KASSERT(cv8 != NULL);
  KASSERT(cv9 != NULL);
  KASSERT(cv10 != NULL);
  KASSERT(cv11 != NULL);
  KASSERT(cv12 != NULL);
  KASSERT(mutex != NULL);

  lock_destroy(mutex);

  array_destroy(pqueue);

  cv_destroy(cv1);
  cv_destroy(cv2);
  cv_destroy(cv3);
  cv_destroy(cv4);
  cv_destroy(cv5);
  cv_destroy(cv6);
  cv_destroy(cv7);
  cv_destroy(cv8);
  cv_destroy(cv9);
  cv_destroy(cv10);
  cv_destroy(cv11);
  cv_destroy(cv12);
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

// check if cv_wait gonna be called; if so, add to priority queue. Else don't add to queue, just increment the count
void
intersection_before_entry(Direction origin, Direction destination) 
{
  KASSERT(cv1 != NULL);
  KASSERT(cv2 != NULL);
  KASSERT(cv3 != NULL);
  KASSERT(cv4 != NULL);
  KASSERT(cv5 != NULL);
  KASSERT(cv6 != NULL);
  KASSERT(cv7 != NULL);
  KASSERT(cv8 != NULL);
  KASSERT(cv9 != NULL);
  KASSERT(cv10 != NULL);
  KASSERT(cv11 != NULL);
  KASSERT(cv12 != NULL);
  KASSERT(mutex != NULL);

  lock_acquire(mutex);

  // make vehicle
  Vehicle *v = kmalloc(sizeof(struct Vehicle));
  KASSERT(v != NULL);
  v->origin = origin;
  v->destination = destination;

  if (v->origin == north) {
    if (v->destination == south) {                                                           // straight line (NS) 
        if cv1falseconditions {
          array_add(pqueue, cv1, NULL);
        }
        while cv1falseconditions {
          cv_wait(cv1, mutex);
        }
        NS++;
    } else if (v->destination == east) {                                                     // left turn (NE)
        if cv2falseconditions {
          array_add(pqueue, cv2, NULL);
        }
        while cv2falseconditions {
          cv_wait(cv2, mutex);
        }
        NE++;
    } else {                                                                                 // right turn (NW)
        if cv3falseconditions {
          array_add(pqueue, cv3, NULL);
        }
        while cv3falseconditions {
          cv_wait(cv3, mutex);
        }
        NW++;
    }
  } else if (v->origin == south) {
    if (v->destination == north) {                                                           // straight line (SN)
        if cv4falseconditions {
          array_add(pqueue, cv4, NULL);
        }
        while cv4falseconditions {
          cv_wait(cv4, mutex);
        }
        SN++;
    } else if (v->destination == east) {                                                     // right turn (SE)
        if cv5falseconditions {
          array_add(pqueue, cv5, NULL);
        }
        while cv5falseconditions {
          cv_wait(cv5, mutex);
        }
        SE++;
    } else {                                                                                 // dest = west, left turn (SW)
        if cv6falseconditions {
          array_add(pqueue, cv6, NULL);
        }
        while cv6falseconditions {
          cv_wait(cv6, mutex);
        }
        SW++;
    }
  } else if (v->origin == east) {
    if (v->destination == north) {                                                           // right turn (EN)
        if cv7falseconditions {
          array_add(pqueue, cv7, NULL);
        }
        while cv7falseconditions {
          cv_wait(cv7, mutex);
        }
        EN++;
    } else if (v->destination == south) {                                                    // left turn (ES)
        if cv8falseconditions {
          array_add(pqueue, cv8, NULL);
        }
        while cv8falseconditions {
          cv_wait(cv8, mutex);
        }
        ES++;
    } else {                                                                                  // straight line (EW)
        if cv9falseconditions {
          array_add(pqueue, cv9, NULL);
        }
        while cv9falseconditions {
          cv_wait(cv9, mutex);
        }
        EW++;
    }
  } else {                                                                                    // Origin is West
    if (v->destination == north) {                                                            // left turn (WN)
        if cv10falseconditions {
          array_add(pqueue, cv10, NULL);
        }
        while cv10falseconditions {
          cv_wait(cv10, mutex);
        }
        WN++;
    } else if (v->destination == south) {                                                     // right turn (WS)
        if cv11falseconditions {
          array_add(pqueue, cv11, NULL);
        }
        while cv11falseconditions {
          cv_wait(cv11, mutex);
        }
        WS++;
    } else {                                                                                  // straight line (WE)
        if cv12falseconditions {
          array_add(pqueue, cv12, NULL);
        }
        while cv12falseconditions {
          cv_wait(cv12, mutex);
        }
        WE++;
    }
  }

    lock_release(mutex);
  }

// helper: scan the priority queue for the highest priority cv that is now true and broadcast to that. Highest priority is the
//         waiting for the longest time (i.e. the one that went onto this queue the first, so i=0 has the highest priority)

void
checkForCvAndBroadcast(void) {

if (array_num(pqueue) > 0) {
  if (array_get(pqueue, 0) == cv1) {
      if cv1trueconditions {
        array_remove(pqueue, 0);
        cv_broadcast(cv1, mutex);
        return;
      }
    } else if (array_get(pqueue, 0) == cv2) {
      if cv2trueconditions {
        array_remove(pqueue, 0);
        cv_broadcast(cv2, mutex);
        return;
      } 
    } else if (array_get(pqueue, 0) == cv3) {
      if cv3trueconditions {
        array_remove(pqueue, 0);
        cv_broadcast(cv3, mutex);
        return;
      } 
    } else if (array_get(pqueue, 0) == cv4) {
      if cv4trueconditions {
        array_remove(pqueue, 0);
        cv_broadcast(cv4, mutex);
        return;
      }
    } else if (array_get(pqueue, 0) == cv5) {
      if cv5trueconditions {
        array_remove(pqueue, 0);
        cv_broadcast(cv5, mutex);
        return;
      } 
    } else if (array_get(pqueue, 0) == cv6) {
      if cv6trueconditions {
        array_remove(pqueue, 0);
        cv_broadcast(cv6, mutex);
        return;
      } 
    } else if (array_get(pqueue, 0) == cv7) {
      if cv7trueconditions {
        array_remove(pqueue, 0);
        cv_broadcast(cv7, mutex);
        return;
      } 
    } else if (array_get(pqueue, 0) == cv8) {
      if cv8trueconditions {
        array_remove(pqueue, 0);
        cv_broadcast(cv8, mutex);
        return;
      }
    } else if (array_get(pqueue, 0) == cv9) {
      if cv9trueconditions {
        array_remove(pqueue, 0);
        cv_broadcast(cv9, mutex);
        return;
      }
    } else if (array_get(pqueue, 0) == cv10) {
      if cv10trueconditions {
        array_remove(pqueue, 0);
        cv_broadcast(cv10, mutex);
        return;
      }
    } else if (array_get(pqueue, 0) == cv11) {
      if cv11trueconditions {
        array_remove(pqueue, 0);
        cv_broadcast(cv11, mutex);
        return;
      }
    } else if (array_get(pqueue, 0) == cv12) {
      if cv1trueconditions {
        array_remove(pqueue, 0);
        cv_broadcast(cv12, mutex);
        return;
      }
    } else {
      // do nothing
      cv1 += 0;
    }
  }
  return;
}
  // for (unsigned int i=0; i<array_num(pqueue); i++) {
  //   if (array_get(pqueue, i) == cv1) {
  //     if cv1trueconditions {
  //       array_remove(pqueue, i);
  //       cv_broadcast(cv1, mutex);
  //       return;
  //     }
  //   } else if (array_get(pqueue, i) == cv2) {
  //     if cv2trueconditions {
  //       array_remove(pqueue, i);
  //       cv_broadcast(cv2, mutex);
  //       return;
  //     }
  //   } else if (array_get(pqueue, i) == cv3) {
  //     if cv3trueconditions {
  //       array_remove(pqueue, i);
  //       cv_broadcast(cv3, mutex);
  //       return;
  //     }
  //   } else if (array_get(pqueue, i) == cv4) {
  //     if cv4trueconditions {
  //       array_remove(pqueue, i);
  //       cv_broadcast(cv4, mutex);
  //       return;
  //     }
  //   } else if (array_get(pqueue, i) == cv5) {
  //     if cv5trueconditions {
  //       array_remove(pqueue, i);
  //       cv_broadcast(cv5, mutex);
  //       return;
  //     }
  //   } else if (array_get(pqueue, i) == cv6) {
  //     if cv6trueconditions {
  //       array_remove(pqueue, i);
  //       cv_broadcast(cv6, mutex);
  //       return;
  //     }
  //   } else if (array_get(pqueue, i) == cv7) {
  //     if cv7trueconditions {
  //       array_remove(pqueue, i);
  //       cv_broadcast(cv7, mutex);
  //       return;
  //     }
  //   } else if (array_get(pqueue, i) == cv8) {
  //     if cv8trueconditions {
  //       array_remove(pqueue, i);
  //       cv_broadcast(cv8, mutex);
  //       return;
  //     }
  //   } else if (array_get(pqueue, i) == cv9) {
  //     if cv9trueconditions {
  //       array_remove(pqueue, i);
  //       cv_broadcast(cv9, mutex);
  //       return;
  //     }
  //   } else if (array_get(pqueue, i) == cv10) {
  //     if cv10trueconditions {
  //       array_remove(pqueue, i);
  //       cv_broadcast(cv10, mutex);
  //       return;
  //     }
  //   } else if (array_get(pqueue, i) == cv11) {
  //     if cv11trueconditions {
  //       array_remove(pqueue, i);
  //       cv_broadcast(cv11, mutex);
  //       return;
  //     }
  //   } else if (array_get(pqueue, i) == cv12) {
  //     if cv12trueconditions {
  //       array_remove(pqueue, i);
  //       cv_broadcast(cv12, mutex);
  //       return;
  //     }
  //   } else {
  //     // never reach here:
  //     panic("should have never reached here!! deadlock!!!");
  //   }
  // }
//}

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

// decrement the cv related to that direction and call check function helper to broadcast to correct cv
void
intersection_after_exit(Direction origin, Direction destination)
{
  KASSERT(cv1 != NULL);
  KASSERT(cv2 != NULL);
  KASSERT(cv3 != NULL);
  KASSERT(cv4 != NULL);
  KASSERT(cv5 != NULL);
  KASSERT(cv6 != NULL);
  KASSERT(cv7 != NULL);
  KASSERT(cv8 != NULL);
  KASSERT(cv9 != NULL);
  KASSERT(cv10 != NULL);
  KASSERT(cv11 != NULL);
  KASSERT(cv12 != NULL);
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
      checkForCvAndBroadcast();
      
    } else if (v->destination == east) {
      NE--;
      checkForCvAndBroadcast();
    } else {                                                                            // dest = west
      NW--;
      checkForCvAndBroadcast();
    }
  } else if (v->origin == south) {
    if (v->destination == north) {
      SN--;
      checkForCvAndBroadcast();
    } else if (v->destination == east) {
      SE--;
      checkForCvAndBroadcast();
    } else {                                                                            // dest = west
      SW--;
      checkForCvAndBroadcast();
    }
  } else if (v->origin == east) {
    if (v->destination == north) {
      EN--;
      checkForCvAndBroadcast();
    } else if (v->destination == south) {
      ES--;
      checkForCvAndBroadcast();
    } else {                                                                            // dest = west
      EW--;
      checkForCvAndBroadcast();
    }
  } else {                                                                              // Origin is West
    if (v->destination == north) {
      WN--;
      checkForCvAndBroadcast();
    } else if (v->destination == south) {
      WS--;
      checkForCvAndBroadcast();
    } else {                                                                            // dest = east
      WE--;
      checkForCvAndBroadcast();
    }
  }

  lock_release(mutex);
}
