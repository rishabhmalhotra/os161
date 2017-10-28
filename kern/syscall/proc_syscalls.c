#include <types.h>
#include <kern/errno.h>
#include <kern/unistd.h>
#include <kern/wait.h>
#include <lib.h>
#include <syscall.h>
#include <current.h>
#include <proc.h>
#include <thread.h>
#include <addrspace.h>
#include <copyinout.h>
#include <array.h>
#include <mips/trapframe.h>
#include <limits.h>
#include <synch.h>

  /* this implementation of sys__exit does not do anything with the exit code */
  /* this needs to be fixed to get exit() and waitpid() working properly */

void sys__exit(int exitcode) {

  struct addrspace *as;
  struct proc *p = curproc;
  /* for now, just include this to keep the compiler from complaining about
     an unused variable */
  (void)exitcode;

  DEBUG(DB_SYSCALL,"Syscall: _exit(%d)\n",exitcode);

  KASSERT(curproc->p_addrspace != NULL);

  // all kids of this proc need their parent pointers updated & release their 
  // locks (which we acquired at the end of sys_fork() remember? :) )

  for (unsigned int i=0; i<array_num(p->childrenprocs); i++) {
    struct proc *childproc = array_get(p->childrenprocs, i);

    // release child's exit lock so it can RIP:
    lock_release(childproc->exitLock);

    // remove child from childrenprocs of p:
    array_remove(p->childrenprocs, i);
  }

  as_deactivate();

  // all children from children array should be gone (parent pointers being set to NULL in my proc_destroy)
  KASSERT(array_num(p->childrenprocs) == 0);
  
  /*
   * clear p_addrspace before calling as_destroy. Otherwise if
   * as_destroy sleeps (which is quite possible) when we
   * come back we'll be calling as_activate on a
   * half-destroyed address space. This tends to be
   * messily fatal.
   */
  as = curproc_setas(NULL);
  as_destroy(as);

  /* detach this thread from its process */
  /* note: curproc cannot be used after this call */
  proc_remthread(curthread);

  // set exitcode
  p->isProcAlive = false;  // remove from proc_destroy
  p->procExitStatus = _MKWAIT_EXIT(exitcode);

  // this proc/thread is gone so let parent know we're done!!
  cv_broadcast(p->w8Cv, p->w8Lock);

  /* if this is the last user process in the system, proc_destroy()
     will wake up the kernel menu thread */
  proc_destroy(p);  // this will also set all kids parent to NULL as well as set isProcAlive to false
  
  thread_exit();
  /* thread_exit() does not return, so we should never get here */
  panic("return from thread_exit in sys_exit\n");
}


/* stub handler for getpid() system call                */
int
sys_getpid(pid_t *retval)
{
  /* for now, this is just a stub that always returns a PID of 1 */
  /* you need to fix this to make it work properly */
  *retval = curproc->pid;
  return(0);
}

/* stub handler for waitpid() system call                */

int
sys_waitpid(pid_t pid,
	    userptr_t status,
	    int options,
	    pid_t *retval)
{
  int exitstatus;
  int result;

  /* this is just a stub implementation that always reports an
     exit status of 0, regardless of the actual exit status of
     the specified process.   
     In fact, this will return 0 even if the specified process
     is still running, and even if it never existed in the first place.

     Fix this!
  */

  // which process calling waitpid (for supplied PID):
  struct proc *p = NULL;
  // check all currently alive processes to see if a proc with this PID exists:
  for (unsigned int i=0; i<array_num(aliveProcs); i++) {
    struct proc *cur = array_get(aliveProcs, i);
    if (cur != NULL) {
      if (cur->pid == pid) {
        p = cur;
      }
    }
  }
  // if couldn't assign to p:
  if (p == NULL) {
    *retval = -1;
    return ESRCH;
  }

  if (options != 0) {
    *retval = -1;
    return(EINVAL);
  }

  // curproc needs to be child, can't w8 on it's own self
  if (p == curproc) {
    *retval = -1;
    return ECHILD;
  }

  // keep w8ing on proc while its alive before returning (sort of block its thread):
  lock_acquire(p->w8Lock);
  while (p->isProcAlive) {
    kprintf("hello\n");
    cv_wait(p->w8Cv, p->w8Lock);
  }
  lock_release(p->w8Lock);

  /* for now, just pretend the exitstatus is 0 --> it is the procExitStatus */
  exitstatus = p->procExitStatus;
  result = copyout((void *)&exitstatus,status,sizeof(int));
  if (result) {
    return(result);
  }
  *retval = pid;
  return(0);
}

#if OPT_A2

pid_t
sys_fork(struct trapframe *tf, pid_t *retval) {

  KASSERT(curproc != NULL);
  KASSERT(curproc->p_addrspace != NULL);

  // create structure for child process
  struct proc *childproc;
  childproc = proc_create_runprogram(curproc->p_name);
  // if error returned:
  if (childproc == NULL) {
    return ENPROC;
  }

  // add childproc to the children array of its parent
  spinlock_acquire(&curproc->p_lock);
  array_add(curproc->childrenprocs, childproc, NULL);
  childproc->parent = curproc;
  spinlock_release(&curproc->p_lock);

  // create new address space, copy pages from old address space to newly created one (in newSpace)
  childproc->p_addrspace = as_create();
  int copy = as_copy(curproc_getas(), &(childproc->p_addrspace));
  
  // either as_copy() failed or no addrspace of childproc:
  if ((copy != 0) || (childproc->p_addrspace == NULL)) {
    proc_destroy(childproc);
    return ENOMEM;
  }

  // thread_fork() to create new thread:
  struct trapframe *heaptf = kmalloc(sizeof(*tf));                  // heaptf is (parent) tf on the heap
  memcpy(heaptf,tf, sizeof(*tf));
  int err_no = thread_fork(curthread->t_name, childproc, &enter_forked_process, heaptf, 0);
  if (err_no !=0) {
    as_deactivate();
    proc_destroy(childproc);
    kfree(heaptf);
    return err_no;
  }

  // acquire lock on child so it can't exit before parent (eliminate the zombie case):
  lock_acquire(childproc->exitLock);

  *retval = childproc->pid;
  return 0;
}

#endif  // OPT_A2
