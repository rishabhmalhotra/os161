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
  as_deactivate();
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

  /* if this is the last user process in the system, proc_destroy()
     will wake up the kernel menu thread */
  proc_destroy(p);
  
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

  if (options != 0) {
    return(EINVAL);
  }
  /* for now, just pretend the exitstatus is 0 */
  exitstatus = 0;
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
  childproc = proc_create_runprogram("new child process");
  // if error returned:
  if (childproc == NULL) {
    return ENPROC;
  }

  // create new address space, copy pages from old address space to newly created one (in newSpace)
  int copy;
  childproc->p_addrspace = as_create();
  copy = as_copy(curproc_getas(), &(childproc->p_addrspace));
  
  // either as_copy() failed or no addrspace of childproc:
  if ((copy != 0) || (childproc->p_addrspace == NULL)) {
    proc_destroy(childproc);
    return ENOMEM;
  }

  // now associate the newSpace with the child proc:
  //proc_setas(childproc, newSpace);

  // set pid of child
  spinlock_acquire(&childproc->p_lock);
  P(pid_var_mutex);
  childproc->pid = pid_var;
  pid_var++;
  V(pid_var_mutex);
  spinlock_release(&childproc->p_lock);


  // thread_fork() to create new thread:
  struct trapframe *heaptf = kmalloc(sizeof(*tf));                  // heaptf is (parent) tf on the heap
  memcpy(heaptf,tf, sizeof(*tf));
  int err_no = thread_fork(curthread->t_name, childproc, &enter_forked_process, heaptf, 0);
  if (err_no !=0) {
    as_deactivate();
    proc_destroy(childproc);
    kfree(heaptf);
    // heaptf = NULL;
    return err_no;
  }

  // add childproc to the children array of its parent
  spinlock_acquire(&curproc->p_lock);
  array_add(curproc->childrenprocs, childproc, NULL);
  childproc->parent = curproc;
  spinlock_release(&curproc->p_lock);

  *retval = childproc->pid;
  return 0;
}

#endif  // OPT_A2
