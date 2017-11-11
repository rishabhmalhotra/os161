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
#if OPT_A2

void sys__exit(int exitcode) {

  struct addrspace *as;
  struct proc *p = curproc;

  DEBUG(DB_SYSCALL,"Syscall: _exit(%d)\n",exitcode);

  KASSERT(curproc->p_addrspace != NULL);

  lock_acquire(procTableLock);
  struct procTable *procTable = NULL;

  for (unsigned int i=0; i<array_num(allProcs); i++) {
    procTable = array_get(allProcs, i);
    if (procTable->pid == curproc->pid) {
      break;
    } else {
      procTable = NULL;
    }
  }

  KASSERT(procTable != NULL);

  if (procTable->parentPid != NoPidForProc) {
    procTable->state = ZombieProc;
    procTable->exitCode = _MKWAIT_EXIT(exitcode);  
    // cv_broadcast(procTableW8Cv, procTableLock);
  } else {
    procTable->state = ProcExited;
  }
  cv_broadcast(procTableW8Cv, procTableLock);

  for (unsigned int i = 0; i < array_num(allProcs); i++) {
    struct procTable *curProcTable = array_get(allProcs,i);
    if((curProcTable->parentPid == procTable->pid) && (curProcTable->state == ZombieProc)) {
      curProcTable->state = ProcExited;
      curProcTable->parentPid = NoPidForProc;
    }
  }

  lock_release(procTableLock);

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

  p->isProcAlive = false;

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
  int result = 0;                             // for program name unknown: waitpid 1 errors

  lock_acquire(procTableLock);

  struct procTable *procTable1;

  for (unsigned int i=0; i<array_num(allProcs); i++) {
    procTable1 = array_get(allProcs, i);
    if (procTable1->pid == pid) {
      break;
    } else {
      procTable1 = NULL;
    }
  }

  if (procTable1 == NULL) {
    result = ESRCH;
  }

  struct proc *parentProc = curproc;

  if (parentProc->pid != procTable1->parentPid) {
    result = ECHILD;
  }

  if (result) {
    lock_release(procTableLock);
    return(result);
  }

  if (options != 0) {
    return(EINVAL);
  }

  // wait to finish
  while (procTable1->state == ActiveProc) {
    cv_wait(procTableW8Cv, procTableLock);
  }


  /* exitstatus is the procExitStatus */
  exitstatus = procTable1->exitCode;
  lock_release(procTableLock);
  result = copyout((void *)&exitstatus,status,sizeof(int));
  if (result) {
    return(result);
  }
  *retval = pid;
  return(0);
}


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

  struct procTable *procTable = NULL;

  for (unsigned int i=0; i<array_num(allProcs); i++) {
    procTable = array_get(allProcs, i);
    if (procTable->pid == childproc->pid) {
      break;
    } else {
      procTable = NULL;
    }
  }


  // add childproc to the children array of its parent
  spinlock_acquire(&curproc->p_lock);
  array_add(curproc->childrenprocs, childproc, NULL);
  childproc->parent = curproc;
  spinlock_release(&curproc->p_lock);

  // create new address space, copy pages from old address space to newly created one (in newSpace)
  childproc->p_addrspace = as_create();
  as_copy(curproc_getas(), &(childproc->p_addrspace));
  
  // either as_copy() failed or no addrspace of childproc:
  if (childproc->p_addrspace == NULL) {
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

  *retval = childproc->pid;
  return 0;
}

int sys_execv(const char *program, char **args) {
	// err chk:
	if (program == NULL) {
		return EFAULT;
	}

	if(strlen((char *)program) > PATH_MAX){
    	return E2BIG;
  	}

	if(args == NULL){
    	return EFAULT;
  	}

	int numArgs = 0;
	int result;

	// bring new prog on heap
	size_t progLen = strlen(program) + 1;								// + 1 for NULL terminating
	char *prog = kmalloc(sizeof(char *) * progLen);
	// put program into prog ie space allocated in kernel (bring in from userspace)
	result = copyinstr((userptr_t)program, prog, progLen, NULL);

	// err chk
	if (prog == NULL) {
		return ENOMEM; 
	}
	// from copyinout
	if (result) return result;

	// calculate number of arguments
	while (args[numArgs] != NULL) {
		numArgs++;
	}

	// put each of args[] onto kernel space which will be held inside kernArgs[]
	char **kernArgs = kmalloc((numArgs+1) * sizeof(char));
	if (kernArgs == NULL) return ENOMEM;

	// NULL terminated
	kernArgs[numArgs] = NULL;

	for (unsigned int i=0; i<numArgs; i++) {
		kernArgLen = strlen(args[i]) + 1;
		kernArgs[i] = kmalloc(sizeof(char) * kernArgLen);
		// put args[i] from userspace into kernArgs[i] ie onto kernel space (kernArgLen bytes; including NULL terminator)
		result = copyinstr((userptr_t)args[i], kernArgs[i], kernArgLen, NULL);
		if (result) return result;
	}

	// next few steps copied from runprogram
	struct addrspace *as;
	struct addrspace *asNew;
	struct vnode *v;
	vaddr_t entrypoint, stackptr;


	// open prog
	char * progNew;
	progNewTemp = kstrdup(program);
	result = vfs_open(progNewTemp, O_RDONLY, 0, &v);
	kfree(progNewTemp);
	if (result) return result;

	// create addrspace(asNew)
	asNew = as_create();
	if (asNew == NULL) {
		vfs_close(v);
		return ENOMEM;
	}

	// switch & activate
	curproc_setas(asNew);
	as_activate();												// flush TLB (as discussed in class)

	// load executable
	result = load_elf(v, &entrypoint);
	if (result) {
		curproc_setas(as);
    	// p_addrspace will go away when curproc is destroyed
    	vfs_close(v);
    	return result;
  	}

  	// done with file now
  	vfs_close(v);

  	// define user stack in the addrspace
  	result = as_define_stack(asNew, &stackptr);
  	if (result) {
  		curproc_setas(as);
    	// p_addrspace will go away when curproc is destroyed
    	return result;
  	}

  	// array to hold where stackptr is pointing at for each arg
  	vaddr_t arrayOfStackAddress[numArgs];
  	// NULL terminate
  	arrayOfStackAddress[numArgs] = NULL;

  	for(int i=(numArgs-1); i>=0; i--) {

  		int kernArgLen = strlen(kernArgs[i]) + 1;

  		// each string to be 8-byte aligned:
  		stackptr -= ROUNDUP(kernArgLen, 8); 			// each char is 1 byte so kernArgLen Bytes

  		// put onto userspace from kern space:
  		result = copyoutstr(kernArgs[i], (userptr_t)stackptr, kernArgLen, NULL);

  		if (result) return result;

  		// set this array for arg i to point to where stackptr points at ie towards the arg i
  		arrayOfStackAddress[i] = stackptr;
  	}

  	// now we need to copy the args (or pointers to, thereof)

  	for (int i=(numArgs-1); i>=0; i--) {
  		stackptr -= ROUNDUP(sizeof(vaddr_t), 4);		// vaddr_t is the type of pointers, round to 4 bytes as in ass. spec
  		result = copyout(&arrayOfStackAddress[i], (userptr_t)stackptr, sizeof(vaddr_t));
  		if (result) return result;
  	}


  	// from runprog:

  	as_destroy(as);

  	// Warp to user mode:
  	enter_new_process(argc, (userptr_t)stackptr, stackptr, entrypoint);
  	/* enter_new_process does not return. */
  	panic("enter_new_process returned\n");
  	return EINVAL;

  }

#endif  // OPT_A2
