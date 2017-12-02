/*
 * Copyright (c) 2000, 2001, 2002, 2003, 2004, 2005, 2008, 2009
 *	The President and Fellows of Harvard College.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE UNIVERSITY AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE UNIVERSITY OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <types.h>
#include <kern/errno.h>
#include <lib.h>
#include <spl.h>
#include <spinlock.h>
#include <proc.h>
#include <current.h>
#include <mips/tlb.h>
#include <addrspace.h>
#include <vm.h>
#include "opt-A3.h"

/*
 * Dumb MIPS-only "VM system" that is intended to only be just barely
 * enough to struggle off the ground.
 */

/* under dumbvm, always have 48k of user stack */
#define DUMBVM_STACKPAGES    12

/*
 * Wrap rma_stealmem in a spinlock.
 */
static struct spinlock stealmem_lock = SPINLOCK_INITIALIZER;

#if OPT_A3

bool coreMapImplemented = false;

struct coreMapMappingAndFrameInfo {
	paddr_t address;		// unnecessary (index into coreMap is enough to figure out as we know start address)
	bool isFrameInUse;
	bool isContiguous;							// is there any more frame(s) after this in a block?
	int numberOfContiguousFramesAfterCurrent;	// if yes, then how many; for free_kpages()
};

struct coreMap {
	// array of coreMapMappings:
	struct coreMapMappingAndFrameInfo* coreMapMappingAndFrameInfoArray;
};

struct coreMap* coreMap;
// struct coreMapMappingAndFrameInfo* arrayOfMappings;
// coreMap->coreMapMappingAndFrameInfo = arrayOfMappings;
int totalNumberOfFrames;

#endif	// OPT_A3

void
vm_bootstrap(void)
{
	/* Do nothing. */

	#if OPT_A3

	paddr_t lo;
	paddr_t hi;

	// get total ram size needed:
	// returns start & end of physical address space of free memory
	// also disables ram_stealmem()
	// returns physical address, convert to virtual address for writing data there
	ram_getsize(&lo, &hi);

	// divide this mem to get no. of frames:
	int numberOfFrames = ((hi - lo) / PAGE_SIZE);

	// keep track of which frame is free:
	// has 1 entry for each frame (as an array)
	coreMap = (struct coreMap*) PADDR_TO_KVADDR(lo);
	// coreMap->coreMapMappingAndFrameInfoArray = (struct coreMapMappingAndFrameInfo*) PADDR_TO_KVADDR(lo);

	// find space for coreMap structure:
	lo += numberOfFrames * (sizeof (struct coreMap));

	// lo has to be multiple because coreMap stores only these
	if ((lo % PAGE_SIZE) != 0) {
		lo = ROUNDUP(lo, PAGE_SIZE);
		// while ((lo % PAGE_SIZE) != 0) {
		// 	lo++;
		// }
	}

	// update numberOfFrames:
	numberOfFrames = ((hi - lo) / PAGE_SIZE);

	totalNumberOfFrames = numberOfFrames;

	// populate coreMap for each frame:
	paddr_t currentvalOfLo = lo;
	for (int i=0; i<numberOfFrames; i++) {
		// coreMap->coreMapMappingAndFrameInfoArray[i] = (struct coreMapMappingAndFrameInfo*) PADDR_TO_KVADDR(lo * (i + 1));
		coreMap->coreMapMappingAndFrameInfoArray[i].address = currentvalOfLo;
		coreMap->coreMapMappingAndFrameInfoArray[i].isFrameInUse = false;
		coreMap->coreMapMappingAndFrameInfoArray[i].isContiguous = false;
		coreMap->coreMapMappingAndFrameInfoArray[i].numberOfContiguousFramesAfterCurrent = 0;
		currentvalOfLo += PAGE_SIZE;
	}

	coreMapImplemented = true;

	#endif	// OPT_A3


}

static
paddr_t
getppages(unsigned long npages)
{
	paddr_t addr;

	spinlock_acquire(&stealmem_lock);

	#if OPT_A3

	// here we will implement fetching non-contiguous memory based on the info from our coreMap
	// data structure. We will only allocate the reqd. mem depending on frames:

	if (coreMapImplemented) {
		int start;
		int numberOfPages = (int)npages;
		bool foundContiguousMemChunk = false;

		if (!foundContiguousMemChunk) {
			for (int i=0; i<totalNumberOfFrames; i++) {
				if (foundContiguousMemChunk) {
					break;
				}

				if (!coreMap->coreMapMappingAndFrameInfoArray[i].isFrameInUse) {
					int numFramesInUse = 1;
					if (numberOfPages > 1) {				// trying to get contiguous block of mem?
						for (int j=i+1; j<(numberOfPages+i); j++) {
							if (!coreMap->coreMapMappingAndFrameInfoArray[j].isFrameInUse) {
								numFramesInUse++;
								if (numFramesInUse == numberOfPages) {
									foundContiguousMemChunk = true;
									start = i;
									coreMap->coreMapMappingAndFrameInfoArray[start].numberOfContiguousFramesAfterCurrent = numFramesInUse-1;
								}
							} else {
								// skip the contiguous block
								i += numFramesInUse;
								break;
							}
						}
					} else {
						start = i;
						foundContiguousMemChunk = true;
					}
				}
			}
		}

		// update the coreMapMapping info for all pages in use inside contiguous mem block we found
		// also set addr (start address) for return
		if (foundContiguousMemChunk) {
			for (int i=0; i<numberOfPages; i++) {
				addr = coreMap->coreMapMappingAndFrameInfoArray[start].address;
				coreMap->coreMapMappingAndFrameInfoArray[start+i].isFrameInUse = true;
				if (i == numberOfPages - 1) {
					coreMap->coreMapMappingAndFrameInfoArray[start+i].isContiguous = false;
				} else {
					coreMap->coreMapMappingAndFrameInfoArray[start+i].isContiguous = true;
				}

				// coreMap->coreMapMappingAndFrameInfo[i].numberOfContiguousFramesAfterCurrent = numFramesInUse;
			}
		} else {
			spinlock_release(&stealmem_lock);
			return ENOMEM;
		}
	} else {
		// this fetches the memory as a contiguous block for npages:
		addr = ram_stealmem(npages);
	}

	#endif	// OPT_A3
	
	spinlock_release(&stealmem_lock);
	return addr;
}

/* Allocate/free some kernel-space virtual pages */
vaddr_t 
alloc_kpages(int npages)
{
	paddr_t pa;
	pa = getppages(npages);
	if (pa==0) {
		return 0;
	} else if (pa == ENOMEM) {
		return ENOMEM;
	}
	return PADDR_TO_KVADDR(pa);
}

void 
free_kpages(vaddr_t addr)
{
	/* nothing - leak the memory. */

	#if OPT_A3

	spinlock_acquire(&stealmem_lock);
	if (coreMapImplemented) {
		if (!addr) {
			spinlock_release(&stealmem_lock);
			kprintf("error!!\n");
			return;
		}
		bool foundStartFrame = false;

		for (int i=0; i<totalNumberOfFrames; i++) {
			if (coreMap->coreMapMappingAndFrameInfoArray[i].address == addr) {
				foundStartFrame = true;
			} else {
				// nothing, not found
			}

			if (foundStartFrame) {
				coreMap->coreMapMappingAndFrameInfoArray[i].isFrameInUse = false;
				if (!coreMap->coreMapMappingAndFrameInfoArray[i].isContiguous) {
					break;
				} else {
					// update isFrameInUse to false for all mappings in contiguous block:
					for (int j=i; j<coreMap->coreMapMappingAndFrameInfoArray[i].numberOfContiguousFramesAfterCurrent; j++) {
						coreMap->coreMapMappingAndFrameInfoArray[j].isFrameInUse = false;
					}
				}
			}
		}
	}

	spinlock_release(&stealmem_lock);

	#endif	// OPT_A3
	return;
	// (void)addr;
}

void
vm_tlbshootdown_all(void)
{
	panic("dumbvm tried to do tlb shootdown?!\n");
}

void
vm_tlbshootdown(const struct tlbshootdown *ts)
{
	(void)ts;
	panic("dumbvm tried to do tlb shootdown?!\n");
}

// A3 changing code to handle TLB when full so no panicks
int
vm_fault(int faulttype, vaddr_t faultaddress)
{
	vaddr_t vbase1, vtop1, vbase2, vtop2, stackbase, stacktop;
	paddr_t paddr;
	int i;
	uint32_t ehi, elo;
	struct addrspace *as;
	int spl;

	faultaddress &= PAGE_FRAME;

	DEBUG(DB_VM, "dumbvm: fault: 0x%x\n", faultaddress);

	switch (faulttype) {
	    case VM_FAULT_READONLY:
		/* We always create pages read-write, so we can't get this */
		#if OPT_A3
		return 1;
		// panic("dumbvm: got VM_FAULT_READONLY\n");
		#endif	// OPT_A3
	    case VM_FAULT_READ:
	    case VM_FAULT_WRITE:
		break;
	    default:
		return EINVAL;
	}

	if (curproc == NULL) {
		/*
		 * No process. This is probably a kernel fault early
		 * in boot. Return EFAULT so as to panic instead of
		 * getting into an infinite faulting loop.
		 */
		return EFAULT;
	}

	as = curproc_getas();
	if (as == NULL) {
		/*
		 * No address space set up. This is probably also a
		 * kernel fault early in boot.
		 */
		return EFAULT;
	}

	/* Assert that the address space has been set up properly. */
	KASSERT(as->as_vbase1 != 0);
	KASSERT(as->as_pbase1 != 0);
	KASSERT(as->as_npages1 != 0);
	KASSERT(as->as_vbase2 != 0);
	KASSERT(as->as_pbase2 != 0);
	KASSERT(as->as_npages2 != 0);
	KASSERT(as->as_stackpbase != 0);
	KASSERT((as->as_vbase1 & PAGE_FRAME) == as->as_vbase1);
	KASSERT((as->as_pbase1 & PAGE_FRAME) == as->as_pbase1);
	KASSERT((as->as_vbase2 & PAGE_FRAME) == as->as_vbase2);
	KASSERT((as->as_pbase2 & PAGE_FRAME) == as->as_pbase2);
	KASSERT((as->as_stackpbase & PAGE_FRAME) == as->as_stackpbase);

	vbase1 = as->as_vbase1;
	vtop1 = vbase1 + as->as_npages1 * PAGE_SIZE;
	vbase2 = as->as_vbase2;
	vtop2 = vbase2 + as->as_npages2 * PAGE_SIZE;
	stackbase = USERSTACK - DUMBVM_STACKPAGES * PAGE_SIZE;
	stacktop = USERSTACK;

	if (faultaddress >= vbase1 && faultaddress < vtop1) {
		paddr = (faultaddress - vbase1) + as->as_pbase1;
	}
	else if (faultaddress >= vbase2 && faultaddress < vtop2) {
		paddr = (faultaddress - vbase2) + as->as_pbase2;
	}
	else if (faultaddress >= stackbase && faultaddress < stacktop) {
		paddr = (faultaddress - stackbase) + as->as_stackpbase;
	}
	else {
		return EFAULT;
	}

	/* make sure it's page-aligned */
	KASSERT((paddr & PAGE_FRAME) == paddr);

	/* Disable interrupts on this CPU while frobbing the TLB. */
	spl = splhigh();

	for (i=0; i<NUM_TLB; i++) {
		tlb_read(&ehi, &elo, i);
		if (elo & TLBLO_VALID) {
			continue;
		}
		ehi = faultaddress;
		elo = paddr | TLBLO_DIRTY | TLBLO_VALID;

		#if OPT_A3

		// Text segment should be read-only
		if (as->flagComplete) {
			if ((faultaddress >= vbase1) && (faultaddress < vtop1)) {
				// Load TLB entries for the text segment with TLBLO_DIRTY off
				elo &= ~TLBLO_DIRTY;
			}
		}

		#endif	// OPT_A3

		DEBUG(DB_VM, "dumbvm: 0x%x -> 0x%x\n", faultaddress, paddr);
		tlb_write(ehi, elo, i);
		splx(spl);
		return 0;
	}

	#if OPT_A3
	// right now we return EFAULT & say ran out of tlb entries, can't handle page fault
	// because the TLB is out of space. Our simple modification solution to this issue:
	// select an existing TLB entry at random, throw it out & put the new entry, then cont
	// without returning an error:

	ehi = faultaddress;
	elo = (TLBLO_VALID) || (TLBLO_DIRTY) | (paddr);

	// Text segment should be read-only
	if (as->flagComplete) {
		if ((faultaddress >= vbase1) && (faultaddress < vtop1)) {
			// Load TLB entries for the text segment with TLBLO_DIRTY off
			elo &= ~TLBLO_DIRTY;
		}
	}

	tlb_random(ehi, elo);

	// kprintf("dumbvm: Ran out of TLB entries - cannot handle page fault\n");

	#endif // OPT_A3

	splx(spl);
	return 0;
	// return EFAULT;
}

struct addrspace *
as_create(void)
{
	struct addrspace *as = kmalloc(sizeof(struct addrspace));
	if (as==NULL) {
		return NULL;
	}

	as->as_vbase1 = 0;
	as->as_pbase1 = 0;
	as->as_npages1 = 0;
	as->as_vbase2 = 0;
	as->as_pbase2 = 0;
	as->as_npages2 = 0;
	as->as_stackpbase = 0;
	as->flagComplete = false;		// whether load_elf has completed, if its true, load entries with dirty bit set to 0

	return as;
}

void
as_destroy(struct addrspace *as)
{
	#if OPT_A3
	free_kpages((vaddr_t)as->as_pbase1);
	free_kpages((vaddr_t)as->as_pbase2);
	free_kpages((vaddr_t)as->as_stackpbase);
	#endif	// OPT_A3
	kfree(as);
}

void
as_activate(void)
{
	int i, spl;
	struct addrspace *as;

	as = curproc_getas();
#ifdef UW
        /* Kernel threads don't have an address spaces to activate */
#endif
	if (as == NULL) {
		return;
	}

	/* Disable interrupts on this CPU while frobbing the TLB. */
	spl = splhigh();

	for (i=0; i<NUM_TLB; i++) {
		tlb_write(TLBHI_INVALID(i), TLBLO_INVALID(), i);
	}

	splx(spl);
}

void
as_deactivate(void)
{
	/* nothing */
}

int
as_define_region(struct addrspace *as, vaddr_t vaddr, size_t sz,
		 int readable, int writeable, int executable)
{
	size_t npages; 

	/* Align the region. First, the base... */
	sz += vaddr & ~(vaddr_t)PAGE_FRAME;
	vaddr &= PAGE_FRAME;

	/* ...and now the length. */
	sz = (sz + PAGE_SIZE - 1) & PAGE_FRAME;

	npages = sz / PAGE_SIZE;

	/* We don't use these - all pages are read-write */
	(void)readable;
	(void)writeable;
	(void)executable;

	if (as->as_vbase1 == 0) {
		as->as_vbase1 = vaddr;
		as->as_npages1 = npages;
		return 0;
	}

	if (as->as_vbase2 == 0) {
		as->as_vbase2 = vaddr;
		as->as_npages2 = npages;
		return 0;
	}

	/*
	 * Support for more than two regions is not available.
	 */
	kprintf("dumbvm: Warning: too many regions\n");
	return EUNIMP;
}

static
void
as_zero_region(paddr_t paddr, unsigned npages)
{
	bzero((void *)PADDR_TO_KVADDR(paddr), npages * PAGE_SIZE);
}

int
as_prepare_load(struct addrspace *as)
{
	KASSERT(as->as_pbase1 == 0);
	KASSERT(as->as_pbase2 == 0);
	KASSERT(as->as_stackpbase == 0);

	as->as_pbase1 = getppages(as->as_npages1);
	if (as->as_pbase1 == 0) {
		return ENOMEM;
	}

	as->as_pbase2 = getppages(as->as_npages2);
	if (as->as_pbase2 == 0) {
		return ENOMEM;
	}

	as->as_stackpbase = getppages(DUMBVM_STACKPAGES);
	if (as->as_stackpbase == 0) {
		return ENOMEM;
	}
	
	as_zero_region(as->as_pbase1, as->as_npages1);
	as_zero_region(as->as_pbase2, as->as_npages2);
	as_zero_region(as->as_stackpbase, DUMBVM_STACKPAGES);

	return 0;
}

int
as_complete_load(struct addrspace *as)
{
	(void)as;
	return 0;
}

int
as_define_stack(struct addrspace *as, vaddr_t *stackptr)
{
	KASSERT(as->as_stackpbase != 0);

	*stackptr = USERSTACK;
	return 0;
}

int
as_copy(struct addrspace *old, struct addrspace **ret)
{
	struct addrspace *new;

	new = as_create();
	if (new==NULL) {
		return ENOMEM;
	}

	new->as_vbase1 = old->as_vbase1;
	new->as_npages1 = old->as_npages1;
	new->as_vbase2 = old->as_vbase2;
	new->as_npages2 = old->as_npages2;

	/* (Mis)use as_prepare_load to allocate some physical memory. */
	if (as_prepare_load(new)) {
		as_destroy(new);
		return ENOMEM;
	}

	KASSERT(new->as_pbase1 != 0);
	KASSERT(new->as_pbase2 != 0);
	KASSERT(new->as_stackpbase != 0);

	memmove((void *)PADDR_TO_KVADDR(new->as_pbase1),
		(const void *)PADDR_TO_KVADDR(old->as_pbase1),
		old->as_npages1*PAGE_SIZE);

	memmove((void *)PADDR_TO_KVADDR(new->as_pbase2),
		(const void *)PADDR_TO_KVADDR(old->as_pbase2),
		old->as_npages2*PAGE_SIZE);

	memmove((void *)PADDR_TO_KVADDR(new->as_stackpbase),
		(const void *)PADDR_TO_KVADDR(old->as_stackpbase),
		DUMBVM_STACKPAGES*PAGE_SIZE);
	
	*ret = new;
	return 0;
}
