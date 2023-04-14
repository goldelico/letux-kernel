#include "inc/tiziano_priv.h"

void *isp_kmalloc(size_t size, gfp_t flags){
	return kmalloc(size, flags);
}

void isp_kfree(void *objp){
	kfree(objp);
}

void isp_spin_lock_init(spinlock_t *lock){
	spin_lock_init(lock);
}

void _isp_spin_lock_irqsave(spinlock_t *lock, unsigned long *flags){
	spin_lock_irqsave(lock, *flags);
}

void isp_spin_unlock_irqrestore(spinlock_t *lock, unsigned long flags){
	spin_unlock_irqrestore(lock, flags);
}

void isp_dma_cache_sync(struct device *dev, void *vaddr, size_t size,
			 enum dma_data_direction direction)
{
	arch_sync_dma_for_device(virt_to_phys(vaddr),size,direction);
}


void ISP_INIT_LIST_HEAD(struct list_head *list){
	INIT_LIST_HEAD(list);
}

void isp_list_add_tail(struct list_head *new, struct list_head *head){
	list_add_tail(new, head);
}

void isp_list_del(struct list_head *entry){
	list_del(entry);
}

int isp_list_empty(const struct list_head *head){
	return list_empty(head);
}

void isp_complete(struct completion *com){
	complete(com);
}

unsigned long isp_wait_for_completion_timeout(struct completion *x, unsigned long timeout){
	return wait_for_completion_timeout(x, timeout);
}

void isp_init_completion(struct completion *x){
	init_completion(x);
}
