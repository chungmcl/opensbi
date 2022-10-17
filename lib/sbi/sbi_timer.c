/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019 Western Digital Corporation or its affiliates.
 *
 * Authors:
 *   Anup Patel <anup.patel@wdc.com>
 */

#include <sbi/riscv_asm.h>
#include <sbi/riscv_barrier.h>
#include <sbi/riscv_encoding.h>
#include <sbi/sbi_console.h>
#include <sbi/sbi_error.h>
#include <sbi/sbi_hart.h>
#include <sbi/sbi_platform.h>
#include <sbi/sbi_pmu.h>
#include <sbi/sbi_scratch.h>
#include <sbi/sbi_timer.h>
#include <sbi/sbi_list.h>
#include <sbi/riscv_locks.h>
#include "sbi/sbi_console.h"

static unsigned long time_delta_off;
static unsigned long timer_list_off;

static u64 (*get_time_val)(void);
static const struct sbi_timer_device *timer_dev = NULL;

#define sbi_scratch_thishart_timerlist() \
	sbi_scratch_offset_ptr(sbi_scratch_thishart_ptr(), timer_list_off);

#define MAX_TIMER_ENTRIES 16

struct sbi_dlist free_list;
static struct tl_entry {
	struct sbi_dlist head;
	u64 next_event;
	int source;
} timer_list_entries[MAX_TIMER_ENTRIES];
static spinlock_t timer_lock = SPIN_LOCK_INITIALIZER;

#if __riscv_xlen == 32
static u64 get_ticks(void)
{
	u32 lo, hi, tmp;
	__asm__ __volatile__("1:\n"
			     "rdtimeh %0\n"
			     "rdtime %1\n"
			     "rdtimeh %2\n"
			     "bne %0, %2, 1b"
			     : "=&r"(hi), "=&r"(lo), "=&r"(tmp));
	return ((u64)hi << 32) | lo;
}
#else
static u64 get_ticks(void)
{
	unsigned long n;

	__asm__ __volatile__("rdtime %0" : "=r"(n));
	return n;
}
#endif

static void nop_delay_fn(void *opaque)
{
	cpu_relax();
}

void sbi_timer_delay_loop(ulong units, u64 unit_freq,
			  void (*delay_fn)(void *), void *opaque)
{
	u64 start_val, delta;

	/* Do nothing if we don't have timer device */
	if (!timer_dev || !get_time_val) {
		sbi_printf("%s: called without timer device\n", __func__);
		return;
	}

	/* Save starting timer value */
	start_val = get_time_val();

	/* Compute desired timer value delta */
	delta = ((u64)timer_dev->timer_freq * (u64)units);
	delta = delta / unit_freq;

	/* Use NOP delay function if delay function not available */
	if (!delay_fn)
		delay_fn = nop_delay_fn;

	/* Busy loop until desired timer value delta reached */
	while ((get_time_val() - start_val) < delta)
		delay_fn(opaque);
}

u64 sbi_timer_value(void)
{
	if (get_time_val)
		return get_time_val();
	return 0;
}

u64 sbi_timer_virt_value(void)
{
	u64 *time_delta = sbi_scratch_offset_ptr(sbi_scratch_thishart_ptr(),
						 time_delta_off);

	return sbi_timer_value() + *time_delta;
}

u64 sbi_timer_get_delta(void)
{
	u64 *time_delta = sbi_scratch_offset_ptr(sbi_scratch_thishart_ptr(),
						 time_delta_off);

	return *time_delta;
}

void sbi_timer_set_delta(ulong delta)
{
	u64 *time_delta = sbi_scratch_offset_ptr(sbi_scratch_thishart_ptr(),
						 time_delta_off);

	*time_delta = (u64)delta;
}

void sbi_timer_set_delta_upper(ulong delta_upper)
{
	u64 *time_delta = sbi_scratch_offset_ptr(sbi_scratch_thishart_ptr(),
						 time_delta_off);

	*time_delta &= 0xffffffffULL;
	*time_delta |= ((u64)delta_upper << 32);
}

void sbi_timer_event_start(u64 next_event, int source)
{
	sbi_pmu_ctr_incr_fw(SBI_PMU_FW_SET_TIMER);

	struct tl_entry *t, *t_new = NULL, *t_next;
	struct sbi_dlist *timer_events = sbi_scratch_thishart_timerlist();

	// First, see if we have another free entry
	spin_lock(&timer_lock);
	if (sbi_list_empty(&free_list)) {
		sbi_printf("Fatal: not enough timer entries");
		sbi_hart_hang();
	} else {
		t_new = sbi_list_first_entry(&free_list, struct tl_entry, head);
		sbi_list_del(&t_new->head);
	}
	spin_unlock(&timer_lock);

	t_new->next_event = next_event;
	t_new->source	  = source;

	// Then, find the correct place to insert this callback
	if (sbi_list_empty(timer_events)) {
		sbi_list_add_tail(&t_new->head, timer_events);
		t_next = t_new;
	} else {
		sbi_list_for_each_entry(t, timer_events, head) {
			if (t->next_event > next_event) {
				break;
			}
		}

        if(&t->head == timer_events) {
            // This entry needs to go to the back of the list
            sbi_list_add_tail(&t_new->head, timer_events);
        } else {
            sbi_list_add_tail(&t_new->head, &t->head);
        }

		t_next = sbi_list_first_entry(timer_events, struct tl_entry, head);
	}

	next_event = t_next->next_event;

	/**
	 * Update the stimecmp directly if available. This allows
	 * the older software to leverage sstc extension on newer hardware.
	 */
	if (sbi_hart_has_extension(sbi_scratch_thishart_ptr(), SBI_HART_EXT_SSTC)) {
#if __riscv_xlen == 32
		csr_write(CSR_STIMECMP, next_event & 0xFFFFFFFF);
		csr_write(CSR_STIMECMPH, next_event >> 32);
#else
		csr_write(CSR_STIMECMP, next_event);
#endif
	} else if (timer_dev && timer_dev->timer_event_start) {
		timer_dev->timer_event_start(next_event);
		csr_clear(CSR_MIP, MIP_STIP);
	}
	csr_set(CSR_MIE, MIP_MTIP);
}

void sbi_timer_process(void)
{
	int source;
	struct tl_entry *t;
	struct sbi_dlist *timer_events = sbi_scratch_thishart_timerlist();

	// Pop the head off the list
	if (unlikely(sbi_list_empty(timer_events))) {
		sbi_printf("Fatal: unexpectedly empty timer queue");
		sbi_hart_hang();
	}

	t = sbi_list_first_entry(timer_events, struct tl_entry, head);
	sbi_list_del(&t->head);

	// Release this entry back to the free list
	source = t->source;
	spin_lock(&timer_lock);
	sbi_list_add_tail(&t->head, &free_list);
	spin_unlock(&timer_lock);

	// Handle any platform-specific interrupt functionality
	csr_clear(CSR_MIE, MIP_MTIP);
	sbi_platform_timer_event_handle(sbi_platform_thishart_ptr(), source);

	if (source == SBI_TIMER_SOURCE_ECALL) {
		/*
		 * If sstc extension is available, supervisor can receive the timer
		 * directly without M-mode come in between. This function should
		 * only invoked if M-mode programs the timer for its own purpose.
		 */

		if (!sbi_hart_has_extension(sbi_scratch_thishart_ptr(), SBI_HART_EXT_SSTC))
			csr_set(CSR_MIP, MIP_STIP);
	}
}

const struct sbi_timer_device *sbi_timer_get_device(void)
{
	return timer_dev;
}

void sbi_timer_set_device(const struct sbi_timer_device *dev)
{
	if (!dev || timer_dev)
		return;

	timer_dev = dev;
	if (!get_time_val && timer_dev->timer_value)
		get_time_val = timer_dev->timer_value;
}

int sbi_timer_init(struct sbi_scratch *scratch, bool cold_boot)
{
	int i;
	struct sbi_dlist *timer_events;
	u64 *time_delta;
	const struct sbi_platform *plat = sbi_platform_ptr(scratch);

	if (cold_boot) {
		time_delta_off = sbi_scratch_alloc_offset(sizeof(*time_delta));
		if (!time_delta_off)
			return SBI_ENOMEM;

		timer_list_off = sbi_scratch_alloc_offset(sizeof(struct sbi_dlist));
		if (!timer_list_off)
			return SBI_ENOMEM;

		free_list.next = free_list.prev = &free_list;
		for (i = 0; i < MAX_TIMER_ENTRIES; i++) {
			sbi_list_add_tail(&timer_list_entries[i].head, &free_list);
		}

		if (sbi_hart_has_extension(scratch, SBI_HART_EXT_TIME))
			get_time_val = get_ticks;
	} else {
		if (!time_delta_off)
			return SBI_ENOMEM;

		if (!timer_list_off)
			return SBI_ENOMEM;
	}

	time_delta = sbi_scratch_offset_ptr(scratch, time_delta_off);
	*time_delta = 0;

	timer_events = sbi_scratch_offset_ptr(scratch, timer_list_off);
	timer_events->next = timer_events->prev = timer_events;

	return sbi_platform_timer_init(plat, cold_boot);
}

void sbi_timer_exit(struct sbi_scratch *scratch)
{
	if (timer_dev && timer_dev->timer_event_stop)
		timer_dev->timer_event_stop();

	csr_clear(CSR_MIP, MIP_STIP);
	csr_clear(CSR_MIE, MIP_MTIP);

	sbi_platform_timer_exit(sbi_platform_ptr(scratch));
}
