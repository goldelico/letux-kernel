// SPDX-License-Identifier: GPL-2.0
/*
 * Deadline Scheduling Class (SCHED_DEADLINE)
 *
 * Earliest Deadline First (EDF) + Constant Bandwidth Server (CBS).
 *
 * Tasks that periodically executes their instances for less than their
 * runtime won't miss any of their deadlines.
 * Tasks that are not periodic or sporadic or that tries to execute more
 * than their reserved bandwidth will be slowed down (and may potentially
 * miss some of their deadlines), and won't affect any other task.
 *
 * Copyright (C) 2012 Dario Faggioli <raistlin@linux.it>,
 *                    Juri Lelli <juri.lelli@gmail.com>,
 *                    Michael Trimarchi <michael@amarulasolutions.com>,
 *                    Fabio Checconi <fchecconi@gmail.com>
 */

#include <linux/cpuset.h>

/*
 * Default limits for DL period; on the top end we guard against small util
 * tasks still getting ridiculously long effective runtimes, on the bottom end we
 * guard against timer DoS.
 */
static unsigned int sysctl_sched_dl_period_max = 1 << 22; /* ~4 seconds */
static unsigned int sysctl_sched_dl_period_min = 100;     /* 100 us */
#ifdef CONFIG_SYSCTL
static struct ctl_table sched_dl_sysctls[] = {
	{
		.procname       = "sched_deadline_period_max_us",
		.data           = &sysctl_sched_dl_period_max,
		.maxlen         = sizeof(unsigned int),
		.mode           = 0644,
		.proc_handler   = proc_douintvec_minmax,
		.extra1         = (void *)&sysctl_sched_dl_period_min,
	},
	{
		.procname       = "sched_deadline_period_min_us",
		.data           = &sysctl_sched_dl_period_min,
		.maxlen         = sizeof(unsigned int),
		.mode           = 0644,
		.proc_handler   = proc_douintvec_minmax,
		.extra2         = (void *)&sysctl_sched_dl_period_max,
	},
};

static int __init sched_dl_sysctl_init(void)
{
	register_sysctl_init("kernel", sched_dl_sysctls);
	return 0;
}
late_initcall(sched_dl_sysctl_init);
#endif

static bool dl_server(struct sched_dl_entity *dl_se)
{
	return dl_se->dl_server;
}

static inline struct task_struct *dl_task_of(struct sched_dl_entity *dl_se)
{
	BUG_ON(dl_server(dl_se));
	return container_of(dl_se, struct task_struct, dl);
}

static inline struct rq *rq_of_dl_rq(struct dl_rq *dl_rq)
{
	return container_of(dl_rq, struct rq, dl);
}

static inline struct rq *rq_of_dl_se(struct sched_dl_entity *dl_se)
{
	struct rq *rq = dl_se->rq;

	if (!dl_server(dl_se))
		rq = task_rq(dl_task_of(dl_se));

	return rq;
}

static inline struct dl_rq *dl_rq_of_se(struct sched_dl_entity *dl_se)
{
	return &rq_of_dl_se(dl_se)->dl;
}

static inline int on_dl_rq(struct sched_dl_entity *dl_se)
{
	return !RB_EMPTY_NODE(&dl_se->rb_node);
}

#ifdef CONFIG_RT_MUTEXES
static inline struct sched_dl_entity *pi_of(struct sched_dl_entity *dl_se)
{
	return dl_se->pi_se;
}

static inline bool is_dl_boosted(struct sched_dl_entity *dl_se)
{
	return pi_of(dl_se) != dl_se;
}
#else
static inline struct sched_dl_entity *pi_of(struct sched_dl_entity *dl_se)
{
	return dl_se;
}

static inline bool is_dl_boosted(struct sched_dl_entity *dl_se)
{
	return false;
}
#endif

#ifdef CONFIG_SMP
static inline struct dl_bw *dl_bw_of(int i)
{
	RCU_LOCKDEP_WARN(!rcu_read_lock_sched_held(),
			 "sched RCU must be held");
	return &cpu_rq(i)->rd->dl_bw;
}

static inline int dl_bw_cpus(int i)
{
	struct root_domain *rd = cpu_rq(i)->rd;
	int cpus;

	RCU_LOCKDEP_WARN(!rcu_read_lock_sched_held(),
			 "sched RCU must be held");

	if (cpumask_subset(rd->span, cpu_active_mask))
		return cpumask_weight(rd->span);

	cpus = 0;

	for_each_cpu_and(i, rd->span, cpu_active_mask)
		cpus++;

	return cpus;
}

static inline unsigned long __dl_bw_capacity(const struct cpumask *mask)
{
	unsigned long cap = 0;
	int i;

	for_each_cpu_and(i, mask, cpu_active_mask)
		cap += arch_scale_cpu_capacity(i);

	return cap;
}

/*
 * XXX Fix: If 'rq->rd == def_root_domain' perform AC against capacity
 * of the CPU the task is running on rather rd's \Sum CPU capacity.
 */
static inline unsigned long dl_bw_capacity(int i)
{
	if (!sched_asym_cpucap_active() &&
	    arch_scale_cpu_capacity(i) == SCHED_CAPACITY_SCALE) {
		return dl_bw_cpus(i) << SCHED_CAPACITY_SHIFT;
	} else {
		RCU_LOCKDEP_WARN(!rcu_read_lock_sched_held(),
				 "sched RCU must be held");

		return __dl_bw_capacity(cpu_rq(i)->rd->span);
	}
}

static inline bool dl_bw_visited(int cpu, u64 gen)
{
	struct root_domain *rd = cpu_rq(cpu)->rd;

	if (rd->visit_gen == gen)
		return true;

	rd->visit_gen = gen;
	return false;
}

static inline
void __dl_update(struct dl_bw *dl_b, s64 bw)
{
	struct root_domain *rd = container_of(dl_b, struct root_domain, dl_bw);
	int i;

	RCU_LOCKDEP_WARN(!rcu_read_lock_sched_held(),
			 "sched RCU must be held");
	for_each_cpu_and(i, rd->span, cpu_active_mask) {
		struct rq *rq = cpu_rq(i);

		rq->dl.extra_bw += bw;
	}
}
#else
static inline struct dl_bw *dl_bw_of(int i)
{
	return &cpu_rq(i)->dl.dl_bw;
}

static inline int dl_bw_cpus(int i)
{
	return 1;
}

static inline unsigned long dl_bw_capacity(int i)
{
	return SCHED_CAPACITY_SCALE;
}

static inline bool dl_bw_visited(int cpu, u64 gen)
{
	return false;
}

static inline
void __dl_update(struct dl_bw *dl_b, s64 bw)
{
	struct dl_rq *dl = container_of(dl_b, struct dl_rq, dl_bw);

	dl->extra_bw += bw;
}
#endif

static inline
void __dl_sub(struct dl_bw *dl_b, u64 tsk_bw, int cpus)
{
	dl_b->total_bw -= tsk_bw;
	__dl_update(dl_b, (s32)tsk_bw / cpus);
}

static inline
void __dl_add(struct dl_bw *dl_b, u64 tsk_bw, int cpus)
{
	dl_b->total_bw += tsk_bw;
	__dl_update(dl_b, -((s32)tsk_bw / cpus));
}

static inline bool
__dl_overflow(struct dl_bw *dl_b, unsigned long cap, u64 old_bw, u64 new_bw)
{
	return dl_b->bw != -1 &&
	       cap_scale(dl_b->bw, cap) < dl_b->total_bw - old_bw + new_bw;
}

static inline
void __add_running_bw(u64 dl_bw, struct dl_rq *dl_rq)
{
	u64 old = dl_rq->running_bw;

	lockdep_assert_rq_held(rq_of_dl_rq(dl_rq));
	dl_rq->running_bw += dl_bw;
	SCHED_WARN_ON(dl_rq->running_bw < old); /* overflow */
	SCHED_WARN_ON(dl_rq->running_bw > dl_rq->this_bw);
	/* kick cpufreq (see the comment in kernel/sched/sched.h). */
	cpufreq_update_util(rq_of_dl_rq(dl_rq), 0);
}

static inline
void __sub_running_bw(u64 dl_bw, struct dl_rq *dl_rq)
{
	u64 old = dl_rq->running_bw;

	lockdep_assert_rq_held(rq_of_dl_rq(dl_rq));
	dl_rq->running_bw -= dl_bw;
	SCHED_WARN_ON(dl_rq->running_bw > old); /* underflow */
	if (dl_rq->running_bw > old)
		dl_rq->running_bw = 0;
	/* kick cpufreq (see the comment in kernel/sched/sched.h). */
	cpufreq_update_util(rq_of_dl_rq(dl_rq), 0);
}

static inline
void __add_rq_bw(u64 dl_bw, struct dl_rq *dl_rq)
{
	u64 old = dl_rq->this_bw;

	lockdep_assert_rq_held(rq_of_dl_rq(dl_rq));
	dl_rq->this_bw += dl_bw;
	SCHED_WARN_ON(dl_rq->this_bw < old); /* overflow */
}

static inline
void __sub_rq_bw(u64 dl_bw, struct dl_rq *dl_rq)
{
	u64 old = dl_rq->this_bw;

	lockdep_assert_rq_held(rq_of_dl_rq(dl_rq));
	dl_rq->this_bw -= dl_bw;
	SCHED_WARN_ON(dl_rq->this_bw > old); /* underflow */
	if (dl_rq->this_bw > old)
		dl_rq->this_bw = 0;
	SCHED_WARN_ON(dl_rq->running_bw > dl_rq->this_bw);
}

static inline
void add_rq_bw(struct sched_dl_entity *dl_se, struct dl_rq *dl_rq)
{
	if (!dl_entity_is_special(dl_se))
		__add_rq_bw(dl_se->dl_bw, dl_rq);
}

static inline
void sub_rq_bw(struct sched_dl_entity *dl_se, struct dl_rq *dl_rq)
{
	if (!dl_entity_is_special(dl_se))
		__sub_rq_bw(dl_se->dl_bw, dl_rq);
}

static inline
void add_running_bw(struct sched_dl_entity *dl_se, struct dl_rq *dl_rq)
{
	if (!dl_entity_is_special(dl_se))
		__add_running_bw(dl_se->dl_bw, dl_rq);
}

static inline
void sub_running_bw(struct sched_dl_entity *dl_se, struct dl_rq *dl_rq)
{
	if (!dl_entity_is_special(dl_se))
		__sub_running_bw(dl_se->dl_bw, dl_rq);
}

static void dl_rq_change_utilization(struct rq *rq, struct sched_dl_entity *dl_se, u64 new_bw)
{
	if (dl_se->dl_non_contending) {
		sub_running_bw(dl_se, &rq->dl);
		dl_se->dl_non_contending = 0;

		/*
		 * If the timer handler is currently running and the
		 * timer cannot be canceled, inactive_task_timer()
		 * will see that dl_not_contending is not set, and
		 * will not touch the rq's active utilization,
		 * so we are still safe.
		 */
		if (hrtimer_try_to_cancel(&dl_se->inactive_timer) == 1) {
			if (!dl_server(dl_se))
				put_task_struct(dl_task_of(dl_se));
		}
	}
	__sub_rq_bw(dl_se->dl_bw, &rq->dl);
	__add_rq_bw(new_bw, &rq->dl);
}

static void dl_change_utilization(struct task_struct *p, u64 new_bw)
{
	WARN_ON_ONCE(p->dl.flags & SCHED_FLAG_SUGOV);

	if (task_on_rq_queued(p))
		return;

	dl_rq_change_utilization(task_rq(p), &p->dl, new_bw);
}

static void __dl_clear_params(struct sched_dl_entity *dl_se);

/*
 * The utilization of a task cannot be immediately removed from
 * the rq active utilization (running_bw) when the task blocks.
 * Instead, we have to wait for the so called "0-lag time".
 *
 * If a task blocks before the "0-lag time", a timer (the inactive
 * timer) is armed, and running_bw is decreased when the timer
 * fires.
 *
 * If the task wakes up again before the inactive timer fires,
 * the timer is canceled, whereas if the task wakes up after the
 * inactive timer fired (and running_bw has been decreased) the
 * task's utilization has to be added to running_bw again.
 * A flag in the deadline scheduling entity (dl_non_contending)
 * is used to avoid race conditions between the inactive timer handler
 * and task wakeups.
 *
 * The following diagram shows how running_bw is updated. A task is
 * "ACTIVE" when its utilization contributes to running_bw; an
 * "ACTIVE contending" task is in the TASK_RUNNING state, while an
 * "ACTIVE non contending" task is a blocked task for which the "0-lag time"
 * has not passed yet. An "INACTIVE" task is a task for which the "0-lag"
 * time already passed, which does not contribute to running_bw anymore.
 *                              +------------------+
 *             wakeup           |    ACTIVE        |
 *          +------------------>+   contending     |
 *          | add_running_bw    |                  |
 *          |                   +----+------+------+
 *          |                        |      ^
 *          |                dequeue |      |
 * +--------+-------+                |      |
 * |                |   t >= 0-lag   |      | wakeup
 * |    INACTIVE    |<---------------+      |
 * |                | sub_running_bw |      |
 * +--------+-------+                |      |
 *          ^                        |      |
 *          |              t < 0-lag |      |
 *          |                        |      |
 *          |                        V      |
 *          |                   +----+------+------+
 *          | sub_running_bw    |    ACTIVE        |
 *          +-------------------+                  |
 *            inactive timer    |  non contending  |
 *            fired             +------------------+
 *
 * The task_non_contending() function is invoked when a task
 * blocks, and checks if the 0-lag time already passed or
 * not (in the first case, it directly updates running_bw;
 * in the second case, it arms the inactive timer).
 *
 * The task_contending() function is invoked when a task wakes
 * up, and checks if the task is still in the "ACTIVE non contending"
 * state or not (in the second case, it updates running_bw).
 */
static void task_non_contending(struct sched_dl_entity *dl_se)
{
	struct hrtimer *timer = &dl_se->inactive_timer;
	struct rq *rq = rq_of_dl_se(dl_se);
	struct dl_rq *dl_rq = &rq->dl;
	s64 zerolag_time;

	/*
	 * If this is a non-deadline task that has been boosted,
	 * do nothing
	 */
	if (dl_se->dl_runtime == 0)
		return;

	if (dl_entity_is_special(dl_se))
		return;

	WARN_ON(dl_se->dl_non_contending);

	zerolag_time = dl_se->deadline -
		 div64_long((dl_se->runtime * dl_se->dl_period),
			dl_se->dl_runtime);

	/*
	 * Using relative times instead of the absolute "0-lag time"
	 * allows to simplify the code
	 */
	zerolag_time -= rq_clock(rq);

	/*
	 * If the "0-lag time" already passed, decrease the active
	 * utilization now, instead of starting a timer
	 */
	if ((zerolag_time < 0) || hrtimer_active(&dl_se->inactive_timer)) {
		if (dl_server(dl_se)) {
			sub_running_bw(dl_se, dl_rq);
		} else {
			struct task_struct *p = dl_task_of(dl_se);

			if (dl_task(p))
				sub_running_bw(dl_se, dl_rq);

			if (!dl_task(p) || READ_ONCE(p->__state) == TASK_DEAD) {
				struct dl_bw *dl_b = dl_bw_of(task_cpu(p));

				if (READ_ONCE(p->__state) == TASK_DEAD)
					sub_rq_bw(dl_se, &rq->dl);
				raw_spin_lock(&dl_b->lock);
				__dl_sub(dl_b, dl_se->dl_bw, dl_bw_cpus(task_cpu(p)));
				raw_spin_unlock(&dl_b->lock);
				__dl_clear_params(dl_se);
			}
		}

		return;
	}

	dl_se->dl_non_contending = 1;
	if (!dl_server(dl_se))
		get_task_struct(dl_task_of(dl_se));

	hrtimer_start(timer, ns_to_ktime(zerolag_time), HRTIMER_MODE_REL_HARD);
}

static void task_contending(struct sched_dl_entity *dl_se, int flags)
{
	struct dl_rq *dl_rq = dl_rq_of_se(dl_se);

	/*
	 * If this is a non-deadline task that has been boosted,
	 * do nothing
	 */
	if (dl_se->dl_runtime == 0)
		return;

	if (flags & ENQUEUE_MIGRATED)
		add_rq_bw(dl_se, dl_rq);

	if (dl_se->dl_non_contending) {
		dl_se->dl_non_contending = 0;
		/*
		 * If the timer handler is currently running and the
		 * timer cannot be canceled, inactive_task_timer()
		 * will see that dl_not_contending is not set, and
		 * will not touch the rq's active utilization,
		 * so we are still safe.
		 */
		if (hrtimer_try_to_cancel(&dl_se->inactive_timer) == 1) {
			if (!dl_server(dl_se))
				put_task_struct(dl_task_of(dl_se));
		}
	} else {
		/*
		 * Since "dl_non_contending" is not set, the
		 * task's utilization has already been removed from
		 * active utilization (either when the task blocked,
		 * when the "inactive timer" fired).
		 * So, add it back.
		 */
		add_running_bw(dl_se, dl_rq);
	}
}

static inline int is_leftmost(struct sched_dl_entity *dl_se, struct dl_rq *dl_rq)
{
	return rb_first_cached(&dl_rq->root) == &dl_se->rb_node;
}

static void init_dl_rq_bw_ratio(struct dl_rq *dl_rq);

void init_dl_bw(struct dl_bw *dl_b)
{
	raw_spin_lock_init(&dl_b->lock);
	if (global_rt_runtime() == RUNTIME_INF)
		dl_b->bw = -1;
	else
		dl_b->bw = to_ratio(global_rt_period(), global_rt_runtime());
	dl_b->total_bw = 0;
}

void init_dl_rq(struct dl_rq *dl_rq)
{
	dl_rq->root = RB_ROOT_CACHED;

#ifdef CONFIG_SMP
	/* zero means no -deadline tasks */
	dl_rq->earliest_dl.curr = dl_rq->earliest_dl.next = 0;

	dl_rq->overloaded = 0;
	dl_rq->pushable_dl_tasks_root = RB_ROOT_CACHED;
#else
	init_dl_bw(&dl_rq->dl_bw);
#endif

	dl_rq->running_bw = 0;
	dl_rq->this_bw = 0;
	init_dl_rq_bw_ratio(dl_rq);
}

#ifdef CONFIG_SMP

static inline int dl_overloaded(struct rq *rq)
{
	return atomic_read(&rq->rd->dlo_count);
}

static inline void dl_set_overload(struct rq *rq)
{
	if (!rq->online)
		return;

	cpumask_set_cpu(rq->cpu, rq->rd->dlo_mask);
	/*
	 * Must be visible before the overload count is
	 * set (as in sched_rt.c).
	 *
	 * Matched by the barrier in pull_dl_task().
	 */
	smp_wmb();
	atomic_inc(&rq->rd->dlo_count);
}

static inline void dl_clear_overload(struct rq *rq)
{
	if (!rq->online)
		return;

	atomic_dec(&rq->rd->dlo_count);
	cpumask_clear_cpu(rq->cpu, rq->rd->dlo_mask);
}

#define __node_2_pdl(node) \
	rb_entry((node), struct task_struct, pushable_dl_tasks)

static inline bool __pushable_less(struct rb_node *a, const struct rb_node *b)
{
	return dl_entity_preempt(&__node_2_pdl(a)->dl, &__node_2_pdl(b)->dl);
}

static inline int has_pushable_dl_tasks(struct rq *rq)
{
	return !RB_EMPTY_ROOT(&rq->dl.pushable_dl_tasks_root.rb_root);
}

/*
 * The list of pushable -deadline task is not a plist, like in
 * sched_rt.c, it is an rb-tree with tasks ordered by deadline.
 */
static void enqueue_pushable_dl_task(struct rq *rq, struct task_struct *p)
{
	struct rb_node *leftmost;

	WARN_ON_ONCE(!RB_EMPTY_NODE(&p->pushable_dl_tasks));

	leftmost = rb_add_cached(&p->pushable_dl_tasks,
				 &rq->dl.pushable_dl_tasks_root,
				 __pushable_less);
	if (leftmost)
		rq->dl.earliest_dl.next = p->dl.deadline;

	if (!rq->dl.overloaded) {
		dl_set_overload(rq);
		rq->dl.overloaded = 1;
	}
}

static void dequeue_pushable_dl_task(struct rq *rq, struct task_struct *p)
{
	struct dl_rq *dl_rq = &rq->dl;
	struct rb_root_cached *root = &dl_rq->pushable_dl_tasks_root;
	struct rb_node *leftmost;

	if (RB_EMPTY_NODE(&p->pushable_dl_tasks))
		return;

	leftmost = rb_erase_cached(&p->pushable_dl_tasks, root);
	if (leftmost)
		dl_rq->earliest_dl.next = __node_2_pdl(leftmost)->dl.deadline;

	RB_CLEAR_NODE(&p->pushable_dl_tasks);

	if (!has_pushable_dl_tasks(rq) && rq->dl.overloaded) {
		dl_clear_overload(rq);
		rq->dl.overloaded = 0;
	}
}

static int push_dl_task(struct rq *rq);

static inline bool need_pull_dl_task(struct rq *rq, struct task_struct *prev)
{
	return rq->online && dl_task(prev);
}

static DEFINE_PER_CPU(struct balance_callback, dl_push_head);
static DEFINE_PER_CPU(struct balance_callback, dl_pull_head);

static void push_dl_tasks(struct rq *);
static void pull_dl_task(struct rq *);

static inline void deadline_queue_push_tasks(struct rq *rq)
{
	if (!has_pushable_dl_tasks(rq))
		return;

	queue_balance_callback(rq, &per_cpu(dl_push_head, rq->cpu), push_dl_tasks);
}

static inline void deadline_queue_pull_task(struct rq *rq)
{
	queue_balance_callback(rq, &per_cpu(dl_pull_head, rq->cpu), pull_dl_task);
}

static struct rq *find_lock_later_rq(struct task_struct *task, struct rq *rq);

static struct rq *dl_task_offline_migration(struct rq *rq, struct task_struct *p)
{
	struct rq *later_rq = NULL;
	struct dl_bw *dl_b;

	later_rq = find_lock_later_rq(p, rq);
	if (!later_rq) {
		int cpu;

		/*
		 * If we cannot preempt any rq, fall back to pick any
		 * online CPU:
		 */
		cpu = cpumask_any_and(cpu_active_mask, p->cpus_ptr);
		if (cpu >= nr_cpu_ids) {
			/*
			 * Failed to find any suitable CPU.
			 * The task will never come back!
			 */
			WARN_ON_ONCE(dl_bandwidth_enabled());

			/*
			 * If admission control is disabled we
			 * try a little harder to let the task
			 * run.
			 */
			cpu = cpumask_any(cpu_active_mask);
		}
		later_rq = cpu_rq(cpu);
		double_lock_balance(rq, later_rq);
	}

	if (p->dl.dl_non_contending || p->dl.dl_throttled) {
		/*
		 * Inactive timer is armed (or callback is running, but
		 * waiting for us to release rq locks). In any case, when it
		 * will fire (or continue), it will see running_bw of this
		 * task migrated to later_rq (and correctly handle it).
		 */
		sub_running_bw(&p->dl, &rq->dl);
		sub_rq_bw(&p->dl, &rq->dl);

		add_rq_bw(&p->dl, &later_rq->dl);
		add_running_bw(&p->dl, &later_rq->dl);
	} else {
		sub_rq_bw(&p->dl, &rq->dl);
		add_rq_bw(&p->dl, &later_rq->dl);
	}

	/*
	 * And we finally need to fix up root_domain(s) bandwidth accounting,
	 * since p is still hanging out in the old (now moved to default) root
	 * domain.
	 */
	dl_b = &rq->rd->dl_bw;
	raw_spin_lock(&dl_b->lock);
	__dl_sub(dl_b, p->dl.dl_bw, cpumask_weight(rq->rd->span));
	raw_spin_unlock(&dl_b->lock);

	dl_b = &later_rq->rd->dl_bw;
	raw_spin_lock(&dl_b->lock);
	__dl_add(dl_b, p->dl.dl_bw, cpumask_weight(later_rq->rd->span));
	raw_spin_unlock(&dl_b->lock);

	set_task_cpu(p, later_rq->cpu);
	double_unlock_balance(later_rq, rq);

	return later_rq;
}

#else

static inline
void enqueue_pushable_dl_task(struct rq *rq, struct task_struct *p)
{
}

static inline
void dequeue_pushable_dl_task(struct rq *rq, struct task_struct *p)
{
}

static inline
void inc_dl_migration(struct sched_dl_entity *dl_se, struct dl_rq *dl_rq)
{
}

static inline
void dec_dl_migration(struct sched_dl_entity *dl_se, struct dl_rq *dl_rq)
{
}

static inline void deadline_queue_push_tasks(struct rq *rq)
{
}

static inline void deadline_queue_pull_task(struct rq *rq)
{
}
#endif /* CONFIG_SMP */

static void
enqueue_dl_entity(struct sched_dl_entity *dl_se, int flags);
static void enqueue_task_dl(struct rq *rq, struct task_struct *p, int flags);
static void dequeue_dl_entity(struct sched_dl_entity *dl_se, int flags);
static void wakeup_preempt_dl(struct rq *rq, struct task_struct *p, int flags);

static inline void replenish_dl_new_period(struct sched_dl_entity *dl_se,
					    struct rq *rq)
{
	/* for non-boosted task, pi_of(dl_se) == dl_se */
	dl_se->deadline = rq_clock(rq) + pi_of(dl_se)->dl_deadline;
	dl_se->runtime = pi_of(dl_se)->dl_runtime;

	/*
	 * If it is a deferred reservation, and the server
	 * is not handling an starvation case, defer it.
	 */
	if (dl_se->dl_defer && !dl_se->dl_defer_running) {
		dl_se->dl_throttled = 1;
		dl_se->dl_defer_armed = 1;
	}
}

/*
 * We are being explicitly informed that a new instance is starting,
 * and this means that:
 *  - the absolute deadline of the entity has to be placed at
 *    current time + relative deadline;
 *  - the runtime of the entity has to be set to the maximum value.
 *
 * The capability of specifying such event is useful whenever a -deadline
 * entity wants to (try to!) synchronize its behaviour with the scheduler's
 * one, and to (try to!) reconcile itself with its own scheduling
 * parameters.
 */
static inline void setup_new_dl_entity(struct sched_dl_entity *dl_se)
{
	struct dl_rq *dl_rq = dl_rq_of_se(dl_se);
	struct rq *rq = rq_of_dl_rq(dl_rq);

	WARN_ON(is_dl_boosted(dl_se));
	WARN_ON(dl_time_before(rq_clock(rq), dl_se->deadline));

	/*
	 * We are racing with the deadline timer. So, do nothing because
	 * the deadline timer handler will take care of properly recharging
	 * the runtime and postponing the deadline
	 */
	if (dl_se->dl_throttled)
		return;

	/*
	 * We use the regular wall clock time to set deadlines in the
	 * future; in fact, we must consider execution overheads (time
	 * spent on hardirq context, etc.).
	 */
	replenish_dl_new_period(dl_se, rq);
}

static int start_dl_timer(struct sched_dl_entity *dl_se);
static bool dl_entity_overflow(struct sched_dl_entity *dl_se, u64 t);

/*
 * Pure Earliest Deadline First (EDF) scheduling does not deal with the
 * possibility of a entity lasting more than what it declared, and thus
 * exhausting its runtime.
 *
 * Here we are interested in making runtime overrun possible, but we do
 * not want a entity which is misbehaving to affect the scheduling of all
 * other entities.
 * Therefore, a budgeting strategy called Constant Bandwidth Server (CBS)
 * is used, in order to confine each entity within its own bandwidth.
 *
 * This function deals exactly with that, and ensures that when the runtime
 * of a entity is replenished, its deadline is also postponed. That ensures
 * the overrunning entity can't interfere with other entity in the system and
 * can't make them miss their deadlines. Reasons why this kind of overruns
 * could happen are, typically, a entity voluntarily trying to overcome its
 * runtime, or it just underestimated it during sched_setattr().
 */
static void replenish_dl_entity(struct sched_dl_entity *dl_se)
{
	struct dl_rq *dl_rq = dl_rq_of_se(dl_se);
	struct rq *rq = rq_of_dl_rq(dl_rq);

	WARN_ON_ONCE(pi_of(dl_se)->dl_runtime <= 0);

	/*
	 * This could be the case for a !-dl task that is boosted.
	 * Just go with full inherited parameters.
	 *
	 * Or, it could be the case of a deferred reservation that
	 * was not able to consume its runtime in background and
	 * reached this point with current u > U.
	 *
	 * In both cases, set a new period.
	 */
	if (dl_se->dl_deadline == 0 ||
	    (dl_se->dl_defer_armed && dl_entity_overflow(dl_se, rq_clock(rq)))) {
		dl_se->deadline = rq_clock(rq) + pi_of(dl_se)->dl_deadline;
		dl_se->runtime = pi_of(dl_se)->dl_runtime;
	}

	if (dl_se->dl_yielded && dl_se->runtime > 0)
		dl_se->runtime = 0;

	/*
	 * We keep moving the deadline away until we get some
	 * available runtime for the entity. This ensures correct
	 * handling of situations where the runtime overrun is
	 * arbitrary large.
	 */
	while (dl_se->runtime <= 0) {
		dl_se->deadline += pi_of(dl_se)->dl_period;
		dl_se->runtime += pi_of(dl_se)->dl_runtime;
	}

	/*
	 * At this point, the deadline really should be "in
	 * the future" with respect to rq->clock. If it's
	 * not, we are, for some reason, lagging too much!
	 * Anyway, after having warn userspace abut that,
	 * we still try to keep the things running by
	 * resetting the deadline and the budget of the
	 * entity.
	 */
	if (dl_time_before(dl_se->deadline, rq_clock(rq))) {
		printk_deferred_once("sched: DL replenish lagged too much\n");
		replenish_dl_new_period(dl_se, rq);
	}

	if (dl_se->dl_yielded)
		dl_se->dl_yielded = 0;
	if (dl_se->dl_throttled)
		dl_se->dl_throttled = 0;

	/*
	 * If this is the replenishment of a deferred reservation,
	 * clear the flag and return.
	 */
	if (dl_se->dl_defer_armed) {
		dl_se->dl_defer_armed = 0;
		return;
	}

	/*
	 * A this point, if the deferred server is not armed, and the deadline
	 * is in the future, if it is not running already, throttle the server
	 * and arm the defer timer.
	 */
	if (dl_se->dl_defer && !dl_se->dl_defer_running &&
	    dl_time_before(rq_clock(dl_se->rq), dl_se->deadline - dl_se->runtime)) {
		if (!is_dl_boosted(dl_se) && dl_se->server_has_tasks(dl_se)) {

			/*
			 * Set dl_se->dl_defer_armed and dl_throttled variables to
			 * inform the start_dl_timer() that this is a deferred
			 * activation.
			 */
			dl_se->dl_defer_armed = 1;
			dl_se->dl_throttled = 1;
			if (!start_dl_timer(dl_se)) {
				/*
				 * If for whatever reason (delays), a previous timer was
				 * queued but not serviced, cancel it and clean the
				 * deferrable server variables intended for start_dl_timer().
				 */
				hrtimer_try_to_cancel(&dl_se->dl_timer);
				dl_se->dl_defer_armed = 0;
				dl_se->dl_throttled = 0;
			}
		}
	}
}

/*
 * Here we check if --at time t-- an entity (which is probably being
 * [re]activated or, in general, enqueued) can use its remaining runtime
 * and its current deadline _without_ exceeding the bandwidth it is
 * assigned (function returns true if it can't). We are in fact applying
 * one of the CBS rules: when a task wakes up, if the residual runtime
 * over residual deadline fits within the allocated bandwidth, then we
 * can keep the current (absolute) deadline and residual budget without
 * disrupting the schedulability of the system. Otherwise, we should
 * refill the runtime and set the deadline a period in the future,
 * because keeping the current (absolute) deadline of the task would
 * result in breaking guarantees promised to other tasks (refer to
 * Documentation/scheduler/sched-deadline.rst for more information).
 *
 * This function returns true if:
 *
 *   runtime / (deadline - t) > dl_runtime / dl_deadline ,
 *
 * IOW we can't recycle current parameters.
 *
 * Notice that the bandwidth check is done against the deadline. For
 * task with deadline equal to period this is the same of using
 * dl_period instead of dl_deadline in the equation above.
 */
static bool dl_entity_overflow(struct sched_dl_entity *dl_se, u64 t)
{
	u64 left, right;

	/*
	 * left and right are the two sides of the equation above,
	 * after a bit of shuffling to use multiplications instead
	 * of divisions.
	 *
	 * Note that none of the time values involved in the two
	 * multiplications are absolute: dl_deadline and dl_runtime
	 * are the relative deadline and the maximum runtime of each
	 * instance, runtime is the runtime left for the last instance
	 * and (deadline - t), since t is rq->clock, is the time left
	 * to the (absolute) deadline. Even if overflowing the u64 type
	 * is very unlikely to occur in both cases, here we scale down
	 * as we want to avoid that risk at all. Scaling down by 10
	 * means that we reduce granularity to 1us. We are fine with it,
	 * since this is only a true/false check and, anyway, thinking
	 * of anything below microseconds resolution is actually fiction
	 * (but still we want to give the user that illusion >;).
	 */
	left = (pi_of(dl_se)->dl_deadline >> DL_SCALE) * (dl_se->runtime >> DL_SCALE);
	right = ((dl_se->deadline - t) >> DL_SCALE) *
		(pi_of(dl_se)->dl_runtime >> DL_SCALE);

	return dl_time_before(right, left);
}

/*
 * Revised wakeup rule [1]: For self-suspending tasks, rather then
 * re-initializing task's runtime and deadline, the revised wakeup
 * rule adjusts the task's runtime to avoid the task to overrun its
 * density.
 *
 * Reasoning: a task may overrun the density if:
 *    runtime / (deadline - t) > dl_runtime / dl_deadline
 *
 * Therefore, runtime can be adjusted to:
 *     runtime = (dl_runtime / dl_deadline) * (deadline - t)
 *
 * In such way that runtime will be equal to the maximum density
 * the task can use without breaking any rule.
 *
 * [1] Luca Abeni, Giuseppe Lipari, and Juri Lelli. 2015. Constant
 * bandwidth server revisited. SIGBED Rev. 11, 4 (January 2015), 19-24.
 */
static void
update_dl_revised_wakeup(struct sched_dl_entity *dl_se, struct rq *rq)
{
	u64 laxity = dl_se->deadline - rq_clock(rq);

	/*
	 * If the task has deadline < period, and the deadline is in the past,
	 * it should already be throttled before this check.
	 *
	 * See update_dl_entity() comments for further details.
	 */
	WARN_ON(dl_time_before(dl_se->deadline, rq_clock(rq)));

	dl_se->runtime = (dl_se->dl_density * laxity) >> BW_SHIFT;
}

/*
 * Regarding the deadline, a task with implicit deadline has a relative
 * deadline == relative period. A task with constrained deadline has a
 * relative deadline <= relative period.
 *
 * We support constrained deadline tasks. However, there are some restrictions
 * applied only for tasks which do not have an implicit deadline. See
 * update_dl_entity() to know more about such restrictions.
 *
 * The dl_is_implicit() returns true if the task has an implicit deadline.
 */
static inline bool dl_is_implicit(struct sched_dl_entity *dl_se)
{
	return dl_se->dl_deadline == dl_se->dl_period;
}

/*
 * When a deadline entity is placed in the runqueue, its runtime and deadline
 * might need to be updated. This is done by a CBS wake up rule. There are two
 * different rules: 1) the original CBS; and 2) the Revisited CBS.
 *
 * When the task is starting a new period, the Original CBS is used. In this
 * case, the runtime is replenished and a new absolute deadline is set.
 *
 * When a task is queued before the begin of the next period, using the
 * remaining runtime and deadline could make the entity to overflow, see
 * dl_entity_overflow() to find more about runtime overflow. When such case
 * is detected, the runtime and deadline need to be updated.
 *
 * If the task has an implicit deadline, i.e., deadline == period, the Original
 * CBS is applied. The runtime is replenished and a new absolute deadline is
 * set, as in the previous cases.
 *
 * However, the Original CBS does not work properly for tasks with
 * deadline < period, which are said to have a constrained deadline. By
 * applying the Original CBS, a constrained deadline task would be able to run
 * runtime/deadline in a period. With deadline < period, the task would
 * overrun the runtime/period allowed bandwidth, breaking the admission test.
 *
 * In order to prevent this misbehave, the Revisited CBS is used for
 * constrained deadline tasks when a runtime overflow is detected. In the
 * Revisited CBS, rather than replenishing & setting a new absolute deadline,
 * the remaining runtime of the task is reduced to avoid runtime overflow.
 * Please refer to the comments update_dl_revised_wakeup() function to find
 * more about the Revised CBS rule.
 */
static void update_dl_entity(struct sched_dl_entity *dl_se)
{
	struct rq *rq = rq_of_dl_se(dl_se);

	if (dl_time_before(dl_se->deadline, rq_clock(rq)) ||
	    dl_entity_overflow(dl_se, rq_clock(rq))) {

		if (unlikely(!dl_is_implicit(dl_se) &&
			     !dl_time_before(dl_se->deadline, rq_clock(rq)) &&
			     !is_dl_boosted(dl_se))) {
			update_dl_revised_wakeup(dl_se, rq);
			return;
		}

		replenish_dl_new_period(dl_se, rq);
	} else if (dl_server(dl_se) && dl_se->dl_defer) {
		/*
		 * The server can still use its previous deadline, so check if
		 * it left the dl_defer_running state.
		 */
		if (!dl_se->dl_defer_running) {
			dl_se->dl_defer_armed = 1;
			dl_se->dl_throttled = 1;
		}
	}
}

static inline u64 dl_next_period(struct sched_dl_entity *dl_se)
{
	return dl_se->deadline - dl_se->dl_deadline + dl_se->dl_period;
}

/*
 * If the entity depleted all its runtime, and if we want it to sleep
 * while waiting for some new execution time to become available, we
 * set the bandwidth replenishment timer to the replenishment instant
 * and try to activate it.
 *
 * Notice that it is important for the caller to know if the timer
 * actually started or not (i.e., the replenishment instant is in
 * the future or in the past).
 */
static int start_dl_timer(struct sched_dl_entity *dl_se)
{
	struct hrtimer *timer = &dl_se->dl_timer;
	struct dl_rq *dl_rq = dl_rq_of_se(dl_se);
	struct rq *rq = rq_of_dl_rq(dl_rq);
	ktime_t now, act;
	s64 delta;

	lockdep_assert_rq_held(rq);

	/*
	 * We want the timer to fire at the deadline, but considering
	 * that it is actually coming from rq->clock and not from
	 * hrtimer's time base reading.
	 *
	 * The deferred reservation will have its timer set to
	 * (deadline - runtime). At that point, the CBS rule will decide
	 * if the current deadline can be used, or if a replenishment is
	 * required to avoid add too much pressure on the system
	 * (current u > U).
	 */
	if (dl_se->dl_defer_armed) {
		WARN_ON_ONCE(!dl_se->dl_throttled);
		act = ns_to_ktime(dl_se->deadline - dl_se->runtime);
	} else {
		/* act = deadline - rel-deadline + period */
		act = ns_to_ktime(dl_next_period(dl_se));
	}

	now = hrtimer_cb_get_time(timer);
	delta = ktime_to_ns(now) - rq_clock(rq);
	act = ktime_add_ns(act, delta);

	/*
	 * If the expiry time already passed, e.g., because the value
	 * chosen as the deadline is too small, don't even try to
	 * start the timer in the past!
	 */
	if (ktime_us_delta(act, now) < 0)
		return 0;

	/*
	 * !enqueued will guarantee another callback; even if one is already in
	 * progress. This ensures a balanced {get,put}_task_struct().
	 *
	 * The race against __run_timer() clearing the enqueued state is
	 * harmless because we're holding task_rq()->lock, therefore the timer
	 * expiring after we've done the check will wait on its task_rq_lock()
	 * and observe our state.
	 */
	if (!hrtimer_is_queued(timer)) {
		if (!dl_server(dl_se))
			get_task_struct(dl_task_of(dl_se));
		hrtimer_start(timer, act, HRTIMER_MODE_ABS_HARD);
	}

	return 1;
}

static void __push_dl_task(struct rq *rq, struct rq_flags *rf)
{
#ifdef CONFIG_SMP
	/*
	 * Queueing this task back might have overloaded rq, check if we need
	 * to kick someone away.
	 */
	if (has_pushable_dl_tasks(rq)) {
		/*
		 * Nothing relies on rq->lock after this, so its safe to drop
		 * rq->lock.
		 */
		rq_unpin_lock(rq, rf);
		push_dl_task(rq);
		rq_repin_lock(rq, rf);
	}
#endif
}

/* a defer timer will not be reset if the runtime consumed was < dl_server_min_res */
static const u64 dl_server_min_res = 1 * NSEC_PER_MSEC;

static enum hrtimer_restart dl_server_timer(struct hrtimer *timer, struct sched_dl_entity *dl_se)
{
	struct rq *rq = rq_of_dl_se(dl_se);
	u64 fw;

	scoped_guard (rq_lock, rq) {
		struct rq_flags *rf = &scope.rf;

		if (!dl_se->dl_throttled || !dl_se->dl_runtime)
			return HRTIMER_NORESTART;

		sched_clock_tick();
		update_rq_clock(rq);

		if (!dl_se->dl_runtime)
			return HRTIMER_NORESTART;

		if (!dl_se->server_has_tasks(dl_se)) {
			replenish_dl_entity(dl_se);
			return HRTIMER_NORESTART;
		}

		if (dl_se->dl_defer_armed) {
			/*
			 * First check if the server could consume runtime in background.
			 * If so, it is possible to push the defer timer for this amount
			 * of time. The dl_server_min_res serves as a limit to avoid
			 * forwarding the timer for a too small amount of time.
			 */
			if (dl_time_before(rq_clock(dl_se->rq),
					   (dl_se->deadline - dl_se->runtime - dl_server_min_res))) {

				/* reset the defer timer */
				fw = dl_se->deadline - rq_clock(dl_se->rq) - dl_se->runtime;

				hrtimer_forward_now(timer, ns_to_ktime(fw));
				return HRTIMER_RESTART;
			}

			dl_se->dl_defer_running = 1;
		}

		enqueue_dl_entity(dl_se, ENQUEUE_REPLENISH);

		if (!dl_task(dl_se->rq->curr) || dl_entity_preempt(dl_se, &dl_se->rq->curr->dl))
			resched_curr(rq);

		__push_dl_task(rq, rf);
	}

	return HRTIMER_NORESTART;
}

/*
 * This is the bandwidth enforcement timer callback. If here, we know
 * a task is not on its dl_rq, since the fact that the timer was running
 * means the task is throttled and needs a runtime replenishment.
 *
 * However, what we actually do depends on the fact the task is active,
 * (it is on its rq) or has been removed from there by a call to
 * dequeue_task_dl(). In the former case we must issue the runtime
 * replenishment and add the task back to the dl_rq; in the latter, we just
 * do nothing but clearing dl_throttled, so that runtime and deadline
 * updating (and the queueing back to dl_rq) will be done by the
 * next call to enqueue_task_dl().
 */
static enum hrtimer_restart dl_task_timer(struct hrtimer *timer)
{
	struct sched_dl_entity *dl_se = container_of(timer,
						     struct sched_dl_entity,
						     dl_timer);
	struct task_struct *p;
	struct rq_flags rf;
	struct rq *rq;

	if (dl_server(dl_se))
		return dl_server_timer(timer, dl_se);

	p = dl_task_of(dl_se);
	rq = task_rq_lock(p, &rf);

	/*
	 * The task might have changed its scheduling policy to something
	 * different than SCHED_DEADLINE (through switched_from_dl()).
	 */
	if (!dl_task(p))
		goto unlock;

	/*
	 * The task might have been boosted by someone else and might be in the
	 * boosting/deboosting path, its not throttled.
	 */
	if (is_dl_boosted(dl_se))
		goto unlock;

	/*
	 * Spurious timer due to start_dl_timer() race; or we already received
	 * a replenishment from rt_mutex_setprio().
	 */
	if (!dl_se->dl_throttled)
		goto unlock;

	sched_clock_tick();
	update_rq_clock(rq);

	/*
	 * If the throttle happened during sched-out; like:
	 *
	 *   schedule()
	 *     deactivate_task()
	 *       dequeue_task_dl()
	 *         update_curr_dl()
	 *           start_dl_timer()
	 *         __dequeue_task_dl()
	 *     prev->on_rq = 0;
	 *
	 * We can be both throttled and !queued. Replenish the counter
	 * but do not enqueue -- wait for our wakeup to do that.
	 */
	if (!task_on_rq_queued(p)) {
		replenish_dl_entity(dl_se);
		goto unlock;
	}

#ifdef CONFIG_SMP
	if (unlikely(!rq->online)) {
		/*
		 * If the runqueue is no longer available, migrate the
		 * task elsewhere. This necessarily changes rq.
		 */
		lockdep_unpin_lock(__rq_lockp(rq), rf.cookie);
		rq = dl_task_offline_migration(rq, p);
		rf.cookie = lockdep_pin_lock(__rq_lockp(rq));
		update_rq_clock(rq);

		/*
		 * Now that the task has been migrated to the new RQ and we
		 * have that locked, proceed as normal and enqueue the task
		 * there.
		 */
	}
#endif

	enqueue_task_dl(rq, p, ENQUEUE_REPLENISH);
	if (dl_task(rq->curr))
		wakeup_preempt_dl(rq, p, 0);
	else
		resched_curr(rq);

	__push_dl_task(rq, &rf);

unlock:
	task_rq_unlock(rq, p, &rf);

	/*
	 * This can free the task_struct, including this hrtimer, do not touch
	 * anything related to that after this.
	 */
	put_task_struct(p);

	return HRTIMER_NORESTART;
}

static void init_dl_task_timer(struct sched_dl_entity *dl_se)
{
	struct hrtimer *timer = &dl_se->dl_timer;

	hrtimer_init(timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_HARD);
	timer->function = dl_task_timer;
}

/*
 * During the activation, CBS checks if it can reuse the current task's
 * runtime and period. If the deadline of the task is in the past, CBS
 * cannot use the runtime, and so it replenishes the task. This rule
 * works fine for implicit deadline tasks (deadline == period), and the
 * CBS was designed for implicit deadline tasks. However, a task with
 * constrained deadline (deadline < period) might be awakened after the
 * deadline, but before the next period. In this case, replenishing the
 * task would allow it to run for runtime / deadline. As in this case
 * deadline < period, CBS enables a task to run for more than the
 * runtime / period. In a very loaded system, this can cause a domino
 * effect, making other tasks miss their deadlines.
 *
 * To avoid this problem, in the activation of a constrained deadline
 * task after the deadline but before the next period, throttle the
 * task and set the replenishing timer to the begin of the next period,
 * unless it is boosted.
 */
static inline void dl_check_constrained_dl(struct sched_dl_entity *dl_se)
{
	struct rq *rq = rq_of_dl_se(dl_se);

	if (dl_time_before(dl_se->deadline, rq_clock(rq)) &&
	    dl_time_before(rq_clock(rq), dl_next_period(dl_se))) {
		if (unlikely(is_dl_boosted(dl_se) || !start_dl_timer(dl_se)))
			return;
		dl_se->dl_throttled = 1;
		if (dl_se->runtime > 0)
			dl_se->runtime = 0;
	}
}

static
int dl_runtime_exceeded(struct sched_dl_entity *dl_se)
{
	return (dl_se->runtime <= 0);
}

/*
 * This function implements the GRUB accounting rule. According to the
 * GRUB reclaiming algorithm, the runtime is not decreased as "dq = -dt",
 * but as "dq = -(max{u, (Umax - Uinact - Uextra)} / Umax) dt",
 * where u is the utilization of the task, Umax is the maximum reclaimable
 * utilization, Uinact is the (per-runqueue) inactive utilization, computed
 * as the difference between the "total runqueue utilization" and the
 * "runqueue active utilization", and Uextra is the (per runqueue) extra
 * reclaimable utilization.
 * Since rq->dl.running_bw and rq->dl.this_bw contain utilizations multiplied
 * by 2^BW_SHIFT, the result has to be shifted right by BW_SHIFT.
 * Since rq->dl.bw_ratio contains 1 / Umax multiplied by 2^RATIO_SHIFT, dl_bw
 * is multiplied by rq->dl.bw_ratio and shifted right by RATIO_SHIFT.
 * Since delta is a 64 bit variable, to have an overflow its value should be
 * larger than 2^(64 - 20 - 8), which is more than 64 seconds. So, overflow is
 * not an issue here.
 */
static u64 grub_reclaim(u64 delta, struct rq *rq, struct sched_dl_entity *dl_se)
{
	u64 u_act;
	u64 u_inact = rq->dl.this_bw - rq->dl.running_bw; /* Utot - Uact */

	/*
	 * Instead of computing max{u, (u_max - u_inact - u_extra)}, we
	 * compare u_inact + u_extra with u_max - u, because u_inact + u_extra
	 * can be larger than u_max. So, u_max - u_inact - u_extra would be
	 * negative leading to wrong results.
	 */
	if (u_inact + rq->dl.extra_bw > rq->dl.max_bw - dl_se->dl_bw)
		u_act = dl_se->dl_bw;
	else
		u_act = rq->dl.max_bw - u_inact - rq->dl.extra_bw;

	u_act = (u_act * rq->dl.bw_ratio) >> RATIO_SHIFT;
	return (delta * u_act) >> BW_SHIFT;
}

s64 dl_scaled_delta_exec(struct rq *rq, struct sched_dl_entity *dl_se, s64 delta_exec)
{
	s64 scaled_delta_exec;

	/*
	 * For tasks that participate in GRUB, we implement GRUB-PA: the
	 * spare reclaimed bandwidth is used to clock down frequency.
	 *
	 * For the others, we still need to scale reservation parameters
	 * according to current frequency and CPU maximum capacity.
	 */
	if (unlikely(dl_se->flags & SCHED_FLAG_RECLAIM)) {
		scaled_delta_exec = grub_reclaim(delta_exec, rq, dl_se);
	} else {
		int cpu = cpu_of(rq);
		unsigned long scale_freq = arch_scale_freq_capacity(cpu);
		unsigned long scale_cpu = arch_scale_cpu_capacity(cpu);

		scaled_delta_exec = cap_scale(delta_exec, scale_freq);
		scaled_delta_exec = cap_scale(scaled_delta_exec, scale_cpu);
	}

	return scaled_delta_exec;
}

static inline void
update_stats_dequeue_dl(struct dl_rq *dl_rq, struct sched_dl_entity *dl_se,
			int flags);
static void update_curr_dl_se(struct rq *rq, struct sched_dl_entity *dl_se, s64 delta_exec)
{
	s64 scaled_delta_exec;

	if (unlikely(delta_exec <= 0)) {
		if (unlikely(dl_se->dl_yielded))
			goto throttle;
		return;
	}

	if (dl_server(dl_se) && dl_se->dl_throttled && !dl_se->dl_defer)
		return;

	if (dl_entity_is_special(dl_se))
		return;

	scaled_delta_exec = delta_exec;
	if (!dl_server(dl_se))
		scaled_delta_exec = dl_scaled_delta_exec(rq, dl_se, delta_exec);

	dl_se->runtime -= scaled_delta_exec;

	/*
	 * The fair server can consume its runtime while throttled (not queued/
	 * running as regular CFS).
	 *
	 * If the server consumes its entire runtime in this state. The server
	 * is not required for the current period. Thus, reset the server by
	 * starting a new period, pushing the activation.
	 */
	if (dl_se->dl_defer && dl_se->dl_throttled && dl_runtime_exceeded(dl_se)) {
		/*
		 * If the server was previously activated - the starving condition
		 * took place, it this point it went away because the fair scheduler
		 * was able to get runtime in background. So return to the initial
		 * state.
		 */
		dl_se->dl_defer_running = 0;

		hrtimer_try_to_cancel(&dl_se->dl_timer);

		replenish_dl_new_period(dl_se, dl_se->rq);

		/*
		 * Not being able to start the timer seems problematic. If it could not
		 * be started for whatever reason, we need to "unthrottle" the DL server
		 * and queue right away. Otherwise nothing might queue it. That's similar
		 * to what enqueue_dl_entity() does on start_dl_timer==0. For now, just warn.
		 */
		WARN_ON_ONCE(!start_dl_timer(dl_se));

		return;
	}

throttle:
	if (dl_runtime_exceeded(dl_se) || dl_se->dl_yielded) {
		dl_se->dl_throttled = 1;

		/* If requested, inform the user about runtime overruns. */
		if (dl_runtime_exceeded(dl_se) &&
		    (dl_se->flags & SCHED_FLAG_DL_OVERRUN))
			dl_se->dl_overrun = 1;

		dequeue_dl_entity(dl_se, 0);
		if (!dl_server(dl_se)) {
			update_stats_dequeue_dl(&rq->dl, dl_se, 0);
			dequeue_pushable_dl_task(rq, dl_task_of(dl_se));
		}

		if (unlikely(is_dl_boosted(dl_se) || !start_dl_timer(dl_se))) {
			if (dl_server(dl_se))
				enqueue_dl_entity(dl_se, ENQUEUE_REPLENISH);
			else
				enqueue_task_dl(rq, dl_task_of(dl_se), ENQUEUE_REPLENISH);
		}

		if (!is_leftmost(dl_se, &rq->dl))
			resched_curr(rq);
	}

	/*
	 * The fair server (sole dl_server) does not account for real-time
	 * workload because it is running fair work.
	 */
	if (dl_se == &rq->fair_server)
		return;

#ifdef CONFIG_RT_GROUP_SCHED
	/*
	 * Because -- for now -- we share the rt bandwidth, we need to
	 * account our runtime there too, otherwise actual rt tasks
	 * would be able to exceed the shared quota.
	 *
	 * Account to the root rt group for now.
	 *
	 * The solution we're working towards is having the RT groups scheduled
	 * using deadline servers -- however there's a few nasties to figure
	 * out before that can happen.
	 */
	if (rt_bandwidth_enabled()) {
		struct rt_rq *rt_rq = &rq->rt;

		raw_spin_lock(&rt_rq->rt_runtime_lock);
		/*
		 * We'll let actual RT tasks worry about the overflow here, we
		 * have our own CBS to keep us inline; only account when RT
		 * bandwidth is relevant.
		 */
		if (sched_rt_bandwidth_account(rt_rq))
			rt_rq->rt_time += delta_exec;
		raw_spin_unlock(&rt_rq->rt_runtime_lock);
	}
#endif
}

/*
 * In the non-defer mode, the idle time is not accounted, as the
 * server provides a guarantee.
 *
 * If the dl_server is in defer mode, the idle time is also considered
 * as time available for the fair server, avoiding a penalty for the
 * rt scheduler that did not consumed that time.
 */
void dl_server_update_idle_time(struct rq *rq, struct task_struct *p)
{
	s64 delta_exec;

	if (!rq->fair_server.dl_defer)
		return;

	/* no need to discount more */
	if (rq->fair_server.runtime < 0)
		return;

	delta_exec = rq_clock_task(rq) - p->se.exec_start;
	if (delta_exec < 0)
		return;

	rq->fair_server.runtime -= delta_exec;

	if (rq->fair_server.runtime < 0) {
		rq->fair_server.dl_defer_running = 0;
		rq->fair_server.runtime = 0;
	}

	p->se.exec_start = rq_clock_task(rq);
}

void dl_server_update(struct sched_dl_entity *dl_se, s64 delta_exec)
{
	/* 0 runtime = fair server disabled */
	if (dl_se->dl_runtime)
		update_curr_dl_se(dl_se->rq, dl_se, delta_exec);
}

void dl_server_start(struct sched_dl_entity *dl_se)
{
	struct rq *rq = dl_se->rq;

	/*
	 * XXX: the apply do not work fine at the init phase for the
	 * fair server because things are not yet set. We need to improve
	 * this before getting generic.
	 */
	if (!dl_server(dl_se)) {
		u64 runtime =  50 * NSEC_PER_MSEC;
		u64 period = 1000 * NSEC_PER_MSEC;

		dl_server_apply_params(dl_se, runtime, period, 1);

		dl_se->dl_server = 1;
		dl_se->dl_defer = 1;
		setup_new_dl_entity(dl_se);
	}

	if (!dl_se->dl_runtime)
		return;

	dl_se->dl_server_active = 1;
	enqueue_dl_entity(dl_se, ENQUEUE_WAKEUP);
	if (!dl_task(dl_se->rq->curr) || dl_entity_preempt(dl_se, &rq->curr->dl))
		resched_curr(dl_se->rq);
}

void dl_server_stop(struct sched_dl_entity *dl_se)
{
	if (!dl_se->dl_runtime)
		return;

	dequeue_dl_entity(dl_se, DEQUEUE_SLEEP);
	hrtimer_try_to_cancel(&dl_se->dl_timer);
	dl_se->dl_defer_armed = 0;
	dl_se->dl_throttled = 0;
	dl_se->dl_server_active = 0;
}

void dl_server_init(struct sched_dl_entity *dl_se, struct rq *rq,
		    dl_server_has_tasks_f has_tasks,
		    dl_server_pick_f pick_task)
{
	dl_se->rq = rq;
	dl_se->server_has_tasks = has_tasks;
	dl_se->server_pick_task = pick_task;
}

void __dl_server_attach_root(struct sched_dl_entity *dl_se, struct rq *rq)
{
	u64 new_bw = dl_se->dl_bw;
	int cpu = cpu_of(rq);
	struct dl_bw *dl_b;

	dl_b = dl_bw_of(cpu_of(rq));
	guard(raw_spinlock)(&dl_b->lock);

	if (!dl_bw_cpus(cpu))
		return;

	__dl_add(dl_b, new_bw, dl_bw_cpus(cpu));
}

int dl_server_apply_params(struct sched_dl_entity *dl_se, u64 runtime, u64 period, bool init)
{
	u64 old_bw = init ? 0 : to_ratio(dl_se->dl_period, dl_se->dl_runtime);
	u64 new_bw = to_ratio(period, runtime);
	struct rq *rq = dl_se->rq;
	int cpu = cpu_of(rq);
	struct dl_bw *dl_b;
	unsigned long cap;
	int retval = 0;
	int cpus;

	dl_b = dl_bw_of(cpu);
	guard(raw_spinlock)(&dl_b->lock);

	cpus = dl_bw_cpus(cpu);
	cap = dl_bw_capacity(cpu);

	if (__dl_overflow(dl_b, cap, old_bw, new_bw))
		return -EBUSY;

	if (init) {
		__add_rq_bw(new_bw, &rq->dl);
		__dl_add(dl_b, new_bw, cpus);
	} else {
		__dl_sub(dl_b, dl_se->dl_bw, cpus);
		__dl_add(dl_b, new_bw, cpus);

		dl_rq_change_utilization(rq, dl_se, new_bw);
	}

	dl_se->dl_runtime = runtime;
	dl_se->dl_deadline = period;
	dl_se->dl_period = period;

	dl_se->runtime = 0;
	dl_se->deadline = 0;

	dl_se->dl_bw = to_ratio(dl_se->dl_period, dl_se->dl_runtime);
	dl_se->dl_density = to_ratio(dl_se->dl_deadline, dl_se->dl_runtime);

	return retval;
}

/*
 * Update the current task's runtime statistics (provided it is still
 * a -deadline task and has not been removed from the dl_rq).
 */
static void update_curr_dl(struct rq *rq)
{
	struct task_struct *curr = rq->curr;
	struct sched_dl_entity *dl_se = &curr->dl;
	s64 delta_exec;

	if (!dl_task(curr) || !on_dl_rq(dl_se))
		return;

	/*
	 * Consumed budget is computed considering the time as
	 * observed by schedulable tasks (excluding time spent
	 * in hardirq context, etc.). Deadlines are instead
	 * computed using hard walltime. This seems to be the more
	 * natural solution, but the full ramifications of this
	 * approach need further study.
	 */
	delta_exec = update_curr_common(rq);
	update_curr_dl_se(rq, dl_se, delta_exec);
}

static enum hrtimer_restart inactive_task_timer(struct hrtimer *timer)
{
	struct sched_dl_entity *dl_se = container_of(timer,
						     struct sched_dl_entity,
						     inactive_timer);
	struct task_struct *p = NULL;
	struct rq_flags rf;
	struct rq *rq;

	if (!dl_server(dl_se)) {
		p = dl_task_of(dl_se);
		rq = task_rq_lock(p, &rf);
	} else {
		rq = dl_se->rq;
		rq_lock(rq, &rf);
	}

	sched_clock_tick();
	update_rq_clock(rq);

	if (dl_server(dl_se))
		goto no_task;

	if (!dl_task(p) || READ_ONCE(p->__state) == TASK_DEAD) {
		struct dl_bw *dl_b = dl_bw_of(task_cpu(p));

		if (READ_ONCE(p->__state) == TASK_DEAD && dl_se->dl_non_contending) {
			sub_running_bw(&p->dl, dl_rq_of_se(&p->dl));
			sub_rq_bw(&p->dl, dl_rq_of_se(&p->dl));
			dl_se->dl_non_contending = 0;
		}

		raw_spin_lock(&dl_b->lock);
		__dl_sub(dl_b, p->dl.dl_bw, dl_bw_cpus(task_cpu(p)));
		raw_spin_unlock(&dl_b->lock);
		__dl_clear_params(dl_se);

		goto unlock;
	}

no_task:
	if (dl_se->dl_non_contending == 0)
		goto unlock;

	sub_running_bw(dl_se, &rq->dl);
	dl_se->dl_non_contending = 0;
unlock:

	if (!dl_server(dl_se)) {
		task_rq_unlock(rq, p, &rf);
		put_task_struct(p);
	} else {
		rq_unlock(rq, &rf);
	}

	return HRTIMER_NORESTART;
}

static void init_dl_inactive_task_timer(struct sched_dl_entity *dl_se)
{
	struct hrtimer *timer = &dl_se->inactive_timer;

	hrtimer_init(timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_HARD);
	timer->function = inactive_task_timer;
}

#define __node_2_dle(node) \
	rb_entry((node), struct sched_dl_entity, rb_node)

#ifdef CONFIG_SMP

static void inc_dl_deadline(struct dl_rq *dl_rq, u64 deadline)
{
	struct rq *rq = rq_of_dl_rq(dl_rq);

	if (dl_rq->earliest_dl.curr == 0 ||
	    dl_time_before(deadline, dl_rq->earliest_dl.curr)) {
		if (dl_rq->earliest_dl.curr == 0)
			cpupri_set(&rq->rd->cpupri, rq->cpu, CPUPRI_HIGHER);
		dl_rq->earliest_dl.curr = deadline;
		cpudl_set(&rq->rd->cpudl, rq->cpu, deadline);
	}
}

static void dec_dl_deadline(struct dl_rq *dl_rq, u64 deadline)
{
	struct rq *rq = rq_of_dl_rq(dl_rq);

	/*
	 * Since we may have removed our earliest (and/or next earliest)
	 * task we must recompute them.
	 */
	if (!dl_rq->dl_nr_running) {
		dl_rq->earliest_dl.curr = 0;
		dl_rq->earliest_dl.next = 0;
		cpudl_clear(&rq->rd->cpudl, rq->cpu);
		cpupri_set(&rq->rd->cpupri, rq->cpu, rq->rt.highest_prio.curr);
	} else {
		struct rb_node *leftmost = rb_first_cached(&dl_rq->root);
		struct sched_dl_entity *entry = __node_2_dle(leftmost);

		dl_rq->earliest_dl.curr = entry->deadline;
		cpudl_set(&rq->rd->cpudl, rq->cpu, entry->deadline);
	}
}

#else

static inline void inc_dl_deadline(struct dl_rq *dl_rq, u64 deadline) {}
static inline void dec_dl_deadline(struct dl_rq *dl_rq, u64 deadline) {}

#endif /* CONFIG_SMP */

static inline
void inc_dl_tasks(struct sched_dl_entity *dl_se, struct dl_rq *dl_rq)
{
	u64 deadline = dl_se->deadline;

	dl_rq->dl_nr_running++;
	add_nr_running(rq_of_dl_rq(dl_rq), 1);

	inc_dl_deadline(dl_rq, deadline);
}

static inline
void dec_dl_tasks(struct sched_dl_entity *dl_se, struct dl_rq *dl_rq)
{
	WARN_ON(!dl_rq->dl_nr_running);
	dl_rq->dl_nr_running--;
	sub_nr_running(rq_of_dl_rq(dl_rq), 1);

	dec_dl_deadline(dl_rq, dl_se->deadline);
}

static inline bool __dl_less(struct rb_node *a, const struct rb_node *b)
{
	return dl_time_before(__node_2_dle(a)->deadline, __node_2_dle(b)->deadline);
}

static __always_inline struct sched_statistics *
__schedstats_from_dl_se(struct sched_dl_entity *dl_se)
{
	if (!schedstat_enabled())
		return NULL;

	if (dl_server(dl_se))
		return NULL;

	return &dl_task_of(dl_se)->stats;
}

static inline void
update_stats_wait_start_dl(struct dl_rq *dl_rq, struct sched_dl_entity *dl_se)
{
	struct sched_statistics *stats = __schedstats_from_dl_se(dl_se);
	if (stats)
		__update_stats_wait_start(rq_of_dl_rq(dl_rq), dl_task_of(dl_se), stats);
}

static inline void
update_stats_wait_end_dl(struct dl_rq *dl_rq, struct sched_dl_entity *dl_se)
{
	struct sched_statistics *stats = __schedstats_from_dl_se(dl_se);
	if (stats)
		__update_stats_wait_end(rq_of_dl_rq(dl_rq), dl_task_of(dl_se), stats);
}

static inline void
update_stats_enqueue_sleeper_dl(struct dl_rq *dl_rq, struct sched_dl_entity *dl_se)
{
	struct sched_statistics *stats = __schedstats_from_dl_se(dl_se);
	if (stats)
		__update_stats_enqueue_sleeper(rq_of_dl_rq(dl_rq), dl_task_of(dl_se), stats);
}

static inline void
update_stats_enqueue_dl(struct dl_rq *dl_rq, struct sched_dl_entity *dl_se,
			int flags)
{
	if (!schedstat_enabled())
		return;

	if (flags & ENQUEUE_WAKEUP)
		update_stats_enqueue_sleeper_dl(dl_rq, dl_se);
}

static inline void
update_stats_dequeue_dl(struct dl_rq *dl_rq, struct sched_dl_entity *dl_se,
			int flags)
{
	struct task_struct *p = dl_task_of(dl_se);

	if (!schedstat_enabled())
		return;

	if ((flags & DEQUEUE_SLEEP)) {
		unsigned int state;

		state = READ_ONCE(p->__state);
		if (state & TASK_INTERRUPTIBLE)
			__schedstat_set(p->stats.sleep_start,
					rq_clock(rq_of_dl_rq(dl_rq)));

		if (state & TASK_UNINTERRUPTIBLE)
			__schedstat_set(p->stats.block_start,
					rq_clock(rq_of_dl_rq(dl_rq)));
	}
}

static void __enqueue_dl_entity(struct sched_dl_entity *dl_se)
{
	struct dl_rq *dl_rq = dl_rq_of_se(dl_se);

	WARN_ON_ONCE(!RB_EMPTY_NODE(&dl_se->rb_node));

	rb_add_cached(&dl_se->rb_node, &dl_rq->root, __dl_less);

	inc_dl_tasks(dl_se, dl_rq);
}

static void __dequeue_dl_entity(struct sched_dl_entity *dl_se)
{
	struct dl_rq *dl_rq = dl_rq_of_se(dl_se);

	if (RB_EMPTY_NODE(&dl_se->rb_node))
		return;

	rb_erase_cached(&dl_se->rb_node, &dl_rq->root);

	RB_CLEAR_NODE(&dl_se->rb_node);

	dec_dl_tasks(dl_se, dl_rq);
}

static void
enqueue_dl_entity(struct sched_dl_entity *dl_se, int flags)
{
	WARN_ON_ONCE(on_dl_rq(dl_se));

	update_stats_enqueue_dl(dl_rq_of_se(dl_se), dl_se, flags);

	/*
	 * Check if a constrained deadline task was activated
	 * after the deadline but before the next period.
	 * If that is the case, the task will be throttled and
	 * the replenishment timer will be set to the next period.
	 */
	if (!dl_se->dl_throttled && !dl_is_implicit(dl_se))
		dl_check_constrained_dl(dl_se);

	if (flags & (ENQUEUE_RESTORE|ENQUEUE_MIGRATING)) {
		struct dl_rq *dl_rq = dl_rq_of_se(dl_se);

		add_rq_bw(dl_se, dl_rq);
		add_running_bw(dl_se, dl_rq);
	}

	/*
	 * If p is throttled, we do not enqueue it. In fact, if it exhausted
	 * its budget it needs a replenishment and, since it now is on
	 * its rq, the bandwidth timer callback (which clearly has not
	 * run yet) will take care of this.
	 * However, the active utilization does not depend on the fact
	 * that the task is on the runqueue or not (but depends on the
	 * task's state - in GRUB parlance, "inactive" vs "active contending").
	 * In other words, even if a task is throttled its utilization must
	 * be counted in the active utilization; hence, we need to call
	 * add_running_bw().
	 */
	if (!dl_se->dl_defer && dl_se->dl_throttled && !(flags & ENQUEUE_REPLENISH)) {
		if (flags & ENQUEUE_WAKEUP)
			task_contending(dl_se, flags);

		return;
	}

	/*
	 * If this is a wakeup or a new instance, the scheduling
	 * parameters of the task might need updating. Otherwise,
	 * we want a replenishment of its runtime.
	 */
	if (flags & ENQUEUE_WAKEUP) {
		task_contending(dl_se, flags);
		update_dl_entity(dl_se);
	} else if (flags & ENQUEUE_REPLENISH) {
		replenish_dl_entity(dl_se);
	} else if ((flags & ENQUEUE_RESTORE) &&
		   !is_dl_boosted(dl_se) &&
		   dl_time_before(dl_se->deadline, rq_clock(rq_of_dl_se(dl_se)))) {
		setup_new_dl_entity(dl_se);
	}

	/*
	 * If the reservation is still throttled, e.g., it got replenished but is a
	 * deferred task and still got to wait, don't enqueue.
	 */
	if (dl_se->dl_throttled && start_dl_timer(dl_se))
		return;

	/*
	 * We're about to enqueue, make sure we're not ->dl_throttled!
	 * In case the timer was not started, say because the defer time
	 * has passed, mark as not throttled and mark unarmed.
	 * Also cancel earlier timers, since letting those run is pointless.
	 */
	if (dl_se->dl_throttled) {
		hrtimer_try_to_cancel(&dl_se->dl_timer);
		dl_se->dl_defer_armed = 0;
		dl_se->dl_throttled = 0;
	}

	__enqueue_dl_entity(dl_se);
}

static void dequeue_dl_entity(struct sched_dl_entity *dl_se, int flags)
{
	__dequeue_dl_entity(dl_se);

	if (flags & (DEQUEUE_SAVE|DEQUEUE_MIGRATING)) {
		struct dl_rq *dl_rq = dl_rq_of_se(dl_se);

		sub_running_bw(dl_se, dl_rq);
		sub_rq_bw(dl_se, dl_rq);
	}

	/*
	 * This check allows to start the inactive timer (or to immediately
	 * decrease the active utilization, if needed) in two cases:
	 * when the task blocks and when it is terminating
	 * (p->state == TASK_DEAD). We can handle the two cases in the same
	 * way, because from GRUB's point of view the same thing is happening
	 * (the task moves from "active contending" to "active non contending"
	 * or "inactive")
	 */
	if (flags & DEQUEUE_SLEEP)
		task_non_contending(dl_se);
}

static void enqueue_task_dl(struct rq *rq, struct task_struct *p, int flags)
{
	if (is_dl_boosted(&p->dl)) {
		/*
		 * Because of delays in the detection of the overrun of a
		 * thread's runtime, it might be the case that a thread
		 * goes to sleep in a rt mutex with negative runtime. As
		 * a consequence, the thread will be throttled.
		 *
		 * While waiting for the mutex, this thread can also be
		 * boosted via PI, resulting in a thread that is throttled
		 * and boosted at the same time.
		 *
		 * In this case, the boost overrides the throttle.
		 */
		if (p->dl.dl_throttled) {
			/*
			 * The replenish timer needs to be canceled. No
			 * problem if it fires concurrently: boosted threads
			 * are ignored in dl_task_timer().
			 *
			 * If the timer callback was running (hrtimer_try_to_cancel == -1),
			 * it will eventually call put_task_struct().
			 */
			if (hrtimer_try_to_cancel(&p->dl.dl_timer) == 1 &&
			    !dl_server(&p->dl))
				put_task_struct(p);
			p->dl.dl_throttled = 0;
		}
	} else if (!dl_prio(p->normal_prio)) {
		/*
		 * Special case in which we have a !SCHED_DEADLINE task that is going
		 * to be deboosted, but exceeds its runtime while doing so. No point in
		 * replenishing it, as it's going to return back to its original
		 * scheduling class after this. If it has been throttled, we need to
		 * clear the flag, otherwise the task may wake up as throttled after
		 * being boosted again with no means to replenish the runtime and clear
		 * the throttle.
		 */
		p->dl.dl_throttled = 0;
		if (!(flags & ENQUEUE_REPLENISH))
			printk_deferred_once("sched: DL de-boosted task PID %d: REPLENISH flag missing\n",
					     task_pid_nr(p));

		return;
	}

	check_schedstat_required();
	update_stats_wait_start_dl(dl_rq_of_se(&p->dl), &p->dl);

	if (p->on_rq == TASK_ON_RQ_MIGRATING)
		flags |= ENQUEUE_MIGRATING;

	enqueue_dl_entity(&p->dl, flags);

	if (dl_server(&p->dl))
		return;

	if (!task_current(rq, p) && !p->dl.dl_throttled && p->nr_cpus_allowed > 1)
		enqueue_pushable_dl_task(rq, p);
}

static bool dequeue_task_dl(struct rq *rq, struct task_struct *p, int flags)
{
	update_curr_dl(rq);

	if (p->on_rq == TASK_ON_RQ_MIGRATING)
		flags |= DEQUEUE_MIGRATING;

	dequeue_dl_entity(&p->dl, flags);
	if (!p->dl.dl_throttled && !dl_server(&p->dl))
		dequeue_pushable_dl_task(rq, p);

	return true;
}

/*
 * Yield task semantic for -deadline tasks is:
 *
 *   get off from the CPU until our next instance, with
 *   a new runtime. This is of little use now, since we
 *   don't have a bandwidth reclaiming mechanism. Anyway,
 *   bandwidth reclaiming is planned for the future, and
 *   yield_task_dl will indicate that some spare budget
 *   is available for other task instances to use it.
 */
static void yield_task_dl(struct rq *rq)
{
	/*
	 * We make the task go to sleep until its current deadline by
	 * forcing its runtime to zero. This way, update_curr_dl() stops
	 * it and the bandwidth timer will wake it up and will give it
	 * new scheduling parameters (thanks to dl_yielded=1).
	 */
	rq->curr->dl.dl_yielded = 1;

	update_rq_clock(rq);
	update_curr_dl(rq);
	/*
	 * Tell update_rq_clock() that we've just updated,
	 * so we don't do microscopic update in schedule()
	 * and double the fastpath cost.
	 */
	rq_clock_skip_update(rq);
}

#ifdef CONFIG_SMP

static inline bool dl_task_is_earliest_deadline(struct task_struct *p,
						 struct rq *rq)
{
	return (!rq->dl.dl_nr_running ||
		dl_time_before(p->dl.deadline,
			       rq->dl.earliest_dl.curr));
}

static int find_later_rq(struct task_struct *task);

static int
select_task_rq_dl(struct task_struct *p, int cpu, int flags)
{
	struct task_struct *curr;
	bool select_rq;
	struct rq *rq;

	if (!(flags & WF_TTWU))
		goto out;

	rq = cpu_rq(cpu);

	rcu_read_lock();
	curr = READ_ONCE(rq->curr); /* unlocked access */

	/*
	 * If we are dealing with a -deadline task, we must
	 * decide where to wake it up.
	 * If it has a later deadline and the current task
	 * on this rq can't move (provided the waking task
	 * can!) we prefer to send it somewhere else. On the
	 * other hand, if it has a shorter deadline, we
	 * try to make it stay here, it might be important.
	 */
	select_rq = unlikely(dl_task(curr)) &&
		    (curr->nr_cpus_allowed < 2 ||
		     !dl_entity_preempt(&p->dl, &curr->dl)) &&
		    p->nr_cpus_allowed > 1;

	/*
	 * Take the capacity of the CPU into account to
	 * ensure it fits the requirement of the task.
	 */
	if (sched_asym_cpucap_active())
		select_rq |= !dl_task_fits_capacity(p, cpu);

	if (select_rq) {
		int target = find_later_rq(p);

		if (target != -1 &&
		    dl_task_is_earliest_deadline(p, cpu_rq(target)))
			cpu = target;
	}
	rcu_read_unlock();

out:
	return cpu;
}

static void migrate_task_rq_dl(struct task_struct *p, int new_cpu __maybe_unused)
{
	struct rq_flags rf;
	struct rq *rq;

	if (READ_ONCE(p->__state) != TASK_WAKING)
		return;

	rq = task_rq(p);
	/*
	 * Since p->state == TASK_WAKING, set_task_cpu() has been called
	 * from try_to_wake_up(). Hence, p->pi_lock is locked, but
	 * rq->lock is not... So, lock it
	 */
	rq_lock(rq, &rf);
	if (p->dl.dl_non_contending) {
		update_rq_clock(rq);
		sub_running_bw(&p->dl, &rq->dl);
		p->dl.dl_non_contending = 0;
		/*
		 * If the timer handler is currently running and the
		 * timer cannot be canceled, inactive_task_timer()
		 * will see that dl_not_contending is not set, and
		 * will not touch the rq's active utilization,
		 * so we are still safe.
		 */
		if (hrtimer_try_to_cancel(&p->dl.inactive_timer) == 1)
			put_task_struct(p);
	}
	sub_rq_bw(&p->dl, &rq->dl);
	rq_unlock(rq, &rf);
}

static void check_preempt_equal_dl(struct rq *rq, struct task_struct *p)
{
	/*
	 * Current can't be migrated, useless to reschedule,
	 * let's hope p can move out.
	 */
	if (rq->curr->nr_cpus_allowed == 1 ||
	    !cpudl_find(&rq->rd->cpudl, rq->curr, NULL))
		return;

	/*
	 * p is migratable, so let's not schedule it and
	 * see if it is pushed or pulled somewhere else.
	 */
	if (p->nr_cpus_allowed != 1 &&
	    cpudl_find(&rq->rd->cpudl, p, NULL))
		return;

	resched_curr(rq);
}

static int balance_dl(struct rq *rq, struct task_struct *p, struct rq_flags *rf)
{
	if (!on_dl_rq(&p->dl) && need_pull_dl_task(rq, p)) {
		/*
		 * This is OK, because current is on_cpu, which avoids it being
		 * picked for load-balance and preemption/IRQs are still
		 * disabled avoiding further scheduler activity on it and we've
		 * not yet started the picking loop.
		 */
		rq_unpin_lock(rq, rf);
		pull_dl_task(rq);
		rq_repin_lock(rq, rf);
	}

	return sched_stop_runnable(rq) || sched_dl_runnable(rq);
}
#endif /* CONFIG_SMP */

/*
 * Only called when both the current and waking task are -deadline
 * tasks.
 */
static void wakeup_preempt_dl(struct rq *rq, struct task_struct *p,
				  int flags)
{
	if (dl_entity_preempt(&p->dl, &rq->curr->dl)) {
		resched_curr(rq);
		return;
	}

#ifdef CONFIG_SMP
	/*
	 * In the unlikely case current and p have the same deadline
	 * let us try to decide what's the best thing to do...
	 */
	if ((p->dl.deadline == rq->curr->dl.deadline) &&
	    !test_tsk_need_resched(rq->curr))
		check_preempt_equal_dl(rq, p);
#endif /* CONFIG_SMP */
}

#ifdef CONFIG_SCHED_HRTICK
static void start_hrtick_dl(struct rq *rq, struct sched_dl_entity *dl_se)
{
	hrtick_start(rq, dl_se->runtime);
}
#else /* !CONFIG_SCHED_HRTICK */
static void start_hrtick_dl(struct rq *rq, struct sched_dl_entity *dl_se)
{
}
#endif

static void set_next_task_dl(struct rq *rq, struct task_struct *p, bool first)
{
	struct sched_dl_entity *dl_se = &p->dl;
	struct dl_rq *dl_rq = &rq->dl;

	p->se.exec_start = rq_clock_task(rq);
	if (on_dl_rq(&p->dl))
		update_stats_wait_end_dl(dl_rq, dl_se);

	/* You can't push away the running task */
	dequeue_pushable_dl_task(rq, p);

	if (!first)
		return;

	if (rq->curr->sched_class != &dl_sched_class)
		update_dl_rq_load_avg(rq_clock_pelt(rq), rq, 0);

	deadline_queue_push_tasks(rq);

	if (hrtick_enabled_dl(rq))
		start_hrtick_dl(rq, &p->dl);
}

static struct sched_dl_entity *pick_next_dl_entity(struct dl_rq *dl_rq)
{
	struct rb_node *left = rb_first_cached(&dl_rq->root);

	if (!left)
		return NULL;

	return __node_2_dle(left);
}

/*
 * __pick_next_task_dl - Helper to pick the next -deadline task to run.
 * @rq: The runqueue to pick the next task from.
 */
static struct task_struct *__pick_task_dl(struct rq *rq)
{
	struct sched_dl_entity *dl_se;
	struct dl_rq *dl_rq = &rq->dl;
	struct task_struct *p;

again:
	if (!sched_dl_runnable(rq))
		return NULL;

	dl_se = pick_next_dl_entity(dl_rq);
	WARN_ON_ONCE(!dl_se);

	if (dl_server(dl_se)) {
		p = dl_se->server_pick_task(dl_se);
		if (!p) {
			if (dl_server_active(dl_se)) {
				dl_se->dl_yielded = 1;
				update_curr_dl_se(rq, dl_se, 0);
			}
			goto again;
		}
		rq->dl_server = dl_se;
	} else {
		p = dl_task_of(dl_se);
	}

	return p;
}

static struct task_struct *pick_task_dl(struct rq *rq)
{
	return __pick_task_dl(rq);
}

static void put_prev_task_dl(struct rq *rq, struct task_struct *p, struct task_struct *next)
{
	struct sched_dl_entity *dl_se = &p->dl;
	struct dl_rq *dl_rq = &rq->dl;

	if (on_dl_rq(&p->dl))
		update_stats_wait_start_dl(dl_rq, dl_se);

	update_curr_dl(rq);

	update_dl_rq_load_avg(rq_clock_pelt(rq), rq, 1);
	if (on_dl_rq(&p->dl) && p->nr_cpus_allowed > 1)
		enqueue_pushable_dl_task(rq, p);
}

/*
 * scheduler tick hitting a task of our scheduling class.
 *
 * NOTE: This function can be called remotely by the tick offload that
 * goes along full dynticks. Therefore no local assumption can be made
 * and everything must be accessed through the @rq and @curr passed in
 * parameters.
 */
static void task_tick_dl(struct rq *rq, struct task_struct *p, int queued)
{
	update_curr_dl(rq);

	update_dl_rq_load_avg(rq_clock_pelt(rq), rq, 1);
	/*
	 * Even when we have runtime, update_curr_dl() might have resulted in us
	 * not being the leftmost task anymore. In that case NEED_RESCHED will
	 * be set and schedule() will start a new hrtick for the next task.
	 */
	if (hrtick_enabled_dl(rq) && queued && p->dl.runtime > 0 &&
	    is_leftmost(&p->dl, &rq->dl))
		start_hrtick_dl(rq, &p->dl);
}

static void task_fork_dl(struct task_struct *p)
{
	/*
	 * SCHED_DEADLINE tasks cannot fork and this is achieved through
	 * sched_fork()
	 */
}

#ifdef CONFIG_SMP

/* Only try algorithms three times */
#define DL_MAX_TRIES 3

static int pick_dl_task(struct rq *rq, struct task_struct *p, int cpu)
{
	if (!task_on_cpu(rq, p) &&
	    cpumask_test_cpu(cpu, &p->cpus_mask))
		return 1;
	return 0;
}

/*
 * Return the earliest pushable rq's task, which is suitable to be executed
 * on the CPU, NULL otherwise:
 */
static struct task_struct *pick_earliest_pushable_dl_task(struct rq *rq, int cpu)
{
	struct task_struct *p = NULL;
	struct rb_node *next_node;

	if (!has_pushable_dl_tasks(rq))
		return NULL;

	next_node = rb_first_cached(&rq->dl.pushable_dl_tasks_root);

next_node:
	if (next_node) {
		p = __node_2_pdl(next_node);

		if (pick_dl_task(rq, p, cpu))
			return p;

		next_node = rb_next(next_node);
		goto next_node;
	}

	return NULL;
}

static DEFINE_PER_CPU(cpumask_var_t, local_cpu_mask_dl);

static int find_later_rq(struct task_struct *task)
{
	struct sched_domain *sd;
	struct cpumask *later_mask = this_cpu_cpumask_var_ptr(local_cpu_mask_dl);
	int this_cpu = smp_processor_id();
	int cpu = task_cpu(task);

	/* Make sure the mask is initialized first */
	if (unlikely(!later_mask))
		return -1;

	if (task->nr_cpus_allowed == 1)
		return -1;

	/*
	 * We have to consider system topology and task affinity
	 * first, then we can look for a suitable CPU.
	 */
	if (!cpudl_find(&task_rq(task)->rd->cpudl, task, later_mask))
		return -1;

	/*
	 * If we are here, some targets have been found, including
	 * the most suitable which is, among the runqueues where the
	 * current tasks have later deadlines than the task's one, the
	 * rq with the latest possible one.
	 *
	 * Now we check how well this matches with task's
	 * affinity and system topology.
	 *
	 * The last CPU where the task run is our first
	 * guess, since it is most likely cache-hot there.
	 */
	if (cpumask_test_cpu(cpu, later_mask))
		return cpu;
	/*
	 * Check if this_cpu is to be skipped (i.e., it is
	 * not in the mask) or not.
	 */
	if (!cpumask_test_cpu(this_cpu, later_mask))
		this_cpu = -1;

	rcu_read_lock();
	for_each_domain(cpu, sd) {
		if (sd->flags & SD_WAKE_AFFINE) {
			int best_cpu;

			/*
			 * If possible, preempting this_cpu is
			 * cheaper than migrating.
			 */
			if (this_cpu != -1 &&
			    cpumask_test_cpu(this_cpu, sched_domain_span(sd))) {
				rcu_read_unlock();
				return this_cpu;
			}

			best_cpu = cpumask_any_and_distribute(later_mask,
							      sched_domain_span(sd));
			/*
			 * Last chance: if a CPU being in both later_mask
			 * and current sd span is valid, that becomes our
			 * choice. Of course, the latest possible CPU is
			 * already under consideration through later_mask.
			 */
			if (best_cpu < nr_cpu_ids) {
				rcu_read_unlock();
				return best_cpu;
			}
		}
	}
	rcu_read_unlock();

	/*
	 * At this point, all our guesses failed, we just return
	 * 'something', and let the caller sort the things out.
	 */
	if (this_cpu != -1)
		return this_cpu;

	cpu = cpumask_any_distribute(later_mask);
	if (cpu < nr_cpu_ids)
		return cpu;

	return -1;
}

/* Locks the rq it finds */
static struct rq *find_lock_later_rq(struct task_struct *task, struct rq *rq)
{
	struct rq *later_rq = NULL;
	int tries;
	int cpu;

	for (tries = 0; tries < DL_MAX_TRIES; tries++) {
		cpu = find_later_rq(task);

		if ((cpu == -1) || (cpu == rq->cpu))
			break;

		later_rq = cpu_rq(cpu);

		if (!dl_task_is_earliest_deadline(task, later_rq)) {
			/*
			 * Target rq has tasks of equal or earlier deadline,
			 * retrying does not release any lock and is unlikely
			 * to yield a different result.
			 */
			later_rq = NULL;
			break;
		}

		/* Retry if something changed. */
		if (double_lock_balance(rq, later_rq)) {
			if (unlikely(task_rq(task) != rq ||
				     !cpumask_test_cpu(later_rq->cpu, &task->cpus_mask) ||
				     task_on_cpu(rq, task) ||
				     !dl_task(task) ||
				     is_migration_disabled(task) ||
				     !task_on_rq_queued(task))) {
				double_unlock_balance(rq, later_rq);
				later_rq = NULL;
				break;
			}
		}

		/*
		 * If the rq we found has no -deadline task, or
		 * its earliest one has a later deadline than our
		 * task, the rq is a good one.
		 */
		if (dl_task_is_earliest_deadline(task, later_rq))
			break;

		/* Otherwise we try again. */
		double_unlock_balance(rq, later_rq);
		later_rq = NULL;
	}

	return later_rq;
}

static struct task_struct *pick_next_pushable_dl_task(struct rq *rq)
{
	struct task_struct *p;

	if (!has_pushable_dl_tasks(rq))
		return NULL;

	p = __node_2_pdl(rb_first_cached(&rq->dl.pushable_dl_tasks_root));

	WARN_ON_ONCE(rq->cpu != task_cpu(p));
	WARN_ON_ONCE(task_current(rq, p));
	WARN_ON_ONCE(p->nr_cpus_allowed <= 1);

	WARN_ON_ONCE(!task_on_rq_queued(p));
	WARN_ON_ONCE(!dl_task(p));

	return p;
}

/*
 * See if the non running -deadline tasks on this rq
 * can be sent to some other CPU where they can preempt
 * and start executing.
 */
static int push_dl_task(struct rq *rq)
{
	struct task_struct *next_task;
	struct rq *later_rq;
	int ret = 0;

	next_task = pick_next_pushable_dl_task(rq);
	if (!next_task)
		return 0;

retry:
	/*
	 * If next_task preempts rq->curr, and rq->curr
	 * can move away, it makes sense to just reschedule
	 * without going further in pushing next_task.
	 */
	if (dl_task(rq->curr) &&
	    dl_time_before(next_task->dl.deadline, rq->curr->dl.deadline) &&
	    rq->curr->nr_cpus_allowed > 1) {
		resched_curr(rq);
		return 0;
	}

	if (is_migration_disabled(next_task))
		return 0;

	if (WARN_ON(next_task == rq->curr))
		return 0;

	/* We might release rq lock */
	get_task_struct(next_task);

	/* Will lock the rq it'll find */
	later_rq = find_lock_later_rq(next_task, rq);
	if (!later_rq) {
		struct task_struct *task;

		/*
		 * We must check all this again, since
		 * find_lock_later_rq releases rq->lock and it is
		 * then possible that next_task has migrated.
		 */
		task = pick_next_pushable_dl_task(rq);
		if (task == next_task) {
			/*
			 * The task is still there. We don't try
			 * again, some other CPU will pull it when ready.
			 */
			goto out;
		}

		if (!task)
			/* No more tasks */
			goto out;

		put_task_struct(next_task);
		next_task = task;
		goto retry;
	}

	deactivate_task(rq, next_task, 0);
	set_task_cpu(next_task, later_rq->cpu);
	activate_task(later_rq, next_task, 0);
	ret = 1;

	resched_curr(later_rq);

	double_unlock_balance(rq, later_rq);

out:
	put_task_struct(next_task);

	return ret;
}

static void push_dl_tasks(struct rq *rq)
{
	/* push_dl_task() will return true if it moved a -deadline task */
	while (push_dl_task(rq))
		;
}

static void pull_dl_task(struct rq *this_rq)
{
	int this_cpu = this_rq->cpu, cpu;
	struct task_struct *p, *push_task;
	bool resched = false;
	struct rq *src_rq;
	u64 dmin = LONG_MAX;

	if (likely(!dl_overloaded(this_rq)))
		return;

	/*
	 * Match the barrier from dl_set_overloaded; this guarantees that if we
	 * see overloaded we must also see the dlo_mask bit.
	 */
	smp_rmb();

	for_each_cpu(cpu, this_rq->rd->dlo_mask) {
		if (this_cpu == cpu)
			continue;

		src_rq = cpu_rq(cpu);

		/*
		 * It looks racy, and it is! However, as in sched_rt.c,
		 * we are fine with this.
		 */
		if (this_rq->dl.dl_nr_running &&
		    dl_time_before(this_rq->dl.earliest_dl.curr,
				   src_rq->dl.earliest_dl.next))
			continue;

		/* Might drop this_rq->lock */
		push_task = NULL;
		double_lock_balance(this_rq, src_rq);

		/*
		 * If there are no more pullable tasks on the
		 * rq, we're done with it.
		 */
		if (src_rq->dl.dl_nr_running <= 1)
			goto skip;

		p = pick_earliest_pushable_dl_task(src_rq, this_cpu);

		/*
		 * We found a task to be pulled if:
		 *  - it preempts our current (if there's one),
		 *  - it will preempt the last one we pulled (if any).
		 */
		if (p && dl_time_before(p->dl.deadline, dmin) &&
		    dl_task_is_earliest_deadline(p, this_rq)) {
			WARN_ON(p == src_rq->curr);
			WARN_ON(!task_on_rq_queued(p));

			/*
			 * Then we pull iff p has actually an earlier
			 * deadline than the current task of its runqueue.
			 */
			if (dl_time_before(p->dl.deadline,
					   src_rq->curr->dl.deadline))
				goto skip;

			if (is_migration_disabled(p)) {
				push_task = get_push_task(src_rq);
			} else {
				deactivate_task(src_rq, p, 0);
				set_task_cpu(p, this_cpu);
				activate_task(this_rq, p, 0);
				dmin = p->dl.deadline;
				resched = true;
			}

			/* Is there any other task even earlier? */
		}
skip:
		double_unlock_balance(this_rq, src_rq);

		if (push_task) {
			preempt_disable();
			raw_spin_rq_unlock(this_rq);
			stop_one_cpu_nowait(src_rq->cpu, push_cpu_stop,
					    push_task, &src_rq->push_work);
			preempt_enable();
			raw_spin_rq_lock(this_rq);
		}
	}

	if (resched)
		resched_curr(this_rq);
}

/*
 * Since the task is not running and a reschedule is not going to happen
 * anytime soon on its runqueue, we try pushing it away now.
 */
static void task_woken_dl(struct rq *rq, struct task_struct *p)
{
	if (!task_on_cpu(rq, p) &&
	    !test_tsk_need_resched(rq->curr) &&
	    p->nr_cpus_allowed > 1 &&
	    dl_task(rq->curr) &&
	    (rq->curr->nr_cpus_allowed < 2 ||
	     !dl_entity_preempt(&p->dl, &rq->curr->dl))) {
		push_dl_tasks(rq);
	}
}

static void set_cpus_allowed_dl(struct task_struct *p,
				struct affinity_context *ctx)
{
	struct root_domain *src_rd;
	struct rq *rq;

	WARN_ON_ONCE(!dl_task(p));

	rq = task_rq(p);
	src_rd = rq->rd;
	/*
	 * Migrating a SCHED_DEADLINE task between exclusive
	 * cpusets (different root_domains) entails a bandwidth
	 * update. We already made space for us in the destination
	 * domain (see cpuset_can_attach()).
	 */
	if (!cpumask_intersects(src_rd->span, ctx->new_mask)) {
		struct dl_bw *src_dl_b;

		src_dl_b = dl_bw_of(cpu_of(rq));
		/*
		 * We now free resources of the root_domain we are migrating
		 * off. In the worst case, sched_setattr() may temporary fail
		 * until we complete the update.
		 */
		raw_spin_lock(&src_dl_b->lock);
		__dl_sub(src_dl_b, p->dl.dl_bw, dl_bw_cpus(task_cpu(p)));
		raw_spin_unlock(&src_dl_b->lock);
	}

	set_cpus_allowed_common(p, ctx);
}

/* Assumes rq->lock is held */
static void rq_online_dl(struct rq *rq)
{
	if (rq->dl.overloaded)
		dl_set_overload(rq);

	cpudl_set_freecpu(&rq->rd->cpudl, rq->cpu);
	if (rq->dl.dl_nr_running > 0)
		cpudl_set(&rq->rd->cpudl, rq->cpu, rq->dl.earliest_dl.curr);
}

/* Assumes rq->lock is held */
static void rq_offline_dl(struct rq *rq)
{
	if (rq->dl.overloaded)
		dl_clear_overload(rq);

	cpudl_clear(&rq->rd->cpudl, rq->cpu);
	cpudl_clear_freecpu(&rq->rd->cpudl, rq->cpu);
}

void __init init_sched_dl_class(void)
{
	unsigned int i;

	for_each_possible_cpu(i)
		zalloc_cpumask_var_node(&per_cpu(local_cpu_mask_dl, i),
					GFP_KERNEL, cpu_to_node(i));
}

void dl_add_task_root_domain(struct task_struct *p)
{
	struct rq_flags rf;
	struct rq *rq;
	struct dl_bw *dl_b;

	raw_spin_lock_irqsave(&p->pi_lock, rf.flags);
	if (!dl_task(p)) {
		raw_spin_unlock_irqrestore(&p->pi_lock, rf.flags);
		return;
	}

	rq = __task_rq_lock(p, &rf);

	dl_b = &rq->rd->dl_bw;
	raw_spin_lock(&dl_b->lock);

	__dl_add(dl_b, p->dl.dl_bw, cpumask_weight(rq->rd->span));

	raw_spin_unlock(&dl_b->lock);

	task_rq_unlock(rq, p, &rf);
}

void dl_clear_root_domain(struct root_domain *rd)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&rd->dl_bw.lock, flags);
	rd->dl_bw.total_bw = 0;
	raw_spin_unlock_irqrestore(&rd->dl_bw.lock, flags);
}

#endif /* CONFIG_SMP */

static void switched_from_dl(struct rq *rq, struct task_struct *p)
{
	/*
	 * task_non_contending() can start the "inactive timer" (if the 0-lag
	 * time is in the future). If the task switches back to dl before
	 * the "inactive timer" fires, it can continue to consume its current
	 * runtime using its current deadline. If it stays outside of
	 * SCHED_DEADLINE until the 0-lag time passes, inactive_task_timer()
	 * will reset the task parameters.
	 */
	if (task_on_rq_queued(p) && p->dl.dl_runtime)
		task_non_contending(&p->dl);

	/*
	 * In case a task is setscheduled out from SCHED_DEADLINE we need to
	 * keep track of that on its cpuset (for correct bandwidth tracking).
	 */
	dec_dl_tasks_cs(p);

	if (!task_on_rq_queued(p)) {
		/*
		 * Inactive timer is armed. However, p is leaving DEADLINE and
		 * might migrate away from this rq while continuing to run on
		 * some other class. We need to remove its contribution from
		 * this rq running_bw now, or sub_rq_bw (below) will complain.
		 */
		if (p->dl.dl_non_contending)
			sub_running_bw(&p->dl, &rq->dl);
		sub_rq_bw(&p->dl, &rq->dl);
	}

	/*
	 * We cannot use inactive_task_timer() to invoke sub_running_bw()
	 * at the 0-lag time, because the task could have been migrated
	 * while SCHED_OTHER in the meanwhile.
	 */
	if (p->dl.dl_non_contending)
		p->dl.dl_non_contending = 0;

	/*
	 * Since this might be the only -deadline task on the rq,
	 * this is the right place to try to pull some other one
	 * from an overloaded CPU, if any.
	 */
	if (!task_on_rq_queued(p) || rq->dl.dl_nr_running)
		return;

	deadline_queue_pull_task(rq);
}

/*
 * When switching to -deadline, we may overload the rq, then
 * we try to push someone off, if possible.
 */
static void switched_to_dl(struct rq *rq, struct task_struct *p)
{
	if (hrtimer_try_to_cancel(&p->dl.inactive_timer) == 1)
		put_task_struct(p);

	/*
	 * In case a task is setscheduled to SCHED_DEADLINE we need to keep
	 * track of that on its cpuset (for correct bandwidth tracking).
	 */
	inc_dl_tasks_cs(p);

	/* If p is not queued we will update its parameters at next wakeup. */
	if (!task_on_rq_queued(p)) {
		add_rq_bw(&p->dl, &rq->dl);

		return;
	}

	if (rq->curr != p) {
#ifdef CONFIG_SMP
		if (p->nr_cpus_allowed > 1 && rq->dl.overloaded)
			deadline_queue_push_tasks(rq);
#endif
		if (dl_task(rq->curr))
			wakeup_preempt_dl(rq, p, 0);
		else
			resched_curr(rq);
	} else {
		update_dl_rq_load_avg(rq_clock_pelt(rq), rq, 0);
	}
}

/*
 * If the scheduling parameters of a -deadline task changed,
 * a push or pull operation might be needed.
 */
static void prio_changed_dl(struct rq *rq, struct task_struct *p,
			    int oldprio)
{
	if (!task_on_rq_queued(p))
		return;

#ifdef CONFIG_SMP
	/*
	 * This might be too much, but unfortunately
	 * we don't have the old deadline value, and
	 * we can't argue if the task is increasing
	 * or lowering its prio, so...
	 */
	if (!rq->dl.overloaded)
		deadline_queue_pull_task(rq);

	if (task_current(rq, p)) {
		/*
		 * If we now have a earlier deadline task than p,
		 * then reschedule, provided p is still on this
		 * runqueue.
		 */
		if (dl_time_before(rq->dl.earliest_dl.curr, p->dl.deadline))
			resched_curr(rq);
	} else {
		/*
		 * Current may not be deadline in case p was throttled but we
		 * have just replenished it (e.g. rt_mutex_setprio()).
		 *
		 * Otherwise, if p was given an earlier deadline, reschedule.
		 */
		if (!dl_task(rq->curr) ||
		    dl_time_before(p->dl.deadline, rq->curr->dl.deadline))
			resched_curr(rq);
	}
#else
	/*
	 * We don't know if p has a earlier or later deadline, so let's blindly
	 * set a (maybe not needed) rescheduling point.
	 */
	resched_curr(rq);
#endif
}

#ifdef CONFIG_SCHED_CORE
static int task_is_throttled_dl(struct task_struct *p, int cpu)
{
	return p->dl.dl_throttled;
}
#endif

DEFINE_SCHED_CLASS(dl) = {

	.enqueue_task		= enqueue_task_dl,
	.dequeue_task		= dequeue_task_dl,
	.yield_task		= yield_task_dl,

	.wakeup_preempt		= wakeup_preempt_dl,

	.pick_task		= pick_task_dl,
	.put_prev_task		= put_prev_task_dl,
	.set_next_task		= set_next_task_dl,

#ifdef CONFIG_SMP
	.balance		= balance_dl,
	.select_task_rq		= select_task_rq_dl,
	.migrate_task_rq	= migrate_task_rq_dl,
	.set_cpus_allowed       = set_cpus_allowed_dl,
	.rq_online              = rq_online_dl,
	.rq_offline             = rq_offline_dl,
	.task_woken		= task_woken_dl,
	.find_lock_rq		= find_lock_later_rq,
#endif

	.task_tick		= task_tick_dl,
	.task_fork              = task_fork_dl,

	.prio_changed           = prio_changed_dl,
	.switched_from		= switched_from_dl,
	.switched_to		= switched_to_dl,

	.update_curr		= update_curr_dl,
#ifdef CONFIG_SCHED_CORE
	.task_is_throttled	= task_is_throttled_dl,
#endif
};

/* Used for dl_bw check and update, used under sched_rt_handler()::mutex */
static u64 dl_generation;

int sched_dl_global_validate(void)
{
	u64 runtime = global_rt_runtime();
	u64 period = global_rt_period();
	u64 new_bw = to_ratio(period, runtime);
	u64 gen = ++dl_generation;
	struct dl_bw *dl_b;
	int cpu, cpus, ret = 0;
	unsigned long flags;

	/*
	 * Here we want to check the bandwidth not being set to some
	 * value smaller than the currently allocated bandwidth in
	 * any of the root_domains.
	 */
	for_each_online_cpu(cpu) {
		rcu_read_lock_sched();

		if (dl_bw_visited(cpu, gen))
			goto next;

		dl_b = dl_bw_of(cpu);
		cpus = dl_bw_cpus(cpu);

		raw_spin_lock_irqsave(&dl_b->lock, flags);
		if (new_bw * cpus < dl_b->total_bw)
			ret = -EBUSY;
		raw_spin_unlock_irqrestore(&dl_b->lock, flags);

next:
		rcu_read_unlock_sched();

		if (ret)
			break;
	}

	return ret;
}

static void init_dl_rq_bw_ratio(struct dl_rq *dl_rq)
{
	if (global_rt_runtime() == RUNTIME_INF) {
		dl_rq->bw_ratio = 1 << RATIO_SHIFT;
		dl_rq->max_bw = dl_rq->extra_bw = 1 << BW_SHIFT;
	} else {
		dl_rq->bw_ratio = to_ratio(global_rt_runtime(),
			  global_rt_period()) >> (BW_SHIFT - RATIO_SHIFT);
		dl_rq->max_bw = dl_rq->extra_bw =
			to_ratio(global_rt_period(), global_rt_runtime());
	}
}

void sched_dl_do_global(void)
{
	u64 new_bw = -1;
	u64 gen = ++dl_generation;
	struct dl_bw *dl_b;
	int cpu;
	unsigned long flags;

	if (global_rt_runtime() != RUNTIME_INF)
		new_bw = to_ratio(global_rt_period(), global_rt_runtime());

	for_each_possible_cpu(cpu) {
		rcu_read_lock_sched();

		if (dl_bw_visited(cpu, gen)) {
			rcu_read_unlock_sched();
			continue;
		}

		dl_b = dl_bw_of(cpu);

		raw_spin_lock_irqsave(&dl_b->lock, flags);
		dl_b->bw = new_bw;
		raw_spin_unlock_irqrestore(&dl_b->lock, flags);

		rcu_read_unlock_sched();
		init_dl_rq_bw_ratio(&cpu_rq(cpu)->dl);
	}
}

/*
 * We must be sure that accepting a new task (or allowing changing the
 * parameters of an existing one) is consistent with the bandwidth
 * constraints. If yes, this function also accordingly updates the currently
 * allocated bandwidth to reflect the new situation.
 *
 * This function is called while holding p's rq->lock.
 */
int sched_dl_overflow(struct task_struct *p, int policy,
		      const struct sched_attr *attr)
{
	u64 period = attr->sched_period ?: attr->sched_deadline;
	u64 runtime = attr->sched_runtime;
	u64 new_bw = dl_policy(policy) ? to_ratio(period, runtime) : 0;
	int cpus, err = -1, cpu = task_cpu(p);
	struct dl_bw *dl_b = dl_bw_of(cpu);
	unsigned long cap;

	if (attr->sched_flags & SCHED_FLAG_SUGOV)
		return 0;

	/* !deadline task may carry old deadline bandwidth */
	if (new_bw == p->dl.dl_bw && task_has_dl_policy(p))
		return 0;

	/*
	 * Either if a task, enters, leave, or stays -deadline but changes
	 * its parameters, we may need to update accordingly the total
	 * allocated bandwidth of the container.
	 */
	raw_spin_lock(&dl_b->lock);
	cpus = dl_bw_cpus(cpu);
	cap = dl_bw_capacity(cpu);

	if (dl_policy(policy) && !task_has_dl_policy(p) &&
	    !__dl_overflow(dl_b, cap, 0, new_bw)) {
		if (hrtimer_active(&p->dl.inactive_timer))
			__dl_sub(dl_b, p->dl.dl_bw, cpus);
		__dl_add(dl_b, new_bw, cpus);
		err = 0;
	} else if (dl_policy(policy) && task_has_dl_policy(p) &&
		   !__dl_overflow(dl_b, cap, p->dl.dl_bw, new_bw)) {
		/*
		 * XXX this is slightly incorrect: when the task
		 * utilization decreases, we should delay the total
		 * utilization change until the task's 0-lag point.
		 * But this would require to set the task's "inactive
		 * timer" when the task is not inactive.
		 */
		__dl_sub(dl_b, p->dl.dl_bw, cpus);
		__dl_add(dl_b, new_bw, cpus);
		dl_change_utilization(p, new_bw);
		err = 0;
	} else if (!dl_policy(policy) && task_has_dl_policy(p)) {
		/*
		 * Do not decrease the total deadline utilization here,
		 * switched_from_dl() will take care to do it at the correct
		 * (0-lag) time.
		 */
		err = 0;
	}
	raw_spin_unlock(&dl_b->lock);

	return err;
}

/*
 * This function initializes the sched_dl_entity of a newly becoming
 * SCHED_DEADLINE task.
 *
 * Only the static values are considered here, the actual runtime and the
 * absolute deadline will be properly calculated when the task is enqueued
 * for the first time with its new policy.
 */
void __setparam_dl(struct task_struct *p, const struct sched_attr *attr)
{
	struct sched_dl_entity *dl_se = &p->dl;

	dl_se->dl_runtime = attr->sched_runtime;
	dl_se->dl_deadline = attr->sched_deadline;
	dl_se->dl_period = attr->sched_period ?: dl_se->dl_deadline;
	dl_se->flags = attr->sched_flags & SCHED_DL_FLAGS;
	dl_se->dl_bw = to_ratio(dl_se->dl_period, dl_se->dl_runtime);
	dl_se->dl_density = to_ratio(dl_se->dl_deadline, dl_se->dl_runtime);
}

void __getparam_dl(struct task_struct *p, struct sched_attr *attr)
{
	struct sched_dl_entity *dl_se = &p->dl;

	attr->sched_priority = p->rt_priority;
	attr->sched_runtime = dl_se->dl_runtime;
	attr->sched_deadline = dl_se->dl_deadline;
	attr->sched_period = dl_se->dl_period;
	attr->sched_flags &= ~SCHED_DL_FLAGS;
	attr->sched_flags |= dl_se->flags;
}

/*
 * This function validates the new parameters of a -deadline task.
 * We ask for the deadline not being zero, and greater or equal
 * than the runtime, as well as the period of being zero or
 * greater than deadline. Furthermore, we have to be sure that
 * user parameters are above the internal resolution of 1us (we
 * check sched_runtime only since it is always the smaller one) and
 * below 2^63 ns (we have to check both sched_deadline and
 * sched_period, as the latter can be zero).
 */
bool __checkparam_dl(const struct sched_attr *attr)
{
	u64 period, max, min;

	/* special dl tasks don't actually use any parameter */
	if (attr->sched_flags & SCHED_FLAG_SUGOV)
		return true;

	/* deadline != 0 */
	if (attr->sched_deadline == 0)
		return false;

	/*
	 * Since we truncate DL_SCALE bits, make sure we're at least
	 * that big.
	 */
	if (attr->sched_runtime < (1ULL << DL_SCALE))
		return false;

	/*
	 * Since we use the MSB for wrap-around and sign issues, make
	 * sure it's not set (mind that period can be equal to zero).
	 */
	if (attr->sched_deadline & (1ULL << 63) ||
	    attr->sched_period & (1ULL << 63))
		return false;

	period = attr->sched_period;
	if (!period)
		period = attr->sched_deadline;

	/* runtime <= deadline <= period (if period != 0) */
	if (period < attr->sched_deadline ||
	    attr->sched_deadline < attr->sched_runtime)
		return false;

	max = (u64)READ_ONCE(sysctl_sched_dl_period_max) * NSEC_PER_USEC;
	min = (u64)READ_ONCE(sysctl_sched_dl_period_min) * NSEC_PER_USEC;

	if (period < min || period > max)
		return false;

	return true;
}

/*
 * This function clears the sched_dl_entity static params.
 */
static void __dl_clear_params(struct sched_dl_entity *dl_se)
{
	dl_se->dl_runtime		= 0;
	dl_se->dl_deadline		= 0;
	dl_se->dl_period		= 0;
	dl_se->flags			= 0;
	dl_se->dl_bw			= 0;
	dl_se->dl_density		= 0;

	dl_se->dl_throttled		= 0;
	dl_se->dl_yielded		= 0;
	dl_se->dl_non_contending	= 0;
	dl_se->dl_overrun		= 0;
	dl_se->dl_server		= 0;

#ifdef CONFIG_RT_MUTEXES
	dl_se->pi_se			= dl_se;
#endif
}

void init_dl_entity(struct sched_dl_entity *dl_se)
{
	RB_CLEAR_NODE(&dl_se->rb_node);
	init_dl_task_timer(dl_se);
	init_dl_inactive_task_timer(dl_se);
	__dl_clear_params(dl_se);
}

bool dl_param_changed(struct task_struct *p, const struct sched_attr *attr)
{
	struct sched_dl_entity *dl_se = &p->dl;

	if (dl_se->dl_runtime != attr->sched_runtime ||
	    dl_se->dl_deadline != attr->sched_deadline ||
	    dl_se->dl_period != attr->sched_period ||
	    dl_se->flags != (attr->sched_flags & SCHED_DL_FLAGS))
		return true;

	return false;
}

#ifdef CONFIG_SMP
int dl_cpuset_cpumask_can_shrink(const struct cpumask *cur,
				 const struct cpumask *trial)
{
	unsigned long flags, cap;
	struct dl_bw *cur_dl_b;
	int ret = 1;

	rcu_read_lock_sched();
	cur_dl_b = dl_bw_of(cpumask_any(cur));
	cap = __dl_bw_capacity(trial);
	raw_spin_lock_irqsave(&cur_dl_b->lock, flags);
	if (__dl_overflow(cur_dl_b, cap, 0, 0))
		ret = 0;
	raw_spin_unlock_irqrestore(&cur_dl_b->lock, flags);
	rcu_read_unlock_sched();

	return ret;
}

enum dl_bw_request {
	dl_bw_req_check_overflow = 0,
	dl_bw_req_alloc,
	dl_bw_req_free
};

static int dl_bw_manage(enum dl_bw_request req, int cpu, u64 dl_bw)
{
	unsigned long flags;
	struct dl_bw *dl_b;
	bool overflow = 0;

	rcu_read_lock_sched();
	dl_b = dl_bw_of(cpu);
	raw_spin_lock_irqsave(&dl_b->lock, flags);

	if (req == dl_bw_req_free) {
		__dl_sub(dl_b, dl_bw, dl_bw_cpus(cpu));
	} else {
		unsigned long cap = dl_bw_capacity(cpu);

		overflow = __dl_overflow(dl_b, cap, 0, dl_bw);

		if (req == dl_bw_req_alloc && !overflow) {
			/*
			 * We reserve space in the destination
			 * root_domain, as we can't fail after this point.
			 * We will free resources in the source root_domain
			 * later on (see set_cpus_allowed_dl()).
			 */
			__dl_add(dl_b, dl_bw, dl_bw_cpus(cpu));
		}
	}

	raw_spin_unlock_irqrestore(&dl_b->lock, flags);
	rcu_read_unlock_sched();

	return overflow ? -EBUSY : 0;
}

int dl_bw_check_overflow(int cpu)
{
	return dl_bw_manage(dl_bw_req_check_overflow, cpu, 0);
}

int dl_bw_alloc(int cpu, u64 dl_bw)
{
	return dl_bw_manage(dl_bw_req_alloc, cpu, dl_bw);
}

void dl_bw_free(int cpu, u64 dl_bw)
{
	dl_bw_manage(dl_bw_req_free, cpu, dl_bw);
}
#endif

#ifdef CONFIG_SCHED_DEBUG
void print_dl_stats(struct seq_file *m, int cpu)
{
	print_dl_rq(m, cpu, &cpu_rq(cpu)->dl);
}
#endif /* CONFIG_SCHED_DEBUG */
