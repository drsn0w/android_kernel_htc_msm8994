/*
 * Intelli Hotplug Driver
 *
 * Copyright (c) 2015, Chad Cormier Roussel <chadcormierroussel@gmail.com>
 * Copyright (c) 2013-2014, Paul Reioux <reioux@gmail.com>
 * Copyright (c) 2010-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/workqueue.h>
#include <linux/cpu.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/kobject.h>
#ifdef CONFIG_STATE_NOTIFIER
#include <linux/state_notifier.h>
#endif
#include <linux/cpufreq.h>

#define HIMA_HOTPLUG			             "hima_hotplug"
#define HIMA_HOTPLUG_MAJOR_VERSION	   0
#define HIMA_HOTPLUG_MINOR_VERSION	   2

#define DEF_SAMPLING_MS                HZ * 2
#define RESUME_SAMPLING_MS             HZ / 5
#define START_DELAY_MS                 HZ * 50
#define MIN_INPUT_INTERVAL	           150 * 1000L
#define BOOST_LOCK_DUR                 2500 * 1000L
#define DEFAULT_NR_CPUS_BOOSTED        1
#define DEFAULT_MIN_CPUS_ONLINE        1
#define DEFAULT_MAX_CPUS_ONLINE        4
#define DEFAULT_NR_FSHIFT              3
#define CAPACITY_RESERVE               50

#if defined(CONFIG_ARCH_MSM8994)
#define THREAD_CAPACITY                (430 - CAPACITY_RESERVE)
#else
#define THREAD_CAPACITY			           (250 - CAPACITY_RESERVE)
#endif

#define CPU_NR_THRESHOLD		((THREAD_CAPACITY << 1) - (THREAD_CAPACITY >> 1))

#define MULT_FACTOR                   6
#define DIV_FACTOR                    100000

static u64 last_boost_time, last_input;

static struct delayed_work hima_hotplug_work;
static struct work_struct up_down_work;
static struct workqueue_struct *hima_hotplug_wq;
static struct mutex hima_hotplug_mutex;
static struct notifier_block notif;

struct ip_cpu_info {
	unsigned long cpu_nr_running;
};
static DEFINE_PER_CPU(struct ip_cpu_info, ip_info);

/* HotPlug Driver controls */
static atomic_t hima_hotplug_active = ATOMIC_INIT(1);
static unsigned int cpus_boosted = DEFAULT_NR_CPUS_BOOSTED;
static unsigned int min_cpus_online = DEFAULT_MIN_CPUS_ONLINE;
static unsigned int max_cpus_online = DEFAULT_MAX_CPUS_ONLINE;
static unsigned int current_profile_no = 0;
static unsigned int cpu_nr_run_threshold = CPU_NR_THRESHOLD;

/* HotPlug Driver Tuning */
static unsigned int target_cpus = 0;
static atomic_t always_on_cpu = ATOMIC_INIT(0);
static u64 boost_lock_duration = BOOST_LOCK_DUR;
static unsigned int def_sampling_ms = DEF_SAMPLING_MS;
static unsigned int nr_fshift = DEFAULT_NR_FSHIFT;
static unsigned int nr_run_hysteresis = DEFAULT_MAX_CPUS_ONLINE;
static unsigned int debug_hima_hotplug = 1;

#define dprintk(msg...)		\
do { 				\
	if (debug_hima_hotplug)		\
		pr_info(msg);	\
} while (0)

static unsigned int nr_run_thresholds_big_cluster[] = {
  (THREAD_CAPACITY * 500 * MULT_FACTOR * 2) / DIV_FACTOR,
  (THREAD_CAPACITY * 800 * MULT_FACTOR * 2) / DIV_FACTOR,
  (THREAD_CAPACITY * 1000 * MULT_FACTOR * 2) / DIV_FACTOR,
  (THREAD_CAPACITY * 1100 * MULT_FACTOR * 2) / DIV_FACTOR,
  UINT_MAX
};

static unsigned int nr_run_thresholds_little_cluster[] = {
  (THREAD_CAPACITY * 400 * MULT_FACTOR * 2) / DIV_FACTOR,
  (THREAD_CAPACITY * 700 * MULT_FACTOR * 2) / DIV_FACTOR,
  (THREAD_CAPACITY * 1000 * MULT_FACTOR * 2) / DIV_FACTOR,
  (THREAD_CAPACITY * 1300 * MULT_FACTOR * 2) / DIV_FACTOR,
  UINT_MAX
};

//static unsigned int little_cluster_sum = 207; 

static unsigned int nr_run_thresholds_conservative[] = {
  (THREAD_CAPACITY * 300 * MULT_FACTOR * 3) / DIV_FACTOR,
  (THREAD_CAPACITY * 550 * MULT_FACTOR * 3) / DIV_FACTOR,
  (THREAD_CAPACITY * 800 * MULT_FACTOR * 3) / DIV_FACTOR,
  UINT_MAX
};

static unsigned int nr_run_thresholds_disable[] = {
	0,  0,  0,  UINT_MAX
};


static unsigned int *nr_run_profiles[] = {
	nr_run_thresholds_big_cluster,
	nr_run_thresholds_little_cluster,
	nr_run_thresholds_conservative,
	nr_run_thresholds_disable
	};

static unsigned int nr_run_last;

static unsigned int calculate_thread_stats(void)
{
	unsigned int avg_nr_run = avg_nr_running();
	unsigned int nr_run;
	unsigned int threshold_size;
	unsigned int *current_profile;

	threshold_size = 4;
	nr_run_hysteresis = 8;
	nr_fshift = 4;

	for (nr_run = 1; nr_run < threshold_size; nr_run++) {
		unsigned int nr_threshold;
		
		current_profile = nr_run_profiles[current_profile_no];
		nr_threshold = current_profile[nr_run - 1];

		if (nr_run_last <= nr_run)
			nr_threshold += nr_run_hysteresis;
		if (avg_nr_run <= (nr_threshold << (FSHIFT - nr_fshift)))
			break;
	}
	nr_run_last = nr_run;

	return nr_run;
}

static void update_per_cpu_stat(void)
{
	unsigned int cpu;
	struct ip_cpu_info *l_ip_info;

	for_each_online_cpu(cpu) {
		l_ip_info = &per_cpu(ip_info, cpu);
		l_ip_info->cpu_nr_running = avg_cpu_nr_running(cpu);
	}
}

static void __ref cpu_up_down_work(struct work_struct *work)
{
	int online_cpus, cpu, l_nr_threshold;
	int target = target_cpus;
	struct ip_cpu_info *l_ip_info;

	if (target < min_cpus_online)
		target = min_cpus_online;
	else if (target > max_cpus_online)
		target = max_cpus_online;

	online_cpus = num_online_cpus();

	if (target < online_cpus) {
		if (online_cpus <= cpus_boosted &&
		    (ktime_to_us(ktime_get()) - last_input < boost_lock_duration))
			return;

		update_per_cpu_stat();
		for_each_online_cpu(cpu) {
			if (cpu == atomic_read(&always_on_cpu))
				continue;
			  
			l_nr_threshold = cpu_nr_run_threshold << 1 / (num_online_cpus());
			l_ip_info = &per_cpu(ip_info, cpu);
			if (l_ip_info->cpu_nr_running < l_nr_threshold)
				cpu_down(cpu);
			if (target >= num_online_cpus())
				break;
		}
	} else if (target > online_cpus) {
		for_each_cpu_not(cpu, cpu_online_mask) {
		  if(cpu < atomic_read(&always_on_cpu))
				continue;
			cpu_up(cpu);
			if (target <= num_online_cpus())
				break;
		}
	}
}

static void hima_hotplug_work_fn(struct work_struct *work)
{
	target_cpus = calculate_thread_stats();
	queue_work_on(atomic_read(&always_on_cpu), hima_hotplug_wq, &up_down_work);

	if (atomic_read(&hima_hotplug_active) == 1)
		queue_delayed_work_on(atomic_read(&always_on_cpu), hima_hotplug_wq, &hima_hotplug_work,
					msecs_to_jiffies(def_sampling_ms));
}

static void __ref hima_hotplug_suspend(void)
{
	int cpu = 0;
  
  mutex_lock(&hima_hotplug_mutex);
  
  /* Flush hotplug workqueue */
	flush_workqueue(hima_hotplug_wq);
	cancel_delayed_work_sync(&hima_hotplug_work);
  cancel_work_sync(&up_down_work);
  
  current_profile_no = 1;
  atomic_set(&always_on_cpu, 0);
  
	/* Bring cpu 0 up if not up already */
	  cpu_up(0);
	
	/* Bring CPU4 down */
	  cpu_down(4);
	
	/* Shut all core beside 0 off */ 
	for_each_online_cpu(cpu) {
		if (cpu == 0)
			continue;
		cpu_down(cpu);
	}
	
  /* Make sure CPU4 is really down */
	  cpu_down(4);
	
	/* Clear task on CPU4 */
	clear_tasks_mm_cpumask(4);
	
	/* Start workqueue on CPU0 */
	INIT_WORK(&up_down_work, cpu_up_down_work);
	INIT_DELAYED_WORK(&hima_hotplug_work, hima_hotplug_work_fn);
	queue_delayed_work_on(atomic_read(&always_on_cpu), hima_hotplug_wq, &hima_hotplug_work,
				      msecs_to_jiffies(DEF_SAMPLING_MS));
				      
	mutex_unlock(&hima_hotplug_mutex);
}

static void __ref hima_hotplug_resume(void)
{
	int cpu = 0;

	mutex_lock(&hima_hotplug_mutex);
	current_profile_no = 0;
	atomic_set(&always_on_cpu, 4);

  /* Flush hotplug workqueue */
	flush_workqueue(hima_hotplug_wq);
	cancel_delayed_work_sync(&hima_hotplug_work);
	cancel_work_sync(&up_down_work);

	/* Bring core 4 on */
	  cpu_up(4);
	
  /* Force CPU0 down */
	  cpu_down(0);
	
	/* Shut all cores beside 4 off */ 
	for_each_online_cpu(cpu) {
		if (cpu == 4)
			continue;
		cpu_down(cpu);
	}
	
  /* Make sure CPU0 is really down */
	  cpu_down(0);
	
	/* Clear task on CPU0 */
	clear_tasks_mm_cpumask(0);
	
	/* Start workqueue on CPU4 */
	INIT_WORK(&up_down_work, cpu_up_down_work);
	INIT_DELAYED_WORK(&hima_hotplug_work, hima_hotplug_work_fn);
	queue_delayed_work_on(atomic_read(&always_on_cpu), hima_hotplug_wq, &hima_hotplug_work,
				      msecs_to_jiffies(RESUME_SAMPLING_MS));
				      
	mutex_unlock(&hima_hotplug_mutex);
}

#ifdef CONFIG_STATE_NOTIFIER
static int state_notifier_callback(struct notifier_block *this,
				unsigned long event, void *data)
{
	if (atomic_read(&hima_hotplug_active) == 0)
		return NOTIFY_OK;

	switch (event) {
		case STATE_NOTIFIER_ACTIVE:
			hima_hotplug_resume();
			break;
		case STATE_NOTIFIER_SUSPEND:
			hima_hotplug_suspend();
			break;
		default:
			break;
	}

	return NOTIFY_OK;
}
#endif

static void hima_hotplug_input_event(struct input_handle *handle,
		unsigned int type, unsigned int code, int value)
{
	u64 now;

	if (cpus_boosted == 1)
		return;

	now = ktime_to_us(ktime_get());
	last_input = now;

	if (now - last_boost_time < MIN_INPUT_INTERVAL)
		return;

	if (num_online_cpus() >= cpus_boosted ||
	    cpus_boosted <= min_cpus_online)
		return;

	target_cpus = cpus_boosted;
	queue_work_on(atomic_read(&always_on_cpu), hima_hotplug_wq, &up_down_work);
	last_boost_time = ktime_to_us(ktime_get());
}

static int hima_hotplug_input_connect(struct input_handler *handler,
				 struct input_dev *dev,
				 const struct input_device_id *id)
{
	struct input_handle *handle;
	int err;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = handler->name;

	err = input_register_handle(handle);
	if (err)
		goto err_register;

	err = input_open_device(handle);
	if (err)
		goto err_open;

	dprintk("%s found and connected!\n", dev->name);

	return 0;
err_open:
	input_unregister_handle(handle);
err_register:
	kfree(handle);
	return err;
}

static void hima_hotplug_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id hima_hotplug_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			 INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
			    BIT_MASK(ABS_MT_POSITION_X) |
			    BIT_MASK(ABS_MT_POSITION_Y) },
	}, /* multi-touch touchscreen */
	{
		.flags = INPUT_DEVICE_ID_MATCH_KEYBIT |
			 INPUT_DEVICE_ID_MATCH_ABSBIT,
		.keybit = { [BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) },
		.absbit = { [BIT_WORD(ABS_X)] =
			    BIT_MASK(ABS_X) | BIT_MASK(ABS_Y) },
	}, /* touchpad */
	{ },
};

static struct input_handler hima_hotplug_input_handler = {
	.event          = hima_hotplug_input_event,
	.connect        = hima_hotplug_input_connect,
	.disconnect     = hima_hotplug_input_disconnect,
	.name           = "hima_hotplug_handler",
	.id_table       = hima_hotplug_ids,
};

static int __ref hima_hotplug_start(void)
{
	int cpu, ret = 0;
	atomic_set(&always_on_cpu, 4);

	hima_hotplug_wq = alloc_workqueue("hima_hotplug", WQ_HIGHPRI | WQ_FREEZABLE, 0);
	if (!hima_hotplug_wq) {
		pr_err("%s: Failed to allocate hotplug workqueue\n",
		       HIMA_HOTPLUG);
		ret = -ENOMEM;
		goto err_out;
	}

#ifdef CONFIG_STATE_NOTIFIER
	notif.notifier_call = state_notifier_callback;
	if (state_register_client(&notif)) {
		pr_err("%s: Failed to register State notifier callback\n",
			HIMA_HOTPLUG);
		goto err_dev;
	}
#endif

	ret = input_register_handler(&hima_hotplug_input_handler);
	if (ret) {
		pr_err("%s: Failed to register input handler: %d\n",
		       HIMA_HOTPLUG, ret);
		goto err_dev;
	}

	mutex_init(&hima_hotplug_mutex);

	INIT_WORK(&up_down_work, cpu_up_down_work);
	INIT_DELAYED_WORK(&hima_hotplug_work, hima_hotplug_work_fn);
	
	/* Fire up all CPUs */
	for_each_cpu_not(cpu, cpu_online_mask) {
		cpu_up(cpu);
	}

	queue_delayed_work_on(atomic_read(&always_on_cpu), hima_hotplug_wq, &hima_hotplug_work,
			      START_DELAY_MS);

	return ret;
err_dev:
	destroy_workqueue(hima_hotplug_wq);
err_out:
	atomic_set(&hima_hotplug_active, 0);
	return ret;
}

static void hima_hotplug_stop(void)
{
	flush_workqueue(hima_hotplug_wq);
	cancel_work_sync(&up_down_work);
	cancel_delayed_work_sync(&hima_hotplug_work);
	mutex_destroy(&hima_hotplug_mutex);
#ifdef CONFIG_STATE_NOTIFIER
	state_unregister_client(&notif);
#endif
	notif.notifier_call = NULL;

	input_unregister_handler(&hima_hotplug_input_handler);
	destroy_workqueue(hima_hotplug_wq);
}

static void hima_hotplug_active_eval_fn(unsigned int status)
{
	int ret = 0;

	if (status == 1) {
		ret = hima_hotplug_start();
		if (ret)
			status = 0;
	} else
		hima_hotplug_stop();

	atomic_set(&hima_hotplug_active, status);
}

#define show_one(file_name, object)				\
static ssize_t show_##file_name					\
(struct kobject *kobj, struct kobj_attribute *attr, char *buf)	\
{								\
	return sprintf(buf, "%u\n", object);			\
}

show_one(cpus_boosted, cpus_boosted);
show_one(min_cpus_online, min_cpus_online);
show_one(max_cpus_online, max_cpus_online);
show_one(current_profile_no, current_profile_no);
show_one(cpu_nr_run_threshold, cpu_nr_run_threshold);
show_one(def_sampling_ms, def_sampling_ms);
show_one(debug_hima_hotplug, debug_hima_hotplug);
show_one(nr_fshift, nr_fshift);
show_one(nr_run_hysteresis, nr_run_hysteresis);

#define store_one(file_name, object)		\
static ssize_t store_##file_name		\
(struct kobject *kobj, 				\
 struct kobj_attribute *attr, 			\
 const char *buf, size_t count)			\
{						\
	unsigned int input;			\
	int ret;				\
	ret = sscanf(buf, "%u", &input);	\
	if (ret != 1 || input > 100)		\
		return -EINVAL;			\
	if (input == object) {			\
		return count;			\
	}					\
	object = input;				\
	return count;				\
}

store_one(cpus_boosted, cpus_boosted);
store_one(current_profile_no, current_profile_no);
store_one(cpu_nr_run_threshold, cpu_nr_run_threshold);
store_one(def_sampling_ms, def_sampling_ms);
store_one(debug_hima_hotplug, debug_hima_hotplug);
store_one(nr_fshift, nr_fshift);
store_one(nr_run_hysteresis, nr_run_hysteresis);

static ssize_t show_hima_hotplug_active(struct kobject *kobj,
					struct kobj_attribute *attr, 
					char *buf)
{
	return sprintf(buf, "%d\n",
			atomic_read(&hima_hotplug_active));
}

static ssize_t store_hima_hotplug_active(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 const char *buf, size_t count)
{
	int ret;
	unsigned int input;

	ret = sscanf(buf, "%d", &input);
	if (ret < 0)
		return ret;

	if (input < 0)
		input = 0;
	else if (input > 0)
		input = 1;

	if (input == atomic_read(&hima_hotplug_active))
		return count;

	hima_hotplug_active_eval_fn(input);

	return count;
}

static ssize_t show_boost_lock_duration(struct kobject *kobj,
					struct kobj_attribute *attr, 
					char *buf)
{
	return sprintf(buf, "%llu\n", div_u64(boost_lock_duration, 1000));
}

static ssize_t store_boost_lock_duration(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 const char *buf, size_t count)
{
	int ret;
	u64 val;

	ret = sscanf(buf, "%llu", &val);
	if (ret != 1)
		return -EINVAL;

	boost_lock_duration = val * 1000;

	return count;
}

static ssize_t store_min_cpus_online(struct kobject *kobj,
				     struct kobj_attribute *attr,
				     const char *buf, size_t count)
{
	int ret;
	unsigned int val;

	ret = sscanf(buf, "%u", &val);
	if (ret != 1 || val < 1 || val > NR_CPUS)
		return -EINVAL;

	if (max_cpus_online < val)
		max_cpus_online = val;

	min_cpus_online = val;

	return count;
}

static ssize_t store_max_cpus_online(struct kobject *kobj,
				     struct kobj_attribute *attr,
				     const char *buf, size_t count)
{
	int ret;
	unsigned int val;

	ret = sscanf(buf, "%u", &val);
	if (ret != 1 || val < 1 || val > NR_CPUS)
		return -EINVAL;

	if (min_cpus_online > val)
		min_cpus_online = val;

	max_cpus_online = val;

	return count;
}

#define KERNEL_ATTR_RW(_name) \
static struct kobj_attribute _name##_attr = \
	__ATTR(_name, 0664, show_##_name, store_##_name)

KERNEL_ATTR_RW(hima_hotplug_active);
KERNEL_ATTR_RW(cpus_boosted);
KERNEL_ATTR_RW(min_cpus_online);
KERNEL_ATTR_RW(max_cpus_online);
KERNEL_ATTR_RW(current_profile_no);
KERNEL_ATTR_RW(cpu_nr_run_threshold);
KERNEL_ATTR_RW(boost_lock_duration);
KERNEL_ATTR_RW(def_sampling_ms);
KERNEL_ATTR_RW(debug_hima_hotplug);
KERNEL_ATTR_RW(nr_fshift);
KERNEL_ATTR_RW(nr_run_hysteresis);

static struct attribute *hima_hotplug_attrs[] = {
	&hima_hotplug_active_attr.attr,
	&cpus_boosted_attr.attr,
	&min_cpus_online_attr.attr,
	&max_cpus_online_attr.attr,
	&current_profile_no_attr.attr,
	&cpu_nr_run_threshold_attr.attr,
	&boost_lock_duration_attr.attr,
	&def_sampling_ms_attr.attr,
	&debug_hima_hotplug_attr.attr,
	&nr_fshift_attr.attr,
	&nr_run_hysteresis_attr.attr,
	NULL,
};

static struct attribute_group hima_hotplug_attr_group = {
	.attrs = hima_hotplug_attrs,
	.name = "hima_hotplug",
};

static int __init hima_hotplug_init(void)
{
	int rc;

	rc = sysfs_create_group(kernel_kobj, &hima_hotplug_attr_group);

	pr_info("HIMA_HOTPLUG: version %d.%d\n",
		 HIMA_HOTPLUG_MAJOR_VERSION,
		 HIMA_HOTPLUG_MINOR_VERSION);

	if (atomic_read(&hima_hotplug_active) == 1)
		hima_hotplug_start();

	return 0;
}

static void __exit hima_hotplug_exit(void)
{

	if (atomic_read(&hima_hotplug_active) == 1)
		hima_hotplug_stop();
	sysfs_remove_group(kernel_kobj, &hima_hotplug_attr_group);
}

late_initcall(hima_hotplug_init);
module_exit(hima_hotplug_exit);

MODULE_AUTHOR("Paul Reioux <reioux@gmail.com>");
MODULE_DESCRIPTION("'intell_plug' - An intelligent cpu hotplug driver for "
	"Low Latency Frequency Transition capable processors");
MODULE_LICENSE("GPLv2");
