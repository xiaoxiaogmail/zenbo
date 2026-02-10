
/*
 *  linux/drivers/cpufreq/cpufreq_userspace.c
 *
 *  Copyright (C)  2001 Russell King
 *            (C)  2002 - 2004 Dominik Brodowski <linux@brodo.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cpufreq.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <asm/uaccess.h>

static DEFINE_PER_CPU(unsigned int, cpu_is_managed);
static DEFINE_PER_CPU(struct cpufreq_policy *, temp_policy);
static DEFINE_MUTEX(userspace_mutex);

//=========dvfs stress test ===================
extern struct cpufreq_frequency_table * cpufreq_frequency_get_table(unsigned int cpu);
struct cpufreq_frequency_table *cpu_freq_table=NULL;
struct workqueue_struct *dvfs_test_work_queue=NULL;
struct delayed_work stress_test;
bool stress_test_enable=false;
unsigned int  max_cpu_freq_index=CPUFREQ_TABLE_END;
int  freq_index=0;
static int cpufreq_set(struct cpufreq_policy *policy, unsigned int freq);
static bool cpufreq_set_userspace_governor(void);
void dvfs_stress_test(struct work_struct *work)
{
	int cpu=0;
	static int start_testing=0;
	if(!start_testing){
		cpufreq_set_userspace_governor();
		start_testing=1;
	}
	cpu=0;

	if(stress_test_enable)
		mutex_lock(&userspace_mutex);

	for_each_online_cpu(cpu){
		if(per_cpu(temp_policy,cpu) && per_cpu(cpu_is_managed, cpu)){
			cpufreq_set(per_cpu(temp_policy,cpu), cpu_freq_table[freq_index].frequency);
		}
	}

	if(stress_test_enable)
		mutex_unlock(&userspace_mutex);

	freq_index--;
	if(freq_index<0)
		freq_index=max_cpu_freq_index;
	if(stress_test_enable)
		queue_delayed_work(dvfs_test_work_queue, &stress_test,1*HZ);
	return ;
}

static bool cpufreq_set_userspace_governor(void)
{
	static char *cpufreq_sysfs_governor_path="/sys/devices/system/cpu/cpu%i/cpufreq/scaling_governor";
	static char *cpufreq_sysfs_online_path="/sys/devices/system/cpu/cpu%i/online";
	struct file *scaling_gov = NULL;
	struct file *cpux_online= NULL;
	static char *governor = "userspace";
	static char cpu_on[1] = {'1'};
	mm_segment_t old_fs;
	char    buf[128];
	int i;
	bool err=0;
	loff_t offset = 0;
	struct cpufreq_policy *policy =NULL;

	printk("cpufreq_set_userspace_governor+\n");
	/* change to KERNEL_DS address limit */
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	for(i = 1; i <= 3; i++) {
		sprintf(buf, cpufreq_sysfs_online_path, i);
		cpux_online = filp_open(buf, O_RDWR, 0);

		if (IS_ERR_OR_NULL(cpux_online)){
			printk("cpufreq_set_userspace_governor: open %s fail\n", buf);
			err = true;
			goto error;
		}

		cpux_online->f_op->write(cpux_online, cpu_on, strlen(cpu_on), &offset);

		memset(buf,0,sizeof(buf));
		sprintf(buf, cpufreq_sysfs_governor_path, i);
		scaling_gov = filp_open(buf, O_RDWR, 0);
		if (IS_ERR_OR_NULL(scaling_gov)) {
			printk("cpufreq_set_userspace_governor: open %s fail\n", buf);
			err = true;
			goto error;
		}

		policy =cpufreq_cpu_get(i);
		if(policy) {
			if(!strncmp(policy->governor->name, governor, 9)) {
				printk("cpufreq_set_userspace_governor cpu%u is userspace\n", i);
				if (policy)
					cpufreq_cpu_put(policy);
				continue;
			}
			cpufreq_cpu_put(policy);
		}

		if (scaling_gov->f_op != NULL && scaling_gov->f_op->write != NULL) {
			scaling_gov->f_op->write(scaling_gov,
					governor,
					strlen(governor),
					&offset);
		} else
			pr_err("f_op might be null\n");

		filp_close(scaling_gov, NULL);
		filp_close(cpux_online, NULL);
	}

	printk("cpufreq_set_userspace_governor-\n");
error:
	set_fs(old_fs);
	return err;
}
static int start_test(struct cpufreq_policy *policy, unsigned int enable)
{
	int i=0;
	if(policy->cpu==0 && enable ){
		cpu_freq_table=cpufreq_frequency_get_table(0);

		for (i = 0; (cpu_freq_table[i].frequency != CPUFREQ_TABLE_END); i++) {
		if (cpu_freq_table[i].frequency == CPUFREQ_ENTRY_INVALID)
			continue;
		printk("start_test cpu_freq_table[%u].frequency=%u\n",i,cpu_freq_table[i].frequency);
		}

		freq_index=max_cpu_freq_index=i-1;
		stress_test_enable=true;
		queue_delayed_work(dvfs_test_work_queue, &stress_test,1*HZ);
	}else
		stress_test_enable=false;
	return true;
}

static ssize_t show_test(struct cpufreq_policy *policy, char *buf)
{
	return sprintf(buf, "%u\n", stress_test_enable);
}

//=========dvfs stress test ===================
/**
 * cpufreq_set - set the CPU frequency
 * @policy: pointer to policy struct where freq is being set
 * @freq: target frequency in kHz
 *
 * Sets the CPU frequency to freq.
 */
static int cpufreq_set(struct cpufreq_policy *policy, unsigned int freq)
{
	int ret = -EINVAL;

	printk("cpufreq_set for cpu %u, freq %u kHz\n", policy->cpu, freq);

	if(!stress_test_enable)
		mutex_lock(&userspace_mutex);
	if (!per_cpu(cpu_is_managed, policy->cpu))
		goto err;

	ret = __cpufreq_driver_target(policy, freq, CPUFREQ_RELATION_L);
 err:
 	if(!stress_test_enable)
		mutex_unlock(&userspace_mutex);
	return ret;
}

static ssize_t show_speed(struct cpufreq_policy *policy, char *buf)
{
	return sprintf(buf, "%u\n", policy->cur);
}

static int cpufreq_governor_userspace(struct cpufreq_policy *policy,
				   unsigned int event)
{
	unsigned int cpu = policy->cpu;
	int rc = 0;

	switch (event) {
	case CPUFREQ_GOV_START:
		if (!cpu_online(cpu))
			return -EINVAL;
		per_cpu(temp_policy, cpu) =policy;
		BUG_ON(!policy->cur);
		pr_debug("started managing cpu %u\n", cpu);

		mutex_lock(&userspace_mutex);
		per_cpu(cpu_is_managed, cpu) = 1;
		mutex_unlock(&userspace_mutex);
		break;
	case CPUFREQ_GOV_STOP:
		pr_debug("managing cpu %u stopped\n", cpu);

		mutex_lock(&userspace_mutex);
		per_cpu(cpu_is_managed, cpu) = 0;
		per_cpu(temp_policy, cpu) =NULL;
		mutex_unlock(&userspace_mutex);
		break;
	case CPUFREQ_GOV_LIMITS:
		mutex_lock(&userspace_mutex);
		pr_debug("limit event for cpu %u: %u - %u kHz, currently %u kHz\n",
			cpu, policy->min, policy->max,
			policy->cur);

		if (policy->max < policy->cur)
			__cpufreq_driver_target(policy, policy->max,
						CPUFREQ_RELATION_H);
		else if (policy->min > policy->cur)
			__cpufreq_driver_target(policy, policy->min,
						CPUFREQ_RELATION_L);
		mutex_unlock(&userspace_mutex);
		break;
	}
	return rc;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_USERSPACE
static
#endif
struct cpufreq_governor cpufreq_gov_userspace = {
	.name		= "userspace",
	.governor	= cpufreq_governor_userspace,
	.store_setspeed	= cpufreq_set,
	.show_setspeed	= show_speed,
	.start_dvfs_test	 =  start_test,
	.show_dvfs_test	=  show_test,
	.owner		= THIS_MODULE,
};

static int __init cpufreq_gov_userspace_init(void)
{
	INIT_DELAYED_WORK(&stress_test,  dvfs_stress_test) ;
	dvfs_test_work_queue = create_singlethread_workqueue("dvfs_test_workqueue");
	return cpufreq_register_governor(&cpufreq_gov_userspace);
}

static void __exit cpufreq_gov_userspace_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_userspace);
}

MODULE_AUTHOR("Dominik Brodowski <linux@brodo.de>, "
		"Russell King <rmk@arm.linux.org.uk>");
MODULE_DESCRIPTION("CPUfreq policy governor 'userspace'");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_USERSPACE
fs_initcall(cpufreq_gov_userspace_init);
#else
module_init(cpufreq_gov_userspace_init);
#endif
module_exit(cpufreq_gov_userspace_exit);
