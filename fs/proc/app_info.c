#include <linux/types.h>
#include <linux/proc_fs.h>
#include <asm/setup.h>
#include <linux/pagemap.h>

struct st_read_proc {
	char *name;
	int (*read_proc)(char *, char **, off_t, int, int *, void *);
};

extern unsigned int get_pd_charge_flag(void);
extern unsigned int resetmode_is_normal(void);

/* same as in proc_misc.c */
static int proc_calc_metrics(char *page, char **start, off_t off,
				 int count, int *eof, int len)
{
	if (len <= off + count)
		*eof = 1;
	*start = page + off;
	len -= off;
	if (len > count)
		len = count;
	if (len < 0)
		len = 0;
	return len;
}

static int app_tag_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len = 0;
	u32 charge_flag = 0;
	u32 recovery_flag = 0;
	u32 reset_normal_flag = 0;

	charge_flag = get_pd_charge_flag();
	reset_normal_flag = resetmode_is_normal();

	len = snprintf(page, PAGE_SIZE,
					"recovery_flag:\n%d\n"
					"charge_flag:\n%d\n"
					"reset_normal_flag:\n%d\n",
					recovery_flag,
					charge_flag,
					reset_normal_flag);
	return proc_calc_metrics(page, start, off, count, eof, len);
}

#define MAX_VERSION_CHAR 40
static char fastboot_version[MAX_VERSION_CHAR + 1];

static int __init early_get_fastboot_version(char *p)
{
	memset(fastboot_version, 0, MAX_VERSION_CHAR);	
	memcpy(fastboot_version, p, MAX_VERSION_CHAR);
	printk("fastboot_version = %s\n", fastboot_version);

	return 0;
}
early_param("fastboot_version", early_get_fastboot_version);

static int fastboot_version_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len = 0;

	len = snprintf(page, PAGE_SIZE,
					"%s\n",
					fastboot_version);

	return proc_calc_metrics(page, start, off, count, eof, len);
}

static int kernel_version_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len = 0;

	len = snprintf(page, PAGE_SIZE,
					"%s\n",
					S10_KERNEL_VERSION);

	return proc_calc_metrics(page, start, off, count, eof, len);
}

extern unsigned long s10_get_mem_size();
static int memsize_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len = 0;
	unsigned long memsize = 0;
	
	memsize = s10_get_mem_size();
	
	len = snprintf(page, PAGE_SIZE,
					"0x%x\n",
					memsize);

	return proc_calc_metrics(page, start, off, count, eof, len);
}
static struct st_read_proc simple_ones[] = {
			{"app_info", app_tag_read_proc},
			{"fastboot_version", fastboot_version_read_proc},
			{"kernel_version", kernel_version_read_proc},
			{"memsize",memsize_read_proc},
			{NULL,}
		};

void __init proc_app_info_init(void)
{
	struct st_read_proc *p;

	for (p = simple_ones; p->name; p++)
		create_proc_read_entry(p->name, 0, NULL, p->read_proc, NULL);
}

