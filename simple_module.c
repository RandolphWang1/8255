#define MODULE
#include <linux/module.h>
/*... other required header files... */

/*
 * ... module declarations and functions ...
 */

int
init_module()
{
 /* code kernel will call when installing module */
	printk("insmod\n");
}

void
cleanup_module()
{
 /* code kernel will call when removing module */
	printk("rmmod\n");
}
