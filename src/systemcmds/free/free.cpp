/**
 * @file free.cpp
 * Display memory usage
 */


#if defined(__PX4_FREERTOS)

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

extern "C" __EXPORT __attribute__((weak)) int px4_systemcmd_free_handler()
{
	printf("Memory info not available on this platform\n");
	return -ENOSYS;
}

static void print_usage()
{
	PRINT_MODULE_DESCRIPTION("Display memory usage information");
	PRINT_MODULE_USAGE_NAME_SIMPLE("free", "command");
}

extern "C" __EXPORT int free_main(int argc, char *argv[])
{
	if (argc > 1 && (!strcmp(argv[1], "-h") || !strcmp(argv[1], "--help"))) {
		print_usage();
		return 0;
	}
	return px4_systemcmd_free_handler();
}

#endif /* __PX4_FREERTOS */
