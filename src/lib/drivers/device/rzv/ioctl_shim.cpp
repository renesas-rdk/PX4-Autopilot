
#if defined(__PX4_FREERTOS)

#include <px4_platform_common/px4_config.h>

#include <stdarg.h>
#include <sys/ioctl.h>

extern "C" int px4_ioctl(int fd, int cmd, unsigned long arg);

int ioctl(int fd, unsigned long request, ...)
{
	va_list ap;
	va_start(ap, request);
	unsigned long arg = va_arg(ap, unsigned long);
	va_end(ap);

	return px4_ioctl(fd, static_cast<int>(request), arg);
}

#endif /* __PX4_FREERTOS */
