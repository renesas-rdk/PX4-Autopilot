
#pragma once

#include <inttypes.h>
#include <string.h>

#define ATOMIC_ENTER lock()
#define ATOMIC_LEAVE unlock()

namespace cdev
{

struct file_operations {
	void *op;
};

using px4_file_operations_t = struct file_operations;
using mode_t = uint32_t;

class CDev;

struct file_t {
	int f_oflags{0};
	void *f_priv{nullptr};
	CDev *cdev{nullptr};

	file_t() = default;
	file_t(int f, CDev *c) : f_oflags(f), cdev(c) {}
};

} // namespace cdev

extern "C" __EXPORT int register_driver(const char *name, const cdev::px4_file_operations_t *fops,
					cdev::mode_t mode, void *data);
extern "C" __EXPORT int unregister_driver(const char *path);
#if defined(__PX4_FREERTOS)
extern "C" __EXPORT void px4_rzv_register_default_uart_devices_once(void);
extern "C" __EXPORT int px4_ioctl(int fd, int cmd, unsigned long arg);
#endif /* __PX4_FREERTOS */
