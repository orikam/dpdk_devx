#ifndef __DEVX_PRIV_H__
#define __DEVX_PRIV_H__

struct devx_context {
	struct devx_device *device;
	int cmd_fd;
};

struct devx_obj_handle {
	struct devx_context *ctx;
	uint32_t handle;
};

#endif
