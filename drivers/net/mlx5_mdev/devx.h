#ifndef __DEVX_H__
#define __DEVX_H__

#include <linux/limits.h>
#include <stdint.h>
#include <rte_bus_pci.h>

struct devx_device {
	char			name[NAME_MAX];
	char			dev_name[NAME_MAX];
	char			dev_path[PATH_MAX];
	char			ibdev_path[PATH_MAX];
};

struct devx_device **devx_get_device_list(int *num_devices);
void devx_free_device_list(struct devx_device **list);
void *devx_open_device(struct devx_device *device);
int devx_close_device(void *context);

int devx_cmd(void *ctx,
	     void *in, size_t inlen,
	     void *out, size_t outlen);

int devx_alloc_uar(void *ctx, uint32_t *idx, void **addr);

struct devx_obj_handle;

struct devx_obj_handle *devx_obj_create(void *ctx,
					void *in, size_t inlen,
					void *out, size_t outlen);
int devx_obj_destroy(struct devx_obj_handle *obj);

struct devx_obj_handle *devx_umem_reg(void *ctx,
				      void *addr, size_t size,
				      int access,
				      uint32_t *id);
int devx_umem_unreg(struct devx_obj_handle *obj);

struct devx_obj_handle *devx_fs_rule_add(void *ctx,
					 void *in, uint32_t inlen);
int devx_fs_rule_del(struct devx_obj_handle *obj);

int devx_device_to_pci_addr(const struct devx_device *device,
			    struct rte_pci_addr *pci_addr);

#endif
