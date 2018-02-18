#include <stdlib.h>
#include <errno.h>
#include "cmd_ioctl.h"
#include "devx.h"
#include "devx_priv.h"

struct devx_obj_handle {
	struct devx_context *ctx;
	uint32_t handle;
};

int devx_cmd(void *ctx,
	     void *in, size_t inlen,
	     void *out, size_t outlen)
{
	DECLARE_COMMAND_BUFFER(cmd,
			       UVERBS_OBJECT_MLX5_MDEV,
			       MLX5_MDEV_OTHER_CMD,
			       2);

	fill_attr_in(cmd, MLX5_MDEV_OBJ_MODIFY_CMD_IN, in, inlen);
	fill_attr_in(cmd, MLX5_MDEV_OBJ_MODIFY_CMD_OUT, out, outlen);
	return execute_ioctl(((struct devx_context *)ctx)->cmd_fd, cmd);
}

struct devx_obj_handle *devx_obj_create(void *ctx,
					void *in, size_t inlen,
					void *out, size_t outlen)
{
	DECLARE_COMMAND_BUFFER(cmd,
			       UVERBS_OBJECT_MLX5_MDEV_OBJ,
			       MLX5_MDEV_OBJ_CREATE,
			       3);
	struct ib_uverbs_attr *handle;
	struct devx_obj_handle *obj;
	int ret = ENOMEM;

	obj = (struct devx_obj_handle *)malloc(sizeof(*obj));
	if (!obj)
		goto err;
	obj->ctx = ctx;

	handle = fill_attr_out_obj(cmd, MLX5_MDEV_OBJ_CREATE_HANDLE);
	fill_attr_in(cmd, MLX5_MDEV_OBJ_CREATE_CMD_IN, in, inlen);
	fill_attr_in(cmd, MLX5_MDEV_OBJ_CREATE_CMD_OUT, out, outlen);

	ret = execute_ioctl(obj->ctx->cmd_fd, cmd);
	if (ret)
		goto err;
	obj->handle = handle->data;

	return obj;
err:
	free(obj);
	errno = ret;
	return NULL;
}

int devx_obj_destroy(struct devx_obj_handle *obj)
{
	DECLARE_COMMAND_BUFFER(cmd,
			       UVERBS_OBJECT_MLX5_MDEV_OBJ,
			       MLX5_MDEV_OBJ_DESTROY,
			       1);

	fill_attr_in_obj(cmd, MLX5_MDEV_OBJ_DESTROY_HANDLE, obj->handle);
	return execute_ioctl(obj->ctx->cmd_fd, cmd);
}

struct devx_obj_handle *devx_umem_reg(void *ctx,
				      void *addr, size_t size,
				      int access,
				      uint32_t *id)
{
	DECLARE_COMMAND_BUFFER(cmd,
			       UVERBS_OBJECT_MLX5_MDEV_UMEM,
			       MLX5_MDEV_UMEM_REG,
			       5);
	struct ib_uverbs_attr *handle, *aid;
	struct devx_obj_handle *obj;
	int ret = ENOMEM;

	obj = (struct devx_obj_handle *)malloc(sizeof(*obj));
	if (!obj)
		goto err;
	obj->ctx = ctx;

	handle = fill_attr_out_obj(cmd, MLX5_MDEV_UMEM_REG_HANDLE);
	fill_attr_in_uint64(cmd, MLX5_MDEV_UMEM_REG_ADDR, (intptr_t)addr);
	fill_attr_in_uint64(cmd, MLX5_MDEV_UMEM_REG_LEN, size);
	fill_attr_in_uint32(cmd, MLX5_MDEV_UMEM_REG_ACCESS, access);
	aid = fill_attr_out_obj(cmd, MLX5_MDEV_UMEM_REG_ID);

	ret = execute_ioctl(obj->ctx->cmd_fd, cmd);
	if (ret)
		goto err;
	obj->handle = handle->data;
	*id = aid->data;

	return obj;
err:
	free(obj);
	errno = ret;
	return NULL;
}

int devx_umem_unreg(struct devx_obj_handle *obj)
{
	DECLARE_COMMAND_BUFFER(cmd,
			       UVERBS_OBJECT_MLX5_MDEV_UMEM,
			       MLX5_MDEV_UMEM_DEREG,
			       1);

	fill_attr_in_obj(cmd, MLX5_MDEV_UMEM_DEREG_HANDLE, obj->handle);
	return execute_ioctl(obj->ctx->cmd_fd, cmd);
}
struct devx_obj_handle *devx_fs_rule_add(void *ctx,
					 void *in, uint32_t inlen)
{
	DECLARE_COMMAND_BUFFER(cmd,
			       UVERBS_OBJECT_MLX5_MDEV_FS_RULE,
			       MLX5_MDEV_FS_RULE_ADD,
			       2);
	struct ib_uverbs_attr *handle;
	struct devx_obj_handle *obj;
	int ret = ENOMEM;

	obj = (struct devx_obj_handle *)malloc(sizeof(*obj));
	if (!obj)
		goto err;
	obj->ctx = ctx;

	handle = fill_attr_out_obj(cmd, MLX5_MDEV_FS_RULE_ADD_HANDLE);
	fill_attr_in(cmd, MLX5_MDEV_FS_RULE_ADD_CMD_IN, in, inlen);

	ret = execute_ioctl(obj->ctx->cmd_fd, cmd);
	if (ret)
		goto err;
	obj->handle = handle->data;

	return obj;
err:
	free(obj);
	errno = ret;
	return NULL;
}

int devx_fs_rule_del(struct devx_obj_handle *obj)
{
	DECLARE_COMMAND_BUFFER(cmd,
			       UVERBS_OBJECT_MLX5_MDEV_FS_RULE,
			       MLX5_MDEV_FS_RULE_DEL,
			       1);

	fill_attr_in_obj(cmd, MLX5_MDEV_FS_RULE_DEL_HANDLE, obj->handle);
	return execute_ioctl(obj->ctx->cmd_fd, cmd);
}

