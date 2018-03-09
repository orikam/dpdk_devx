#include "cmd_ioctl.h"
#include "devx.h"
#include "devx_priv.h"
#include "mlx5.h"

int devx_alloc_uar(void *context, uint32_t *idx, void **addr)
{
	struct mlx5_context *ctx = to_mctx((struct devx_context *)context);
	DECLARE_COMMAND_BUFFER(cmd,
			       UVERBS_OBJECT_MLX5_MDEV,
			       MLX5_MDEV_QUERY_UAR,
			       2);
	int ret;

	fill_attr_in_uint32(cmd, MLX5_MDEV_QUERY_UAR_INDEX, *idx);
	fill_attr_out(cmd, MLX5_MDEV_QUERY_UAR_PAGE, idx, sizeof(*idx));
	ret = execute_ioctl(((struct devx_context *)context)->cmd_fd, cmd);
	if (ret)
		return ret;

	*addr = ctx->uar[0].reg;
	return 0;
}
