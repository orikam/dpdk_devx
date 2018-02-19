#include <stddef.h>
#include <stdio.h>
#include <rte_byteorder.h>
#include "devx.h"
#include "mdev_prm.h"

int main(void) {
	int num;
	struct devx_device **list = devx_get_device_list(&num);
	void *ctx = devx_open_device(list[2]);
	u8 in[MLX5_ST_SZ_BYTES(query_hca_cap_in)] = {0};
	u8 out[MLX5_ST_SZ_BYTES(query_hca_cap_out)] = {0};
	int ret;

	devx_free_device_list(list);
	MLX5_SET(query_hca_cap_in, in, opcode, MLX5_CMD_OP_QUERY_HCA_CAP);
	MLX5_SET(query_hca_cap_in, in, op_mod, MLX5_HCA_CAP_OPMOD_GET_MAX | (MLX5_CAP_GENERAL << 1));
	ret = devx_cmd(ctx, in, sizeof(in), out, sizeof(out));
	printf("%d %d\n", ret, MLX5_GET(query_hca_cap_out, out, capability.cmd_hca_cap.log_max_cq_sz));

	devx_close_device(ctx);
	return 0;
}
