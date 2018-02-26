#include <stddef.h>
#include <stdio.h>
#include <sys/mman.h>
#include <unistd.h>
#include <rte_byteorder.h>
#include "devx.h"
#include "mdev_prm.h"

void test_query(void *ctx);
void test_query(void *ctx) {
	u8 in[MLX5_ST_SZ_BYTES(query_hca_cap_in)] = {0};
	u8 out[MLX5_ST_SZ_BYTES(query_hca_cap_out)] = {0};
	int ret;

	MLX5_SET(query_hca_cap_in, in, opcode, MLX5_CMD_OP_QUERY_HCA_CAP);
	MLX5_SET(query_hca_cap_in, in, op_mod, MLX5_HCA_CAP_OPMOD_GET_MAX | (MLX5_CAP_GENERAL << 1));
	ret = devx_cmd(ctx, in, sizeof(in), out, sizeof(out));
	printf("%s:%d %d %d\n", __func__, __LINE__, ret,
			MLX5_GET(query_hca_cap_out, out,
				capability.cmd_hca_cap.log_max_cq_sz));
}

int alloc_pd(void *ctx);
int alloc_pd(void *ctx) {
	u8 in[MLX5_ST_SZ_BYTES(alloc_pd_in)] = {0};
	u8 out[MLX5_ST_SZ_BYTES(alloc_pd_out)] = {0};
	struct devx_obj_handle *pd;

	MLX5_SET(alloc_pd_in, in, opcode, MLX5_CMD_OP_ALLOC_PD);
	pd = devx_obj_create(ctx, in, sizeof(in), out, sizeof(out));
	if (!pd)
		return -1;

	return MLX5_GET(alloc_pd_out, out, pd);
}

void test_mr(void *ctx, int pd);
void test_mr(void *ctx, int pd) {
	u8 in[MLX5_ST_SZ_BYTES(create_mkey_in) + MLX5_ST_SZ_BYTES(cmd_pas) * 2] = {0};
	u8 out[MLX5_ST_SZ_BYTES(create_mkey_out)] = {0};
	struct devx_obj_handle *mem, *mr;
	uint32_t mem_id;
	void *buff;

	buff = mmap(NULL, 0x1000, PROT_READ|PROT_WRITE, MAP_PRIVATE|MAP_ANON, -1, 0);
	mem = devx_umem_reg(ctx, buff, 0x1000, 7, &mem_id);
	printf("%s:%d %p %d %d\n", __func__, __LINE__, mem, errno, mem_id);

	MLX5_SET(create_mkey_in, in, opcode, MLX5_CMD_OP_CREATE_MKEY);
	MLX5_SET(create_mkey_in, in, ctx.access_mode, MLX5_MKC_ACCESS_MODE_MTT);
	MLX5_SET(create_mkey_in, in, ctx.a, 1);
	MLX5_SET(create_mkey_in, in, ctx.rw, 1);
	MLX5_SET(create_mkey_in, in, ctx.rr, 1);
	MLX5_SET(create_mkey_in, in, ctx.lw, 1);
	MLX5_SET(create_mkey_in, in, ctx.lr, 1);
	MLX5_SET64(create_mkey_in, in, ctx.start_addr, (intptr_t)buff);
	MLX5_SET64(create_mkey_in, in, ctx.len, 0x1000);
	MLX5_SET(create_mkey_in, in, ctx.pd, pd);
	MLX5_SET(create_mkey_in, in, ctx.translations_octword_size, 1);
	MLX5_SET(create_mkey_in, in, ctx.log_page_size, 12);
	MLX5_SET(create_mkey_in, in, ctx.qpn, 0xffffff);
	MLX5_SET(create_mkey_in, in, ctx.pas_umem_id, mem_id);
	MLX5_SET(create_mkey_in, in, translations_octword_actual_size, 1);
	MLX5_SET(create_mkey_in, in, pg_access, 1);
	mr = devx_obj_create(ctx, in, sizeof(in), out, sizeof(out));
	printf("%s:%d %p %p\n", __func__, __LINE__, mem, mr);
}

void test_cq(void *ctx);
void test_cq(void *ctx) {
	u8 in[MLX5_ST_SZ_BYTES(create_cq_in)] = {0};
	u8 out[MLX5_ST_SZ_BYTES(create_cq_out)] = {0};
	struct devx_obj_handle *pas, *cq;
	uint32_t pas_id, dbr_id, uar_id;
	size_t dbr_off;
	void *buff, *uar_ptr, *dbr;
	int ret;

	buff = mmap(NULL, 0x1000, PROT_READ|PROT_WRITE, MAP_PRIVATE|MAP_ANON, -1, 0);
	pas = devx_umem_reg(ctx, buff, 0x1000, 7, &pas_id);

	dbr = devx_alloc_db(ctx, &dbr_id, &dbr_off);
	printf("%s:%d %p %d %zd\n", __func__, __LINE__, dbr, dbr_id, dbr_off);
	ret = devx_alloc_uar(ctx, &uar_id, &uar_ptr);
	printf("%s:%d %d %d %p\n", __func__, __LINE__, ret, uar_id, uar_ptr);

	MLX5_SET(create_cq_in, in, opcode, MLX5_CMD_OP_CREATE_CQ);
	MLX5_SET(create_cq_in, in, ctx.cqe_sz, 0);
	MLX5_SET(create_cq_in, in, ctx.log_cq_size, 5);
	MLX5_SET(create_cq_in, in, ctx.uar_page, uar_id);
	MLX5_SET(create_cq_in, in, ctx.pas_umem_id, pas_id);
	MLX5_SET(create_cq_in, in, ctx.dbr_umem_id, dbr_id);
	MLX5_SET64(create_cq_in, in, ctx.dbr_addr, dbr_off);
	cq = devx_obj_create(ctx, in, sizeof(in), out, sizeof(out));
	printf("%s:%d %p %p\n", __func__, __LINE__, cq, pas);
}

int main(void) {
	int num;
	struct devx_device **list = devx_get_device_list(&num);
	void *ctx;
	int pd;

	ctx = devx_open_device(list[0]);
	devx_free_device_list(list);

	test_query(ctx);
	pd = alloc_pd(ctx);
	test_mr(ctx, pd);
	test_cq(ctx);

	devx_close_device(ctx);
	return 0;
}
