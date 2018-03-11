/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2015 Mellanox.
 */

#include <stddef.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <net/if.h>

#include <rte_io.h>
#include <rte_pci.h>
#include <rte_ethdev_pci.h>
#include <rte_ether.h>
#include <rte_ethdev.h>
#include <rte_eth_ctrl.h>

#include "mdev_lib.h"

/* todo: rte should be switched to local implementation */
#define MDEV_READ_16_BE(a)		rte_be_to_cpu_16(rte_read16(a))
#define MDEV_READ_32_BE(a)		rte_be_to_cpu_32(rte_read32(a))
#define MDEV_WRITE_32_BE(val, a)	rte_write32(rte_cpu_to_be_32(val), a)
#define MDEV_CPU_TO_BE_64(x)		rte_cpu_to_be_64(x)

#define MDEV_CPU_TO_BE_64(x)		rte_cpu_to_be_64(x)
#define MDEV_CPU_TO_BE_32(x)		rte_cpu_to_be_32(x)
#define MDEV_CPU_TO_BE_16(x)		rte_cpu_to_be_16(x)
#define MDEV_ALLOC(size, align) rte_zmalloc("mdev", size, align)

static inline unsigned int
log2above(unsigned int v)
{
	unsigned int l;
	unsigned int r;

	for (l = 0, r = 0; (v >> 1); ++l, v >>= 1)
		r |= (v & 1);
	return l + r;
}


int mlx5_mdev_cmd_exec(struct mlx5_mdev_context *ctx, void *in, int ilen,
			  void *out, int olen)
{
	struct mlx5_mdev_cmd *cmd = ctx->cmd;
	char *cin = (char *)in;
	char *cout = (char *)out;
	int output_len = olen;
	int temp_len;
	int number_of_obox = olen > (512+16) ? (olen-16) / 512 + 1 : 1;
	int i;
	struct mlx5_cmd_block	*obox = &cmd->obox;
	volatile uint8_t *status_own = &cmd->entry.status_own;

	if (ilen > 512)  {
		RTE_LOG(ERR, PMD, "mlx5e_cmd_exec: len>512 ilen=%d olen=%d\n",
			ilen, olen);
		return -EIO;
	}
	for(i=1; i < number_of_obox; i++) {
		obox->next = MDEV_CPU_TO_BE_64(ctx->ms.phys_addr + (i-1)*1024);
		obox = (struct mlx5_cmd_block *)((uint8_t*)ctx->ms.addr
				+ (i-1) * 1024);
		memset(obox,0,sizeof(struct mlx5_cmd_block));
		obox->block_num = MDEV_CPU_TO_BE_32(i);

	}
	obox->next = 0x0;
	obox = &cmd->obox;

	memcpy(cmd->entry.in, in, 16);
	memcpy(cmd->ibox.data, cin + 16, (ilen > 16) ? (ilen - 16) : 0);
	cmd->entry.ilen = rte_cpu_to_be_32(ilen);
	cmd->entry.olen = rte_cpu_to_be_32(olen);
	cmd->entry.status_own = 0x1;
	MDEV_WRITE_32_BE(0x1, &ctx->iseg->cmd_dbell);
	while (*status_own & 0x1)
		;
	memcpy(out, cmd->entry.out, 16);
	output_len -= 16;
	if(output_len > 0) {
		for (i=0; i<number_of_obox; i++) {
			temp_len = RTE_MIN(512, output_len);
			memcpy(cout + olen-output_len, obox->data, temp_len);
			output_len -= temp_len;
			obox = (struct mlx5_cmd_block *)((uint8_t*)ctx->ms.addr + i * sizeof(struct mlx5_cmd_block));
		}
	}
	if (cmd->entry.status_own) {
		uint32_t *c = (uint32_t *)&ctx->cmd->entry;
		unsigned int i;

		RTE_LOG(ERR, PMD, "mlx5e_cmd_exec error:\n");
		for (i = 0; i < sizeof(cmd->entry); i += 4)
			RTE_LOG(ERR, PMD, "[0x%x] 0x%.8x 0x%.8x 0x%.8x\
				0x%.8x\n", i*4, c[i], c[i+1], c[i+2], c[i+3]);
	}

	return cmd->entry.status_own;
}

static int mlx5_mdev_cmd_init(struct mlx5_mdev_context *ctx)
{
	const struct mlx5_mdev_memzone *mz;

	mz = ctx->alloc_function(ctx->owner,
			"cmd",
			sizeof(struct mlx5_mdev_cmd),
			sizeof(struct mlx5_mdev_cmd));
	if (!mz)
		return -ENOMEM;
	memset(mz->addr, 0, sizeof(struct mlx5_mdev_cmd));

	ctx->cmd_pa = mz->phys_addr;
	ctx->cmd = mz->addr;
	ctx->cmd->entry.type = 0x7;
	ctx->cmd->entry.iptr = MDEV_CPU_TO_BE_64(ctx->cmd_pa + 2048);
	ctx->cmd->entry.optr = MDEV_CPU_TO_BE_64(ctx->cmd_pa + 2048 + 1024);

	mz = ctx->alloc_function(ctx->owner,
				"mailboxspace",
				4096*2,
				4096);
	if (!mz)
		return -ENOMEM;
	ctx->ms.addr = mz->addr;
	ctx->ms.phys_addr = mz->phys_addr;
	return 0;
}

static void mlx5_mdev_cmd_enable_reset_nic_interface(struct mlx5_mdev_context *ctx)
{
	MDEV_WRITE_32_BE((uint32_t)(ctx->cmd_pa >> 32), &ctx->iseg->cmdq_pa_h);
	MDEV_WRITE_32_BE((uint32_t)(ctx->cmd_pa), &ctx->iseg->cmdq_pa_l_sz);
	rte_wmb();
}


static int mlx5_mdev_enable_hca(struct mlx5_mdev_context *ctx)
{
	uint32_t in[MLX5_ST_SZ_DW(enable_hca_in)]   = {0};
	uint32_t out[MLX5_ST_SZ_DW(enable_hca_out)] = {0};

	MLX5_SET(enable_hca_in, in, opcode, MLX5_CMD_OP_ENABLE_HCA);
	return mlx5_mdev_cmd_exec(ctx, in, sizeof(in), out, sizeof(out));
}

static void mlx5_mdev_disable_hca(struct mlx5_mdev_context *ctx)
{
	uint32_t in[MLX5_ST_SZ_DW(disable_hca_in)]   = {0};
	uint32_t out[MLX5_ST_SZ_DW(disable_hca_out)] = {0};

	MLX5_SET(disable_hca_in, in, opcode, MLX5_CMD_OP_DISABLE_HCA);
	mlx5_mdev_cmd_exec(ctx, in, sizeof(in), out, sizeof(out));
}

static int mlx5_mdev_init_hca(struct mlx5_mdev_context *ctx)
{
	uint32_t in[MLX5_ST_SZ_DW(init_hca_in)]   = {0};
	uint32_t out[MLX5_ST_SZ_DW(init_hca_out)] = {0};
	uint32_t status;
	int err;

	MLX5_SET(init_hca_in, in, opcode, MLX5_CMD_OP_INIT_HCA);

	err = mlx5_mdev_cmd_exec(ctx, in, sizeof(in), out, sizeof(out));
	if (err)
		return err;
	
	status = MLX5_GET(init_hca_out,out,status);
	return status;
}

static int mlx5_mdev_set_issi(struct mlx5_mdev_context *ctx)
{
	uint32_t query_in[MLX5_ST_SZ_DW(query_issi_in)]   = {0};
	uint32_t query_out[MLX5_ST_SZ_DW(query_issi_out)] = {0};
	uint32_t set_in[MLX5_ST_SZ_DW(set_issi_in)]       = {0};
	uint32_t set_out[MLX5_ST_SZ_DW(set_issi_out)]     = {0};
	uint32_t sup_issi;
	uint32_t status;
	int err;

	MLX5_SET(query_issi_in, query_in, opcode, MLX5_CMD_OP_QUERY_ISSI);
	err = mlx5_mdev_cmd_exec(ctx, query_in, sizeof(query_in), query_out,
			    sizeof(query_out));
	if (err)
		return err;

	sup_issi = MLX5_GET(query_issi_out, query_out, supported_issi_dw0);

	if (!(sup_issi & (1 << 1)))
		return -EOPNOTSUPP;

	MLX5_SET(set_issi_in, set_in, opcode, MLX5_CMD_OP_SET_ISSI);
	MLX5_SET(set_issi_in, set_in, current_issi, 1);
	err = mlx5_mdev_cmd_exec(ctx, set_in, sizeof(set_in), set_out,
			    sizeof(set_out));
	if (err)
		return err;
	status = MLX5_GET(set_issi_out,set_out,status);

	return status;
}

static int mlx5_mdev_query_pages(struct mlx5_mdev_context *ctx, int boot, uint16_t *func_id,
				int32_t *npages)
{
	uint32_t query_pages_in[MLX5_ST_SZ_DW(query_pages_in)]   = {0};
	uint32_t query_pages_out[MLX5_ST_SZ_DW(query_pages_out)]   = {0};
	int res;

	MLX5_SET(query_pages_in, query_pages_in, opcode, 0x107);
	MLX5_SET(query_pages_in, query_pages_in, op_mod, boot ? 0x01 : 0x02);


	res = mlx5_mdev_cmd_exec(ctx, (uint8_t*)&query_pages_in, sizeof(query_pages_in),
			(uint8_t*)&query_pages_out, sizeof(query_pages_out));
	if(res)
		return res;
	*npages = MLX5_GET(query_pages_out,query_pages_out,num_pages);
	*func_id = MLX5_GET(query_pages_out,query_pages_out,function_id);

	return res;
}

static int mlx5_mdev_give_pages(struct mlx5_mdev_context *ctx, uint16_t func_id,
				int32_t npages )
{
	uint32_t in[MLX5_ST_SZ_DW(manage_pages_in)]   = {0};
	uint32_t out[MLX5_ST_SZ_DW(manage_pages_out)] = {0};
	int ilen = MLX5_ST_SZ_BYTES(manage_pages_in) -
			((32-npages) * sizeof(uint64_t));
	int res = 0;
	int i;
	const struct mlx5_mdev_memzone *mz;
	static int last_page_index = 0;
	char page_name[20];
	__be64 *pas	= (__be64 *)MLX5_ADDR_OF(manage_pages_in, in, pas);
	int status;

	for (i=0; i < npages; i++) {
		snprintf(page_name, sizeof(page_name), "page_%d",
				 last_page_index);
		mz = ctx->alloc_function(ctx->owner,
						page_name,
						4096,
						4096);

		if (!mz)
			return -ENOMEM;
		pas[i] = rte_cpu_to_be_64(mz->phys_addr);
		last_page_index++;
	}
	MLX5_SET(manage_pages_in, in, opcode, MLX5_CMD_OP_MANAGE_PAGES);
	MLX5_SET(manage_pages_in, in, op_mod, 1);
	MLX5_SET(manage_pages_in, in, function_id, func_id);
	MLX5_SET(manage_pages_in, in, input_num_entries, npages);

	res = mlx5_mdev_cmd_exec(ctx, (uint8_t*)in, ilen,
				(uint8_t*)&out, sizeof(out));
	if (res)
		return res;

	status = MLX5_GET(manage_pages_out,out,status);
	return status;
}

static int mlx5_mdev_satisfy_startup_pages(struct mlx5_mdev_context *ctx, int boot)
{
	uint16_t function_id;
	int32_t	npages = 0;
	int res;

	res = mlx5_mdev_query_pages(ctx, boot, &function_id, &npages);

	if (npages>0)
		res = mlx5_mdev_give_pages(ctx, function_id, npages);

	return res;
}

static void mlx5_mdev_teardown_hca(struct mlx5_mdev_context *ctx)
{
	uint32_t out[MLX5_ST_SZ_DW(teardown_hca_out)] = {0};
	uint32_t in[MLX5_ST_SZ_DW(teardown_hca_in)]   = {0};

	MLX5_SET(teardown_hca_in, in, opcode, MLX5_CMD_OP_TEARDOWN_HCA);
	mlx5_mdev_cmd_exec(ctx, in, sizeof(in), out, sizeof(out));
}

#if 0
static void mlx5_mdev_get_hca_cap_gen(struct mlx5_mdev_context *ctx)
{
	uint32_t in[MLX5_ST_SZ_DW(query_hca_cap_in)]   = {0};
	uint32_t out[MLX5_ST_SZ_DW(query_hca_cap_out)] = {0};

	MLX5_SET(query_hca_cap_in, in, opcode, MLX5_CMD_OP_QUERY_HCA_CAP);
	MLX5_SET(query_hca_cap_in, in, op_mod,
		 (MLX5_CAP_GENERAL << 1) |
		 (MLX5_HCA_CAP_OPMOD_GET_CUR & 0x1));
	mlx5_mdev_cmd_exec(ctx, in, sizeof(in), out, sizeof(out));
	memcpy(ctx->cap.gen,
	       MLX5_ADDR_OF(query_hca_cap_out, out, capability),
	       sizeof(ctx->cap.gen));
}

static void mlx5_mdev_get_hca_cap_eth(struct mlx5_mdev_context *ctx)
{
	uint32_t in[MLX5_ST_SZ_DW(query_hca_cap_in)]   = {0};
	uint32_t out[MLX5_ST_SZ_DW(query_hca_cap_out)] = {0};

	MLX5_SET(query_hca_cap_in, in, opcode, MLX5_CMD_OP_QUERY_HCA_CAP);
	MLX5_SET(query_hca_cap_in, in, op_mod,
		 (MLX5_CAP_ETHERNET_OFFLOADS << 1) |
		 (MLX5_HCA_CAP_OPMOD_GET_CUR & 0x1));
	mlx5_mdev_cmd_exec(ctx, in, sizeof(in), out, sizeof(out));
	memcpy(ctx->cap.eth,
	       MLX5_ADDR_OF(query_hca_cap_out, out, capability),
	       sizeof(ctx->cap.eth));
}

static void mlx5_mdev_get_hca_cap_ftn(struct mlx5_mdev_context *ctx)
{
	uint32_t in[MLX5_ST_SZ_DW(query_hca_cap_in)]   = {0};
	uint32_t out[MLX5_ST_SZ_DW(query_hca_cap_out)] = {0};

	MLX5_SET(query_hca_cap_in, in, opcode, MLX5_CMD_OP_QUERY_HCA_CAP);
	MLX5_SET(query_hca_cap_in, in, op_mod,
		 (MLX5_CAP_FLOW_TABLE << 1) |
		 (MLX5_HCA_CAP_OPMOD_GET_CUR & 0x1));
	mlx5_mdev_cmd_exec(ctx, in, sizeof(in), out, sizeof(out));
	memcpy(ctx->cap.ftn,
	       MLX5_ADDR_OF(query_hca_cap_out, out, capability),
	       sizeof(ctx->cap.ftn));
}

static void mlx5_mdev_get_hca_cap(struct mlx5_mdev_context *ctx)
{
	mlx5_mdev_get_hca_cap_gen(ctx);
	mlx5_mdev_get_hca_cap_eth(ctx);
	mlx5_mdev_get_hca_cap_ftn(ctx);
}

static int mlx5_mdev_check_hca_cap(struct mlx5_mdev_context *ctx)
{
	if ((MLX5_CAP_GEN(ctx, port_type) != MLX5_CAP_PORT_TYPE_ETH)	    ||
	    (MLX5_CAP_GEN(ctx, num_ports) > 1)				    ||
	    (MLX5_CAP_GEN(ctx, cqe_version) != 1)			    ||
	     0) {
		RTE_LOG(ERR, PMD, "mlx5_mdev_check_hca_cap failed\n");
		return -EOPNOTSUPP;
	}

	return 0;
}
#endif
static void mlx5_mdev_query_nic_vport_mac_addr(struct mlx5_mdev_context *ctx,
					   struct ether_addr *addr)
{
	uint32_t in[MLX5_ST_SZ_DW(query_nic_vport_context_in)]   = {0};
	uint32_t out[MLX5_ST_SZ_DW(query_nic_vport_context_out)] = {0};

	MLX5_SET(query_nic_vport_context_in, in, opcode,
		 MLX5_CMD_OP_QUERY_NIC_VPORT_CONTEXT);
	mlx5_mdev_cmd_exec(ctx, in, sizeof(in), out, sizeof(out));

	ether_addr_copy(addr,
			(void *)MLX5_ADDR_OF(query_nic_vport_context_out, out,
					nic_vport_context.permanent_address));
}

static int mlx5_mdev_set_edev_addr(struct mlx5_mdev_context *ctx)
{
	struct rte_eth_dev *edev = ctx->owner;

	edev->data->mac_addrs = rte_zmalloc("mlx5e mac", ETHER_ADDR_LEN, 0);
	if (!edev->data->mac_addrs)
		return -ENOMEM;

	mlx5_mdev_query_nic_vport_mac_addr(ctx, &edev->data->mac_addrs[0]);
	if (!is_valid_assigned_ether_addr(&edev->data->mac_addrs[0]))
		eth_random_addr((uint8_t *)&edev->data->mac_addrs[0]);

	return 0;
}

static void mlx5_mdev_unset_edev_addr(struct mlx5_mdev_context *ctx)
{
	rte_free(((struct rte_eth_dev *)(ctx->owner))->data->mac_addrs);
}

static int mlx5_mdev_alloc_pd(struct mlx5_mdev_context *ctx)
{
	uint32_t in[MLX5_ST_SZ_DW(alloc_pd_in)]   = {0};
	uint32_t out[MLX5_ST_SZ_DW(alloc_pd_out)] = {0};
	uint32_t status;
	int err;

	MLX5_SET(alloc_pd_in, in, opcode, MLX5_CMD_OP_ALLOC_PD);
	err = mlx5_mdev_cmd_exec(ctx, in, sizeof(in), out, sizeof(out));
	if (err)
		return err;

	status = MLX5_GET(alloc_pd_out, out, status);
	ctx->pd = MLX5_GET(alloc_pd_out, out, pd);

	return status;
}

static void mlx5_mdev_dealloc_pd(struct mlx5_mdev_context *ctx)
{
	uint32_t in[MLX5_ST_SZ_DW(dealloc_pd_in)]   = {0};
	uint32_t out[MLX5_ST_SZ_DW(dealloc_pd_out)] = {0};

	MLX5_SET(dealloc_pd_in, in, opcode, MLX5_CMD_OP_DEALLOC_PD);
	MLX5_SET(dealloc_pd_in, in, pd, ctx->pd);
	mlx5_mdev_cmd_exec(ctx, in, sizeof(in), out, sizeof(out));
}

static int mlx5_mdev_alloc_td(struct mlx5_mdev_context *ctx)
{
	uint32_t in[MLX5_ST_SZ_DW(alloc_transport_domain_in)]   = {0};
	uint32_t out[MLX5_ST_SZ_DW(alloc_transport_domain_out)] = {0};
	int err;
	uint32_t status;


	MLX5_SET(alloc_transport_domain_in, in, opcode,
		 MLX5_CMD_OP_ALLOC_TRANSPORT_DOMAIN);
	err = mlx5_mdev_cmd_exec(ctx, in, sizeof(in), out, sizeof(out));
	if (err)
		return err;

	ctx->td = MLX5_GET(alloc_transport_domain_out, out, transport_domain);

	status = MLX5_GET(alloc_pd_out, out, status);
	return status;
}

static void mlx5_mdev_dealloc_td(struct mlx5_mdev_context *ctx)
{
	uint32_t in[MLX5_ST_SZ_DW(dealloc_transport_domain_in)]   = {0};
	uint32_t out[MLX5_ST_SZ_DW(dealloc_transport_domain_out)] = {0};

	MLX5_SET(dealloc_transport_domain_in, in, opcode,
		 MLX5_CMD_OP_DEALLOC_TRANSPORT_DOMAIN);
	MLX5_SET(dealloc_transport_domain_in, in, transport_domain, ctx->td);
	mlx5_mdev_cmd_exec(ctx, in, sizeof(in), out, sizeof(out));
}

static int mlx5_mdev_alloc_uar(struct mlx5_mdev_context *ctx)
{
	uint32_t in[MLX5_ST_SZ_DW(alloc_uar_in)]   = {0};
	uint32_t out[MLX5_ST_SZ_DW(alloc_uar_out)] = {0};
	int err;
	uint32_t status;

	MLX5_SET(alloc_uar_in, in, opcode, MLX5_CMD_OP_ALLOC_UAR);
	err = mlx5_mdev_cmd_exec(ctx, in, sizeof(in), out, sizeof(out));
	if (err)
		return err;

	ctx->uar = MLX5_GET(alloc_uar_out, out, uar);
	status = MLX5_GET(alloc_uar_out, out, status);
	return status;
}

struct mlx5_mdev_context * mdev_open_device(void *owner,
					void *iseg,
					alloc_dma_memory_t *alloc_function)
{
	struct mlx5_mdev_context *ctx = MDEV_ALLOC(sizeof(struct mlx5_mdev_context), 64);
	int err;

	if(!ctx)
		return ctx;
	ctx->owner = owner;
	ctx->alloc_function = alloc_function;
	ctx->iseg = iseg;
	ctx->cache_line_size = 64; //todo: should be based on system
	ctx->page_size = 4096; //todo should be based on system

	if (MDEV_READ_16_BE(&ctx->iseg->cmdif_rev) != 5)
			return NULL;

	err = mlx5_mdev_cmd_init(ctx);
	if (err)
		return NULL;

	mlx5_mdev_cmd_enable_reset_nic_interface(ctx);
	while (MDEV_READ_32_BE(&ctx->iseg->initializing) >> 31)
			;
	err = mlx5_mdev_enable_hca(ctx);
	if (err)
		return NULL;

	err = mlx5_mdev_set_issi(ctx);
	if (err)
		return NULL;

	if (err)
		goto err_disable_hca;

	err = mlx5_mdev_satisfy_startup_pages(ctx, 0);
	err = mlx5_mdev_init_hca(ctx);

	// mlx5_mdev_get_hca_cap(ctx);
	//err = mlx5_mdev_check_hca_cap(ctx);
	if (err)
		goto err_teardown_hca;
	err = mlx5_mdev_set_edev_addr(ctx);
	if (err)
		goto err_teardown_hca;
	err = mlx5_mdev_alloc_pd(ctx);
	if (err)
		goto err_unset_edev_addr;
	err = mlx5_mdev_alloc_td(ctx);
	if (err)
		goto err_dealloc_pd;
	err = mlx5_mdev_alloc_uar(ctx);
	if (err)
		goto err_dealloc_td;
	return ctx;
err_dealloc_td:
	mlx5_mdev_dealloc_td(ctx);
err_dealloc_pd:
	mlx5_mdev_dealloc_pd(ctx);
err_unset_edev_addr:
	mlx5_mdev_unset_edev_addr(ctx);
err_teardown_hca:
	mlx5_mdev_teardown_hca(ctx);
err_disable_hca:
	mlx5_mdev_disable_hca(ctx);
	return NULL;
}
