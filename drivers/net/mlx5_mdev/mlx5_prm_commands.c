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
#include <math.h>


#include <sys/mman.h>
#include <unistd.h>


#include <rte_io.h>
#include <rte_pci.h>
#include <rte_ethdev_pci.h>
#include <rte_ether.h>
#include <rte_ethdev.h>
#include <rte_eth_ctrl.h>

#include "mlx5_mdev_utils.h"
#include "mdev_prm.h"
#include "mlx5_mdev_priv.h"
#include "devx.h"

static int
mdev_priv_create_eq(void *dev __rte_unused, struct mdev_eq *eq)
{
	void *cqc;
//	uint32_t in[MLX5_ST_SZ_DW(create_eq_in)] = {0};
	uint32_t in[MLX5_ST_SZ_DW(create_eq_in) + MLX5_ST_SZ_DW(cmd_pas) * 2] = {0};
	uint32_t out[MLX5_ST_SZ_DW(create_eq_out)] = {0};
	int status, syndrome;

	cqc = MLX5_ADDR_OF(create_eq_in, in, ctx);
	MLX5_SET(create_eq_in, in, opcode, MLX5_CMD_OP_CREATE_EQ);
//	MLX5_ARRAY_SET64(create_eq_in, in, pas, 0,eq->buf->iova);
	MLX5_SET(eqc, cqc, log_eq_size, log2(eq->neqe));
	MLX5_SET(eqc, cqc, uar_page, eq->uar_page);
	MLX5_SET(eqc, cqc, pas_umem_id, eq->mem.index);
	MLX5_SET(eqc, cqc, log_page_size, log2(eq->buf->len /4096)); // TODO: from where ? MTT ???

	eq->object = devx_obj_create(eq->ctx, in, sizeof(in), out, sizeof(out));
	status = 100;
	status = MLX5_GET(create_eq_out, out, status);
	syndrome = MLX5_GET(create_eq_out, out, syndrome);
	eq->eqn = MLX5_GET(create_eq_out, out, eqn);


	printf("mdev_priv_create_eq status %x, syndrome = %x. eq num %d\n",status, syndrome,eq->eqn);
	if (!eq->object)

		return 1;
	return status;
}

#define cqe_sz_to_mlx_sz(size) \
	(size)== 64 ? 0 : 1


static int
mdev_priv_create_cq(void *dev __rte_unused, struct mdev_cq *cq)
{
	void *cqc;
	uint32_t *in;
	uint32_t out[MLX5_ST_SZ_DW(create_cq_out)+ MLX5_ST_SZ_DW(cmd_pas) * 2] = {0};
	int status, syndrome, size;

	size = MLX5_ST_SZ_BYTES(create_cq_in) + MLX5_ST_SZ_BYTES(cmd_pas) * (cq->buf->len/4096);
	
	in = malloc(size);
	memset(in,0,size);

	cqc = MLX5_ADDR_OF(create_cq_in, in, ctx);
	MLX5_SET(create_cq_in, in, opcode, MLX5_CMD_OP_CREATE_CQ);
//	MLX5_ARRAY_SET64(create_cq_in, in, pas, 0, cq->buf->iova);
	MLX5_SET(cqc, cqc, c_eqn, cq->eqn);
	MLX5_SET(cqc, cqc, cqe_sz, cqe_sz_to_mlx_sz(cq->cqe_size));
	MLX5_SET(cqc, cqc, uar_page, cq->uar_page);
	MLX5_SET(cqc, cqc, log_page_size,log2(cq->buf->len/4096)); // TODO: from where ? MTT ???
	MLX5_SET(cqc, cqc, dbr_umem_id, cq->dbrec.index);
	MLX5_SET(cqc, cqc, log_cq_size, cq->ncqe); // WAS: cq->buf->len
	MLX5_SET(cqc, cqc, pas_umem_id, cq->mem.index);
	MLX5_SET64(cqc, cqc, dbr_addr, cq->dbrec.offset);

	MLX5_SET(cqc, cqc, oi, 0);
	printf("mdev_priv_create_cq uar %x, dbrec = %d\n",cq->uar_page, cq->dbrec.index);
	cq->object = devx_obj_create(cq->dev, in, size, out, sizeof(out));
	if (!cq->object) {
		printf("Error creating cq object error from kernel %d\n",errno);
		return 1;
	}

	cq->cqn = MLX5_GET(create_cq_out, out, cqn);
	cq->cons_index = 0;
	// cq->arm_sn     = 0;
	status = MLX5_GET(create_cq_out, out, status);
	syndrome = MLX5_GET(create_cq_out, out, syndrome);
	printf("mdev_priv_create_cq status %x, syndrome = %x, id %d\n",status, syndrome, cq->cqn);

	return status;
}

static int
mdev_priv_create_tis(void *dev __rte_unused, struct mdev_tis *tis)
{
	void *tisc;
	uint32_t in[MLX5_ST_SZ_DW(create_tis_in)] = {0};
	uint32_t out[MLX5_ST_SZ_DW(create_tis_out)];
	int status, syndrome;

	tisc = MLX5_ADDR_OF(create_tis_in, in, ctx);
	MLX5_SET(create_tis_in, in, opcode, MLX5_CMD_OP_CREATE_TIS);
	MLX5_SET(tisc, tisc, prio, (tis->priority)<<1);
	MLX5_SET(tisc, tisc, transport_domain, tis->td);
	tis->object = devx_obj_create(tis->dev, in, sizeof(in), out, sizeof(out));
	if (!tis->object)
		return 1;

	status = MLX5_GET(create_tis_out, out, status);
	syndrome = MLX5_GET(create_tis_out, out, syndrome);
	//if(!status)
	tis->tisn = MLX5_GET(create_tis_out, out, tisn);
	printf("mdev_priv_create_tis status %x, syndrome = %x, tis %d\n",status, syndrome, tis->tisn);

	return status;
}

static int
mdev_priv_create_sq(void *dev __rte_unused, struct mdev_sq *sq)
{
	void *sqc;
	void *wqc;
	//uint32_t in[MLX5_ST_SZ_DW(create_sq_in) + MLX5_ST_SZ_DW(cmd_pas) * (sq->wq.buf->len/4096)] = {0};
	uint32_t *in;
	uint32_t out[MLX5_ST_SZ_DW(create_sq_out)] = {0};
	int status, syndrome;
	int size;

	size = MLX5_ST_SZ_BYTES(create_sq_in) + MLX5_ST_SZ_BYTES(cmd_pas) * (sq->wq.buf->len/4096);
	
	in = malloc(size);
	memset(in,0,size);
	
	MLX5_SET(create_sq_in, in, opcode, MLX5_CMD_OP_CREATE_SQ);

	sqc = MLX5_ADDR_OF(create_sq_in, in, ctx);
	MLX5_SET(sqc, sqc, cqn, sq->cqn);
	MLX5_SET(sqc, sqc, tis_lst_sz, 1);
	MLX5_SET(sqc, sqc, tis_num_0, sq->tisn);

	wqc = MLX5_ADDR_OF(sqc, sqc, wq);
	MLX5_SET(wq, wqc, wq_type, 0x1);
	MLX5_SET(wq, wqc, uar_page, sq->wq.uar_page);
	MLX5_SET(wq, wqc, pd, sq->wq.pd);
	MLX5_SET(wq, wqc, log_wq_stride, 6);
	MLX5_SET(wq, wqc, log_wq_pg_sz, log2(sq->wq.buf->len /4096));
	MLX5_SET(wq, wqc, log_wq_sz, log2(sq->wq.buf->len>>6));
	MLX5_SET(wq, wqc, dbr_umem_id, sq->wq.dbrec.index);
	MLX5_SET(wq, wqc, pas_umem_id, sq->wq.mem.index);
	MLX5_SET64(wq, wqc, dbr_addr, sq->wq.dbrec.offset);
//	MLX5_ARRAY_SET64(wq, wqc, pas, 0, sq->wq.buf->iova);

	sq->object = devx_obj_create(sq->ctx, in, size, out, sizeof(out));
	if (!sq->object) {
		printf("create_sq devx object fail %d",errno);
		return 1;
	}

	status = MLX5_GET(create_sq_out, out, status);
	syndrome = MLX5_GET(create_sq_out, out, syndrome);
	sq->sqn = MLX5_GET(create_sq_out, out, sqn);
	printf("mdev_priv_create_sq status %x, syndrome = %x sq %d uar %d\n",status, syndrome, sq->sqn, sq->wq.uar_page);

	return status;
}

static int
mdev_priv_modify_sq(struct mdev_sq *sq, struct mdev_sq_attr *sq_attr)
{
	void *sqc;
	uint32_t in[MLX5_ST_SZ_DW(modify_sq_in)] = {0};
	uint32_t out[MLX5_ST_SZ_DW(modify_sq_out)] = {0};
	int status, syndrome;

	MLX5_SET(modify_sq_in, in, opcode, MLX5_CMD_OP_MODIFY_SQ);
	MLX5_SET(modify_sq_in, in, sqn, sq->sqn);
	MLX5_SET(modify_sq_in, in, sq_state, sq_attr->state);
	//MLX5_SET64(modify_sq_in, in, modify_bitmask, 0x01);
	sqc = MLX5_ADDR_OF(create_sq_in, in, ctx);
	MLX5_SET(sqc, sqc, state, 0x1);

	status = devx_cmd(sq->ctx, in, sizeof(in), out, sizeof(out));
	if (status) {
		printf("modify_sq object fail %d",errno);
		return status;
	}

	status = MLX5_GET(create_sq_out, out, status);
	syndrome = MLX5_GET(create_sq_out, out, syndrome);
	printf("mdev_priv_modify_sq status %x, syndrome = %x \n",status, syndrome);

	return status;
}
static int
mdev_priv_query_sq(struct mdev_sq *sq, struct mdev_sq_attr *sq_attr)
{
	void *sqc;
	void *wqc;
	uint32_t in[MLX5_ST_SZ_DW(query_sq_in)] = {0};
	uint32_t out[MLX5_ST_SZ_DW(query_sq_out)] = {0};
	int status, syndrome;
	uint32_t state;
	uint32_t uar_page;

	MLX5_SET(query_sq_in, in, opcode, MLX5_CMD_OP_QUERY_SQ);
	MLX5_SET(query_sq_in, in, sqn, sq->sqn);

	devx_cmd(sq->ctx, in, sizeof(in), out, sizeof(out));
	sqc = MLX5_ADDR_OF(query_sq_out, out, sq_context);
	wqc = MLX5_ADDR_OF(sqc, sqc, wq);
	state = MLX5_GET(sqc,sqc,state);
	
	sq_attr->state = state;	
	status = MLX5_GET(query_sq_out, out, status);
	uar_page = MLX5_GET(wq,wqc,uar_page);
	syndrome = MLX5_GET(query_sq_out, out, syndrome);
	printf("mdev_priv_query_sq status %x, syndrome = %x state %d , uar page %d\n",status, syndrome, state,uar_page);

	return status;
}
struct mdev_eq *
mlx5_mdev_create_eq(struct mlx5_mdev_priv *priv,
		    struct mdev_eq_attr *eq_attr)
{
	uint32_t eqe_size = 64;
	int log_max_cq_sz = 24; // TODO take from QUERY_HCA_CAP
	struct mdev_eq *eq;
	uint32_t neqe, eq_size;
	int ret;
	if (!eq_attr->eqe) {
		return NULL;
	}
	eq = rte_zmalloc("mdev_eq", sizeof(*eq), priv->cache_line_size); // TODO: make it numa node aware ?
	if(!eq)
		return NULL;
	neqe = 1UL << log2above(eq_attr->eqe + 1);
	eq_size = neqe * eqe_size;
	if ((neqe > 1UL << log_max_cq_sz) ||
	    (neqe < (eq_attr->eqe + 1))) {
		goto err_spl;
	}
	eq->ctx = eq_attr->ctx;
	eq->neqe = neqe;
	eq->buf = rte_eth_dma_zone_reserve(priv->dev, "eq_buffer", 0, eq_size, eq_size,
						priv->dev->data->numa_node);

	eq->mem.object = devx_umem_reg(priv->ctx,
			eq->buf->addr, eq_size,
					      7,
					      &eq->mem.index);

	if(!eq->mem.object)
		printf("Erorrrr!!! no umem reg\n");
	if (!eq->buf)
		goto err_spl;

	eq->uar_page = eq_attr->uar;

	ret = mdev_priv_create_eq(eq_attr->ctx, eq);
	printf("create eq res == %d\n", ret);
	if (ret)
		goto err_ccq;

	return eq;
err_ccq:
	//mlx5_mdev_dealloc_uar(ctx, eq->uar_page); // fixme : remove
err_spl:
	//if (eq->buf)
	//	mdev_dealloc_cq_buf(ctx, eq->buf);
	//if (eq->dbrec)
	//	mlx5_return_dbrec(priv, eq->dbrec);
	//rte_free(eq);
	return NULL;
}

struct mdev_cq *
mlx5_mdev_create_cq(struct mlx5_mdev_priv *priv,
		    struct mdev_cq_attr *cq_attr)
{
	uint32_t cqe_size = 64; // TODO make it a user parameter ?
	struct mdev_cq *cq;
	void *dev = priv->ctx;
	uint32_t cqe_count, cq_size;
	struct mlx5_cqe *cqe;	
	int ret;
	int i;


	if (!cq_attr->ncqe) {
		ERROR("number of cq elements is 0");
		return NULL;
	}
	cq = rte_zmalloc("mdev_cq", sizeof(*cq), 64); // TODO: make it numa node aware ?
	if(!cq)
		return NULL;
	cqe_count = 1UL << cq_attr->ncqe;
	cq_size = cqe_count * cqe_size;
	cq->dbrec.addr = devx_alloc_db(cq_attr->ctx, &cq->dbrec.index, &cq->dbrec.offset);
	if (!cq->dbrec.addr) {
		ERROR("can't allocate dbrec in create cq");
		goto err_spl;
	}

	cq->uar_page = priv->uar.index;
	/* Fill info for create CQ */
	cq->eqn = cq_attr->eqn;
	cq->buf = rte_eth_dma_zone_reserve(priv->dev, "cq_buffer", 0, cq_size, 4096,
						priv->dev->data->numa_node);
	cq->mem.object = devx_umem_reg(priv->ctx,
				cq->buf->addr, cq_size,
						      7,
						      &cq->mem.index);

	if(!cq->mem.object)
		printf("Erorrrr!!! no umem reg in create cq\n");
	cq->dev = dev;
	cq->ncqe = cq_attr->ncqe;
	cq->cqe_size = cqe_size;
	ret = mdev_priv_create_cq(dev, cq);
	if (ret)
		goto err_ccq;
	cqe = (struct mlx5_cqe *)cq->buf->addr;
	for (i = 0; i < cqe_count; i++) {
		cqe->op_own = 0xf0;
		cqe++;
	}	
	printf("create CQ res == %d, uar page %d\n",ret, cq->uar_page);
	return cq;
err_ccq:
	//mlx5_mdev_dealloc_uar(ctx, cq->uar_page); // fixme : remove
err_spl:
	//if (cq->buf)
	//	mdev_dealloc_cq_buf(ctx, cq->buf);
	//if (cq->dbrec)
	//	mlx5_return_dbrec(priv, cq->dbrec);
	//rte_free(cq);
	return NULL;
}

struct mdev_tis *
mlx5_mdev_create_tis(struct mlx5_mdev_priv *priv,
		    struct mdev_tis_attr *tis_attr)
{
	void *dev = priv->ctx;
	int ret;
	struct mdev_tis *tis;

	if (!tis_attr->td) {
		return NULL;
	}
	tis = rte_zmalloc("tis", sizeof(*tis), priv->cache_line_size);
	if(!tis)
		return NULL;

	tis->dev = dev;
	tis->td = tis_attr->td;

	ret = mdev_priv_create_tis(dev, tis);
	printf("create tis res == %d\n", ret);
	if (ret)
		goto err_tis;
	return tis;
err_tis:

	return NULL;
}


struct mdev_sq *
mlx5_mdev_create_sq(struct mlx5_mdev_priv *priv,
		    struct mdev_sq_attr *sq_attr)
{
	int ret;
	struct mdev_sq *sq;


	sq = rte_zmalloc("sq", sizeof(*sq), priv->cache_line_size);
	if(!sq)
		return NULL;
	sq->wq.dbrec.addr = devx_alloc_db(sq_attr->ctx, &sq->wq.dbrec.index, &sq->wq.dbrec.offset);
	if (!sq->wq.dbrec.addr) {
		ERROR("can't allocate dbrec in create sq");
//		goto err_spl;
	}

	sq->ctx = priv->ctx;
	sq->wq.pd = sq_attr->wq.pd;
	sq->cqn = sq_attr->cqn;
	sq->tisn = sq_attr->tisn;
	sq->wq.dbr_addr = sq->wq.dbrec.addr;
	sq->wq.uar_page = sq_attr->uar;
	sq->wq.buf = rte_eth_dma_zone_reserve(priv->dev, "sq_buffer", 0, sq_attr->nelements * 64, 4096,
							priv->dev->data->numa_node);
	sq->wq.mem.object = devx_umem_reg(priv->ctx,
					sq->wq.buf->addr, sq_attr->nelements * 64,
							      7,
							      &sq->wq.mem.index);

	if(!sq->wq.mem.object)
		printf("Erorrrr!!! no umem reg in create sq \n");
	ret = mdev_priv_create_sq(priv->ctx, sq);
	printf("create sq res == %d, uar page %d\n", ret, sq->wq.uar_page);
	if (ret)
		goto err_sq;
	return sq;
err_sq:

	return NULL;
}
/*
struct mdev_sq *
mlx5_mdev_modify_sq(struct mlx5_mdev_priv *priv,
		    struct mdev_sq_attr *sq_attr)
{
	int ret;
	struct mdev_sq *sq;


	sq = rte_zmalloc("sq", sizeof(*sq), priv->cache_line_size);
	if(!sq)
		return NULL;
	sq->wq.dbrec.addr = devx_alloc_db(sq_attr->ctx, &sq->wq.dbrec.index, &sq->wq.dbrec.offset);
	if (!sq->wq.dbrec.addr) {
		ERROR("can't allocate dbrec in create sq");
//		goto err_spl;
	}

	sq->ctx = priv->ctx;
	sq->wq.pd = sq_attr->wq.pd;
	sq->cqn = sq_attr->cqn;
	sq->tisn = sq_attr->tisn;
	sq->wq.dbr_addr = sq->wq.dbrec.addr;
	sq->wq.uar_page = sq_attr->uar;
	sq->wq.buf = rte_eth_dma_zone_reserve(priv->dev, "sq_buffer", 0, sq_attr->nelements * 64, 4096,
							priv->dev->data->numa_node);
	sq->wq.mem.object = devx_umem_reg(priv->ctx,
					sq->wq.buf->addr, sq_attr->nelements * 64,
							      7,
							      &sq->wq.mem.index);

	if(!sq->wq.mem.object)
		printf("Erorrrr!!! no umem reg in create sq \n");
	ret = mdev_priv_create_sq(priv->ctx, sq);
	printf("create sq res == %d, uar page %d\n", ret, sq->wq.uar_page);
	if (ret)
		goto err_sq;
	return sq;
err_sq:

	return NULL;
}
*/
int
mlx5_mdev_query_sq( struct mdev_sq *sq,
		    struct mdev_sq_attr *sq_attr)
{
	int ret;
	memset(sq_attr, 0, sizeof(*sq_attr));
	ret = mdev_priv_query_sq(sq, sq_attr);
	printf("query sq res == %d\n", ret);
	if (ret)
		goto err_sq;
	return 0;
err_sq:

	return 1;
}

int
mlx5_mdev_modify_sq( struct mdev_sq *sq,
		    struct mdev_sq_attr *sq_attr)
{
	int ret;
	memset(sq_attr, 0, sizeof(*sq_attr));
	ret = mdev_priv_modify_sq(sq, sq_attr);
	printf("modify sq res == %d\n", ret);
	if (ret)
		goto err_sq;
	return 0;
err_sq:

	return 1;
}

int mlx5_mdev_alloc_pd(struct mlx5_mdev_priv *priv)
{
	uint32_t in[MLX5_ST_SZ_DW(alloc_pd_in)]   = {0};
	uint32_t out[MLX5_ST_SZ_DW(alloc_pd_out)] = {0};
	uint32_t status;

	MLX5_SET(alloc_pd_in, in, opcode, MLX5_CMD_OP_ALLOC_PD);
	priv->pd.pd_object = devx_obj_create(priv->ctx, in, sizeof(in), out, sizeof(out));
	if (!priv->pd.pd_object)
		return 1;

	status = MLX5_GET(alloc_pd_out, out, status);
	priv->pd.pd = MLX5_GET(alloc_pd_out, out, pd);
	printf("oooOri pd status %d pd value %d\n",status, priv->pd.pd);
	return status;
}

int mlx5_mdev_alloc_td(struct mlx5_mdev_priv *priv)
{
	uint32_t in[MLX5_ST_SZ_DW(alloc_transport_domain_in)]   = {0};
	uint32_t out[MLX5_ST_SZ_DW(alloc_transport_domain_out)] = {0};
	uint32_t status;


	MLX5_SET(alloc_transport_domain_in, in, opcode,
		 MLX5_CMD_OP_ALLOC_TRANSPORT_DOMAIN);
	priv->td.td_object = devx_obj_create(priv->ctx, in, sizeof(in), out, sizeof(out));
	if (!priv->td.td_object)
		return 1;

	priv->td.td = MLX5_GET(alloc_transport_domain_out, out, transport_domain);

	status = MLX5_GET(alloc_pd_out, out, status);
	printf("oooOri td status %d td value %d\n",status, priv->td.td);
	return status;
}

struct mlx5_mdev_mkey *mlx5_mdev_create_mkey(struct mlx5_mdev_priv *priv __rte_unused,
			struct mlx5_mdev_mkey_attr *mkey_attr __rte_unused)
{
	uint32_t *in;
	uint32_t out[MLX5_ST_SZ_DW(create_mkey_out)] = {0};
	uint32_t status;
	int size ;
	void * mkc;
	struct mlx5_mdev_mkey *mkey = NULL;

	size = MLX5_ST_SZ_BYTES(create_mkey_in) + MLX5_ST_SZ_BYTES(cmd_pas) * ((mkey_attr->size / 4096)+1);
	in = malloc(size);
	memset(in,0,size);

	mkey = rte_zmalloc("mkey", sizeof(*mkey), priv->cache_line_size);

	MLX5_SET(create_mkey_in, in, opcode, MLX5_CMD_OP_CREATE_MKEY);

	mkc = MLX5_ADDR_OF(create_mkey_in, in, ctx);
	MLX5_SET(mkc, mkc, lw, 0x1);
	MLX5_SET(mkc, mkc, lr, 0x1);
	MLX5_SET(mkc, mkc, rw, 0x1);
	MLX5_SET(mkc, mkc, rr, 0x1);
	MLX5_SET(mkc, mkc, access_mode, MLX5_MKC_ACCESS_MODE_MTT);
	MLX5_SET(mkc, mkc, qpn, 0xffffff);
	MLX5_SET(mkc, mkc, length64, 0x0);
	MLX5_SET(mkc, mkc, pd, mkey_attr->pd);
	MLX5_SET(mkc, mkc, mkey_7_0, 0x50);//FIXME: should be dynamic
	MLX5_SET(mkc, mkc, translations_octword_size, ((mkey_attr->size / 4096)+1)/2);
	MLX5_SET(create_mkey_in, in, translations_octword_actual_size, ((mkey_attr->size / 4096)+1)/2);
	MLX5_SET(create_mkey_in, in, pg_access, 1);

#if 1
	MLX5_SET(mkc, mkc, pas_umem_id, mkey_attr->pas_id);
	MLX5_SET64(mkc, mkc, start_addr,mkey_attr->addr);
	MLX5_SET64(mkc, mkc, len, mkey_attr->size);
	MLX5_SET(mkc, mkc, log_page_size, 12);
#else
	struct devx_obj_handle *mem;
	uint32_t mem_id;
	void *buff;

	buff = mmap(NULL, 0x1000, PROT_READ|PROT_WRITE, MAP_PRIVATE|MAP_ANON, -1, 0);
	mem = devx_umem_reg(priv->ctx, buff, 0x1000, 7, &mem_id);
	printf("%s:%d %p %d %d\n", __func__, __LINE__, mem, errno, mem_id);

	MLX5_SET64(mkc, mkc, start_addr,(intptr_t) buff);
	MLX5_SET64(mkc, mkc, len, 0x1000);
	MLX5_SET(mkc, mkc, log_page_size, log2(0x1000));
	MLX5_SET(mkc, mkc, pas_umem_id, mem_id);
#endif

	mkey->obj = devx_obj_create(priv->ctx, in, size, out, sizeof(out));

	if(!mkey->obj) {
		printf("Can't create mkey error %d\n", errno);
		return NULL;
	}
	status = MLX5_GET(create_mkey_out, out, status);
	mkey->key = MLX5_GET(create_mkey_out, out, mkey_index);
	mkey->key = (mkey->key<<8) | 0x50;
	printf("oooOri create mkey status %d mkey value %d\n",status, (mkey->key << 8) | 0x50 );
	if(status)
		return NULL;



	return mkey;
}

static int
mlx5_mdev_query_hca_cap_gen(void *ctx, struct mlx5_mdev_cap *caps)
{
	uint32_t in[MLX5_ST_SZ_DW(query_hca_cap_in) + MLX5_ST_SZ_DW(cmd_pas) * 2] = {0};
	uint32_t out[MLX5_ST_SZ_DW(query_hca_cap_out)] = {0};
	int status, syndrome, err;

	MLX5_SET(query_hca_cap_in, in, opcode, MLX5_CMD_OP_QUERY_HCA_CAP);
	MLX5_SET(query_hca_cap_in, in, op_mod,
		 (MLX5_CAP_GENERAL << 1) |
		 (MLX5_HCA_CAP_OPMOD_GET_CUR & 0x1));

	err = devx_cmd(ctx, in, sizeof(in), out, sizeof(out));
	if (err) {
		printf("query HCA capabilities failed %d", errno);
		return err;
	}
	status = MLX5_GET(query_hca_cap_out, out, status);
	syndrome = MLX5_GET(query_hca_cap_out, out, syndrome);
	printf("mdev_priv_query_hca_cap status %x, syndrome = %x\n",status, syndrome);
	if (!status)
		rte_memcpy(caps->gen,
		           MLX5_ADDR_OF(query_hca_cap_out, out, capability),
		           sizeof(caps->gen));
	return status;
}

static int
mlx5_mdev_query_offload_cap(void *ctx, struct mlx5_mdev_cap *caps)
{
	uint32_t in[MLX5_ST_SZ_DW(query_hca_cap_in) + MLX5_ST_SZ_DW(cmd_pas) * 2] = {0};
	uint32_t out[MLX5_ST_SZ_DW(query_hca_cap_out)] = {0};
	int status, syndrome, err;

	MLX5_SET(query_hca_cap_in, in, opcode, MLX5_CMD_OP_QUERY_HCA_CAP);
	MLX5_SET(query_hca_cap_in, in, op_mod,
		 (MLX5_CAP_ETHERNET_OFFLOADS << 1) |
		 (MLX5_HCA_CAP_OPMOD_GET_CUR & 0x1));

	err = devx_cmd(ctx, in, sizeof(in), out, sizeof(out));
	if (err) {
		printf("query offload capabilities failed %d", errno);
		return err;
	}
	status = MLX5_GET(query_hca_cap_out, out, status);
	syndrome = MLX5_GET(query_hca_cap_out, out, syndrome);
	printf("mlx5_mdev_query_offload_cap status %x, syndrome = %x\n",status, syndrome);
	if (!status)
		rte_memcpy(caps->eth,
		           MLX5_ADDR_OF(query_hca_cap_out, out, capability),
		           sizeof(caps->eth));
	return status;
}

static int
mlx5_mdev_query_hca_cap_ftn(void *ctx, struct mlx5_mdev_cap *caps)
{
	uint32_t in[MLX5_ST_SZ_DW(query_hca_cap_in) + MLX5_ST_SZ_DW(cmd_pas) * 2] = {0};
	uint32_t out[MLX5_ST_SZ_DW(query_hca_cap_out)] = {0};
	int status, syndrome, err;

	MLX5_SET(query_hca_cap_in, in, opcode, MLX5_CMD_OP_QUERY_HCA_CAP);
	MLX5_SET(query_hca_cap_in, in, op_mod,
		 (MLX5_CAP_FLOW_TABLE << 1) |
		 (MLX5_HCA_CAP_OPMOD_GET_CUR & 0x1));
	err = devx_cmd(ctx, in, sizeof(in), out, sizeof(out));
	if (err) {
		ERROR("query flow capabilities failed %d", errno);
		return err;
	}
	status = MLX5_GET(query_hca_cap_out, out, status);
	syndrome = MLX5_GET(query_hca_cap_out, out, syndrome);
	printf("mlx5_mdev_query_hca_cap_ftn status %x, syndrome = %x, sizeof %lu\n",
		status, syndrome, sizeof(caps->ftn));
	if(!status)
		rte_memcpy(caps->ftn,
		           MLX5_ADDR_OF(query_hca_cap_out, out, capability),
		           sizeof(caps->ftn));
	return status;
}


int
mlx5_mdev_query_hca_cap(void *ctx, struct mlx5_mdev_cap *caps)
{
	if (mlx5_mdev_query_hca_cap_gen(ctx, caps) ||
	    mlx5_mdev_query_offload_cap(ctx, caps) ||
 	    mlx5_mdev_query_hca_cap_ftn(ctx, caps) ||
	    0)
		return -EFAULT;

	return 0;
}

int
mlx5_mdev_check_hca_cap(struct mlx5_mdev_cap *caps)
{
	if ((MLX5_CAP_GEN(caps, port_type) != MLX5_CAP_PORT_TYPE_ETH) ||
	    (MLX5_CAP_GEN(caps, num_ports) > 1)	||
	    (MLX5_CAP_GEN(caps, cqe_version) != 1) ||
	    0) {
		RTE_LOG(ERR, PMD, "mlx5_mdev_check_hca_cap failed\n");
		return -EOPNOTSUPP;
	}

	return 0;
}

int
mlx5_mdev_query_vport_state(void *ctx, uint8_t *admin_state, uint8_t *state)
{
	uint32_t in[MLX5_ST_SZ_DW(query_vport_state_in) + MLX5_ST_SZ_DW(cmd_pas) * 2] = {0};
	uint32_t out[MLX5_ST_SZ_DW(query_vport_state_out)] = {0};
	int status, syndrome, err;

	MLX5_SET(query_vport_state_in, in, opcode,
		 MLX5_CMD_OP_QUERY_VPORT_STATE);
	MLX5_SET(query_vport_state_in, in, op_mod,
		 MLX5_QUERY_VPORT_STATE_IN_OP_MOD_VNIC_VPORT);
	MLX5_SET(query_vport_state_in, in, other_vport, 0);

	err = devx_cmd(ctx, in, sizeof(in), out, sizeof(out));
	if (err) {
		printf("query HCA capabilities failed %d", errno);
		return err;
	}
	status = MLX5_GET(query_vport_state_out, out, status);
	syndrome = MLX5_GET(query_vport_state_out, out, syndrome);
	printf("mlx5_mdev_query_vport_state status %x, syndrome = %x\n",status, syndrome);
	if (!status) {
		*admin_state = MLX5_GET(query_vport_state_out, out, admin_state);
		*state = MLX5_GET(query_vport_state_out, out, state);
	}
	return status;
}
