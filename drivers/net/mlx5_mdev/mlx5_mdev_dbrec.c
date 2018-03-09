//#include <stdint.h>
//#include <stdlib.h>
//#include <errno.h>
//#include <string.h>

#include <rte_io.h>
#include <rte_pci.h>
#include <rte_log.h>
#include <rte_malloc.h>
#include <rte_memzone.h>
#include <rte_ethdev.h>

#include <rte_ether.h>
#include <rte_ethdev.h>
#include <rte_ethdev_pci.h>
#include <rte_eth_ctrl.h>
#include "mlx5_mdev_priv.h"

static int32_t ffsl(int64_t value)
{
	int i = 0;

	for (i=0;i < 64; i++) {
		if (((value >> i) & 0x01) == 1)
			return i + 1;
	}
	return 0;
}


int64_t mlx5_get_dbrec(struct mlx5_mdev_priv *priv)
{
	struct mlx5_mdev_db_page *page = (struct mlx5_mdev_db_page *)priv->db_page;
	int i, j;
	uint64_t db_rec;
	if (!page) {
		int ps = 4096; // todo detirmine what size to put here. priv->page_size;
		int pp = ps / priv->cache_line_size;
		int nlong = (pp + 8 * sizeof(long) - 1) / (8 * sizeof(long));
		int i;

		pp = ps / priv->cache_line_size;
		nlong = (pp + 8 * sizeof(long) - 1) / (8 * sizeof(long));
		priv->db_page =
			rte_zmalloc_socket(__func__,
					   sizeof *page + nlong * sizeof(long),
					   priv->cache_line_size,
					   priv->edev->device->numa_node);
		if(!priv->db_page) {
			return 0;
		}
		page = priv->db_page;
		for (i = 0; i < nlong; ++i)
			page->free_records[i] = ~0;
		page->use_cnt = 0;
		page->num_db = pp;
		page->rte_mz = rte_eth_dma_zone_reserve(priv->edev,
				"dbrec_page", 0, ps, ps,
				priv->edev->device->numa_node);
		if (!page->rte_mz) {
			rte_free(priv->db_page);
			return 0;
		}
#if 0 // fixme: what is this ?
		rte_malloc_socket(__func__, ps, ps,
		 priv->edev->device->numa_node);
#endif
	}

	printf("mlx5_get_dbrec num_db %d", page->num_db);

	if(page->use_cnt > page->num_db) {
		return 0; // TODO -ENOSPC ?
	}
	++page->use_cnt;

	for (i = 0; !page->free_records[i]; ++i)
		/* nothing */;

	j = ffsl(page->free_records[i]);
	--j;
	page->free_records[i] &= ~(1UL << j);
	db_rec = (uint64_t)page->rte_mz->iova + (i * 8 * sizeof(long) + j) * priv->cache_line_size;
	return db_rec;
}

