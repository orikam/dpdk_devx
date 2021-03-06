#   BSD LICENSE
#
#   Copyright 2015 Mellanox.
#
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in
#       the documentation and/or other materials provided with the
#       distribution.
#     * Neither the name of 6WIND S.A. nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

include $(RTE_SDK)/mk/rte.vars.mk

# Library name.
LIB = librte_pmd_mlx5_mdev.a

# Sources.

SRCS-$(CONFIG_RTE_LIBRTE_MLX5_MDEV_PMD) += mlx5_mdev.c
SRCS-$(CONFIG_RTE_LIBRTE_MLX5_MDEV_PMD) += mdev_lib.c
SRCS-$(CONFIG_RTE_LIBRTE_MLX5_MDEV_PMD) += mlx5_prm_commands.c
SRCS-$(CONFIG_RTE_LIBRTE_MLX5_MDEV_PMD) += mlx5_mdev_ethdev.c
SRCS-$(CONFIG_RTE_LIBRTE_MLX5_MDEV_PMD) += mlx5_mdev_mac.c
SRCS-$(CONFIG_RTE_LIBRTE_MLX5_MDEV_PMD) += mlx5_mdev_txq.c
SRCS-$(CONFIG_RTE_LIBRTE_MLX5_MDEV_PMD) += mlx5_mdev_mr.c
SRCS-$(CONFIG_RTE_LIBRTE_MLX5_MDEV_PMD) += mlx5_mdev_rxtx.c
SRCS-$(CONFIG_RTE_LIBRTE_MLX5_MDEV_PMD) += mlx5_mdev_trigger.c
SRCS-$(CONFIG_RTE_LIBRTE_MLX5_MDEV_PMD) += mlx5_mdev_stats.c
SRCS-$(CONFIG_RTE_LIBRTE_MLX5_MDEV_PMD) += cmd_ioctl.c
SRCS-$(CONFIG_RTE_LIBRTE_MLX5_MDEV_PMD) += devx.c
SRCS-$(CONFIG_RTE_LIBRTE_MLX5_MDEV_PMD) += devx_uar.c
SRCS-$(CONFIG_RTE_LIBRTE_MLX5_MDEV_PMD) += devx_dbrec.c
SRCS-$(CONFIG_RTE_LIBRTE_MLX5_MDEV_PMD) += rdma_core/device.c
SRCS-$(CONFIG_RTE_LIBRTE_MLX5_MDEV_PMD) += rdma_core/cmd.c
SRCS-$(CONFIG_RTE_LIBRTE_MLX5_MDEV_PMD) += rdma_core/init.c
SRCS-$(CONFIG_RTE_LIBRTE_MLX5_MDEV_PMD) += rdma_core/sysfs.c
SRCS-$(CONFIG_RTE_LIBRTE_MLX5_MDEV_PMD) += rdma_core/mlx5.c
SRCS-$(CONFIG_RTE_LIBRTE_MLX5_MDEV_PMD) += rdma_core/ccan/str.c
SRCS-$(CONFIG_RTE_LIBRTE_MLX5_MDEV_PMD) += rdma_core/ccan/list.c

# Basic CFLAGS.
CFLAGS += -O3
CFLAGS += -std=c11 -Wall -Wextra
CFLAGS += -g
CFLAGS += -I.
CFLAGS += -I$(SRCDIR)
CFLAGS += -I$(SRCDIR)/rdma_core
CFLAGS += -D_BSD_SOURCE
CFLAGS += -D_DEFAULT_SOURCE
CFLAGS += -D_XOPEN_SOURCE=600
CFLAGS += $(WERROR_FLAGS)
CFLAGS += -Wno-strict-prototypes
LDLIBS += -libverbs

# A few warnings cannot be avoided in external headers.
CFLAGS += -Wno-cast-qual
CFLAGS += -Wno-unused-parameter
CFLAGS += -Wno-sign-compare
CFLAGS += -Wno-pointer-arith

EXPORT_MAP := rte_pmd_mlx5_mdev_version.map
LIBABIVER := 1

# DEBUG which is usually provided on the command-line may enable
# CONFIG_RTE_LIBRTE_MLX5_MDEV_DEBUG.
ifeq ($(DEBUG),1)
CONFIG_RTE_LIBRTE_MLX5_MDEV_DEBUG := y
endif

# User-defined CFLAGS.
ifeq ($(CONFIG_RTE_LIBRTE_MLX5_MDEV_DEBUG),y)
CFLAGS += -UNDEBUG -DPEDANTIC
else
CFLAGS += -DNDEBUG -UPEDANTIC
endif

include $(RTE_SDK)/mk/rte.lib.mk

# Generate and clean-up mlx5_mdev_autoconf.h.

export CC CFLAGS CPPFLAGS EXTRA_CFLAGS EXTRA_CPPFLAGS
export AUTO_CONFIG_CFLAGS = -Wno-error

ifndef V
AUTOCONF_OUTPUT := >/dev/null
endif
