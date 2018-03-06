/*-
 *   BSD LICENSE
 *
 *   Copyright 2017 Mellanox.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of 6WIND S.A. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __PRM_H_
#define __PRM_H_

#ifndef u8
#define u8          uint8_t
#endif
#ifndef __be16
#define __be16          uint16_t
#endif
#ifndef __be32
#define __be32          uint32_t
#endif
#ifndef __be64
#define __be64          uint64_t
#endif

struct mlx5_cmd_block {
	u8              data[512];
	u8              rsvd0[48];
	__be64          next;
	__be32          block_num;
	u8              rsvd1;
	u8              token;
	u8              ctrl_sig;
	u8              sig;
};

struct mlx5_cmdq_entry {
	u8		type;
	u8		rsvd0[3];
	__be32		ilen;
	__be64		iptr;
	__be32		in[4];
	__be32		out[4];
	__be64		optr;
	__be32		olen;
	u8		token;
	u8		sig;
	u8		rsvd1;
	u8		status_own;
};

struct mlx5_iseg {
	__be32			fw_rev;
	__be16			cmdif_rev;
	__be16			fw_rev_sub;
	__be32			rsvd0[2];
	__be32			cmdq_pa_h;
	__be32			cmdq_pa_l_sz;
	__be32			cmd_dbell;
	__be32			rsvd1[120];
	__be32			initializing;
	__be32			health[16];
	__be32			rsvd2[880];
	__be32			internal_timer_h;
	__be32			internal_timer_l;
	__be32			rsvd3[2];
	__be32			health_counter;
	__be32			rsvd4[1019];
	__be64			ieee1588_clk;
	__be32			ieee1588_clk_type;
	__be32			clr_intx;
};

enum {
	MLX5_CMD_OP_QUERY_HCA_CAP                 = 0x100,
	MLX5_CMD_OP_INIT_HCA                      = 0x102,
	MLX5_CMD_OP_TEARDOWN_HCA                  = 0x103,
	MLX5_CMD_OP_ENABLE_HCA                    = 0x104,
	MLX5_CMD_OP_DISABLE_HCA                   = 0x105,
	MLX5_CMD_OP_MANAGE_PAGES		  = 0x108,
	MLX5_CMD_OP_QUERY_ISSI                    = 0x10a,
	MLX5_CMD_OP_SET_ISSI                      = 0x10b,
	MLX5_CMD_OP_CREATE_MKEY                   = 0x200,
	MLX5_CMD_OP_DESTROY_MKEY                  = 0x202,
	MLX5_CMD_OP_CREATE_EQ                     = 0x301,
	MLX5_CMD_OP_DESTROY_EQ                    = 0x302,
	MLX5_CMD_OP_CREATE_CQ                     = 0x400,
	MLX5_CMD_OP_DESTROY_CQ                    = 0x401,
	MLX5_CMD_OP_QUERY_NIC_VPORT_CONTEXT       = 0x754,
	MLX5_CMD_OP_ALLOC_PD                      = 0x800,
	MLX5_CMD_OP_DEALLOC_PD                    = 0x801,
	MLX5_CMD_OP_ALLOC_UAR                     = 0x802,
	MLX5_CMD_OP_DEALLOC_UAR                   = 0x803,
	MLX5_CMD_OP_ALLOC_TRANSPORT_DOMAIN        = 0x816,
	MLX5_CMD_OP_DEALLOC_TRANSPORT_DOMAIN      = 0x817,
	MLX5_CMD_OP_CREATE_SQ			  = 0x904,
	MLX5_CMD_OP_CREATE_RQ                     = 0x908,
	MLX5_CMD_OP_MODIFY_RQ                     = 0x909,
	MLX5_CMD_OP_DESTROY_RQ                    = 0x90a,
	MLX5_CMD_OP_CREATE_TIS                    = 0x912,
	MLX5_CMD_OP_DESTROY_TIS                   = 0x914,
	MLX5_CMD_OP_CREATE_RQT                    = 0x916,
	MLX5_CMD_OP_MODIFY_RQT                    = 0x917,
	MLX5_CMD_OP_DESTROY_RQT                   = 0x918,
};

struct mlx5_ifc_enable_hca_out_bits {
	u8         status[0x8];
	u8         reserved_at_8[0x18];

	u8         syndrome[0x20];

	u8         reserved_at_40[0x20];
};

struct mlx5_ifc_enable_hca_in_bits {
	u8         opcode[0x10];
	u8         reserved_at_10[0x10];

	u8         reserved_at_20[0x10];
	u8         op_mod[0x10];

	u8         reserved_at_40[0x10];
	u8         function_id[0x10];

	u8         reserved_at_60[0x20];
};

struct mlx5_ifc_disable_hca_out_bits {
	u8         status[0x8];
	u8         reserved_at_8[0x18];

	u8         syndrome[0x20];

	u8         reserved_at_40[0x20];
};

struct mlx5_ifc_disable_hca_in_bits {
	u8         opcode[0x10];
	u8         reserved_at_10[0x10];

	u8         reserved_at_20[0x10];
	u8         op_mod[0x10];

	u8         reserved_at_40[0x10];
	u8         function_id[0x10];

	u8         reserved_at_60[0x20];
};

struct mlx5_ifc_query_issi_out_bits {
	u8         status[0x8];
	u8         reserved_at_8[0x18];

	u8         syndrome[0x20];

	u8         reserved_at_40[0x10];
	u8         current_issi[0x10];

	u8         reserved_at_60[0xa0];

	u8         reserved_at_100[76][0x8];
	u8         supported_issi_dw0[0x20];
};

struct mlx5_ifc_query_issi_in_bits {
	u8         opcode[0x10];
	u8         reserved_at_10[0x10];

	u8         reserved_at_20[0x10];
	u8         op_mod[0x10];

	u8         reserved_at_40[0x40];
};

struct mlx5_ifc_set_issi_out_bits {
	u8         status[0x8];
	u8         reserved_at_8[0x18];

	u8         syndrome[0x20];

	u8         reserved_at_40[0x40];
};

struct mlx5_ifc_set_issi_in_bits {
	u8         opcode[0x10];
	u8         reserved_at_10[0x10];

	u8         reserved_at_20[0x10];
	u8         op_mod[0x10];

	u8         reserved_at_40[0x10];
	u8         current_issi[0x10];

	u8         reserved_at_60[0x20];
};

enum {
	MLX5_CAP_PORT_TYPE_IB  = 0x0,
	MLX5_CAP_PORT_TYPE_ETH = 0x1,
};

struct mlx5_ifc_cmd_hca_cap_bits {
	u8         reserved_at_0[0x80];

	u8         log_max_srq_sz[0x8];
	u8         log_max_qp_sz[0x8];
	u8         reserved_at_90[0xb];
	u8         log_max_qp[0x5];

	u8         reserved_at_a0[0xb];
	u8         log_max_srq[0x5];
	u8         reserved_at_b0[0x10];

	u8         reserved_at_c0[0x8];
	u8         log_max_cq_sz[0x8];
	u8         reserved_at_d0[0xb];
	u8         log_max_cq[0x5];

	u8         log_max_eq_sz[0x8];
	u8         reserved_at_e8[0x2];
	u8         log_max_mkey[0x6];
	u8         reserved_at_f0[0xc];
	u8         log_max_eq[0x4];

	u8         max_indirection[0x8];
	u8         fixed_buffer_size[0x1];
	u8         log_max_mrw_sz[0x7];
	u8         force_teardown[0x1];
	u8         reserved_at_111[0x1];
	u8         log_max_bsf_list_size[0x6];
	u8         umr_extended_translation_offset[0x1];
	u8         null_mkey[0x1];
	u8         log_max_klm_list_size[0x6];

	u8         reserved_at_120[0xa];
	u8         log_max_ra_req_dc[0x6];
	u8         reserved_at_130[0xa];
	u8         log_max_ra_res_dc[0x6];

	u8         reserved_at_140[0xa];
	u8         log_max_ra_req_qp[0x6];
	u8         reserved_at_150[0xa];
	u8         log_max_ra_res_qp[0x6];

	u8         end_pad[0x1];
	u8         cc_query_allowed[0x1];
	u8         cc_modify_allowed[0x1];
	u8         start_pad[0x1];
	u8         cache_line_128byte[0x1];
	u8         reserved_at_165[0xb];
	u8         gid_table_size[0x10];

	u8         out_of_seq_cnt[0x1];
	u8         vport_counters[0x1];
	u8         retransmission_q_counters[0x1];
	u8         reserved_at_183[0x1];
	u8         modify_rq_counter_set_id[0x1];
	u8         rq_delay_drop[0x1];
	u8         max_qp_cnt[0xa];
	u8         pkey_table_size[0x10];

	u8         vport_group_manager[0x1];
	u8         vhca_group_manager[0x1];
	u8         ib_virt[0x1];
	u8         eth_virt[0x1];
	u8         reserved_at_1a4[0x1];
	u8         ets[0x1];
	u8         nic_flow_table[0x1];
	u8         eswitch_flow_table[0x1];
	u8	   early_vf_enable[0x1];
	u8         mcam_reg[0x1];
	u8         pcam_reg[0x1];
	u8         local_ca_ack_delay[0x5];
	u8         port_module_event[0x1];
	u8         enhanced_error_q_counters[0x1];
	u8         ports_check[0x1];
	u8         reserved_at_1b3[0x1];
	u8         disable_link_up[0x1];
	u8         beacon_led[0x1];
	u8         port_type[0x2];
	u8         num_ports[0x8];

	u8         reserved_at_1c0[0x1];
	u8         pps[0x1];
	u8         pps_modify[0x1];
	u8         log_max_msg[0x5];
	u8         reserved_at_1c8[0x4];
	u8         max_tc[0x4];
	u8         reserved_at_1d0[0x1];
	u8         dcbx[0x1];
	u8         general_notification_event[0x1];
	u8         reserved_at_1d3[0x2];
	u8         fpga[0x1];
	u8         rol_s[0x1];
	u8         rol_g[0x1];
	u8         reserved_at_1d8[0x1];
	u8         wol_s[0x1];
	u8         wol_g[0x1];
	u8         wol_a[0x1];
	u8         wol_b[0x1];
	u8         wol_m[0x1];
	u8         wol_u[0x1];
	u8         wol_p[0x1];

	u8         stat_rate_support[0x10];
	u8         reserved_at_1f0[0xc];
	u8         cqe_version[0x4];

	u8         compact_address_vector[0x1];
	u8         striding_rq[0x1];
	u8         reserved_at_202[0x1];
	u8         ipoib_enhanced_offloads[0x1];
	u8         ipoib_basic_offloads[0x1];
	u8         reserved_at_205[0x5];
	u8         umr_fence[0x2];
	u8         reserved_at_20c[0x3];
	u8         drain_sigerr[0x1];
	u8         cmdif_checksum[0x2];
	u8         sigerr_cqe[0x1];
	u8         reserved_at_213[0x1];
	u8         wq_signature[0x1];
	u8         sctr_data_cqe[0x1];
	u8         reserved_at_216[0x1];
	u8         sho[0x1];
	u8         tph[0x1];
	u8         rf[0x1];
	u8         dct[0x1];
	u8         qos[0x1];
	u8         eth_net_offloads[0x1];
	u8         roce[0x1];
	u8         atomic[0x1];
	u8         reserved_at_21f[0x1];

	u8         cq_oi[0x1];
	u8         cq_resize[0x1];
	u8         cq_moderation[0x1];
	u8         reserved_at_223[0x3];
	u8         cq_eq_remap[0x1];
	u8         pg[0x1];
	u8         block_lb_mc[0x1];
	u8         reserved_at_229[0x1];
	u8         scqe_break_moderation[0x1];
	u8         cq_period_start_from_cqe[0x1];
	u8         cd[0x1];
	u8         reserved_at_22d[0x1];
	u8         apm[0x1];
	u8         vector_calc[0x1];
	u8         umr_ptr_rlky[0x1];
	u8	   imaicl[0x1];
	u8         reserved_at_232[0x4];
	u8         qkv[0x1];
	u8         pkv[0x1];
	u8         set_deth_sqpn[0x1];
	u8         reserved_at_239[0x3];
	u8         xrc[0x1];
	u8         ud[0x1];
	u8         uc[0x1];
	u8         rc[0x1];

	u8         uar_4k[0x1];
	u8         reserved_at_241[0x9];
	u8         uar_sz[0x6];
	u8         reserved_at_250[0x8];
	u8         log_pg_sz[0x8];

	u8         bf[0x1];
	u8         driver_version[0x1];
	u8         pad_tx_eth_packet[0x1];
	u8         reserved_at_263[0x8];
	u8         log_bf_reg_size[0x5];

	u8         reserved_at_270[0xb];
	u8         lag_master[0x1];
	u8         num_lag_ports[0x4];

	u8         reserved_at_280[0x10];
	u8         max_wqe_sz_sq[0x10];

	u8         reserved_at_2a0[0x10];
	u8         max_wqe_sz_rq[0x10];

	u8         max_flow_counter_31_16[0x10];
	u8         max_wqe_sz_sq_dc[0x10];

	u8         reserved_at_2e0[0x7];
	u8         max_qp_mcg[0x19];

	u8         reserved_at_300[0x18];
	u8         log_max_mcg[0x8];

	u8         reserved_at_320[0x3];
	u8         log_max_transport_domain[0x5];
	u8         reserved_at_328[0x3];
	u8         log_max_pd[0x5];
	u8         reserved_at_330[0xb];
	u8         log_max_xrcd[0x5];

	u8         reserved_at_340[0x8];
	u8         log_max_flow_counter_bulk[0x8];
	u8         max_flow_counter_15_0[0x10];


	u8         reserved_at_360[0x3];
	u8         log_max_rq[0x5];
	u8         reserved_at_368[0x3];
	u8         log_max_sq[0x5];
	u8         reserved_at_370[0x3];
	u8         log_max_tir[0x5];
	u8         reserved_at_378[0x3];
	u8         log_max_tis[0x5];

	u8         basic_cyclic_rcv_wqe[0x1];
	u8         reserved_at_381[0x2];
	u8         log_max_rmp[0x5];
	u8         reserved_at_388[0x3];
	u8         log_max_rqt[0x5];
	u8         reserved_at_390[0x3];
	u8         log_max_rqt_size[0x5];
	u8         reserved_at_398[0x3];
	u8         log_max_tis_per_sq[0x5];

	u8         reserved_at_3a0[0x3];
	u8         log_max_stride_sz_rq[0x5];
	u8         reserved_at_3a8[0x3];
	u8         log_min_stride_sz_rq[0x5];
	u8         reserved_at_3b0[0x3];
	u8         log_max_stride_sz_sq[0x5];
	u8         reserved_at_3b8[0x3];
	u8         log_min_stride_sz_sq[0x5];

	u8         reserved_at_3c0[0x1b];
	u8         log_max_wq_sz[0x5];

	u8         nic_vport_change_event[0x1];
	u8         disable_local_lb[0x1];
	u8         reserved_at_3e2[0x9];
	u8         log_max_vlan_list[0x5];
	u8         reserved_at_3f0[0x3];
	u8         log_max_current_mc_list[0x5];
	u8         reserved_at_3f8[0x3];
	u8         log_max_current_uc_list[0x5];

	u8         reserved_at_400[0x80];

	u8         reserved_at_480[0x3];
	u8         log_max_l2_table[0x5];
	u8         reserved_at_488[0x8];
	u8         log_uar_page_sz[0x10];

	u8         reserved_at_4a0[0x20];
	u8         device_frequency_mhz[0x20];
	u8         device_frequency_khz[0x20];

	u8         reserved_at_500[0x20];
	u8	   num_of_uars_per_page[0x20];
	u8         reserved_at_540[0x40];

	u8         reserved_at_580[0x3f];
	u8         cqe_compression[0x1];

	u8         cqe_compression_timeout[0x10];
	u8         cqe_compression_max_num[0x10];

	u8         reserved_at_5e0[0x10];
	u8         tag_matching[0x1];
	u8         rndv_offload_rc[0x1];
	u8         rndv_offload_dc[0x1];
	u8         log_tag_matching_list_sz[0x5];
	u8         reserved_at_5f8[0x3];
	u8         log_max_xrq[0x5];

	u8         reserved_at_600[0x200];
};

struct mlx5_ifc_per_protocol_networking_offload_caps_bits {
	u8         csum_cap[0x1];
	u8         vlan_cap[0x1];
	u8         lro_cap[0x1];
	u8         lro_psh_flag[0x1];
	u8         lro_time_stamp[0x1];
	u8         reserved_at_5[0x2];
	u8         wqe_vlan_insert[0x1];
	u8         self_lb_en_modifiable[0x1];
	u8         self_lb_mc[0x1];
	u8         self_lb_uc[0x1];
	u8         max_lso_cap[0x5];
	u8         multi_pkt_send_wqe[0x2];
	u8	   wqe_inline_mode[0x2];
	u8         rss_ind_tbl_cap[0x4];
	u8         reg_umr_sq[0x1];
	u8         scatter_fcs[0x1];
	u8         enhanced_multi_pkt_send_wqe[0x1];
	u8         tunnel_lso_const_out_ip_id[0x1];
	u8         reserved_at_1c[0x2];
	u8         tunnel_stateless_gre[0x1];
	u8         tunnel_stateless_vxlan[0x1];

	u8         swp[0x1];
	u8         swp_csum[0x1];
	u8         swp_lso[0x1];
	u8         reserved_at_23[0x1d];

	u8         reserved_at_40[0x10];
	u8         lro_min_mss_size[0x10];

	u8         reserved_at_60[0x120];

	u8         lro_timer_supported_periods[4][0x20];

	u8         reserved_at_200[0x600];
};

struct mlx5_ifc_flow_table_fields_supported_bits {
	u8         outer_dmac[0x1];
	u8         outer_smac[0x1];
	u8         outer_ether_type[0x1];
	u8         outer_ip_version[0x1];
	u8         outer_first_prio[0x1];
	u8         outer_first_cfi[0x1];
	u8         outer_first_vid[0x1];
	u8         outer_ipv4_ttl[0x1];
	u8         outer_second_prio[0x1];
	u8         outer_second_cfi[0x1];
	u8         outer_second_vid[0x1];
	u8         reserved_at_b[0x1];
	u8         outer_sip[0x1];
	u8         outer_dip[0x1];
	u8         outer_frag[0x1];
	u8         outer_ip_protocol[0x1];
	u8         outer_ip_ecn[0x1];
	u8         outer_ip_dscp[0x1];
	u8         outer_udp_sport[0x1];
	u8         outer_udp_dport[0x1];
	u8         outer_tcp_sport[0x1];
	u8         outer_tcp_dport[0x1];
	u8         outer_tcp_flags[0x1];
	u8         outer_gre_protocol[0x1];
	u8         outer_gre_key[0x1];
	u8         outer_vxlan_vni[0x1];
	u8         reserved_at_1a[0x5];
	u8         source_eswitch_port[0x1];

	u8         inner_dmac[0x1];
	u8         inner_smac[0x1];
	u8         inner_ether_type[0x1];
	u8         inner_ip_version[0x1];
	u8         inner_first_prio[0x1];
	u8         inner_first_cfi[0x1];
	u8         inner_first_vid[0x1];
	u8         inner_ipv4_ttl[0x1];
	u8         inner_second_prio[0x1];
	u8         inner_second_cfi[0x1];
	u8         inner_second_vid[0x1];
	u8         reserved_at_2b[0x1];
	u8         inner_sip[0x1];
	u8         inner_dip[0x1];
	u8         inner_frag[0x1];
	u8         inner_ip_protocol[0x1];
	u8         inner_ip_ecn[0x1];
	u8         inner_ip_dscp[0x1];
	u8         inner_udp_sport[0x1];
	u8         inner_udp_dport[0x1];
	u8         inner_tcp_sport[0x1];
	u8         inner_tcp_dport[0x1];
	u8         inner_tcp_flags[0x1];
	u8         reserved_at_37[0x9];
	u8         reserved_at_40[0x1a];
	u8         bth_dst_qp[0x1];

	u8         reserved_at_5b[0x25];
};

struct mlx5_ifc_flow_table_prop_layout_bits {
	u8         ft_support[0x1];
	u8         reserved_at_1[0x1];
	u8         flow_counter[0x1];
	u8	   flow_modify_en[0x1];
	u8         modify_root[0x1];
	u8         identified_miss_table_mode[0x1];
	u8         flow_table_modify[0x1];
	u8         encap[0x1];
	u8         decap[0x1];
	u8         reserved_at_9[0x17];

	u8         reserved_at_20[0x2];
	u8         log_max_ft_size[0x6];
	u8         log_max_modify_header_context[0x8];
	u8         max_modify_header_actions[0x8];
	u8         max_ft_level[0x8];

	u8         reserved_at_40[0x20];

	u8         reserved_at_60[0x18];
	u8         log_max_ft_num[0x8];

	u8         reserved_at_80[0x18];
	u8         log_max_destination[0x8];

	u8         reserved_at_a0[0x18];
	u8         log_max_flow[0x8];

	u8         reserved_at_c0[0x40];

	struct mlx5_ifc_flow_table_fields_supported_bits ft_field_support;

	struct mlx5_ifc_flow_table_fields_supported_bits ft_field_bitmask_support;
};

struct mlx5_ifc_flow_table_nic_cap_bits {
	u8         nic_rx_multi_path_tirs[0x1];
	u8         nic_rx_multi_path_tirs_fts[0x1];
	u8         allow_sniffer_and_nic_rx_shared_tir[0x1];
	u8         reserved_at_3[0x1fd];

	struct mlx5_ifc_flow_table_prop_layout_bits nic_rx;

	u8         reserved_at_400[0x200];

	struct mlx5_ifc_flow_table_prop_layout_bits nic_rx_sniffer;

	struct mlx5_ifc_flow_table_prop_layout_bits nic_tx;

	u8         reserved_at_a00[0x200];

	struct mlx5_ifc_flow_table_prop_layout_bits nic_tx_sniffer;

	u8         reserved_at_e00[0x7200];
};

union mlx5_ifc_hca_cap_union_bits {
	struct mlx5_ifc_cmd_hca_cap_bits cmd_hca_cap;
	struct mlx5_ifc_per_protocol_networking_offload_caps_bits per_protocol_networking_offload_caps;
	struct mlx5_ifc_flow_table_nic_cap_bits flow_table_nic_cap; //oooOri todo: should add memory
	u8         reserved_at_0[0x8000];
};

struct mlx5_ifc_query_hca_cap_in_bits {
	u8         opcode[0x10];
	u8         reserved_at_10[0x10];

	u8         reserved_at_20[0x10];
	u8         op_mod[0x10];

	u8         reserved_at_40[0x40];
};

struct mlx5_ifc_query_hca_cap_out_bits {
	u8         status[0x8];
	u8         reserved_at_8[0x18];

	u8         syndrome[0x20];

	u8         reserved_at_40[0x40];

	union mlx5_ifc_hca_cap_union_bits capability;
};

struct mlx5_ifc_init_hca_out_bits {
	u8         status[0x8];
	u8         reserved_at_8[0x18];

	u8         syndrome[0x20];

	u8         reserved_at_40[0x40];
};

struct mlx5_ifc_init_hca_in_bits {
	u8         opcode[0x10];
	u8         reserved_at_10[0x10];

	u8         reserved_at_20[0x10];
	u8         op_mod[0x10];

	u8         reserved_at_40[0x40];
};

struct mlx5_ifc_teardown_hca_out_bits {
	u8         status[0x8];
	u8         reserved_at_8[0x18];

	u8         syndrome[0x20];

	u8         reserved_at_40[0x3f];

	u8         force_state[0x1];
};

struct mlx5_ifc_teardown_hca_in_bits {
	u8         opcode[0x10];
	u8         reserved_at_10[0x10];

	u8         reserved_at_20[0x10];
	u8         op_mod[0x10];

	u8         reserved_at_40[0x10];
	u8         profile[0x10];

	u8         reserved_at_60[0x20];
};

struct mlx5_ifc_query_pages_out_bits {
	u8         status[0x8];
	u8         reserved_at_8[0x18];

	u8         syndrome[0x20];

	u8	   embedded_cpu_function[0x01];
	u8	   reserved_bits[0x0f];
	u8	   function_id[0x10];

	u8	   num_pages[0x20];
};

struct mlx5_ifc_query_pages_in_bits {
	u8	opcode[0x10];
	u8	reserved_at_10[0x10];

	u8	reserved_at_20[0x10];
	u8	op_mod[0x10];

	u8	reserved_at_40[0x10];
	u8	function_id[0x10];

	u8	reserved_at_60[0x20];
};

struct mlx5_ifc_manage_pages_out_bits {
	u8	status[0x8];
	u8	reserved_at_8[0x18];

	u8	syndrome[0x20];

	u8	output_num_entries[0x20];

	u8	reserved_at_60[0x20];

	__be64	pas[0x20];
};

struct mlx5_ifc_manage_pages_in_bits {
	u8	opcode[0x10];
	u8	reserved_at_10[0x10];

	u8	reserved_at_20[0x10];
	u8	op_mod[0x10];

	u8	embedded_cpu_function[0x01];
	u8	reserved_bits[0x0f];
	u8	function_id[0x10];

	u8	input_num_entries[0x20];
	__be64	pas[0x20];
};


enum {
	MLX5_HCA_CAP_OPMOD_GET_MAX	= 0,
	MLX5_HCA_CAP_OPMOD_GET_CUR	= 1,
};

enum {
	MLX5_CAP_GENERAL = 0,
	MLX5_CAP_ETHERNET_OFFLOADS,
	MLX5_CAP_ODP,
	MLX5_CAP_ATOMIC,
	MLX5_CAP_ROCE,
	MLX5_CAP_IPOIB_OFFLOADS,
	MLX5_CAP_IPOIB_ENHANCED_OFFLOADS,
	MLX5_CAP_FLOW_TABLE,
	MLX5_CAP_ESWITCH_FLOW_TABLE,
	MLX5_CAP_ESWITCH,
	MLX5_CAP_RESERVED,
	MLX5_CAP_VECTOR_CALC,
	MLX5_CAP_QOS,
	MLX5_CAP_FPGA,
	/* NUM OF CAP Types */
	MLX5_CAP_NUM
};

struct mlx5_ifc_mac_address_layout_bits {
	u8         reserved_at_0[0x10];
	u8         mac_addr_47_32[0x10];

	u8         mac_addr_31_0[0x20];
};

struct mlx5_ifc_nic_vport_context_bits {
	u8         reserved_at_0[0x5];
	u8         min_wqe_inline_mode[0x3];
	u8         reserved_at_8[0x15];
	u8         disable_mc_local_lb[0x1];
	u8         disable_uc_local_lb[0x1];
	u8         roce_en[0x1];

	u8         arm_change_event[0x1];
	u8         reserved_at_21[0x1a];
	u8         event_on_mtu[0x1];
	u8         event_on_promisc_change[0x1];
	u8         event_on_vlan_change[0x1];
	u8         event_on_mc_address_change[0x1];
	u8         event_on_uc_address_change[0x1];

	u8         reserved_at_40[0xf0];

	u8         mtu[0x10];

	u8         system_image_guid[0x40];
	u8         port_guid[0x40];
	u8         node_guid[0x40];

	u8         reserved_at_200[0x140];
	u8         qkey_violation_counter[0x10];
	u8         reserved_at_350[0x430];

	u8         promisc_uc[0x1];
	u8         promisc_mc[0x1];
	u8         promisc_all[0x1];
	u8         reserved_at_783[0x2];
	u8         allowed_list_type[0x3];
	u8         reserved_at_788[0xc];
	u8         allowed_list_size[0xc];

	struct mlx5_ifc_mac_address_layout_bits permanent_address;

	u8         reserved_at_7e0[0x20];

	u8         current_uc_mac_address[0x40];
};

struct mlx5_ifc_query_nic_vport_context_out_bits {
	u8         status[0x8];
	u8         reserved_at_8[0x18];

	u8         syndrome[0x20];

	u8         reserved_at_40[0x40];

	struct mlx5_ifc_nic_vport_context_bits nic_vport_context;
};

struct mlx5_ifc_query_nic_vport_context_in_bits {
	u8         opcode[0x10];
	u8         reserved_at_10[0x10];

	u8         reserved_at_20[0x10];
	u8         op_mod[0x10];

	u8         other_vport[0x1];
	u8         reserved_at_41[0xf];
	u8         vport_number[0x10];

	u8         reserved_at_60[0x5];
	u8         allowed_list_type[0x3];
	u8         reserved_at_68[0x18];
};

struct mlx5_ifc_alloc_pd_out_bits {
	u8         status[0x8];
	u8         reserved_at_8[0x18];

	u8         syndrome[0x20];

	u8         reserved_at_40[0x8];
	u8         pd[0x18];

	u8         reserved_at_60[0x20];
};

struct mlx5_ifc_alloc_pd_in_bits {
	u8         opcode[0x10];
	u8         reserved_at_10[0x10];

	u8         reserved_at_20[0x10];
	u8         op_mod[0x10];

	u8         reserved_at_40[0x40];
};

struct mlx5_ifc_dealloc_pd_out_bits {
	u8         status[0x8];
	u8         reserved_at_8[0x18];

	u8         syndrome[0x20];

	u8         reserved_at_40[0x40];
};

struct mlx5_ifc_dealloc_pd_in_bits {
	u8         opcode[0x10];
	u8         reserved_at_10[0x10];

	u8         reserved_at_20[0x10];
	u8         op_mod[0x10];

	u8         reserved_at_40[0x8];
	u8         pd[0x18];

	u8         reserved_at_60[0x20];
};

struct mlx5_ifc_alloc_uar_out_bits {
	u8         status[0x8];
	u8         reserved_at_8[0x18];

	u8         syndrome[0x20];

	u8         reserved_at_40[0x8];
	u8         uar[0x18];

	u8         reserved_at_60[0x20];
};

struct mlx5_ifc_alloc_uar_in_bits {
	u8         opcode[0x10];
	u8         reserved_at_10[0x10];

	u8         reserved_at_20[0x10];
	u8         op_mod[0x10];

	u8         reserved_at_40[0x40];
};

struct mlx5_ifc_dealloc_uar_out_bits {
	u8         status[0x8];
	u8         reserved_at_8[0x18];

	u8         syndrome[0x20];

	u8         reserved_at_40[0x40];
};

struct mlx5_ifc_dealloc_uar_in_bits {
	u8         opcode[0x10];
	u8         reserved_at_10[0x10];

	u8         reserved_at_20[0x10];
	u8         op_mod[0x10];

	u8         reserved_at_40[0x8];
	u8         uar[0x18];

	u8         reserved_at_60[0x20];
};

struct mlx5_ifc_alloc_transport_domain_out_bits {
	u8         status[0x8];
	u8         reserved_at_8[0x18];

	u8         syndrome[0x20];

	u8         reserved_at_40[0x8];
	u8         transport_domain[0x18];

	u8         reserved_at_60[0x20];
};

struct mlx5_ifc_alloc_transport_domain_in_bits {
	u8         opcode[0x10];
	u8         reserved_at_10[0x10];

	u8         reserved_at_20[0x10];
	u8         op_mod[0x10];

	u8         reserved_at_40[0x40];
};

struct mlx5_ifc_dealloc_transport_domain_out_bits {
	u8         status[0x8];
	u8         reserved_at_8[0x18];

	u8         syndrome[0x20];

	u8         reserved_at_40[0x40];
};

struct mlx5_ifc_dealloc_transport_domain_in_bits {
	u8         opcode[0x10];
	u8         reserved_at_10[0x10];

	u8         reserved_at_20[0x10];
	u8         op_mod[0x10];

	u8         reserved_at_40[0x8];
	u8         transport_domain[0x18];

	u8         reserved_at_60[0x20];
};

enum {
	MLX5_MKC_ACCESS_MODE_PA    = 0x0,
	MLX5_MKC_ACCESS_MODE_MTT   = 0x1,
	MLX5_MKC_ACCESS_MODE_KLMS  = 0x2,
	MLX5_MKC_ACCESS_MODE_KSM   = 0x3,
};

struct mlx5_ifc_mkc_bits {
	u8         reserved_at_0[0x1];
	u8         free[0x1];
	u8         reserved_at_2[0xd];
	u8         small_fence_on_rdma_read_response[0x1];
	u8         umr_en[0x1];
	u8         a[0x1];
	u8         rw[0x1];
	u8         rr[0x1];
	u8         lw[0x1];
	u8         lr[0x1];
	u8         access_mode[0x2];
	u8         reserved_at_18[0x8];

	u8         qpn[0x18];
	u8         mkey_7_0[0x8];

	u8         reserved_at_40[0x20];

	u8         length64[0x1];
	u8         bsf_en[0x1];
	u8         sync_umr[0x1];
	u8         reserved_at_63[0x2];
	u8         expected_sigerr_count[0x1];
	u8         reserved_at_66[0x1];
	u8         en_rinval[0x1];
	u8         pd[0x18];

	u8         start_addr[0x40];

	u8         len[0x40];

	u8         bsf_octword_size[0x20];

	u8         reserved_at_120[0x80];

	u8         translations_octword_size[0x20];

	u8         reserved_at_1c0[0x1b];
	u8         log_page_size[0x5];

	u8         pas_umem_id[0x20];
};

struct mlx5_ifc_create_mkey_out_bits {
	u8         status[0x8];
	u8         reserved_at_8[0x18];

	u8         syndrome[0x20];

	u8         reserved_at_40[0x8];
	u8         mkey_index[0x18];

	u8         reserved_at_60[0x20];
};

struct mlx5_ifc_create_mkey_in_bits {
	u8         opcode[0x10];
	u8         reserved_at_10[0x10];

	u8         reserved_at_20[0x10];
	u8         op_mod[0x10];

	u8         reserved_at_40[0x20];

	u8         pg_access[0x1];
	u8         reserved_at_61[0x1f];

	struct mlx5_ifc_mkc_bits ctx;

	u8         reserved_at_280[0x80];

	u8         translations_octword_actual_size[0x20];

	u8         reserved_at_320[0x560];

	u8         klm_pas_mtt[0][0x20];
};

struct mlx5_ifc_destroy_mkey_out_bits {
	u8         status[0x8];
	u8         reserved_at_8[0x18];

	u8         syndrome[0x20];

	u8         reserved_at_40[0x40];
};

struct mlx5_ifc_destroy_mkey_in_bits {
	u8         opcode[0x10];
	u8         reserved_at_10[0x10];

	u8         reserved_at_20[0x10];
	u8         op_mod[0x10];

	u8         reserved_at_40[0x8];
	u8         mkey_index[0x18];

	u8         reserved_at_60[0x20];
};

struct mlx5_ifc_tisc_bits {
	u8         strict_lag_tx_port_affinity[0x1];
	u8         reserved_at_1[0x3];
	u8         lag_tx_port_affinity[0x04];

	u8         reserved_at_8[0x4];
	u8         prio[0x4];
	u8         reserved_at_10[0x10];

	u8         reserved_at_20[0x100];

	u8         reserved_at_120[0x8];
	u8         transport_domain[0x18];

	u8         reserved_at_140[0x8];
	u8         underlay_qpn[0x18];
	u8         reserved_at_160[0x3a0];
};

struct mlx5_ifc_create_tis_out_bits {
	u8         status[0x8];
	u8         reserved_at_8[0x18];

	u8         syndrome[0x20];

	u8         reserved_at_40[0x8];
	u8         tisn[0x18];

	u8         reserved_at_60[0x20];
};

struct mlx5_ifc_create_tis_in_bits {
	u8         opcode[0x10];
	u8         reserved_at_10[0x10];

	u8         reserved_at_20[0x10];
	u8         op_mod[0x10];

	u8         reserved_at_40[0xc0];

	struct mlx5_ifc_tisc_bits ctx;
};

struct mlx5_ifc_destroy_tis_out_bits {
	u8         status[0x8];
	u8         reserved_at_8[0x18];

	u8         syndrome[0x20];

	u8         reserved_at_40[0x40];
};

struct mlx5_ifc_destroy_tis_in_bits {
	u8         opcode[0x10];
	u8         reserved_at_10[0x10];

	u8         reserved_at_20[0x10];
	u8         op_mod[0x10];

	u8         reserved_at_40[0x8];
	u8         tisn[0x18];

	u8         reserved_at_60[0x20];
};

struct mlx5_ifc_eqc_bits {
	u8         status[0x4];
	u8         reserved_at_4[0x9];
	u8         ec[0x1];
	u8         oi[0x1];
	u8         reserved_at_f[0x5];
	u8         st[0x4];
	u8         reserved_at_18[0x8];

	u8         reserved_at_20[0x20];

	u8         reserved_at_40[0x14];
	u8         page_offset[0x6];
	u8         reserved_at_5a[0x6];

	u8         reserved_at_60[0x3];
	u8         log_eq_size[0x5];
	u8         uar_page[0x18];

	u8         reserved_at_80[0x20];

	u8         reserved_at_a0[0x18];
	u8         intr[0x8];

	u8         reserved_at_c0[0x3];
	u8         log_page_size[0x5];
	u8         reserved_at_c8[0x18];

	u8         reserved_at_e0[0x60];

	u8         reserved_at_140[0x8];
	u8         consumer_counter[0x18];

	u8         reserved_at_160[0x8];
	u8         producer_counter[0x18];

	u8         reserved_at_180[0x60];

	u8         pas_umem_id[0x20];
};

struct mlx5_ifc_create_eq_out_bits {
	u8         status[0x8];
	u8         reserved_at_8[0x18];

	u8         syndrome[0x20];

	u8         reserved_at_40[0x18];
	u8         eqn[0x8];

	u8         reserved_at_60[0x20];
};

struct mlx5_ifc_create_eq_in_bits {
	u8         opcode[0x10];
	u8         reserved_at_10[0x10];

	u8         reserved_at_20[0x10];
	u8         op_mod[0x10];

	u8         reserved_at_40[0x40];

	struct mlx5_ifc_eqc_bits ctx;

	u8         reserved_at_280[0x40];

	u8         event_bitmask[0x40];

	u8         reserved_at_300[0x580];

	u8         pas[0x40];
};

struct mlx5_ifc_destroy_eq_out_bits {
	u8         status[0x8];
	u8         reserved_at_8[0x18];

	u8         syndrome[0x20];

	u8         reserved_at_40[0x40];
};

struct mlx5_ifc_destroy_eq_in_bits {
	u8         opcode[0x10];
	u8         reserved_at_10[0x10];

	u8         reserved_at_20[0x10];
	u8         op_mod[0x10];

	u8         reserved_at_40[0x18];
	u8         eqn[0x8];

	u8         reserved_at_60[0x20];
};

struct mlx5_ifc_cqc_bits {
	u8         status[0x4];
	u8         reserved_at_4[0x4];
	u8         cqe_sz[0x3];
	u8         cc[0x1];
	u8         reserved_at_c[0x1];
	u8         scqe_break_moderation_en[0x1];
	u8         oi[0x1];
	u8         cq_period_mode[0x2];
	u8         cqe_comp_en[0x1];
	u8         mini_cqe_res_format[0x2];
	u8         st[0x4];
	u8         reserved_at_18[0x8];

	u8         reserved_at_20[0x20];

	u8         reserved_at_40[0x14];
	u8         page_offset[0x6];
	u8         reserved_at_5a[0x6];

	u8         reserved_at_60[0x3];
	u8         log_cq_size[0x5];
	u8         uar_page[0x18];

	u8         reserved_at_80[0x4];
	u8         cq_period[0xc];
	u8         cq_max_count[0x10];

	u8         reserved_at_a0[0x18];
	u8         c_eqn[0x8];

	u8         reserved_at_c0[0x3];
	u8         log_page_size[0x5];
	u8         reserved_at_c8[0x18];

	u8         reserved_at_e0[0x20];

	u8         reserved_at_100[0x8];
	u8         last_notified_index[0x18];

	u8         reserved_at_120[0x8];
	u8         last_solicit_index[0x18];

	u8         reserved_at_140[0x8];
	u8         consumer_counter[0x18];

	u8         reserved_at_160[0x8];
	u8         producer_counter[0x18];

	u8         pas_umem_id[0x20];

	u8         dbr_umem_id[0x20];

	u8         dbr_addr[0x40];
};

struct mlx5_ifc_create_cq_out_bits {
	u8         status[0x8];
	u8         reserved_at_8[0x18];

	u8         syndrome[0x20];

	u8         reserved_at_40[0x8];
	u8         cqn[0x18];

	u8         reserved_at_60[0x20];
};

struct mlx5_ifc_create_cq_in_bits {
	u8         opcode[0x10];
	u8         reserved_at_10[0x10];

	u8         reserved_at_20[0x10];
	u8         op_mod[0x10];

	u8         reserved_at_40[0x40];

	struct mlx5_ifc_cqc_bits ctx;

	u8         reserved_at_280[0x600];

	__be64     pas[32];
};

struct mlx5_ifc_destroy_cq_out_bits {
	u8         status[0x8];
	u8         reserved_at_8[0x18];

	u8         syndrome[0x20];

	u8         reserved_at_40[0x40];
};

struct mlx5_ifc_destroy_cq_in_bits {
	u8         opcode[0x10];
	u8         reserved_at_10[0x10];

	u8         reserved_at_20[0x10];
	u8         op_mod[0x10];

	u8         reserved_at_40[0x8];
	u8         cqn[0x18];

	u8         reserved_at_60[0x20];
};

enum {
	MLX5_WQ_WQ_TYPE_WQ_LINKED_LIST  = 0x0,
	MLX5_WQ_WQ_TYPE_WQ_CYCLIC       = 0x1,
};

enum {
	MLX5_WQ_END_PADDING_MODE_END_PAD_NONE   = 0x0,
	MLX5_WQ_END_PADDING_MODE_END_PAD_ALIGN  = 0x1,
};

struct mlx5_ifc_cmd_pas_bits {
	u8         pa_h[0x20];

	u8         pa_l[0x14];
	u8         reserved_at_34[0xc];
};

struct mlx5_ifc_wq_bits {
	u8         wq_type[0x4];
	u8         wq_signature[0x1];
	u8         end_padding_mode[0x2];
	u8         cd_slave[0x1];
	u8         reserved_at_8[0x18];

	u8         hds_skip_first_sge[0x1];
	u8         log2_hds_buf_size[0x3];
	u8         reserved_at_24[0x7];
	u8         page_offset[0x5];
	u8         lwm[0x10];

	u8         reserved_at_40[0x8];
	u8         pd[0x18];

	u8         reserved_at_60[0x8];
	u8         uar_page[0x18];

	u8         dbr_addr[0x40];

	u8         hw_counter[0x20];

	u8         sw_counter[0x20];

	u8         reserved_at_100[0xc];
	u8         log_wq_stride[0x4];
	u8         reserved_at_110[0x3];
	u8         log_wq_pg_sz[0x5];
	u8         reserved_at_118[0x3];
	u8         log_wq_sz[0x5];

	u8         reserved_at_120[0x15];
	u8         log_wqe_num_of_strides[0x3];
	u8         two_byte_shift_en[0x1];
	u8         reserved_at_139[0x4];
	u8         log_wqe_stride_size[0x3];

	u8         reserved_at_140[0x480];

	u8         pas_umem_id[0x20];

	u8         dbr_umem_id[0x20];

	struct mlx5_ifc_cmd_pas_bits pas[16];
};

enum {
	MLX5_RQC_MEM_RQ_TYPE_MEMORY_RQ_INLINE  = 0x0,
	MLX5_RQC_MEM_RQ_TYPE_MEMORY_RQ_RMP     = 0x1,
};

enum {
	MLX5_RQC_STATE_RST  = 0x0,
	MLX5_RQC_STATE_RDY  = 0x1,
	MLX5_RQC_STATE_ERR  = 0x3,
};

struct mlx5_ifc_rqc_bits {
	u8         rlky[0x1];
	u8	   delay_drop_en[0x1];
	u8         scatter_fcs[0x1];
	u8         vsd[0x1];
	u8         mem_rq_type[0x4];
	u8         state[0x4];
	u8         reserved_at_c[0x1];
	u8         flush_in_error_en[0x1];
	u8         reserved_at_e[0x12];

	u8         reserved_at_20[0x8];
	u8         user_index[0x18];

	u8         reserved_at_40[0x8];
	u8         cqn[0x18];

	u8         counter_set_id[0x8];
	u8         reserved_at_68[0x18];

	u8         reserved_at_80[0x8];
	u8         rmpn[0x18];

	u8         reserved_at_a0[0xe0];

	struct mlx5_ifc_wq_bits wq;
};

struct mlx5_ifc_sqc_bits {
	u8         rlky[0x1];
	u8         cd_master[0x1];
	u8         fre[0x1];
	u8         flush_in_error_en[0x1];
	u8         allow_multi_pkt_send_wqe[0x1];
	u8	   min_wqe_inline_mode[0x3];
	u8         state[0x4];
	u8         reg_umr[0x1];
	u8         allow_swp[0x1];
	u8         reserved_at_e[0x12];

	u8         reserved_at_20[0x8];
	u8         user_index[0x18];

	u8         reserved_at_40[0x8];
	u8         cqn[0x18];

	u8         reserved_at_60[0x90];

	u8         packet_pacing_rate_limit_index[0x10];
	u8         tis_lst_sz[0x10];
	u8         reserved_at_110[0x10];

	u8         reserved_at_120[0x40];

	u8         reserved_at_160[0x8];
	u8         tis_num_0[0x18];

	struct mlx5_ifc_wq_bits wq;
};

struct mlx5_ifc_create_rq_out_bits {
	u8         status[0x8];
	u8         reserved_at_8[0x18];

	u8         syndrome[0x20];

	u8         reserved_at_40[0x8];
	u8         rqn[0x18];

	u8         reserved_at_60[0x20];
};

struct mlx5_ifc_create_rq_in_bits {
	u8         opcode[0x10];
	u8         reserved_at_10[0x10];

	u8         reserved_at_20[0x10];
	u8         op_mod[0x10];

	u8         reserved_at_40[0xc0];

	struct mlx5_ifc_rqc_bits ctx;
};

struct mlx5_ifc_modify_rq_out_bits {
	u8         status[0x8];
	u8         reserved_at_8[0x18];

	u8         syndrome[0x20];

	u8         reserved_at_40[0x40];
};

struct mlx5_ifc_create_sq_out_bits {
	u8         status[0x8];
	u8         reserved_at_8[0x18];

	u8         syndrome[0x20];

	u8         reserved_at_40[0x8];
	u8         sqn[0x18];

	u8         reserved_at_60[0x20];
};

struct mlx5_ifc_create_sq_in_bits {
	u8         opcode[0x10];
	u8         reserved_at_10[0x10];

	u8         reserved_at_20[0x10];
	u8         op_mod[0x10];

	u8         reserved_at_40[0xc0];

	struct mlx5_ifc_sqc_bits ctx;
};
enum {
	MLX5_MODIFY_RQ_IN_MODIFY_BITMASK_VSD = 1ULL << 1,
	MLX5_MODIFY_RQ_IN_MODIFY_BITMASK_SCATTER_FCS = 1ULL << 2,
	MLX5_MODIFY_RQ_IN_MODIFY_BITMASK_RQ_COUNTER_SET_ID = 1ULL << 3,
};

struct mlx5_ifc_modify_rq_in_bits {
	u8         opcode[0x10];
	u8         reserved_at_10[0x10];

	u8         reserved_at_20[0x10];
	u8         op_mod[0x10];

	u8         rq_state[0x4];
	u8         reserved_at_44[0x4];
	u8         rqn[0x18];

	u8         reserved_at_60[0x20];

	u8         modify_bitmask[0x40];

	u8         reserved_at_c0[0x40];

	struct mlx5_ifc_rqc_bits ctx;
};

struct mlx5_ifc_destroy_rq_out_bits {
	u8         status[0x8];
	u8         reserved_at_8[0x18];

	u8         syndrome[0x20];

	u8         reserved_at_40[0x40];
};

struct mlx5_ifc_destroy_rq_in_bits {
	u8         opcode[0x10];
	u8         reserved_at_10[0x10];

	u8         reserved_at_20[0x10];
	u8         op_mod[0x10];

	u8         reserved_at_40[0x8];
	u8         rqn[0x18];

	u8         reserved_at_60[0x20];
};

struct mlx5_ifc_rqtc_bits {
	u8         reserved_at_0[0xa0];

	u8         reserved_at_a0[0x10];
	u8         rqt_max_size[0x10];

	u8         reserved_at_c0[0x10];
	u8         rqt_actual_size[0x10];

	u8         reserved_at_e0[0x6a0];

	__be32     rq_num[128];
};

struct mlx5_ifc_create_rqt_out_bits {
	u8         status[0x8];
	u8         reserved_at_8[0x18];

	u8         syndrome[0x20];

	u8         reserved_at_40[0x8];
	u8         rqtn[0x18];

	u8         reserved_at_60[0x20];
};

struct mlx5_ifc_create_rqt_in_bits {
	u8         opcode[0x10];
	u8         reserved_at_10[0x10];

	u8         reserved_at_20[0x10];
	u8         op_mod[0x10];

	u8         reserved_at_40[0xc0];

	struct mlx5_ifc_rqtc_bits ctx;
};

struct mlx5_ifc_modify_rqt_out_bits {
	u8         status[0x8];
	u8         reserved_at_8[0x18];

	u8         syndrome[0x20];

	u8         reserved_at_40[0x40];
};

struct mlx5_ifc_rqt_bitmask_bits {
	u8	   reserved_at_0[0x20];

	u8         reserved_at_20[0x1f];
	u8         rqn_list[0x1];
};

struct mlx5_ifc_modify_rqt_in_bits {
	u8         opcode[0x10];
	u8         reserved_at_10[0x10];

	u8         reserved_at_20[0x10];
	u8         op_mod[0x10];

	u8         reserved_at_40[0x8];
	u8         rqtn[0x18];

	u8         reserved_at_60[0x20];

	struct mlx5_ifc_rqt_bitmask_bits bitmask;

	u8         reserved_at_c0[0x40];

	struct mlx5_ifc_rqtc_bits ctx;
};

struct mlx5_ifc_destroy_rqt_out_bits {
	u8         status[0x8];
	u8         reserved_at_8[0x18];

	u8         syndrome[0x20];

	u8         reserved_at_40[0x40];
};

struct mlx5_ifc_destroy_rqt_in_bits {
	u8         opcode[0x10];
	u8         reserved_at_10[0x10];

	u8         reserved_at_20[0x10];
	u8         op_mod[0x10];

	u8         reserved_at_40[0x8];
	u8         rqtn[0x18];

	u8         reserved_at_60[0x20];
};

struct mlx5_cqe64 {
	u8		outer_l3_tunneled;
	u8		rsvd0;
	__be16		wqe_id;
	u8		lro_tcppsh_abort_dupack;
	u8		lro_min_ttl;
	__be16		lro_tcp_win;
	__be32		lro_ack_seq_num;
	__be32		rss_hash_result;
	u8		rss_hash_type;
	u8		ml_path;
	u8		rsvd20[2];
	__be16		check_sum;
	__be16		slid;
	__be32		flags_rqpn;
	u8		hds_ip_ext;
	u8		l4_l3_hdr_type;
	__be16		vlan_info;
	__be32		srqn; /* [31:24]: lro_num_seg, [23:0]: srqn */
	__be32		imm_inval_pkey;
	u8		rsvd40[4];
	__be32		byte_cnt;
	__be32		timestamp_h;
	__be32		timestamp_l;
	__be32		sop_drop_qpn;
	__be16		wqe_counter;
	u8		signature;
	u8		op_own;
};

/* helper macros */
#define __mlx5_nullp(typ) ((struct mlx5_ifc_##typ##_bits *)0)
#define __mlx5_bit_sz(typ, fld) sizeof(__mlx5_nullp(typ)->fld)
#define __mlx5_bit_off(typ, fld) (offsetof(struct mlx5_ifc_##typ##_bits, fld))
#define __mlx5_dw_off(typ, fld) (__mlx5_bit_off(typ, fld) / 32)
#define __mlx5_64_off(typ, fld) (__mlx5_bit_off(typ, fld) / 64)
#define __mlx5_dw_bit_off(typ, fld) (32 - __mlx5_bit_sz(typ, fld) - (__mlx5_bit_off(typ, fld) & 0x1f))
#define __mlx5_mask(typ, fld) ((uint32_t)((1ull << __mlx5_bit_sz(typ, fld)) - 1))
#define __mlx5_dw_mask(typ, fld) (__mlx5_mask(typ, fld) << __mlx5_dw_bit_off(typ, fld))
#define __mlx5_st_sz_bits(typ) sizeof(struct mlx5_ifc_##typ##_bits)

#define MLX5_FLD_SZ_BYTES(typ, fld) (__mlx5_bit_sz(typ, fld) / 8)
#define MLX5_ST_SZ_BYTES(typ) (sizeof(struct mlx5_ifc_##typ##_bits) / 8)
#define MLX5_ST_SZ_DW(typ) (sizeof(struct mlx5_ifc_##typ##_bits) / 32)
#define MLX5_ST_SZ_QW(typ) (sizeof(struct mlx5_ifc_##typ##_bits) / 64)
#define MLX5_UN_SZ_BYTES(typ) (sizeof(union mlx5_ifc_##typ##_bits) / 8)
#define MLX5_UN_SZ_DW(typ) (sizeof(union mlx5_ifc_##typ##_bits) / 32)
#define MLX5_BYTE_OFF(typ, fld) (__mlx5_bit_off(typ, fld) / 8)
#define MLX5_ADDR_OF(typ, p, fld) ((char *)(p) + MLX5_BYTE_OFF(typ, fld))

#define BUILD_BUG_ON(a) /*TODO*/
/* insert a value to a struct */
#define MLX5_SET(typ, p, fld, v) do { \
	uint32_t _v = v; \
	BUILD_BUG_ON(__mlx5_st_sz_bits(typ) % 32);             \
	*((__be32 *)(p) + __mlx5_dw_off(typ, fld)) = \
	rte_cpu_to_be_32((rte_be_to_cpu_32(*((__be32 *)(p) + __mlx5_dw_off(typ, fld))) & \
		     (~__mlx5_dw_mask(typ, fld))) | (((_v) & __mlx5_mask(typ, fld)) \
		     << __mlx5_dw_bit_off(typ, fld))); \
} while (0)

#define MLX5_SET_TO_ONES(typ, p, fld) do { \
	BUILD_BUG_ON(__mlx5_st_sz_bits(typ) % 32);             \
	*((__be32 *)(p) + __mlx5_dw_off(typ, fld)) = \
	rte_cpu_to_be32((rte_be_to_cpu_32(*((__be32 *)(p) + __mlx5_dw_off(typ, fld))) & \
		     (~__mlx5_dw_mask(typ, fld))) | ((__mlx5_mask(typ, fld)) \
		     << __mlx5_dw_bit_off(typ, fld))); \
} while (0)

#define MLX5_GET(typ, p, fld) ((rte_be_to_cpu_32(*((__be32 *)(p) +\
__mlx5_dw_off(typ, fld))) >> __mlx5_dw_bit_off(typ, fld)) & \
__mlx5_mask(typ, fld))

#define MLX5_GET_PR(typ, p, fld) ({ \
	uint32_t ___t = MLX5_GET(typ, p, fld); \
	pr_debug(#fld " = 0x%x\n", ___t); \
	___t; \
})

#define __MLX5_SET64(typ, p, fld, v) do { \
	BUILD_BUG_ON(__mlx5_bit_sz(typ, fld) != 64); \
	*((__be64 *)(p) + __mlx5_64_off(typ, fld)) = rte_cpu_to_be_64(v); \
} while (0)

#define MLX5_SET64(typ, p, fld, v) do { \
	BUILD_BUG_ON(__mlx5_bit_off(typ, fld) % 64); \
	__MLX5_SET64(typ, p, fld, v); \
} while (0)

#define MLX5_ARRAY_SET64(typ, p, fld, idx, v) do { \
	BUILD_BUG_ON(__mlx5_bit_off(typ, fld) % 64); \
	__MLX5_SET64(typ, p, fld[idx], v); \
} while (0)

#define MLX5_GET64(typ, p, fld) rte_be_64_to_cpu(*((__be64 *)(p) +\
						   __mlx5_64_off(typ, fld)))

#define MLX5_GET64_PR(typ, p, fld) ({ \
	u64 ___t = MLX5_GET64(typ, p, fld); \
	pr_debug(#fld " = 0x%llx\n", ___t); \
	___t; \
})

/* Big endian getters */
#define MLX5_GET64_BE(typ, p, fld) (*((__be64 *)(p) +\
	__mlx5_64_off(typ, fld)))

#define MLX5_GET_BE(type_t, typ, p, fld) ({				  \
		type_t tmp;						  \
		switch (sizeof(tmp)) {					  \
		case sizeof(uint8_t):					  \
			tmp = (__force type_t)MLX5_GET(typ, p, fld);	  \
			break;						  \
		case sizeof(uint16_t):					  \
			tmp = (__force type_t)rte_cpu_to_be16(MLX5_GET(typ, p, fld)); \
			break;						  \
		case sizeof(uint32_t):					  \
			tmp = (__force type_t)rte_cpu_to_be32(MLX5_GET(typ, p, fld)); \
			break;						  \
		case sizeof(uint64_t):					  \
			tmp = (__force type_t)MLX5_GET64_BE(typ, p, fld); \
			break;						  \
			}						  \
		tmp;							  \
		})
#endif
