/**************************************************************************

Copyright (c) 2016 - 2021, Intel Corporation

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Intel Corporation nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

***************************************************************************/

#ifndef _ICE_FLEX_PIPE_H_
#define _ICE_FLEX_PIPE_H_

#include "ice_type.h"

/* Package minimal version supported */
#define ICE_PKG_SUPP_VER_MAJ	1
#define ICE_PKG_SUPP_VER_MNR	3

/* Package format version */
#define ICE_PKG_FMT_VER_MAJ	1
#define ICE_PKG_FMT_VER_MNR	0
#define ICE_PKG_FMT_VER_UPD	0
#define ICE_PKG_FMT_VER_DFT	0

#define ICE_PKG_CNT 4

#ifndef LINUX_SUPPORT
enum ice_status
ice_update_pkg(struct ice_hw *hw, struct ice_buf *bufs, u32 count);
#endif /* !LINUX_SUPPORT */
#ifdef QV_SUPPORT
void ice_release_global_cfg_lock(struct ice_hw *hw);
struct ice_generic_seg_hdr *
ice_find_seg_in_pkg(struct ice_hw *hw, u32 seg_type,
		    struct ice_pkg_hdr *pkg_hdr);
enum ice_ddp_state
ice_verify_pkg(struct ice_pkg_hdr *pkg, u32 len);
enum ice_ddp_state
ice_get_pkg_info(struct ice_hw *hw);
void ice_init_pkg_hints(struct ice_hw *hw, struct ice_seg *ice_seg);
struct ice_buf_table *ice_find_buf_table(struct ice_seg *ice_seg);
enum ice_status
ice_acquire_global_cfg_lock(struct ice_hw *hw,
			    enum ice_aq_res_access_type access);
#endif /* QV_SUPPORT */
#ifndef NO_ADV_SW_SUPPORT
enum ice_status
ice_acquire_change_lock(struct ice_hw *hw, enum ice_aq_res_access_type access);
void ice_release_change_lock(struct ice_hw *hw);
#endif /* NO_ADV_SW_SUPPORT */
#ifndef NO_UNUSED_PACKAGE_CODE
enum ice_status
ice_find_prot_off(struct ice_hw *hw, enum ice_block blk, u8 prof, u16 fv_idx,
		  u8 *prot, u16 *off);
enum ice_status
ice_find_label_value(struct ice_seg *ice_seg, char const *name, u32 type,
		     u16 *value);
void
ice_get_sw_fv_bitmap(struct ice_hw *hw, enum ice_prof_type type,
		     ice_bitmap_t *bm);
void
ice_init_prof_result_bm(struct ice_hw *hw);
enum ice_status
ice_get_sw_fv_list(struct ice_hw *hw, u8 *prot_ids, u16 ids_cnt,
		   ice_bitmap_t *bm, struct LIST_HEAD_TYPE *fv_list);
enum ice_status
ice_pkg_buf_unreserve_section(struct ice_buf_build *bld, u16 count);
u16 ice_pkg_buf_get_free_space(struct ice_buf_build *bld);
#endif /* !NO_UNUSED_PACKAGE_CODE */
enum ice_status
ice_aq_upload_section(struct ice_hw *hw, struct ice_buf_hdr *pkg_buf,
		      u16 buf_size, struct ice_sq_cd *cd);
#if defined(FDIR_SUPPORT) || !defined(NO_UNUSED_TUNNEL_CODE)
#ifndef ICE_TDD
bool
ice_get_open_tunnel_port(struct ice_hw *hw, enum ice_tunnel_type type,
			 u16 *port);
#endif
#endif /* FDIR_SUPPORT || !NO_UNUSED_TUNNEL_CODE */
#ifdef LINUX_SUPPORT
enum ice_status
ice_is_create_tunnel_possible(struct ice_hw *hw, enum ice_tunnel_type type,
			      u16 port);
#ifdef DCF_SUPPORT
bool ice_is_tunnel_empty(struct ice_hw *hw);
#endif
#endif
enum ice_status
ice_create_tunnel(struct ice_hw *hw, enum ice_tunnel_type type, u16 port);
#ifdef DVM_SUPPORT
enum ice_status ice_set_dvm_boost_entries(struct ice_hw *hw);
#endif /* DVM_SUPPORT */
enum ice_status ice_destroy_tunnel(struct ice_hw *hw, u16 port, bool all);
#if !defined(LINUX_SUPPORT) || !defined(NO_ADV_SW_SUPPORT)
bool ice_tunnel_port_in_use(struct ice_hw *hw, u16 port, u16 *index);
#endif
#ifndef NO_UNUSED_TUNNEL_CODE
bool
ice_tunnel_get_type(struct ice_hw *hw, u16 port, enum ice_tunnel_type *type);
#ifndef NO_UNUSED_CODE
enum ice_status ice_replay_tunnels(struct ice_hw *hw);
#endif /* !NO_UNUSED_CODE */
#endif /* NO_UNUSED_TUNNEL_CODE */

#if defined(DPDK_SUPPORT) || defined(ADV_AVF_SUPPORT)
/* RX parser PType functions */
bool ice_hw_ptype_ena(struct ice_hw *hw, u16 ptype);
#endif /* DPDK_SUPPORT || ADV_AVF_SUPPORT */

#ifndef NO_UNUSED_CODE
/* XLT1/PType group functions */
enum ice_status ice_ptg_update_xlt1(struct ice_hw *hw, enum ice_block blk);
void ice_ptg_free(struct ice_hw *hw, enum ice_block blk, u8 ptg);
#endif /* !NO_UNUSED_CODE */

/* XLT2/VSI group functions */
#ifndef ICE_TDD
#ifndef NO_UNUSED_CODE
enum ice_status ice_vsig_update_xlt2(struct ice_hw *hw, enum ice_block blk);
#endif /* !NO_UNUSED_CODE */
#ifndef LINUX_SUPPORT
enum ice_status
ice_vsig_find_vsi(struct ice_hw *hw, enum ice_block blk, u16 vsi, u16 *vsig);
#endif /* !LINUX_SUPPORT */
#ifdef DPDK_SUPPORT
enum ice_status
ice_add_prof(struct ice_hw *hw, enum ice_block blk, u64 id,
	     ice_bitmap_t *ptypes, const struct ice_ptype_attributes *attr,
	     u16 attr_cnt, struct ice_fv_word *es, u16 *masks, bool fd_swap);
#elif defined(ADV_AVF_SUPPORT)
enum ice_status
ice_add_prof(struct ice_hw *hw, enum ice_block blk, u64 id,
	     ice_bitmap_t *ptypes, const struct ice_ptype_attributes *attr,
	     u16 attr_cnt, struct ice_fv_word *es, u16 *masks);
#else /* !DPDK_SUPPORT && !ADV_AVF_SUPPORT */
#ifndef NO_FLEXP_SUPPORT
enum ice_status
ice_add_prof(struct ice_hw *hw, enum ice_block blk, u64 id,
	     ice_bitmap_t *ptypes, struct ice_fv_word *es);
#endif /* !NO_FLEXP_SUPPORT */
#endif /* DPDK_SUPPORT || ADV_AVF_SUPPORT */
#if defined(DPDK_SUPPORT) || defined(ADV_AVF_SUPPORT)
void ice_init_all_prof_masks(struct ice_hw *hw);
void ice_shutdown_all_prof_masks(struct ice_hw *hw);
#endif /* DPDK_SUPPORT || ADV_AVF_SUPPORT */
#endif /* !ICE_TDD */
#ifndef NO_UNUSED_PACKAGE_CODE
struct ice_prof_map *
ice_search_prof_id(struct ice_hw *hw, enum ice_block blk, u64 id);
enum ice_status
ice_add_vsi_flow(struct ice_hw *hw, enum ice_block blk, u16 vsi, u16 vsig);
#endif /* !NO_UNUSED_PACKAGE_CODE */
#ifndef NO_FLEXP_SUPPORT
enum ice_status
ice_add_prof_id_flow(struct ice_hw *hw, enum ice_block blk, u16 vsi, u64 hdl);
#ifndef ICE_TDD
enum ice_status
ice_rem_prof_id_flow(struct ice_hw *hw, enum ice_block blk, u16 vsi, u64 hdl);
#ifdef DPDK_SUPPORT
enum ice_status
ice_flow_assoc_hw_prof(struct ice_hw *hw, enum ice_block blk,
		       u16 dest_vsi_handle, u16 fdir_vsi_handle, int id);
#endif /* DPDK_SUPPORT */
#endif /* !NO_FLEXP_SUPPORT */
#endif /* ICE_TDD */
#ifndef NO_UNUSED_PACKAGE_CODE
#ifndef NO_UNUSED_CODE
enum ice_status
ice_set_prof_context(struct ice_hw *hw, enum ice_block blk, u64 id, u64 cntxt);
enum ice_status
ice_get_prof_context(struct ice_hw *hw, enum ice_block blk, u64 id, u64 *cntxt);
#endif /* !NO_UNUSED_CODE */
#endif /* !NO_UNUSED_PACKAGE_CODE */
enum ice_ddp_state ice_init_pkg(struct ice_hw *hw, u8 *buff, u32 len);
enum ice_ddp_state
ice_copy_and_init_pkg(struct ice_hw *hw, const u8 *buf, u32 len);
bool ice_is_init_pkg_successful(enum ice_ddp_state state);
enum ice_status ice_init_hw_tbls(struct ice_hw *hw);
void ice_free_seg(struct ice_hw *hw);
void ice_fill_blk_tbls(struct ice_hw *hw);
void ice_clear_hw_tbls(struct ice_hw *hw);
void ice_free_hw_tbls(struct ice_hw *hw);
#ifndef NO_UNUSED_CODE
enum ice_status
ice_add_flow(struct ice_hw *hw, enum ice_block blk, u16 vsi[], u8 count,
	     u64 id);
enum ice_status
ice_rem_flow(struct ice_hw *hw, enum ice_block blk, u16 vsi[], u8 count,
	     u64 id);
#endif /* !NO_UNUSED_CODE */
#ifndef NO_FLEXP_SUPPORT
enum ice_status
ice_rem_prof(struct ice_hw *hw, enum ice_block blk, u64 id);
struct ice_buf_build *
ice_pkg_buf_alloc_single_section(struct ice_hw *hw, u32 type, u16 size,
				 void **section);
struct ice_buf *ice_pkg_buf(struct ice_buf_build *bld);
void ice_pkg_buf_free(struct ice_hw *hw, struct ice_buf_build *bld);
#endif /* !NO_FLEXP_SUPPORT */

#ifndef NO_ACL_SUPPORT
enum ice_status
ice_set_key(u8 *key, u16 size, u8 *val, u8 *upd, u8 *dc, u8 *nm, u16 off,
	    u16 len);
#endif /* !NO_ACL_SUPPORT */
#ifdef DPDK_SUPPORT
void *
ice_pkg_enum_entry(struct ice_seg *ice_seg, struct ice_pkg_enum *state,
		   u32 sect_type, u32 *offset,
		   void *(*handler)(u32 sect_type, void *section,
				    u32 index, u32 *offset));
void *
ice_pkg_enum_section(struct ice_seg *ice_seg, struct ice_pkg_enum *state,
		     u32 sect_type);
#endif /* DPDK_SUPPORT */
#endif /* _ICE_FLEX_PIPE_H_ */
