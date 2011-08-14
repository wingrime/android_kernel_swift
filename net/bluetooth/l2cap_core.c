/*
   BlueZ - Bluetooth protocol stack for Linux
   Copyright (c) 2000-2001, 2010-2011 Code Aurora Forum.  All rights reserved.
   Copyright (C) 2009-2010 Gustavo F. Padovan <gustavo@padovan.org>
   Copyright (C) 2010 Google Inc.

   Written 2000,2001 by Maxim Krasnyansky <maxk@qualcomm.com>

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License version 2 as
   published by the Free Software Foundation;

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
   OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT OF THIRD PARTY RIGHTS.
   IN NO EVENT SHALL THE COPYRIGHT HOLDER(S) AND AUTHOR(S) BE LIABLE FOR ANY
   CLAIM, OR ANY SPECIAL INDIRECT OR CONSEQUENTIAL DAMAGES, OR ANY DAMAGES
   WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
   ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
   OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

   ALL LIABILITY, INCLUDING LIABILITY FOR INFRINGEMENT OF ANY PATENTS,
   COPYRIGHTS, TRADEMARKS OR OTHER RIGHTS, RELATING TO USE OF THIS
   SOFTWARE IS DISCLAIMED.
*/

/* Bluetooth L2CAP core and sockets. */

#include <linux/module.h>

#include <linux/types.h>
#include <linux/capability.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/fcntl.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/socket.h>
#include <linux/skbuff.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/crc16.h>
#include <net/sock.h>

#include <asm/system.h>
#include <asm/unaligned.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>
#include <net/bluetooth/l2cap.h>
#include <net/bluetooth/amp.h>

#define VERSION "2.15"

static int disable_ertm = 0;

static u32 l2cap_feat_mask = L2CAP_FEAT_FIXED_CHAN;
static u8 l2cap_fixed_chan[8] = { L2CAP_FC_L2CAP | L2CAP_FC_A2MP, };

static const struct proto_ops l2cap_sock_ops;

static struct workqueue_struct *_l2cap_wq;

static struct bt_sock_list l2cap_sk_list = {
	.lock = __RW_LOCK_UNLOCKED(l2cap_sk_list.lock)
};

static void l2cap_send_move_chan_req(struct l2cap_conn *conn,
			struct l2cap_pinfo *pi, u16 icid, u8 dest_amp_id);
static void l2cap_send_move_chan_cfm(struct l2cap_conn *conn,
			struct l2cap_pinfo *pi, u16 icid, u16 result);
static void l2cap_send_move_chan_rsp(struct l2cap_conn *conn, u8 ident,
			u16 icid, u16 result);

static void l2cap_amp_move_setup(struct sock *sk);
static void l2cap_amp_move_success(struct sock *sk);
static void l2cap_amp_move_revert(struct sock *sk);

static int l2cap_data_channel(struct sock *sk, struct sk_buff *skb);
static int l2cap_ertm_rx_queued_iframes(struct sock *sk);
static int l2cap_ertm_send_txq(struct sock *sk);

static void __l2cap_sock_close(struct sock *sk, int reason);
static void l2cap_sock_close(struct sock *sk);
static void l2cap_sock_kill(struct sock *sk);

static int l2cap_build_conf_req(struct sock *sk, void *data);
static struct sk_buff *l2cap_build_cmd(struct l2cap_conn *conn,
				u8 code, u8 ident, u16 dlen, void *data);
static int l2cap_answer_move_poll(struct sock *sk);

/* ---- L2CAP timers ---- */
static void l2cap_sock_timeout(unsigned long arg)
{
	struct sock *sk = (struct sock *) arg;
	int reason;

	BT_DBG("sock %p state %d", sk, sk->sk_state);

	bh_lock_sock(sk);

	if (sk->sk_state == BT_CONNECTED || sk->sk_state == BT_CONFIG)
		reason = ECONNREFUSED;
	else if (sk->sk_state == BT_CONNECT &&
				l2cap_pi(sk)->sec_level != BT_SECURITY_SDP)
		reason = ECONNREFUSED;
	else
		reason = ETIMEDOUT;

	__l2cap_sock_close(sk, reason);

	bh_unlock_sock(sk);

	l2cap_sock_kill(sk);
	sock_put(sk);
}

static void l2cap_sock_set_timer(struct sock *sk, long timeout)
{
	BT_DBG("sk %p state %d timeout %ld", sk, sk->sk_state, timeout);
	sk_reset_timer(sk, &sk->sk_timer, jiffies + timeout);
}

static void l2cap_sock_clear_timer(struct sock *sk)
{
	BT_DBG("sock %p state %d", sk, sk->sk_state);
	sk_stop_timer(sk, &sk->sk_timer);
}

/* ---- L2CAP channels ---- */
static struct sock *__l2cap_get_chan_by_dcid(struct l2cap_chan_list *l, u16 cid)
{
	struct sock *s;
	for (s = l->head; s; s = l2cap_pi(s)->next_c) {
		if (l2cap_pi(s)->dcid == cid)
			break;
	}
	return s;
}

/* Find channel with given DCID.
 * Returns locked socket */
static inline struct sock *l2cap_get_chan_by_dcid(struct l2cap_chan_list *l,
						u16 cid)
{
	struct sock *s;
	read_lock(&l->lock);
	s = __l2cap_get_chan_by_dcid(l, cid);
	if (s)
		bh_lock_sock(s);
	read_unlock(&l->lock);
	return s;
}

static struct sock *__l2cap_get_chan_by_scid(struct l2cap_chan_list *l, u16 cid)
{
	struct sock *s;
	for (s = l->head; s; s = l2cap_pi(s)->next_c) {
		if (l2cap_pi(s)->scid == cid)
			break;
	}
	return s;
}

/* Find channel with given SCID.
 * Returns locked socket */
static inline struct sock *l2cap_get_chan_by_scid(struct l2cap_chan_list *l, u16 cid)
{
	struct sock *s;
	read_lock(&l->lock);
	s = __l2cap_get_chan_by_scid(l, cid);
	if (s)
		bh_lock_sock(s);
	read_unlock(&l->lock);
	return s;
}

static struct sock *__l2cap_get_chan_by_ident(struct l2cap_chan_list *l, u8 ident)
{
	struct sock *s;
	for (s = l->head; s; s = l2cap_pi(s)->next_c) {
		if (l2cap_pi(s)->ident == ident)
			break;
	}
	return s;
}

static inline struct sock *l2cap_get_chan_by_ident(struct l2cap_chan_list *l, u8 ident)
{
	struct sock *s;
	read_lock(&l->lock);
	s = __l2cap_get_chan_by_ident(l, ident);
	if (s)
		bh_lock_sock(s);
	read_unlock(&l->lock);
	return s;
}

static inline struct sk_buff *l2cap_ertm_seq_in_queue(struct sk_buff_head *head,
						u16 seq)
{
	struct sk_buff *skb;

	skb_queue_walk(head, skb) {
		if (bt_cb(skb)->control.txseq == seq)
			return skb;
	}

	return NULL;
}

static int l2cap_seq_list_init(struct l2cap_seq_list *seq_list, u16 size)
{
	u16 allocSize = 1;
	int err = 0;
	int i;

	/* Actual allocated size must be a power of 2 */
	while (allocSize && allocSize <= size)
		allocSize <<= 1;
	if (!allocSize)
		return -ENOMEM;

	seq_list->list = kzalloc(sizeof(u16) * allocSize, GFP_ATOMIC);
	if (!seq_list->list)
		return -ENOMEM;

	seq_list->size = allocSize;
	seq_list->mask = allocSize - 1;
	seq_list->head = L2CAP_SEQ_LIST_CLEAR;
	seq_list->tail = L2CAP_SEQ_LIST_CLEAR;
	for (i = 0; i < allocSize; i++)
		seq_list->list[i] = L2CAP_SEQ_LIST_CLEAR;

	return err;
}

static inline void l2cap_seq_list_free(struct l2cap_seq_list *seq_list)
{
	kfree(seq_list->list);
}

static inline bool l2cap_seq_list_contains(struct l2cap_seq_list *seq_list,
					u16 seq)
{
	return seq_list->list[seq & seq_list->mask] != L2CAP_SEQ_LIST_CLEAR;
}

static u16 l2cap_seq_list_remove(struct l2cap_seq_list *seq_list, u16 seq)
{
	u16 mask = seq_list->mask;

	BT_DBG("seq_list %p, seq %d", seq_list, (int) seq);

	if (seq_list->head == L2CAP_SEQ_LIST_CLEAR) {
		/* In case someone tries to pop the head of an empty list */
		BT_DBG("List empty");
		return L2CAP_SEQ_LIST_CLEAR;
	} else if (seq_list->head == seq) {
		/* Head can be removed quickly */
		BT_DBG("Remove head");
		seq_list->head = seq_list->list[seq & mask];
		seq_list->list[seq & mask] = L2CAP_SEQ_LIST_CLEAR;

		if (seq_list->head == L2CAP_SEQ_LIST_TAIL) {
			seq_list->head = L2CAP_SEQ_LIST_CLEAR;
			seq_list->tail = L2CAP_SEQ_LIST_CLEAR;
		}
	} else {
		/* Non-head item must be found first */
		u16 prev = seq_list->head;
		BT_DBG("Find and remove");
		while (seq_list->list[prev & mask] != seq) {
			prev = seq_list->list[prev & mask];
			if (prev == L2CAP_SEQ_LIST_TAIL) {
				BT_DBG("seq %d not in list", (int) seq);
				return L2CAP_SEQ_LIST_CLEAR;
			}
		}

		seq_list->list[prev & mask] = seq_list->list[seq & mask];
		seq_list->list[seq & mask] = L2CAP_SEQ_LIST_CLEAR;
		if (seq_list->tail == seq)
			seq_list->tail = prev;
	}
	return seq;
}

static inline u16 l2cap_seq_list_pop(struct l2cap_seq_list *seq_list)
{
	return l2cap_seq_list_remove(seq_list, seq_list->head);
}

static void l2cap_seq_list_clear(struct l2cap_seq_list *seq_list)
{
	if (seq_list->head != L2CAP_SEQ_LIST_CLEAR) {
		u16 i;
		for (i = 0; i < seq_list->size; i++)
			seq_list->list[i] = L2CAP_SEQ_LIST_CLEAR;

		seq_list->head = L2CAP_SEQ_LIST_CLEAR;
		seq_list->tail = L2CAP_SEQ_LIST_CLEAR;
	}
}

static void l2cap_seq_list_append(struct l2cap_seq_list *seq_list, u16 seq)
{
	u16 mask = seq_list->mask;

	BT_DBG("seq_list %p, seq %d", seq_list, (int) seq);

	if (seq_list->list[seq & mask] == L2CAP_SEQ_LIST_CLEAR) {
		if (seq_list->tail == L2CAP_SEQ_LIST_CLEAR)
			seq_list->head = seq;
		else
			seq_list->list[seq_list->tail & mask] = seq;

		seq_list->tail = seq;
		seq_list->list[seq & mask] = L2CAP_SEQ_LIST_TAIL;
	}
}

static u16 __pack_enhanced_control(struct bt_l2cap_control *control)
{
	u16 packed;

	packed = (control->reqseq << L2CAP_CTRL_REQSEQ_SHIFT) &
		L2CAP_CTRL_REQSEQ;
	packed |= (control->final << L2CAP_CTRL_FINAL_SHIFT) &
		L2CAP_CTRL_FINAL;

	if (control->frame_type == 's') {
		packed |= (control->poll << L2CAP_CTRL_POLL_SHIFT) &
			L2CAP_CTRL_POLL;
		packed |= (control->super << L2CAP_CTRL_SUPERVISE_SHIFT) &
			L2CAP_CTRL_SUPERVISE;
		packed |= L2CAP_CTRL_FRAME_TYPE;
	} else {
		packed |= (control->sar << L2CAP_CTRL_SAR_SHIFT) &
			L2CAP_CTRL_SAR;
		packed |= (control->txseq << L2CAP_CTRL_TXSEQ_SHIFT) &
			L2CAP_CTRL_TXSEQ;
	}

	return packed;
}

static void __get_enhanced_control(u16 enhanced,
					struct bt_l2cap_control *control)
{
	control->reqseq = (enhanced & L2CAP_CTRL_REQSEQ) >>
		L2CAP_CTRL_REQSEQ_SHIFT;
	control->final = (enhanced & L2CAP_CTRL_FINAL) >>
		L2CAP_CTRL_FINAL_SHIFT;

	if (enhanced & L2CAP_CTRL_FRAME_TYPE) {
		control->frame_type = 's';
		control->poll = (enhanced & L2CAP_CTRL_POLL) >>
			L2CAP_CTRL_POLL_SHIFT;
		control->super = (enhanced & L2CAP_CTRL_SUPERVISE) >>
			L2CAP_CTRL_SUPERVISE_SHIFT;

		control->sar = 0;
		control->txseq = 0;
	} else {
		control->frame_type = 'i';
		control->sar = (enhanced & L2CAP_CTRL_SAR) >>
			L2CAP_CTRL_SAR_SHIFT;
		control->txseq = (enhanced & L2CAP_CTRL_TXSEQ) >>
			L2CAP_CTRL_TXSEQ_SHIFT;

		control->poll = 0;
		control->super = 0;
	}
}

static u32 __pack_extended_control(struct bt_l2cap_control *control)
{
	u32 packed;

	packed = (control->reqseq << L2CAP_EXT_CTRL_REQSEQ_SHIFT) &
		L2CAP_EXT_CTRL_REQSEQ;
	packed |= (control->final << L2CAP_EXT_CTRL_FINAL_SHIFT) &
		L2CAP_EXT_CTRL_FINAL;

	if (control->frame_type == 's') {
		packed |= (control->poll << L2CAP_EXT_CTRL_POLL_SHIFT) &
			L2CAP_EXT_CTRL_POLL;
		packed |= (control->super << L2CAP_EXT_CTRL_SUPERVISE_SHIFT) &
			L2CAP_EXT_CTRL_SUPERVISE;
		packed |= L2CAP_EXT_CTRL_FRAME_TYPE;
	} else {
		packed |= (control->sar << L2CAP_EXT_CTRL_SAR_SHIFT) &
			L2CAP_EXT_CTRL_SAR;
		packed |= (control->txseq << L2CAP_EXT_CTRL_TXSEQ_SHIFT) &
			L2CAP_EXT_CTRL_TXSEQ;
	}

	return packed;
}

static void __get_extended_control(u32 extended,
				struct bt_l2cap_control *control)
{
	control->reqseq = (extended & L2CAP_EXT_CTRL_REQSEQ) >>
		L2CAP_EXT_CTRL_REQSEQ_SHIFT;
	control->final = (extended & L2CAP_EXT_CTRL_FINAL) >>
		L2CAP_EXT_CTRL_FINAL_SHIFT;

	if (extended & L2CAP_EXT_CTRL_FRAME_TYPE) {
		control->frame_type = 's';
		control->poll = (extended & L2CAP_EXT_CTRL_POLL) >>
			L2CAP_EXT_CTRL_POLL_SHIFT;
		control->super = (extended & L2CAP_EXT_CTRL_SUPERVISE) >>
			L2CAP_EXT_CTRL_SUPERVISE_SHIFT;

		control->sar = 0;
		control->txseq = 0;
	} else {
		control->frame_type = 'i';
		control->sar = (extended & L2CAP_EXT_CTRL_SAR) >>
			L2CAP_EXT_CTRL_SAR_SHIFT;
		control->txseq = (extended & L2CAP_EXT_CTRL_TXSEQ) >>
			L2CAP_EXT_CTRL_TXSEQ_SHIFT;

		control->poll = 0;
		control->super = 0;
	}
}

static inline void l2cap_ertm_stop_ack_timer(struct l2cap_pinfo *pi)
{
	BT_DBG("pi %p", pi);
	__cancel_delayed_work(&pi->ack_work);
}

static inline void l2cap_ertm_start_ack_timer(struct l2cap_pinfo *pi)
{
	BT_DBG("pi %p, pending %d", pi, delayed_work_pending(&pi->ack_work));
	if (!delayed_work_pending(&pi->ack_work)) {
		queue_delayed_work(_l2cap_wq, &pi->ack_work,
				msecs_to_jiffies(L2CAP_DEFAULT_ACK_TO));
	}
}

static inline void l2cap_ertm_stop_retrans_timer(struct l2cap_pinfo *pi)
{
	BT_DBG("pi %p", pi);
	__cancel_delayed_work(&pi->retrans_work);
}

static inline void l2cap_ertm_start_retrans_timer(struct l2cap_pinfo *pi)
{
	BT_DBG("pi %p", pi);
	if (!delayed_work_pending(&pi->monitor_work) && pi->retrans_timeout) {
		__cancel_delayed_work(&pi->retrans_work);
		queue_delayed_work(_l2cap_wq, &pi->retrans_work,
			msecs_to_jiffies(pi->retrans_timeout));
	}
}

static inline void l2cap_ertm_stop_monitor_timer(struct l2cap_pinfo *pi)
{
	BT_DBG("pi %p", pi);
	__cancel_delayed_work(&pi->monitor_work);
}

static inline void l2cap_ertm_start_monitor_timer(struct l2cap_pinfo *pi)
{
	BT_DBG("pi %p", pi);
	l2cap_ertm_stop_retrans_timer(pi);
	__cancel_delayed_work(&pi->monitor_work);
	if (pi->monitor_timeout) {
		queue_delayed_work(_l2cap_wq, &pi->monitor_work,
				msecs_to_jiffies(pi->monitor_timeout));
	}
}

static u16 l2cap_alloc_cid(struct l2cap_chan_list *l)
{
	u16 cid = L2CAP_CID_DYN_START;

	for (; cid < L2CAP_CID_DYN_END; cid++) {
		if (!__l2cap_get_chan_by_scid(l, cid))
			return cid;
	}

	return 0;
}

static inline void __l2cap_chan_link(struct l2cap_chan_list *l, struct sock *sk)
{
	sock_hold(sk);

	if (l->head)
		l2cap_pi(l->head)->prev_c = sk;

	l2cap_pi(sk)->next_c = l->head;
	l2cap_pi(sk)->prev_c = NULL;
	l->head = sk;
}

static inline void l2cap_chan_unlink(struct l2cap_chan_list *l, struct sock *sk)
{
	struct sock *next = l2cap_pi(sk)->next_c, *prev = l2cap_pi(sk)->prev_c;

	write_lock_bh(&l->lock);
	if (sk == l->head)
		l->head = next;

	if (next)
		l2cap_pi(next)->prev_c = prev;
	if (prev)
		l2cap_pi(prev)->next_c = next;
	write_unlock_bh(&l->lock);

	__sock_put(sk);
}

static void __l2cap_chan_add(struct l2cap_conn *conn, struct sock *sk, struct sock *parent)
{
	struct l2cap_chan_list *l = &conn->chan_list;

	BT_DBG("conn %p, psm 0x%2.2x, dcid 0x%4.4x", conn,
			l2cap_pi(sk)->psm, l2cap_pi(sk)->dcid);

	conn->disc_reason = 0x13;

	l2cap_pi(sk)->conn = conn;

	if (!l2cap_pi(sk)->fixed_channel &&
		(sk->sk_type == SOCK_SEQPACKET || sk->sk_type == SOCK_STREAM)) {
		/* Alloc CID for connection-oriented socket */
		l2cap_pi(sk)->scid = l2cap_alloc_cid(l);
	} else if (sk->sk_type == SOCK_DGRAM) {
		/* Connectionless socket */
		l2cap_pi(sk)->scid = L2CAP_CID_CONN_LESS;
		l2cap_pi(sk)->dcid = L2CAP_CID_CONN_LESS;
		l2cap_pi(sk)->omtu = L2CAP_DEFAULT_MTU;
	} else if (sk->sk_type == SOCK_RAW) {
		/* Raw socket can send/recv signalling messages only */
		l2cap_pi(sk)->scid = L2CAP_CID_SIGNALING;
		l2cap_pi(sk)->dcid = L2CAP_CID_SIGNALING;
		l2cap_pi(sk)->omtu = L2CAP_DEFAULT_MTU;
	}
	/* Otherwise, do not set scid/dcid/omtu.  These will be set up
	 * by l2cap_fixed_channel_config()
	 */

	__l2cap_chan_link(l, sk);

	if (parent)
		bt_accept_enqueue(parent, sk);
}

/* Delete channel.
 * Must be called on the locked socket. */
static void l2cap_chan_del(struct sock *sk, int err)
{
	struct l2cap_conn *conn = l2cap_pi(sk)->conn;
	struct sock *parent = bt_sk(sk)->parent;

	l2cap_sock_clear_timer(sk);

	BT_DBG("sk %p, conn %p, err %d", sk, conn, err);

	if (conn) {
		/* Unlink from channel list */
		l2cap_chan_unlink(&conn->chan_list, sk);
		l2cap_pi(sk)->conn = NULL;
		if (!l2cap_pi(sk)->fixed_channel)
			hci_conn_put(conn->hcon);
	}

	if (l2cap_pi(sk)->ampcon) {
		l2cap_pi(sk)->ampcon->l2cap_data = NULL;
		l2cap_pi(sk)->ampcon = NULL;
		if (l2cap_pi(sk)->ampchan)
			hci_chan_put(l2cap_pi(sk)->ampchan);
		l2cap_pi(sk)->ampchan = NULL;
		l2cap_pi(sk)->amp_id = 0;
	}

	sk->sk_state = BT_CLOSED;
	sock_set_flag(sk, SOCK_ZAPPED);

	if (err)
		sk->sk_err = err;

	if (parent) {
		bt_accept_unlink(sk);
		parent->sk_data_ready(parent, 0);
	} else
		sk->sk_state_change(sk);

	skb_queue_purge(TX_QUEUE(sk));

	if (l2cap_pi(sk)->mode == L2CAP_MODE_ERTM) {
		if (l2cap_pi(sk)->sdu)
			kfree_skb(l2cap_pi(sk)->sdu);

		skb_queue_purge(SREJ_QUEUE(sk));

		__cancel_delayed_work(&l2cap_pi(sk)->ack_work);
		__cancel_delayed_work(&l2cap_pi(sk)->retrans_work);
		__cancel_delayed_work(&l2cap_pi(sk)->monitor_work);
	}
}

/* Service level security */
static inline int l2cap_check_security(struct sock *sk)
{
	struct l2cap_conn *conn = l2cap_pi(sk)->conn;
	__u8 auth_type;

	if (l2cap_pi(sk)->psm == cpu_to_le16(0x0001)) {
		if (l2cap_pi(sk)->sec_level == BT_SECURITY_HIGH)
			auth_type = HCI_AT_NO_BONDING_MITM;
		else
			auth_type = HCI_AT_NO_BONDING;

		if (l2cap_pi(sk)->sec_level == BT_SECURITY_LOW)
			l2cap_pi(sk)->sec_level = BT_SECURITY_SDP;
	} else {
		switch (l2cap_pi(sk)->sec_level) {
		case BT_SECURITY_HIGH:
			auth_type = HCI_AT_GENERAL_BONDING_MITM;
			break;
		case BT_SECURITY_MEDIUM:
			auth_type = HCI_AT_GENERAL_BONDING;
			break;
		default:
			auth_type = HCI_AT_NO_BONDING;
			break;
		}
	}

	return hci_conn_security(conn->hcon, l2cap_pi(sk)->sec_level,
								auth_type);
}

static inline u8 l2cap_get_ident(struct l2cap_conn *conn)
{
	u8 id;

	/* Get next available identificator.
	 *    1 - 128 are used by kernel.
	 *  129 - 199 are reserved.
	 *  200 - 254 are used by utilities like l2ping, etc.
	 */

	spin_lock_bh(&conn->lock);

	if (++conn->tx_ident > 128)
		conn->tx_ident = 1;

	id = conn->tx_ident;

	spin_unlock_bh(&conn->lock);

	return id;
}

static void apply_fcs(struct sk_buff *skb)
{
	size_t len;
	u16 partial_crc;
	struct sk_buff *iter;
	struct sk_buff *final_frag = skb;

	if (skb_has_frags(skb))
		len = skb_headlen(skb);
	else
		len = skb->len - L2CAP_FCS_SIZE;

	partial_crc = crc16(0, (u8 *) skb->data, len);

	skb_walk_frags(skb, iter) {
		len = iter->len;
		if (!iter->next)
			len -= L2CAP_FCS_SIZE;

		partial_crc = crc16(partial_crc, iter->data, len);
		final_frag = iter;
	}

	put_unaligned_le16(partial_crc,
		final_frag->data + final_frag->len - L2CAP_FCS_SIZE);
}

static inline void l2cap_send_cmd(struct l2cap_conn *conn, u8 ident, u8 code, u16 len, void *data)
{
	struct sk_buff *skb = l2cap_build_cmd(conn, code, ident, len, data);
	u8 flags;

	BT_DBG("code 0x%2.2x", code);

	if (!skb)
		return;

	if (lmp_no_flush_capable(conn->hcon->hdev))
		flags = ACL_START_NO_FLUSH;
	else
		flags = ACL_START;

	bt_cb(skb)->force_active = 1;
	hci_send_acl(conn->hcon, NULL, skb, flags);
}

static inline int __l2cap_no_conn_pending(struct sock *sk)
{
	return !(l2cap_pi(sk)->conf_state & L2CAP_CONF_CONNECT_PEND);
}

static void l2cap_do_start(struct sock *sk)
{
	struct l2cap_conn *conn = l2cap_pi(sk)->conn;

	if (conn->info_state & L2CAP_INFO_FEAT_MASK_REQ_SENT) {
		if (!(conn->info_state & L2CAP_INFO_FEAT_MASK_REQ_DONE))
			return;

		if (l2cap_check_security(sk) && __l2cap_no_conn_pending(sk)) {
			struct l2cap_conn_req req;
			req.scid = cpu_to_le16(l2cap_pi(sk)->scid);
			req.psm  = l2cap_pi(sk)->psm;

			l2cap_pi(sk)->ident = l2cap_get_ident(conn);
			l2cap_pi(sk)->conf_state |= L2CAP_CONF_CONNECT_PEND;

			l2cap_send_cmd(conn, l2cap_pi(sk)->ident,
					L2CAP_CONN_REQ, sizeof(req), &req);
		}
	} else {
		struct l2cap_info_req req;
		req.type = cpu_to_le16(L2CAP_IT_FEAT_MASK);

		conn->info_state |= L2CAP_INFO_FEAT_MASK_REQ_SENT;
		conn->info_ident = l2cap_get_ident(conn);

		mod_timer(&conn->info_timer, jiffies +
					msecs_to_jiffies(L2CAP_INFO_TIMEOUT));

		l2cap_send_cmd(conn, conn->info_ident,
					L2CAP_INFO_REQ, sizeof(req), &req);
	}
}

static inline int l2cap_mode_supported(__u8 mode, __u32 feat_mask)
{
	u32 local_feat_mask = l2cap_feat_mask;
	if (!disable_ertm)
		local_feat_mask |= L2CAP_FEAT_ERTM | L2CAP_FEAT_STREAMING;

	switch (mode) {
	case L2CAP_MODE_ERTM:
		return L2CAP_FEAT_ERTM & feat_mask & local_feat_mask;
	case L2CAP_MODE_STREAMING:
		return L2CAP_FEAT_STREAMING & feat_mask & local_feat_mask;
	default:
		return 0x00;
	}
}

static void l2cap_send_disconn_req(struct l2cap_conn *conn, struct sock *sk, int err)
{
	struct l2cap_disconn_req req;

	if (!conn)
		return;

	skb_queue_purge(TX_QUEUE(sk));

	if (l2cap_pi(sk)->mode == L2CAP_MODE_ERTM) {
		skb_queue_purge(SREJ_QUEUE(sk));

		__cancel_delayed_work(&l2cap_pi(sk)->ack_work);
		__cancel_delayed_work(&l2cap_pi(sk)->retrans_work);
		__cancel_delayed_work(&l2cap_pi(sk)->monitor_work);
	}

	req.dcid = cpu_to_le16(l2cap_pi(sk)->dcid);
	req.scid = cpu_to_le16(l2cap_pi(sk)->scid);
	l2cap_send_cmd(conn, l2cap_get_ident(conn),
			L2CAP_DISCONN_REQ, sizeof(req), &req);

	sk->sk_state = BT_DISCONN;
	sk->sk_err = err;
}

/* ---- L2CAP connections ---- */
static void l2cap_conn_start(struct l2cap_conn *conn)
{
	struct l2cap_chan_list *l = &conn->chan_list;
	struct sock_del_list del, *tmp1, *tmp2;
	struct sock *sk;

	BT_DBG("conn %p", conn);

	INIT_LIST_HEAD(&del.list);

	read_lock(&l->lock);

	for (sk = l->head; sk; sk = l2cap_pi(sk)->next_c) {
		bh_lock_sock(sk);

		if (sk->sk_type != SOCK_SEQPACKET &&
				sk->sk_type != SOCK_STREAM) {
			bh_unlock_sock(sk);
			continue;
		}

		if (sk->sk_state == BT_CONNECT) {
			struct l2cap_conn_req req;

			if (!l2cap_check_security(sk) ||
					!__l2cap_no_conn_pending(sk)) {
				bh_unlock_sock(sk);
				continue;
			}

			if (!l2cap_mode_supported(l2cap_pi(sk)->mode,
					conn->feat_mask)
					&& l2cap_pi(sk)->conf_state &
					L2CAP_CONF_STATE2_DEVICE) {
				tmp1 = kzalloc(sizeof(struct sock_del_list),
						GFP_ATOMIC);
				tmp1->sk = sk;
				list_add_tail(&tmp1->list, &del.list);
				bh_unlock_sock(sk);
				continue;
			}

			req.scid = cpu_to_le16(l2cap_pi(sk)->scid);
			req.psm  = l2cap_pi(sk)->psm;

			l2cap_pi(sk)->ident = l2cap_get_ident(conn);
			l2cap_pi(sk)->conf_state |= L2CAP_CONF_CONNECT_PEND;

			l2cap_send_cmd(conn, l2cap_pi(sk)->ident,
				L2CAP_CONN_REQ, sizeof(req), &req);

		} else if (sk->sk_state == BT_CONNECT2) {
			struct l2cap_conn_rsp rsp;
			char buf[128];
			rsp.scid = cpu_to_le16(l2cap_pi(sk)->dcid);
			rsp.dcid = cpu_to_le16(l2cap_pi(sk)->scid);

			if (l2cap_check_security(sk)) {
				if (bt_sk(sk)->defer_setup) {
					struct sock *parent = bt_sk(sk)->parent;
					rsp.result = cpu_to_le16(L2CAP_CR_PEND);
					rsp.status = cpu_to_le16(L2CAP_CS_AUTHOR_PEND);
					if (parent)
						parent->sk_data_ready(parent, 0);

				} else {
					sk->sk_state = BT_CONFIG;
					rsp.result = cpu_to_le16(L2CAP_CR_SUCCESS);
					rsp.status = cpu_to_le16(L2CAP_CS_NO_INFO);
				}
			} else {
				rsp.result = cpu_to_le16(L2CAP_CR_PEND);
				rsp.status = cpu_to_le16(L2CAP_CS_AUTHEN_PEND);
			}

			l2cap_send_cmd(conn, l2cap_pi(sk)->ident,
					L2CAP_CONN_RSP, sizeof(rsp), &rsp);

			if (l2cap_pi(sk)->conf_state & L2CAP_CONF_REQ_SENT ||
					rsp.result != L2CAP_CR_SUCCESS) {
				bh_unlock_sock(sk);
				continue;
			}

			l2cap_pi(sk)->conf_state |= L2CAP_CONF_REQ_SENT;
			l2cap_send_cmd(conn, l2cap_get_ident(conn), L2CAP_CONF_REQ,
						l2cap_build_conf_req(sk, buf), buf);
			l2cap_pi(sk)->num_conf_req++;
		}

		bh_unlock_sock(sk);
	}

	read_unlock(&l->lock);

	list_for_each_entry_safe(tmp1, tmp2, &del.list, list) {
		bh_lock_sock(tmp1->sk);
		__l2cap_sock_close(tmp1->sk, ECONNRESET);
		bh_unlock_sock(tmp1->sk);
		list_del(&tmp1->list);
		kfree(tmp1);
	}
}

static void l2cap_conn_ready(struct l2cap_conn *conn)
{
	struct l2cap_chan_list *l = &conn->chan_list;
	struct sock *sk;

	BT_DBG("conn %p", conn);

	read_lock(&l->lock);

	for (sk = l->head; sk; sk = l2cap_pi(sk)->next_c) {
		bh_lock_sock(sk);

		if (sk->sk_type != SOCK_SEQPACKET &&
				sk->sk_type != SOCK_STREAM) {
			l2cap_sock_clear_timer(sk);
			sk->sk_state = BT_CONNECTED;
			sk->sk_state_change(sk);
		} else if (sk->sk_state == BT_CONNECT)
			l2cap_do_start(sk);

		bh_unlock_sock(sk);
	}

	read_unlock(&l->lock);
}

/* Notify sockets that we cannot guaranty reliability anymore */
static void l2cap_conn_unreliable(struct l2cap_conn *conn, int err)
{
	struct l2cap_chan_list *l = &conn->chan_list;
	struct sock *sk;

	BT_DBG("conn %p", conn);

	read_lock(&l->lock);

	for (sk = l->head; sk; sk = l2cap_pi(sk)->next_c) {
		if (l2cap_pi(sk)->force_reliable)
			sk->sk_err = err;
	}

	read_unlock(&l->lock);
}

static void l2cap_info_timeout(unsigned long arg)
{
	struct l2cap_conn *conn = (void *) arg;

	conn->info_state |= L2CAP_INFO_FEAT_MASK_REQ_DONE;
	conn->info_ident = 0;

	l2cap_conn_start(conn);
}

static struct l2cap_conn *l2cap_conn_add(struct hci_conn *hcon, u8 status)
{
	struct l2cap_conn *conn = hcon->l2cap_data;

	if (conn || status)
		return conn;

	conn = kzalloc(sizeof(struct l2cap_conn), GFP_ATOMIC);
	if (!conn)
		return NULL;

	hcon->l2cap_data = conn;
	conn->hcon = hcon;

	BT_DBG("hcon %p conn %p", hcon, conn);

	conn->mtu = hcon->hdev->acl_mtu;
	conn->src = &hcon->hdev->bdaddr;
	conn->dst = &hcon->dst;

	conn->feat_mask = 0;

	spin_lock_init(&conn->lock);
	rwlock_init(&conn->chan_list.lock);

	setup_timer(&conn->info_timer, l2cap_info_timeout,
						(unsigned long) conn);

	conn->disc_reason = 0x13;

	return conn;
}

static void l2cap_conn_del(struct hci_conn *hcon, int err)
{
	struct l2cap_conn *conn = hcon->l2cap_data;
	struct sock *sk;
	struct sock *next;

	if (!conn)
		return;

	BT_DBG("hcon %p conn %p, err %d", hcon, conn, err);

	if ((conn->hcon == hcon) && (conn->rx_skb))
		kfree_skb(conn->rx_skb);

	BT_DBG("conn->hcon %p", conn->hcon);

	/* Kill channels */
	for (sk = conn->chan_list.head; sk; ) {
		BT_DBG("ampcon %p", l2cap_pi(sk)->ampcon);
		if ((conn->hcon == hcon) || (l2cap_pi(sk)->ampcon == hcon)) {
			next = l2cap_pi(sk)->next_c;
			bh_lock_sock(sk);
			l2cap_chan_del(sk, err);
			bh_unlock_sock(sk);
			l2cap_sock_kill(sk);
			sk = next;
		} else
			sk = l2cap_pi(sk)->next_c;
	}

	if (conn->hcon == hcon) {
		if (conn->info_state & L2CAP_INFO_FEAT_MASK_REQ_SENT)
			del_timer_sync(&conn->info_timer);

		hcon->l2cap_data = NULL;

		kfree(conn);
	}
}

static inline void l2cap_chan_add(struct l2cap_conn *conn, struct sock *sk, struct sock *parent)
{
	struct l2cap_chan_list *l = &conn->chan_list;
	write_lock_bh(&l->lock);
	__l2cap_chan_add(conn, sk, parent);
	write_unlock_bh(&l->lock);
}

/* ---- Socket interface ---- */
static struct sock *__l2cap_get_sock_by_addr(__le16 psm, bdaddr_t *src)
{
	struct sock *sk;
	struct hlist_node *node;
	sk_for_each(sk, node, &l2cap_sk_list.head)
		if (l2cap_pi(sk)->sport == psm && !bacmp(&bt_sk(sk)->src, src))
			goto found;
	sk = NULL;
found:
	return sk;
}

/* Find socket with psm and source bdaddr.
 * Returns closest match.
 */
static struct sock *__l2cap_get_sock_by_psm(int state, __le16 psm, bdaddr_t *src)
{
	struct sock *sk = NULL, *sk1 = NULL;
	struct hlist_node *node;

	sk_for_each(sk, node, &l2cap_sk_list.head) {
		if (state && sk->sk_state != state)
			continue;

		if (l2cap_pi(sk)->psm == psm) {
			/* Exact match. */
			if (!bacmp(&bt_sk(sk)->src, src))
				break;

			/* Closest match */
			if (!bacmp(&bt_sk(sk)->src, BDADDR_ANY))
				sk1 = sk;
		}
	}
	return node ? sk : sk1;
}

/* Find socket with given address (psm, src).
 * Returns locked socket */
static inline struct sock *l2cap_get_sock_by_psm(int state, __le16 psm, bdaddr_t *src)
{
	struct sock *s;
	read_lock(&l2cap_sk_list.lock);
	s = __l2cap_get_sock_by_psm(state, psm, src);
	if (s)
		bh_lock_sock(s);
	read_unlock(&l2cap_sk_list.lock);
	return s;
}

static void l2cap_sock_destruct(struct sock *sk)
{
	BT_DBG("sk %p", sk);

	skb_queue_purge(&sk->sk_receive_queue);
	skb_queue_purge(&sk->sk_write_queue);

	l2cap_seq_list_free(&l2cap_pi(sk)->srej_list);
	l2cap_seq_list_free(&l2cap_pi(sk)->retrans_list);
}

static void l2cap_sock_cleanup_listen(struct sock *parent)
{
	struct sock *sk;

	BT_DBG("parent %p", parent);

	/* Close not yet accepted channels */
	while ((sk = bt_accept_dequeue(parent, NULL)))
		l2cap_sock_close(sk);

	parent->sk_state = BT_CLOSED;
	sock_set_flag(parent, SOCK_ZAPPED);
}

/* Kill socket (only if zapped and orphan)
 * Must be called on unlocked socket.
 */
static void l2cap_sock_kill(struct sock *sk)
{
	if (!sock_flag(sk, SOCK_ZAPPED) || sk->sk_socket)
		return;

	BT_DBG("sk %p state %d", sk, sk->sk_state);

	/* Kill poor orphan */
	bt_sock_unlink(&l2cap_sk_list, sk);
	sock_set_flag(sk, SOCK_DEAD);
	sock_put(sk);
}

static void __l2cap_sock_close(struct sock *sk, int reason)
{
	BT_DBG("sk %p state %d socket %p", sk, sk->sk_state, sk->sk_socket);

	switch (sk->sk_state) {
	case BT_LISTEN:
		l2cap_sock_cleanup_listen(sk);
		break;

	case BT_CONNECTED:
	case BT_CONFIG:
		if (sk->sk_type == SOCK_SEQPACKET ||
				sk->sk_type == SOCK_STREAM) {
			struct l2cap_conn *conn = l2cap_pi(sk)->conn;

			l2cap_sock_set_timer(sk, sk->sk_sndtimeo);
			l2cap_send_disconn_req(conn, sk, reason);
		} else
			l2cap_chan_del(sk, reason);
		break;

	case BT_CONNECT2:
		if (sk->sk_type == SOCK_SEQPACKET ||
				sk->sk_type == SOCK_STREAM) {
			struct l2cap_conn *conn = l2cap_pi(sk)->conn;
			struct l2cap_conn_rsp rsp;
			__u16 result;

			if (bt_sk(sk)->defer_setup)
				result = L2CAP_CR_SEC_BLOCK;
			else
				result = L2CAP_CR_BAD_PSM;

			rsp.scid   = cpu_to_le16(l2cap_pi(sk)->dcid);
			rsp.dcid   = cpu_to_le16(l2cap_pi(sk)->scid);
			rsp.result = cpu_to_le16(result);
			rsp.status = cpu_to_le16(L2CAP_CS_NO_INFO);
			l2cap_send_cmd(conn, l2cap_pi(sk)->ident,
					L2CAP_CONN_RSP, sizeof(rsp), &rsp);
		} else
			l2cap_chan_del(sk, reason);
		break;

	case BT_CONNECT:
	case BT_DISCONN:
		l2cap_chan_del(sk, reason);
		break;

	default:
		sock_set_flag(sk, SOCK_ZAPPED);
		break;
	}
}

/* Must be called on unlocked socket. */
static void l2cap_sock_close(struct sock *sk)
{
	l2cap_sock_clear_timer(sk);
	lock_sock(sk);
	__l2cap_sock_close(sk, ECONNRESET);
	release_sock(sk);
	l2cap_sock_kill(sk);
}

static void set_default_config(struct l2cap_conf_prm *conf_prm)
{
	conf_prm->fcs = L2CAP_FCS_CRC16;
	conf_prm->retrans_timeout = 0;
	conf_prm->monitor_timeout = 0;
	conf_prm->flush_to = L2CAP_DEFAULT_FLUSH_TO;
}

static void l2cap_sock_init(struct sock *sk, struct sock *parent)
{
	struct l2cap_pinfo *pi = l2cap_pi(sk);

	BT_DBG("sk %p parent %p", sk, parent);

	if (parent) {
		sk->sk_type = parent->sk_type;
		sk->sk_rcvbuf = parent->sk_rcvbuf;
		sk->sk_sndbuf = parent->sk_sndbuf;
		bt_sk(sk)->defer_setup = bt_sk(parent)->defer_setup;

		pi->imtu = l2cap_pi(parent)->imtu;
		pi->omtu = l2cap_pi(parent)->omtu;
		pi->conf_state = l2cap_pi(parent)->conf_state;
		pi->mode = l2cap_pi(parent)->mode;
		pi->fcs  = l2cap_pi(parent)->fcs;
		pi->max_tx = l2cap_pi(parent)->max_tx;
		pi->tx_win = l2cap_pi(parent)->tx_win;
		pi->sec_level = l2cap_pi(parent)->sec_level;
		pi->role_switch = l2cap_pi(parent)->role_switch;
		pi->force_reliable = l2cap_pi(parent)->force_reliable;
		pi->flushable = l2cap_pi(parent)->flushable;
		pi->force_active = l2cap_pi(parent)->force_active;
		pi->amp_pref = l2cap_pi(parent)->amp_pref;
	} else {
		pi->imtu = L2CAP_DEFAULT_MTU;
		pi->omtu = 0;
		if (!disable_ertm && sk->sk_type == SOCK_STREAM) {
			pi->mode = L2CAP_MODE_ERTM;
			pi->conf_state |= L2CAP_CONF_STATE2_DEVICE;
		} else {
			pi->mode = L2CAP_MODE_BASIC;
		}
		pi->reconf_state = L2CAP_RECONF_NONE;
		pi->max_tx = L2CAP_DEFAULT_MAX_TX;
		pi->fcs = L2CAP_FCS_CRC16;
		pi->tx_win = L2CAP_DEFAULT_TX_WINDOW;
		pi->sec_level = BT_SECURITY_LOW;
		pi->role_switch = 0;
		pi->force_reliable = 0;
		pi->flushable = 0;
		pi->force_active = 1;
		pi->amp_pref = BT_AMP_POLICY_REQUIRE_BR_EDR;
	}

	/* Default config options */
	sk->sk_backlog_rcv = l2cap_data_channel;
	pi->ampcon = NULL;
	pi->ampchan = NULL;
	pi->conf_len = 0;
	pi->flush_to = L2CAP_DEFAULT_FLUSH_TO;
	pi->scid = 0;
	pi->dcid = 0;
	pi->tx_win_max = L2CAP_TX_WIN_MAX_ENHANCED;
	pi->extended_control = 0;

	pi->local_conf.fcs = pi->fcs;
	if (pi->mode == L2CAP_MODE_BASIC) {
		pi->local_conf.retrans_timeout = 0;
		pi->local_conf.monitor_timeout = 0;
	} else {
		pi->local_conf.retrans_timeout = L2CAP_DEFAULT_RETRANS_TO;
		pi->local_conf.monitor_timeout = L2CAP_DEFAULT_MONITOR_TO;
	}

	pi->local_conf.flush_to = pi->flush_to;

	set_default_config(&pi->remote_conf);

	skb_queue_head_init(TX_QUEUE(sk));
	skb_queue_head_init(SREJ_QUEUE(sk));
}

static struct proto l2cap_proto = {
	.name		= "L2CAP",
	.owner		= THIS_MODULE,
	.obj_size	= sizeof(struct l2cap_pinfo)
};

static struct sock *l2cap_sock_alloc(struct net *net, struct socket *sock, int proto, gfp_t prio)
{
	struct sock *sk;

	sk = sk_alloc(net, PF_BLUETOOTH, prio, &l2cap_proto);
	if (!sk)
		return NULL;

	sock_init_data(sock, sk);
	INIT_LIST_HEAD(&bt_sk(sk)->accept_q);

	sk->sk_destruct = l2cap_sock_destruct;
	sk->sk_sndtimeo = msecs_to_jiffies(L2CAP_CONN_TIMEOUT);

	sock_reset_flag(sk, SOCK_ZAPPED);

	sk->sk_protocol = proto;
	sk->sk_state = BT_OPEN;

	setup_timer(&sk->sk_timer, l2cap_sock_timeout, (unsigned long) sk);

	bt_sock_link(&l2cap_sk_list, sk);
	return sk;
}

static int l2cap_sock_create(struct net *net, struct socket *sock, int protocol,
							int kern)
{
	struct sock *sk;

	BT_DBG("sock %p", sock);

	sock->state = SS_UNCONNECTED;

	if (sock->type != SOCK_SEQPACKET && sock->type != SOCK_STREAM &&
			sock->type != SOCK_DGRAM && sock->type != SOCK_RAW)
		return -ESOCKTNOSUPPORT;

	if (sock->type == SOCK_RAW && !kern && !capable(CAP_NET_RAW))
		return -EPERM;

	sock->ops = &l2cap_sock_ops;

	sk = l2cap_sock_alloc(net, sock, protocol, GFP_ATOMIC);
	if (!sk)
		return -ENOMEM;

	l2cap_sock_init(sk, NULL);
	return 0;
}

static int l2cap_sock_bind(struct socket *sock, struct sockaddr *addr, int alen)
{
	struct sock *sk = sock->sk;
	struct sockaddr_l2 la;
	int len, err = 0;

	BT_DBG("sk %p", sk);

	if (!addr || addr->sa_family != AF_BLUETOOTH)
		return -EINVAL;

	memset(&la, 0, sizeof(la));
	len = min_t(unsigned int, sizeof(la), alen);
	memcpy(&la, addr, len);

	if (la.l2_cid)
		return -EINVAL;

	lock_sock(sk);

	if (sk->sk_state != BT_OPEN) {
		err = -EBADFD;
		goto done;
	}

	if (la.l2_psm) {
		__u16 psm = __le16_to_cpu(la.l2_psm);

		/* PSM must be odd and lsb of upper byte must be 0 */
		if ((psm & 0x0101) != 0x0001) {
			err = -EINVAL;
			goto done;
		}

		/* Restrict usage of well-known PSMs */
		if (psm < 0x1001 && !capable(CAP_NET_BIND_SERVICE)) {
			err = -EACCES;
			goto done;
		}
	}

	write_lock_bh(&l2cap_sk_list.lock);

	if (la.l2_psm && __l2cap_get_sock_by_addr(la.l2_psm, &la.l2_bdaddr)) {
		err = -EADDRINUSE;
	} else {
		/* Save source address */
		bacpy(&bt_sk(sk)->src, &la.l2_bdaddr);
		l2cap_pi(sk)->psm   = la.l2_psm;
		l2cap_pi(sk)->sport = la.l2_psm;
		sk->sk_state = BT_BOUND;

		if (__le16_to_cpu(la.l2_psm) == 0x0001 ||
					__le16_to_cpu(la.l2_psm) == 0x0003)
			l2cap_pi(sk)->sec_level = BT_SECURITY_SDP;
	}

	write_unlock_bh(&l2cap_sk_list.lock);

done:
	release_sock(sk);
	return err;
}

static int l2cap_do_connect(struct sock *sk)
{
	bdaddr_t *src = &bt_sk(sk)->src;
	bdaddr_t *dst = &bt_sk(sk)->dst;
	struct l2cap_conn *conn;
	struct hci_conn *hcon;
	struct hci_dev *hdev;
	__u8 auth_type;
	int err;

	BT_DBG("%s -> %s psm 0x%2.2x", batostr(src), batostr(dst),
							l2cap_pi(sk)->psm);

	hdev = hci_get_route(dst, src);
	if (!hdev)
		return -EHOSTUNREACH;

	hci_dev_lock_bh(hdev);

	err = -ENOMEM;

	if (sk->sk_type == SOCK_RAW) {
		switch (l2cap_pi(sk)->sec_level) {
		case BT_SECURITY_HIGH:
			auth_type = HCI_AT_DEDICATED_BONDING_MITM;
			break;
		case BT_SECURITY_MEDIUM:
			auth_type = HCI_AT_DEDICATED_BONDING;
			break;
		default:
			auth_type = HCI_AT_NO_BONDING;
			break;
		}
	} else if (l2cap_pi(sk)->psm == cpu_to_le16(0x0001)) {
		if (l2cap_pi(sk)->sec_level == BT_SECURITY_HIGH)
			auth_type = HCI_AT_NO_BONDING_MITM;
		else
			auth_type = HCI_AT_NO_BONDING;

		if (l2cap_pi(sk)->sec_level == BT_SECURITY_LOW)
			l2cap_pi(sk)->sec_level = BT_SECURITY_SDP;
	} else {
		switch (l2cap_pi(sk)->sec_level) {
		case BT_SECURITY_HIGH:
			auth_type = HCI_AT_GENERAL_BONDING_MITM;
			break;
		case BT_SECURITY_MEDIUM:
			auth_type = HCI_AT_GENERAL_BONDING;
			break;
		default:
			auth_type = HCI_AT_NO_BONDING;
			break;
		}
	}

	if (l2cap_pi(sk)->fixed_channel) {
		/* Fixed channels piggyback on existing ACL connections */
		hcon = hci_conn_hash_lookup_ba(hdev, ACL_LINK, dst);
		if (!hcon || !hcon->l2cap_data)
			goto done;

		conn = hcon->l2cap_data;
	} else {
		hcon = hci_connect(hdev, ACL_LINK, 0, dst,
				l2cap_pi(sk)->sec_level, auth_type);
		if (!hcon)
			goto done;

		conn = l2cap_conn_add(hcon, 0);
		if (!conn) {
			hci_conn_put(hcon);
			goto done;
		}
	}

	err = 0;

	/* Update source addr of the socket */
	bacpy(src, conn->src);

	l2cap_chan_add(conn, sk, NULL);

	BT_DBG("hcon->state %d", (int) hcon->state);

	if (l2cap_pi(sk)->fixed_channel) {
		sk->sk_state = BT_CONNECTED;
		sk->sk_state_change(sk);
	} else {
		sk->sk_state = BT_CONNECT;
		l2cap_sock_set_timer(sk, sk->sk_sndtimeo);
		sk->sk_state_change(sk);

		if (hcon->state == BT_CONNECTED) {
			if (sk->sk_type != SOCK_SEQPACKET &&
				sk->sk_type != SOCK_STREAM) {
				l2cap_sock_clear_timer(sk);
				sk->sk_state = BT_CONNECTED;
				sk->sk_state_change(sk);
			} else
				l2cap_do_start(sk);
		}
	}

done:
	hci_dev_unlock_bh(hdev);
	hci_dev_put(hdev);
	return err;
}

static int l2cap_sock_connect(struct socket *sock, struct sockaddr *addr, int alen, int flags)
{
	struct sock *sk = sock->sk;
	struct sockaddr_l2 la;
	int len, err = 0;

	BT_DBG("sk %p type %d mode %d state %d", sk, sk->sk_type,
		l2cap_pi(sk)->mode, sk->sk_state);

	if (!addr || alen < sizeof(addr->sa_family) ||
		addr->sa_family != AF_BLUETOOTH)
		return -EINVAL;

	memset(&la, 0, sizeof(la));
	len = min_t(unsigned int, sizeof(la), alen);
	memcpy(&la, addr, len);

	if (la.l2_cid)
		return -EINVAL;

	lock_sock(sk);

	if ((sk->sk_type == SOCK_SEQPACKET || sk->sk_type == SOCK_STREAM)
		&& (!la.l2_psm && !l2cap_pi(sk)->fixed_channel)) {
		err = -EINVAL;
		goto done;
	}

	switch (l2cap_pi(sk)->mode) {
	case L2CAP_MODE_BASIC:
		break;
	case L2CAP_MODE_ERTM:
	case L2CAP_MODE_STREAMING:
		if (!disable_ertm)
			break;
		/* fall through */
	default:
		err = -ENOTSUPP;
		goto done;
	}

	switch (sk->sk_state) {
	case BT_CONNECT:
	case BT_CONNECT2:
	case BT_CONFIG:
		/* Already connecting */
		goto wait;

	case BT_CONNECTED:
		/* Already connected */
		err = -EISCONN;
		goto done;

	case BT_OPEN:
	case BT_BOUND:
		/* Can connect */
		break;

	default:
		err = -EBADFD;
		goto done;
	}

	/* PSM must be odd and lsb of upper byte must be 0 */
	if ((__le16_to_cpu(la.l2_psm) & 0x0101) != 0x0001 &&
		!l2cap_pi(sk)->fixed_channel &&
		sk->sk_type != SOCK_RAW) {
		BT_DBG("Bad PSM 0x%x", (int)__le16_to_cpu(la.l2_psm));
		err = -EINVAL;
		goto done;
	}

	/* Set destination address and psm */
	bacpy(&bt_sk(sk)->dst, &la.l2_bdaddr);
	l2cap_pi(sk)->psm = la.l2_psm;

	err = l2cap_do_connect(sk);
	if (err)
		goto done;

wait:
	err = bt_sock_wait_state(sk, BT_CONNECTED,
			sock_sndtimeo(sk, flags & O_NONBLOCK));
done:
	if (err)
		BT_ERR("failed %d", err);
	release_sock(sk);
	return err;
}

static int l2cap_sock_listen(struct socket *sock, int backlog)
{
	struct sock *sk = sock->sk;
	int err = 0;

	BT_DBG("sk %p backlog %d", sk, backlog);

	lock_sock(sk);

	if ((sock->type != SOCK_SEQPACKET && sock->type != SOCK_STREAM)
			|| sk->sk_state != BT_BOUND) {
		err = -EBADFD;
		goto done;
	}

	switch (l2cap_pi(sk)->mode) {
	case L2CAP_MODE_BASIC:
		break;
	case L2CAP_MODE_ERTM:
	case L2CAP_MODE_STREAMING:
		if (!disable_ertm)
			break;
		/* fall through */
	default:
		err = -ENOTSUPP;
		goto done;
	}

	if (!l2cap_pi(sk)->psm) {
		bdaddr_t *src = &bt_sk(sk)->src;
		u16 psm;

		err = -EINVAL;

		write_lock_bh(&l2cap_sk_list.lock);

		for (psm = 0x1001; psm < 0x1100; psm += 2)
			if (!__l2cap_get_sock_by_addr(cpu_to_le16(psm), src)) {
				l2cap_pi(sk)->psm   = cpu_to_le16(psm);
				l2cap_pi(sk)->sport = cpu_to_le16(psm);
				err = 0;
				break;
			}

		write_unlock_bh(&l2cap_sk_list.lock);

		if (err < 0)
			goto done;
	}

	sk->sk_max_ack_backlog = backlog;
	sk->sk_ack_backlog = 0;
	sk->sk_state = BT_LISTEN;

done:
	release_sock(sk);
	return err;
}

static int l2cap_sock_accept(struct socket *sock, struct socket *newsock, int flags)
{
	DECLARE_WAITQUEUE(wait, current);
	struct sock *sk = sock->sk, *nsk;
	long timeo;
	int err = 0;

	lock_sock_nested(sk, SINGLE_DEPTH_NESTING);

	if (sk->sk_state != BT_LISTEN) {
		err = -EBADFD;
		goto done;
	}

	timeo = sock_rcvtimeo(sk, flags & O_NONBLOCK);

	BT_DBG("sk %p timeo %ld", sk, timeo);

	/* Wait for an incoming connection. (wake-one). */
	add_wait_queue_exclusive(sk_sleep(sk), &wait);
	while (!(nsk = bt_accept_dequeue(sk, newsock))) {
		set_current_state(TASK_INTERRUPTIBLE);
		if (!timeo) {
			err = -EAGAIN;
			break;
		}

		release_sock(sk);
		timeo = schedule_timeout(timeo);
		lock_sock_nested(sk, SINGLE_DEPTH_NESTING);

		if (sk->sk_state != BT_LISTEN) {
			err = -EBADFD;
			break;
		}

		if (signal_pending(current)) {
			err = sock_intr_errno(timeo);
			break;
		}
	}
	set_current_state(TASK_RUNNING);
	remove_wait_queue(sk_sleep(sk), &wait);

	if (err)
		goto done;

	newsock->state = SS_CONNECTED;

	BT_DBG("new socket %p", nsk);

done:
	release_sock(sk);
	return err;
}

static int l2cap_sock_getname(struct socket *sock, struct sockaddr *addr, int *len, int peer)
{
	struct sockaddr_l2 *la = (struct sockaddr_l2 *) addr;
	struct sock *sk = sock->sk;

	BT_DBG("sock %p, sk %p", sock, sk);

	addr->sa_family = AF_BLUETOOTH;
	*len = sizeof(struct sockaddr_l2);

	if (peer) {
		la->l2_psm = l2cap_pi(sk)->psm;
		bacpy(&la->l2_bdaddr, &bt_sk(sk)->dst);
		la->l2_cid = cpu_to_le16(l2cap_pi(sk)->dcid);
	} else {
		la->l2_psm = l2cap_pi(sk)->sport;
		bacpy(&la->l2_bdaddr, &bt_sk(sk)->src);
		la->l2_cid = cpu_to_le16(l2cap_pi(sk)->scid);
	}

	return 0;
}

static int __l2cap_wait_ack(struct sock *sk)
{
	DECLARE_WAITQUEUE(wait, current);
	int err = 0;
	int timeo = HZ/5;

	add_wait_queue(sk_sleep(sk), &wait);
	while (l2cap_pi(sk)->unacked_frames > 0 && l2cap_pi(sk)->conn &&
		atomic_read(&l2cap_pi(sk)->ertm_queued)) {
		set_current_state(TASK_INTERRUPTIBLE);

		if (!timeo)
			timeo = HZ/5;

		if (signal_pending(current)) {
			err = sock_intr_errno(timeo);
			break;
		}

		release_sock(sk);
		timeo = schedule_timeout(timeo);
		lock_sock(sk);

		err = sock_error(sk);
		if (err)
			break;
	}
	set_current_state(TASK_RUNNING);
	remove_wait_queue(sk_sleep(sk), &wait);
	return err;
}

static void l2cap_ertm_tx_worker(struct work_struct *work)
{
	struct l2cap_pinfo *pi =
		container_of(work, struct l2cap_pinfo, tx_work);
	struct sock *sk = (struct sock *)pi;
	BT_DBG("%p", pi);

	lock_sock(sk);
	l2cap_ertm_send_txq(sk);
	release_sock(sk);
}

static void l2cap_skb_destructor(struct sk_buff *skb)
{
	struct sock *sk = skb->sk;
	int queued;

	queued = atomic_sub_return(1, &l2cap_pi(sk)->ertm_queued);
	if (queued < L2CAP_MIN_ERTM_QUEUED)
		queue_work(_l2cap_wq, &l2cap_pi(sk)->tx_work);
}

static inline void l2cap_do_send(struct sock *sk, struct sk_buff *skb)
{
	struct l2cap_pinfo *pi = l2cap_pi(sk);

	BT_DBG("sk %p, skb %p len %d", sk, skb, skb->len);

	if (pi->ampcon && (pi->amp_move_state == L2CAP_AMP_STATE_STABLE)) {
		BT_DBG("Sending on AMP connection %p %p",
			pi->ampcon, pi->ampchan);
		if (pi->ampchan)
			hci_send_acl(pi->ampcon, pi->ampchan, skb, 0);
		else
			kfree_skb(skb);
	} else {
		u16 flags;

		BT_DBG("Sending on BR/EDR connection %p", pi->conn->hcon);

		if (lmp_no_flush_capable(pi->conn->hcon->hdev) &&
			!l2cap_pi(sk)->flushable)
			flags = ACL_START_NO_FLUSH;
		else
			flags = ACL_START;
		bt_cb(skb)->force_active = pi->force_active;

		hci_send_acl(pi->conn->hcon, NULL, skb, flags);
	}
}

static int l2cap_ertm_send_txq(struct sock *sk)
{
	struct sk_buff *skb, *tx_skb;
	struct l2cap_pinfo *pi = l2cap_pi(sk);
	struct bt_l2cap_control *control;
	int sent = 0;

	BT_DBG("sk %p", sk);

	if (sk->sk_state != BT_CONNECTED)
		return -ENOTCONN;

	if (pi->conn_state & L2CAP_CONN_REMOTE_BUSY)
		return 0;

	if (pi->amp_move_state != L2CAP_AMP_STATE_STABLE)
		return 0;

	while (sk->sk_send_head && (pi->unacked_frames < pi->remote_tx_win) &&
		atomic_read(&pi->ertm_queued) < L2CAP_MAX_ERTM_QUEUED &&
		(pi->tx_state == L2CAP_ERTM_TX_STATE_XMIT)) {

		skb = sk->sk_send_head;

		bt_cb(skb)->retries = 1;
		control = &bt_cb(skb)->control;

		if (pi->conn_state & L2CAP_CONN_SEND_FBIT) {
			control->final = 1;
			pi->conn_state &= ~L2CAP_CONN_SEND_FBIT;
		}
		control->reqseq = pi->buffer_seq;
		pi->last_acked_seq = pi->buffer_seq;
		control->txseq = pi->next_tx_seq;

		if (pi->extended_control) {
			put_unaligned_le32(__pack_extended_control(control),
					skb->data + L2CAP_HDR_SIZE);
		} else {
			put_unaligned_le16(__pack_enhanced_control(control),
					skb->data + L2CAP_HDR_SIZE);
		}

		if (pi->fcs == L2CAP_FCS_CRC16)
			apply_fcs(skb);

		/* Clone after data has been modified. Data is assumed to be
		   read-only (for locking purposes) on cloned sk_buffs.
		 */
		tx_skb = skb_clone(skb, GFP_ATOMIC);

		tx_skb->sk = sk;
		tx_skb->destructor = l2cap_skb_destructor;
		atomic_inc(&pi->ertm_queued);

		l2cap_do_send(sk, tx_skb);

		BT_DBG("Sent txseq %d", (int)control->txseq);

		l2cap_ertm_start_retrans_timer(pi);

		pi->next_tx_seq = __next_seq(pi->next_tx_seq, pi);
		pi->unacked_frames += 1;
		pi->frames_sent += 1;
		sent += 1;

		if (skb_queue_is_last(TX_QUEUE(sk), skb))
			sk->sk_send_head = NULL;
		else
			sk->sk_send_head = skb_queue_next(TX_QUEUE(sk), skb);
	}

	BT_DBG("Sent %d, %d unacked, %d in ERTM queue, %d in HCI queue", sent,
		(int) pi->unacked_frames, skb_queue_len(TX_QUEUE(sk)),
		atomic_read(&pi->ertm_queued));

	return sent;
}

static int l2cap_strm_tx(struct sock *sk, struct sk_buff_head *skbs)
{
	struct sk_buff *skb;
	struct l2cap_pinfo *pi = l2cap_pi(sk);
	struct bt_l2cap_control *control;
	int sent = 0;

	BT_DBG("sk %p, skbs %p", sk, skbs);

	if (sk->sk_state != BT_CONNECTED)
		return -ENOTCONN;

	if (pi->amp_move_state != L2CAP_AMP_STATE_STABLE)
		return 0;

	skb_queue_splice_tail_init(skbs, TX_QUEUE(sk));

	BT_DBG("skb queue empty 0x%2.2x", skb_queue_empty(TX_QUEUE(sk)));
	while (!skb_queue_empty(TX_QUEUE(sk))) {

		skb = skb_dequeue(TX_QUEUE(sk));

		BT_DBG("skb %p", skb);

		bt_cb(skb)->retries = 1;
		control = &bt_cb(skb)->control;

		BT_DBG("control %p", control);

		control->reqseq = 0;
		control->txseq = pi->next_tx_seq;

		if (pi->extended_control) {
			put_unaligned_le32(__pack_extended_control(control),
					skb->data + L2CAP_HDR_SIZE);
		} else {
			put_unaligned_le16(__pack_enhanced_control(control),
					skb->data + L2CAP_HDR_SIZE);
		}

		if (pi->fcs == L2CAP_FCS_CRC16)
			apply_fcs(skb);

		l2cap_do_send(sk, skb);

		BT_DBG("Sent txseq %d", (int)control->txseq);

		pi->next_tx_seq = __next_seq(pi->next_tx_seq, pi);
		pi->frames_sent += 1;
		sent += 1;
	}

	BT_DBG("Sent %d", sent);

	return 0;
}

static int memcpy_fromkvec(unsigned char *kdata, struct kvec *iv, int len)
{
	while (len > 0) {
		if (iv->iov_len) {
			int copy = min_t(unsigned int, len, iv->iov_len);
			memcpy(kdata, iv->iov_base, copy);
			len -= copy;
			kdata += copy;
			iv->iov_base += copy;
			iv->iov_len -= copy;
		}
		iv++;
	}

	return 0;
}

static inline int l2cap_skbuff_fromiovec(struct sock *sk, struct msghdr *msg,
					int len, int count, struct sk_buff *skb,
					int reseg)
{
	struct l2cap_conn *conn = l2cap_pi(sk)->conn;
	struct sk_buff **frag;
	struct sk_buff *final;
	int err, sent = 0;

	BT_DBG("sk %p, msg %p, len %d, count %d, skb %p", sk,
		msg, (int)len, (int)count, skb);

	if (!conn)
		return -ENOTCONN;

	/* When resegmenting, data is copied from kernel space */
	if (reseg) {
		err = memcpy_fromkvec(skb_put(skb, count),
				(struct kvec *) msg->msg_iov, count);
	} else {
		err = memcpy_fromiovec(skb_put(skb, count), msg->msg_iov,
					count);
	}

	if (err)
		return -EFAULT;

	sent += count;
	len  -= count;
	final = skb;

	/* Continuation fragments (no L2CAP header) */
	frag = &skb_shinfo(skb)->frag_list;
	while (len) {
		int skblen;
		count = min_t(unsigned int, conn->mtu, len);

		/* Add room for the FCS if it fits */
		if (bt_cb(skb)->control.fcs == L2CAP_FCS_CRC16 &&
			len + L2CAP_FCS_SIZE <= conn->mtu)
			skblen = count + L2CAP_FCS_SIZE;
		else
			skblen = count;

		/* Don't use bt_skb_send_alloc() while resegmenting, since
		 * it is not ok to block.
		 */
		if (reseg) {
			*frag = bt_skb_alloc(skblen, GFP_ATOMIC);
			if (*frag)
				skb_set_owner_w(*frag, sk);
		} else {
			*frag = bt_skb_send_alloc(sk, skblen,
					msg->msg_flags & MSG_DONTWAIT, &err);
		}

		if (!*frag)
			return -EFAULT;

		/* When resegmenting, data is copied from kernel space */
		if (reseg) {
			err = memcpy_fromkvec(skb_put(*frag, count),
						(struct kvec *) msg->msg_iov,
						count);
		} else {
			err = memcpy_fromiovec(skb_put(*frag, count),
						msg->msg_iov, count);
		}

		if (err)
			return -EFAULT;

		sent += count;
		len  -= count;

		final = *frag;

		frag = &(*frag)->next;
	}

	if (bt_cb(skb)->control.fcs == L2CAP_FCS_CRC16) {
		if (skb_tailroom(final) < L2CAP_FCS_SIZE) {
			if (reseg) {
				*frag = bt_skb_alloc(L2CAP_FCS_SIZE,
						GFP_ATOMIC);
				if (*frag)
					skb_set_owner_w(*frag, sk);
			} else {
				*frag = bt_skb_send_alloc(sk, L2CAP_FCS_SIZE,
						msg->msg_flags & MSG_DONTWAIT,
						&err);
			}

			if (!*frag)
				return -EFAULT;

			final = *frag;
		}

		skb_put(final, L2CAP_FCS_SIZE);
	}

	return sent;
}

static struct sk_buff *l2cap_create_connless_pdu(struct sock *sk, struct msghdr *msg, size_t len)
{
	struct l2cap_conn *conn = l2cap_pi(sk)->conn;
	struct sk_buff *skb;
	int err, count, hlen = L2CAP_HDR_SIZE + 2;
	struct l2cap_hdr *lh;

	BT_DBG("sk %p len %d", sk, (int)len);

	count = min_t(unsigned int, (conn->mtu - hlen), len);
	skb = bt_skb_send_alloc(sk, count + hlen,
			msg->msg_flags & MSG_DONTWAIT, &err);
	if (!skb)
		return ERR_PTR(-ENOMEM);

	/* Create L2CAP header */
	lh = (struct l2cap_hdr *) skb_put(skb, L2CAP_HDR_SIZE);
	lh->cid = cpu_to_le16(l2cap_pi(sk)->dcid);
	lh->len = cpu_to_le16(len + (hlen - L2CAP_HDR_SIZE));
	put_unaligned_le16(l2cap_pi(sk)->psm, skb_put(skb, 2));

	err = l2cap_skbuff_fromiovec(sk, msg, len, count, skb, 0);
	if (unlikely(err < 0)) {
		kfree_skb(skb);
		return ERR_PTR(err);
	}
	return skb;
}

static struct sk_buff *l2cap_create_basic_pdu(struct sock *sk, struct msghdr *msg, size_t len)
{
	struct l2cap_conn *conn = l2cap_pi(sk)->conn;
	struct sk_buff *skb;
	int err, count, hlen = L2CAP_HDR_SIZE;
	struct l2cap_hdr *lh;

	BT_DBG("sk %p len %d", sk, (int)len);

	count = min_t(unsigned int, (conn->mtu - hlen), len);
	skb = bt_skb_send_alloc(sk, count + hlen,
			msg->msg_flags & MSG_DONTWAIT, &err);
	if (!skb)
		return ERR_PTR(-ENOMEM);

	/* Create L2CAP header */
	lh = (struct l2cap_hdr *) skb_put(skb, L2CAP_HDR_SIZE);
	lh->cid = cpu_to_le16(l2cap_pi(sk)->dcid);
	lh->len = cpu_to_le16(len + (hlen - L2CAP_HDR_SIZE));

	err = l2cap_skbuff_fromiovec(sk, msg, len, count, skb, 0);
	if (unlikely(err < 0)) {
		kfree_skb(skb);
		return ERR_PTR(err);
	}
	return skb;
}

static struct sk_buff *l2cap_create_iframe_pdu(struct sock *sk,
					struct msghdr *msg, size_t len,
					u16 sdulen, int reseg)
{
	struct sk_buff *skb;
	int err, count, hlen;
	int reserve = 0;
	struct l2cap_hdr *lh;
	u8 fcs = l2cap_pi(sk)->fcs;

	if (l2cap_pi(sk)->extended_control)
		hlen = L2CAP_EXTENDED_HDR_SIZE;
	else
		hlen = L2CAP_ENHANCED_HDR_SIZE;

	if (sdulen)
		hlen += L2CAP_SDULEN_SIZE;

	if (fcs == L2CAP_FCS_CRC16)
		hlen += L2CAP_FCS_SIZE;

	BT_DBG("sk %p, msg %p, len %d, sdulen %d, hlen %d",
		sk, msg, (int)len, (int)sdulen, hlen);

	count = min_t(unsigned int, (l2cap_pi(sk)->conn->mtu - hlen), len);

	/* Allocate extra headroom for Qualcomm PAL.  This is only
	 * necessary in two places (here and when creating sframes)
	 * because only unfragmented iframes and sframes are sent
	 * using AMP controllers.
	 */
	if (l2cap_pi(sk)->ampcon &&
			l2cap_pi(sk)->ampcon->hdev->manufacturer == 0x001d)
		reserve = BT_SKB_RESERVE_80211;

	/* Don't use bt_skb_send_alloc() while resegmenting, since
	 * it is not ok to block.
	 */
	if (reseg) {
		skb = bt_skb_alloc(count + hlen + reserve, GFP_ATOMIC);
		if (skb)
			skb_set_owner_w(skb, sk);
	} else {
		skb = bt_skb_send_alloc(sk, count + hlen + reserve,
					msg->msg_flags & MSG_DONTWAIT, &err);
	}
	if (!skb)
		return ERR_PTR(-ENOMEM);

	if (reserve)
		skb_reserve(skb, reserve);

	bt_cb(skb)->control.fcs = fcs;

	/* Create L2CAP header */
	lh = (struct l2cap_hdr *) skb_put(skb, L2CAP_HDR_SIZE);
	lh->cid = cpu_to_le16(l2cap_pi(sk)->dcid);
	lh->len = cpu_to_le16(len + hlen - L2CAP_HDR_SIZE);

	/* Control header is populated later */
	if (l2cap_pi(sk)->extended_control)
		put_unaligned_le32(0, skb_put(skb, 4));
	else
		put_unaligned_le16(0, skb_put(skb, 2));

	if (sdulen)
		put_unaligned_le16(sdulen, skb_put(skb, L2CAP_SDULEN_SIZE));

	err = l2cap_skbuff_fromiovec(sk, msg, len, count, skb, reseg);
	if (unlikely(err < 0)) {
		BT_DBG("err %d", err);
		kfree_skb(skb);
		return ERR_PTR(err);
	}

	bt_cb(skb)->retries = 0;
	return skb;
}

static void l2cap_ertm_process_reqseq(struct sock *sk, u16 reqseq)
{
	struct l2cap_pinfo *pi;
	struct sk_buff *acked_skb;
	u16 ackseq;

	BT_DBG("sk %p, reqseq %d", sk, (int) reqseq);

	pi = l2cap_pi(sk);

	if (pi->unacked_frames == 0 || reqseq == pi->expected_ack_seq)
		return;

	BT_DBG("expected_ack_seq %d, unacked_frames %d",
		(int) pi->expected_ack_seq, (int) pi->unacked_frames);

	for (ackseq = pi->expected_ack_seq; ackseq != reqseq;
		ackseq = __next_seq(ackseq, pi)) {

		acked_skb = l2cap_ertm_seq_in_queue(TX_QUEUE(sk), ackseq);
		if (acked_skb) {
			skb_unlink(acked_skb, TX_QUEUE(sk));
			kfree_skb(acked_skb);
			pi->unacked_frames--;
		}
	}

	pi->expected_ack_seq = reqseq;

	if (pi->unacked_frames == 0)
		l2cap_ertm_stop_retrans_timer(pi);

	BT_DBG("unacked_frames %d", (int) pi->unacked_frames);
}

static struct sk_buff *l2cap_create_sframe_pdu(struct sock *sk, u32 control)
{
	struct sk_buff *skb;
	int len;
	int reserve = 0;
	struct l2cap_hdr *lh;

	if (l2cap_pi(sk)->extended_control)
		len = L2CAP_EXTENDED_HDR_SIZE;
	else
		len = L2CAP_ENHANCED_HDR_SIZE;

	if (l2cap_pi(sk)->fcs == L2CAP_FCS_CRC16)
		len += L2CAP_FCS_SIZE;

	/* Allocate extra headroom for Qualcomm PAL */
	if (l2cap_pi(sk)->ampcon &&
			l2cap_pi(sk)->ampcon->hdev->manufacturer == 0x001d)
		reserve = BT_SKB_RESERVE_80211;

	skb = bt_skb_alloc(len + reserve, GFP_ATOMIC);

	if (!skb)
		return ERR_PTR(-ENOMEM);

	if (reserve)
		skb_reserve(skb, reserve);

	lh = (struct l2cap_hdr *) skb_put(skb, L2CAP_HDR_SIZE);
	lh->cid = cpu_to_le16(l2cap_pi(sk)->dcid);
	lh->len = cpu_to_le16(len - L2CAP_HDR_SIZE);

	if (l2cap_pi(sk)->extended_control)
		put_unaligned_le32(control, skb_put(skb, 4));
	else
		put_unaligned_le16(control, skb_put(skb, 2));

	if (l2cap_pi(sk)->fcs == L2CAP_FCS_CRC16) {
		u16 fcs = crc16(0, (u8 *) skb->data, skb->len);
		put_unaligned_le16(fcs, skb_put(skb, L2CAP_FCS_SIZE));
	}

	return skb;
}

static void l2cap_ertm_send_sframe(struct sock *sk,
				struct bt_l2cap_control *control)
{
	struct l2cap_pinfo *pi;
	struct sk_buff *skb;
	u32 control_field;

	BT_DBG("sk %p, control %p", sk, control);

	if (control->frame_type != 's')
		return;

	pi = l2cap_pi(sk);

	if (pi->amp_move_state != L2CAP_AMP_STATE_STABLE &&
		pi->amp_move_state != L2CAP_AMP_STATE_RESEGMENT) {
		BT_DBG("AMP error - attempted S-Frame send during AMP move");
		return;
	}

	if ((pi->conn_state & L2CAP_CONN_SEND_FBIT) && !control->poll) {
		control->final = 1;
		pi->conn_state &= ~L2CAP_CONN_SEND_FBIT;
	}

	if (control->super == L2CAP_SFRAME_RR)
		pi->conn_state &= ~L2CAP_CONN_SENT_RNR;
	else if (control->super == L2CAP_SFRAME_RNR)
		pi->conn_state |= L2CAP_CONN_SENT_RNR;

	if (control->super != L2CAP_SFRAME_SREJ) {
		pi->last_acked_seq = control->reqseq;
		l2cap_ertm_stop_ack_timer(pi);
	}

	BT_DBG("reqseq %d, final %d, poll %d, super %d", (int) control->reqseq,
		(int) control->final, (int) control->poll,
		(int) control->super);

	if (pi->extended_control)
		control_field = __pack_extended_control(control);
	else
		control_field = __pack_enhanced_control(control);

	skb = l2cap_create_sframe_pdu(sk, control_field);
	if (!IS_ERR(skb))
		l2cap_do_send(sk, skb);
}

static void l2cap_ertm_send_ack(struct sock *sk)
{
	struct l2cap_pinfo *pi = l2cap_pi(sk);
	struct bt_l2cap_control control;
	u16 frames_to_ack = __delta_seq(pi->buffer_seq, pi->last_acked_seq, pi);
	int threshold;

	BT_DBG("sk %p", sk);
	BT_DBG("last_acked_seq %d, buffer_seq %d", (int)pi->last_acked_seq,
		(int)pi->buffer_seq);

	memset(&control, 0, sizeof(control));
	control.frame_type = 's';

	if ((pi->conn_state & L2CAP_CONN_LOCAL_BUSY) &&
		pi->rx_state == L2CAP_ERTM_RX_STATE_RECV) {
		l2cap_ertm_stop_ack_timer(pi);
		control.super = L2CAP_SFRAME_RNR;
		control.reqseq = pi->buffer_seq;
		l2cap_ertm_send_sframe(sk, &control);
	} else {
		if (!(pi->conn_state & L2CAP_CONN_REMOTE_BUSY)) {
			l2cap_ertm_send_txq(sk);
			/* If any i-frames were sent, they included an ack */
			if (pi->buffer_seq == pi->last_acked_seq)
				frames_to_ack = 0;
		}

		/* Ack now if the tx window is 3/4ths full.
		 * Calculate without mul or div
		 */
		threshold = pi->tx_win;
		threshold += threshold << 1;
		threshold >>= 2;

		BT_DBG("frames_to_ack %d, threshold %d", (int)frames_to_ack,
			threshold);

		if (frames_to_ack >= threshold) {
			l2cap_ertm_stop_ack_timer(pi);
			control.super = L2CAP_SFRAME_RR;
			control.reqseq = pi->buffer_seq;
			l2cap_ertm_send_sframe(sk, &control);
			frames_to_ack = 0;
		}

		if (frames_to_ack)
			l2cap_ertm_start_ack_timer(pi);
	}
}

static void l2cap_ertm_send_rr_or_rnr(struct sock *sk, bool poll)
{
	struct l2cap_pinfo *pi;
	struct bt_l2cap_control control;

	BT_DBG("sk %p, poll %d", sk, (int) poll);

	pi = l2cap_pi(sk);

	memset(&control, 0, sizeof(control));
	control.frame_type = 's';
	control.poll = poll;

	if (pi->conn_state & L2CAP_CONN_LOCAL_BUSY)
		control.super = L2CAP_SFRAME_RNR;
	else
		control.super = L2CAP_SFRAME_RR;

	control.reqseq = pi->buffer_seq;
	l2cap_ertm_send_sframe(sk, &control);
}

static void l2cap_ertm_send_i_or_rr_or_rnr(struct sock *sk)
{
	struct l2cap_pinfo *pi;
	struct bt_l2cap_control control;

	BT_DBG("sk %p", sk);

	pi = l2cap_pi(sk);

	memset(&control, 0, sizeof(control));
	control.frame_type = 's';
	control.final = 1;
	control.reqseq = pi->buffer_seq;
	pi->conn_state |= L2CAP_CONN_SEND_FBIT;

	if (pi->conn_state & L2CAP_CONN_LOCAL_BUSY) {
		control.super = L2CAP_SFRAME_RNR;
		l2cap_ertm_send_sframe(sk, &control);
	}

	if ((pi->conn_state & L2CAP_CONN_REMOTE_BUSY) &&
		(pi->unacked_frames > 0))
		l2cap_ertm_start_retrans_timer(pi);

	pi->conn_state &= ~L2CAP_CONN_REMOTE_BUSY;

	/* Send pending iframes */
	l2cap_ertm_send_txq(sk);

	if (pi->conn_state & L2CAP_CONN_SEND_FBIT) {
		/* F-bit wasn't sent in an s-frame or i-frame yet, so
		 * send it now.
		 */
		control.super = L2CAP_SFRAME_RR;
		l2cap_ertm_send_sframe(sk, &control);
	}
}

static void l2cap_ertm_send_srej(struct sock *sk, u16 txseq)
{
	struct bt_l2cap_control control;
	struct l2cap_pinfo *pi;
	u16 seq;

	BT_DBG("sk %p, txseq %d", sk, (int)txseq);

	pi = l2cap_pi(sk);
	memset(&control, 0, sizeof(control));
	control.frame_type = 's';
	control.super = L2CAP_SFRAME_SREJ;

	for (seq = pi->expected_tx_seq; seq != txseq;
		seq = __next_seq(seq, pi)) {
		if (!l2cap_ertm_seq_in_queue(SREJ_QUEUE(pi), seq)) {
			control.reqseq = seq;
			l2cap_ertm_send_sframe(sk, &control);
			l2cap_seq_list_append(&pi->srej_list, seq);
		}
	}

	pi->expected_tx_seq = __next_seq(txseq, pi);
}

static void l2cap_ertm_send_srej_tail(struct sock *sk)
{
	struct bt_l2cap_control control;
	struct l2cap_pinfo *pi;

	BT_DBG("sk %p", sk);

	pi = l2cap_pi(sk);

	if (pi->srej_list.tail == L2CAP_SEQ_LIST_CLEAR)
		return;

	memset(&control, 0, sizeof(control));
	control.frame_type = 's';
	control.super = L2CAP_SFRAME_SREJ;
	control.reqseq = pi->srej_list.tail;
	l2cap_ertm_send_sframe(sk, &control);
}

static void l2cap_ertm_send_srej_list(struct sock *sk, u16 txseq)
{
	struct bt_l2cap_control control;
	struct l2cap_pinfo *pi;
	u16 initial_head;
	u16 seq;

	BT_DBG("sk %p, txseq %d", sk, (int) txseq);

	pi = l2cap_pi(sk);
	memset(&control, 0, sizeof(control));
	control.frame_type = 's';
	control.super = L2CAP_SFRAME_SREJ;

	/* Capture initial list head to allow only one pass through the list. */
	initial_head = pi->srej_list.head;

	do {
		seq = l2cap_seq_list_pop(&pi->srej_list);
		if ((seq == txseq) || (seq == L2CAP_SEQ_LIST_CLEAR))
			break;

		control.reqseq = seq;
		l2cap_ertm_send_sframe(sk, &control);
		l2cap_seq_list_append(&pi->srej_list, seq);
	} while (pi->srej_list.head != initial_head);
}

static void l2cap_ertm_abort_rx_srej_sent(struct sock *sk)
{
	struct l2cap_pinfo *pi = l2cap_pi(sk);
	BT_DBG("sk %p", sk);

	pi->expected_tx_seq = pi->buffer_seq;
	l2cap_seq_list_clear(&l2cap_pi(sk)->srej_list);
	skb_queue_purge(SREJ_QUEUE(sk));
	pi->rx_state = L2CAP_ERTM_RX_STATE_RECV;
}

static int l2cap_ertm_tx_state_xmit(struct sock *sk,
				struct bt_l2cap_control *control,
				struct sk_buff_head *skbs, u8 event)
{
	struct l2cap_pinfo *pi;
	int err = 0;

	BT_DBG("sk %p, control %p, skbs %p, event %d", sk, control, skbs,
		(int)event);
	pi = l2cap_pi(sk);

	switch (event) {
	case L2CAP_ERTM_EVENT_DATA_REQUEST:
		if (sk->sk_send_head == NULL)
			sk->sk_send_head = skb_peek(skbs);

		skb_queue_splice_tail_init(skbs, TX_QUEUE(sk));
		l2cap_ertm_send_txq(sk);
		break;
	case L2CAP_ERTM_EVENT_LOCAL_BUSY_DETECTED:
		BT_DBG("Enter LOCAL_BUSY");
		pi->conn_state |= L2CAP_CONN_LOCAL_BUSY;

		if (pi->rx_state == L2CAP_ERTM_RX_STATE_SREJ_SENT) {
			/* The SREJ_SENT state must be aborted if we are to
			 * enter the LOCAL_BUSY state.
			 */
			l2cap_ertm_abort_rx_srej_sent(sk);
		}

		l2cap_ertm_send_ack(sk);

		break;
	case L2CAP_ERTM_EVENT_LOCAL_BUSY_CLEAR:
		BT_DBG("Exit LOCAL_BUSY");
		pi->conn_state &= ~L2CAP_CONN_LOCAL_BUSY;

		if (pi->amp_move_state == L2CAP_AMP_STATE_WAIT_LOCAL_BUSY) {
			if (pi->amp_move_role == L2CAP_AMP_MOVE_INITIATOR) {
				pi->amp_move_state =
					L2CAP_AMP_STATE_WAIT_MOVE_CONFIRM_RSP;
				l2cap_send_move_chan_cfm(pi->conn, pi,
						pi->scid,
						L2CAP_MOVE_CHAN_CONFIRMED);
				l2cap_sock_set_timer(sk, L2CAP_MOVE_TIMEOUT);
			} else if (pi->amp_move_role ==
					L2CAP_AMP_MOVE_RESPONDER) {
				pi->amp_move_state =
					L2CAP_AMP_STATE_WAIT_MOVE_CONFIRM;
				l2cap_send_move_chan_rsp(pi->conn,
						pi->amp_move_cmd_ident,
						pi->dcid,
						L2CAP_MOVE_CHAN_SUCCESS);
			}
			break;
		}

		if (pi->amp_move_role == L2CAP_AMP_MOVE_NONE &&
			(pi->conn_state & L2CAP_CONN_SENT_RNR)) {
			struct bt_l2cap_control local_control;

			memset(&local_control, 0, sizeof(local_control));
			local_control.frame_type = 's';
			local_control.super = L2CAP_SFRAME_RR;
			local_control.poll = 1;
			local_control.reqseq = pi->buffer_seq;
			l2cap_ertm_send_sframe(sk, &local_control);

			pi->retry_count = 1;
			l2cap_ertm_start_monitor_timer(pi);
			pi->tx_state = L2CAP_ERTM_TX_STATE_WAIT_F;
		}
		break;
	case L2CAP_ERTM_EVENT_RECV_REQSEQ_AND_FBIT:
		l2cap_ertm_process_reqseq(sk, control->reqseq);
		break;
	case L2CAP_ERTM_EVENT_EXPLICIT_POLL:
		l2cap_ertm_send_rr_or_rnr(sk, 1);
		pi->retry_count = 1;
		l2cap_ertm_start_monitor_timer(pi);
		l2cap_ertm_stop_ack_timer(pi);
		pi->tx_state = L2CAP_ERTM_TX_STATE_WAIT_F;
		break;
	case L2CAP_ERTM_EVENT_RETRANS_TIMER_EXPIRES:
		l2cap_ertm_send_rr_or_rnr(sk, 1);
		pi->retry_count = 1;
		l2cap_ertm_start_monitor_timer(pi);
		pi->tx_state = L2CAP_ERTM_TX_STATE_WAIT_F;
		break;
	case L2CAP_ERTM_EVENT_RECV_FBIT:
		/* Nothing to process */
		break;
	default:
		break;
	}

	return err;
}

static int l2cap_ertm_tx_state_wait_f(struct sock *sk,
				struct bt_l2cap_control *control,
				struct sk_buff_head *skbs, u8 event)
{
	struct l2cap_pinfo *pi;
	int err = 0;

	BT_DBG("sk %p, control %p, skbs %p, event %d", sk, control, skbs,
		(int)event);
	pi = l2cap_pi(sk);

	switch (event) {
	case L2CAP_ERTM_EVENT_DATA_REQUEST:
		if (sk->sk_send_head == NULL)
			sk->sk_send_head = skb_peek(skbs);
		/* Queue data, but don't send. */
		skb_queue_splice_tail_init(skbs, TX_QUEUE(sk));
		break;
	case L2CAP_ERTM_EVENT_LOCAL_BUSY_DETECTED:
		BT_DBG("Enter LOCAL_BUSY");
		pi->conn_state |= L2CAP_CONN_LOCAL_BUSY;

		if (pi->rx_state == L2CAP_ERTM_RX_STATE_SREJ_SENT) {
			/* The SREJ_SENT state must be aborted if we are to
			 * enter the LOCAL_BUSY state.
			 */
			l2cap_ertm_abort_rx_srej_sent(sk);
		}

		l2cap_ertm_send_ack(sk);

		break;
	case L2CAP_ERTM_EVENT_LOCAL_BUSY_CLEAR:
		BT_DBG("Exit LOCAL_BUSY");
		pi->conn_state &= ~L2CAP_CONN_LOCAL_BUSY;

		if (pi->conn_state & L2CAP_CONN_SENT_RNR) {
			struct bt_l2cap_control local_control;
			memset(&local_control, 0, sizeof(local_control));
			local_control.frame_type = 's';
			local_control.super = L2CAP_SFRAME_RR;
			local_control.poll = 1;
			local_control.reqseq = pi->buffer_seq;
			l2cap_ertm_send_sframe(sk, &local_control);

			pi->retry_count = 1;
			l2cap_ertm_start_monitor_timer(pi);
			pi->tx_state = L2CAP_ERTM_TX_STATE_WAIT_F;
		}
		break;
	case L2CAP_ERTM_EVENT_RECV_REQSEQ_AND_FBIT:
		l2cap_ertm_process_reqseq(sk, control->reqseq);

		/* Fall through */

	case L2CAP_ERTM_EVENT_RECV_FBIT:
		if (control && control->final) {
			l2cap_ertm_stop_monitor_timer(pi);
			if (pi->unacked_frames > 0)
				l2cap_ertm_start_retrans_timer(pi);
			pi->retry_count = 0;
			pi->tx_state = L2CAP_ERTM_TX_STATE_XMIT;
			BT_DBG("recv fbit tx_state 0x2.2%x", pi->tx_state);
		}
		break;
	case L2CAP_ERTM_EVENT_EXPLICIT_POLL:
		/* Ignore */
		break;
	case L2CAP_ERTM_EVENT_MONITOR_TIMER_EXPIRES:
		if ((pi->max_tx == 0) || (pi->retry_count < pi->max_tx)) {
			l2cap_ertm_send_rr_or_rnr(sk, 1);
			l2cap_ertm_start_monitor_timer(pi);
			pi->retry_count += 1;
		} else
			l2cap_send_disconn_req(pi->conn, sk, ECONNABORTED);
		break;
	default:
		break;
	}

	return err;
}

static int l2cap_ertm_tx(struct sock *sk, struct bt_l2cap_control *control,
			struct sk_buff_head *skbs, u8 event)
{
	struct l2cap_pinfo *pi;
	int err = 0;

	BT_DBG("sk %p, control %p, skbs %p, event %d, state %d",
		sk, control, skbs, (int)event, l2cap_pi(sk)->tx_state);

	pi = l2cap_pi(sk);

	switch (pi->tx_state) {
	case L2CAP_ERTM_TX_STATE_XMIT:
		err = l2cap_ertm_tx_state_xmit(sk, control, skbs, event);
		break;
	case L2CAP_ERTM_TX_STATE_WAIT_F:
		err = l2cap_ertm_tx_state_wait_f(sk, control, skbs, event);
		break;
	default:
		/* Ignore event */
		break;
	}

	return err;
}

static int l2cap_segment_sdu(struct sock *sk, struct sk_buff_head* seg_queue,
			struct msghdr *msg, size_t len, int reseg)
{
	struct sk_buff *skb;
	u16 sdu_len;
	size_t pdu_len;
	int err = 0;
	u8 sar;

	BT_DBG("sk %p, msg %p, len %d", sk, msg, (int)len);

	/* It is critical that ERTM PDUs fit in a single HCI fragment,
	 * so fragmented skbs are not used.  The HCI layer's handling
	 * of fragmented skbs is not compatible with ERTM's queueing.
	 */

	/* PDU size is derived from the HCI MTU */
	pdu_len = l2cap_pi(sk)->conn->mtu;

	/* Constrain BR/EDR PDU size to fit within the largest radio packet */
	if (!l2cap_pi(sk)->ampcon)
		pdu_len = min_t(size_t, pdu_len, L2CAP_BREDR_MAX_PAYLOAD);

	/* Adjust for largest possible L2CAP overhead. */
	pdu_len -= L2CAP_EXTENDED_HDR_SIZE + L2CAP_FCS_SIZE;

	/* Remote device may have requested smaller PDUs */
	pdu_len = min_t(size_t, pdu_len, l2cap_pi(sk)->remote_mps);

	if (len <= pdu_len) {
		sar = L2CAP_SAR_UNSEGMENTED;
		sdu_len = 0;
		pdu_len = len;
	} else {
		sar = L2CAP_SAR_START;
		sdu_len = len;
		pdu_len -= L2CAP_SDULEN_SIZE;
	}

	while (len) {
		skb = l2cap_create_iframe_pdu(sk, msg, pdu_len, sdu_len, reseg);

		BT_DBG("iframe skb %p", skb);

		if (IS_ERR(skb)) {
			__skb_queue_purge(seg_queue);
			return PTR_ERR(skb);
		}

		bt_cb(skb)->control.sar = sar;
		__skb_queue_tail(seg_queue, skb);

		len -= pdu_len;
		if (sdu_len) {
			sdu_len = 0;
			pdu_len += L2CAP_SDULEN_SIZE;
		}

		if (len <= pdu_len) {
			sar = L2CAP_SAR_END;
			pdu_len = len;
		} else {
			sar = L2CAP_SAR_CONTINUE;
		}
	}

	return err;
}

static inline int is_initial_frame(u8 sar)
{
	return (sar == L2CAP_SAR_UNSEGMENTED ||
		sar == L2CAP_SAR_START);
}

static inline int l2cap_skbuff_to_kvec(struct sk_buff *skb, struct kvec *iv,
					size_t veclen)
{
	struct sk_buff *frag_iter;

	BT_DBG("skb %p (len %d), iv %p", skb, (int)skb->len, iv);

	if (iv->iov_len + skb->len > veclen)
		return -ENOMEM;

	memcpy(iv->iov_base + iv->iov_len, skb->data, skb->len);
	iv->iov_len += skb->len;

	skb_walk_frags(skb, frag_iter) {
		if (iv->iov_len + skb->len > veclen)
			return -ENOMEM;

		BT_DBG("Copying %d bytes", (int)frag_iter->len);
		memcpy(iv->iov_base + iv->iov_len, frag_iter->data,
			frag_iter->len);
		iv->iov_len += frag_iter->len;
	}

	return 0;
}

static int l2cap_resegment_queue(struct sock *sk, struct sk_buff_head *queue)
{
	void *buf;
	int buflen;
	int err = 0;
	struct sk_buff *skb;
	struct msghdr msg;
	struct kvec iv;
	struct sk_buff_head old_frames;
	struct l2cap_pinfo *pi = l2cap_pi(sk);

	BT_DBG("sk %p", sk);

	if (skb_queue_empty(queue))
		return 0;

	memset(&msg, 0, sizeof(msg));
	msg.msg_iov = (struct iovec *) &iv;

	buflen = pi->omtu + L2CAP_FCS_SIZE;
	buf = kzalloc(buflen, GFP_TEMPORARY);

	if (!buf) {
		BT_DBG("Could not allocate resegmentation buffer");
		return -ENOMEM;
	}

	/* Move current frames off the original queue */
	__skb_queue_head_init(&old_frames);
	skb_queue_splice_tail_init(queue, &old_frames);

	while (!skb_queue_empty(&old_frames)) {
		struct sk_buff_head current_sdu;
		u8 original_sar;

		/* Reassemble each SDU from one or more PDUs */

		iv.iov_base = buf;
		iv.iov_len = 0;

		skb = skb_peek(&old_frames);
		original_sar = bt_cb(skb)->control.sar;

		__skb_unlink(skb, &old_frames);

		/* Append data to SDU */
		if (pi->extended_control)
			skb_pull(skb, L2CAP_EXTENDED_HDR_SIZE);
		else
			skb_pull(skb, L2CAP_ENHANCED_HDR_SIZE);

		if (original_sar == L2CAP_SAR_START)
			skb_pull(skb, L2CAP_SDULEN_SIZE);

		err = l2cap_skbuff_to_kvec(skb, &iv, buflen);

		if (bt_cb(skb)->control.fcs == L2CAP_FCS_CRC16)
			iv.iov_len -= L2CAP_FCS_SIZE;

		/* Free skb */
		kfree_skb(skb);

		if (err)
			break;

		while (!skb_queue_empty(&old_frames) && !err) {
			/* Check next frame */
			skb = skb_peek(&old_frames);

			if (is_initial_frame(bt_cb(skb)->control.sar))
				break;

			__skb_unlink(skb, &old_frames);

			/* Append data to SDU */
			if (pi->extended_control)
				skb_pull(skb, L2CAP_EXTENDED_HDR_SIZE);
			else
				skb_pull(skb, L2CAP_ENHANCED_HDR_SIZE);

			if (bt_cb(skb)->control.sar == L2CAP_SAR_START)
				skb_pull(skb, L2CAP_SDULEN_SIZE);

			err = l2cap_skbuff_to_kvec(skb, &iv, buflen);

			if (bt_cb(skb)->control.fcs == L2CAP_FCS_CRC16)
				iv.iov_len -= L2CAP_FCS_SIZE;

			/* Free skb */
			kfree_skb(skb);
		}

		if (err)
			break;

		/* Segment data */

		__skb_queue_head_init(&current_sdu);

		/* skbs for the SDU were just freed, but the
		 * resegmenting process could produce more, smaller
		 * skbs due to smaller PDUs and reduced HCI MTU.  The
		 * overhead from the sk_buff structs could put us over
		 * the sk_sndbuf limit.
		 *
		 * Since this code is running in response to a
		 * received poll/final packet, it cannot block.
		 * Therefore, memory allocation needs to be allowed by
		 * falling back to bt_skb_alloc() (with
		 * skb_set_owner_w() to maintain sk_wmem_alloc
		 * correctly).
		 */
		msg.msg_iovlen = iv.iov_len;
		err = l2cap_segment_sdu(sk, &current_sdu, &msg,
					msg.msg_iovlen, 1);

		if (err || skb_queue_empty(&current_sdu)) {
			BT_DBG("Error %d resegmenting data for socket %p",
				err, sk);
			__skb_queue_purge(&current_sdu);
			break;
		}

		/* Fix up first PDU SAR bits */
		if (!is_initial_frame(original_sar)) {
			BT_DBG("Changing SAR bits, %d PDUs",
				skb_queue_len(&current_sdu));
			skb = skb_peek(&current_sdu);

			if (skb_queue_len(&current_sdu) == 1) {
				/* Change SAR from 'unsegmented' to 'end' */
				bt_cb(skb)->control.sar = L2CAP_SAR_END;
			} else {
				struct l2cap_hdr *lh;
				size_t hdrlen;

				/* Change SAR from 'start' to 'continue' */
				bt_cb(skb)->control.sar = L2CAP_SAR_CONTINUE;

				/* Start frames contain 2 bytes for
				 * sdulen and continue frames don't.
				 * Must rewrite header to eliminate
				 * sdulen and then adjust l2cap frame
				 * length.
				 */
				if (pi->extended_control)
					hdrlen = L2CAP_EXTENDED_HDR_SIZE;
				else
					hdrlen = L2CAP_ENHANCED_HDR_SIZE;

				memmove(skb->data + L2CAP_SDULEN_SIZE,
					skb->data, hdrlen);
				skb_pull(skb, L2CAP_SDULEN_SIZE);
				lh = (struct l2cap_hdr *)skb->data;
				lh->len = cpu_to_le16(le16_to_cpu(lh->len) -
							L2CAP_SDULEN_SIZE);
			}
		}

		/* Add to queue */
		skb_queue_splice_tail(&current_sdu, queue);
	}

	__skb_queue_purge(&old_frames);
	if (err)
		__skb_queue_purge(queue);

	kfree(buf);

	BT_DBG("Queue resegmented, err=%d", err);
	return err;
}

static void l2cap_resegment_worker(struct work_struct *work)
{
	int err = 0;
	struct l2cap_resegment_work *seg_work =
		container_of(work, struct l2cap_resegment_work, work);
	struct sock *sk = seg_work->sk;

	kfree(seg_work);

	BT_DBG("sk %p", sk);
	lock_sock(sk);

	if (l2cap_pi(sk)->amp_move_state != L2CAP_AMP_STATE_RESEGMENT) {
		release_sock(sk);
		return;
	}

	err = l2cap_resegment_queue(sk, TX_QUEUE(sk));

	l2cap_pi(sk)->amp_move_state = L2CAP_AMP_STATE_STABLE;

	if (skb_queue_empty(TX_QUEUE(sk)))
		sk->sk_send_head = NULL;
	else
		sk->sk_send_head = skb_peek(TX_QUEUE(sk));

	if (err)
		l2cap_send_disconn_req(l2cap_pi(sk)->conn, sk, ECONNRESET);
	else
		l2cap_ertm_send_txq(sk);

	release_sock(sk);
}

static int l2cap_setup_resegment(struct sock *sk)
{
	struct l2cap_resegment_work *seg_work;

	BT_DBG("sk %p", sk);

	if (skb_queue_empty(TX_QUEUE(sk)))
		return 0;

	seg_work = kzalloc(sizeof(*seg_work), GFP_ATOMIC);
	if (!seg_work)
		return -ENOMEM;

	INIT_WORK(&seg_work->work, l2cap_resegment_worker);
	seg_work->sk = sk;

	if (!queue_work(_l2cap_wq, &seg_work->work)) {
		kfree(seg_work);
		return -ENOMEM;
	}

	l2cap_pi(sk)->amp_move_state = L2CAP_AMP_STATE_RESEGMENT;

	return 0;
}

static int l2cap_sock_sendmsg(struct kiocb *iocb, struct socket *sock, struct msghdr *msg, size_t len)
{
	struct sock *sk = sock->sk;
	struct l2cap_pinfo *pi = l2cap_pi(sk);
	struct sk_buff *skb;
	struct sk_buff_head seg_queue;
	int err;
	u8 amp_id;

	BT_DBG("sock %p, sk %p", sock, sk);

	err = sock_error(sk);
	if (err)
		return err;

	if (msg->msg_flags & MSG_OOB)
		return -EOPNOTSUPP;

	lock_sock(sk);

	if (sk->sk_state != BT_CONNECTED) {
		err = -ENOTCONN;
		goto done;
	}

	/* Connectionless channel */
	if (sk->sk_type == SOCK_DGRAM) {
		skb = l2cap_create_connless_pdu(sk, msg, len);
		if (IS_ERR(skb)) {
			err = PTR_ERR(skb);
		} else {
			l2cap_do_send(sk, skb);
			err = len;
		}
		goto done;
	}

	switch (pi->mode) {
	case L2CAP_MODE_BASIC:
		/* Check outgoing MTU */
		if (len > pi->omtu) {
			err = -EMSGSIZE;
			goto done;
		}

		/* Create a basic PDU */
		skb = l2cap_create_basic_pdu(sk, msg, len);
		if (IS_ERR(skb)) {
			err = PTR_ERR(skb);
			goto done;
		}

		l2cap_do_send(sk, skb);
		err = len;
		break;

	case L2CAP_MODE_ERTM:
	case L2CAP_MODE_STREAMING:

		/* Check outgoing MTU */
		if (len > pi->omtu) {
			err = -EMSGSIZE;
			goto done;
		}

		__skb_queue_head_init(&seg_queue);

		/* Do segmentation before calling in to the state machine,
		 * since it's possible to block while waiting for memory
		 * allocation.
		 */
		amp_id = pi->amp_id;
		err = l2cap_segment_sdu(sk, &seg_queue, msg, len, 0);

		/* The socket lock is released while segmenting, so check
		 * that the socket is still connected
		 */
		if (sk->sk_state != BT_CONNECTED) {
			__skb_queue_purge(&seg_queue);
			err = -ENOTCONN;
		}

		if (err) {
			BT_DBG("Error %d, sk_sndbuf %d, sk_wmem_alloc %d",
				err, sk->sk_sndbuf,
				atomic_read(&sk->sk_wmem_alloc));
			break;
		}

		if (pi->amp_id != amp_id) {
			/* Channel moved while unlocked. Resegment. */
			err = l2cap_resegment_queue(sk, &seg_queue);

			if (err)
				break;
		}

		if (pi->mode != L2CAP_MODE_STREAMING)
			err = l2cap_ertm_tx(sk, 0, &seg_queue,
				L2CAP_ERTM_EVENT_DATA_REQUEST);
		else
			err = l2cap_strm_tx(sk, &seg_queue);
		if (!err)
			err = len;

		/* If the skbs were not queued for sending, they'll still be in
		 * seg_queue and need to be purged.
		 */
		__skb_queue_purge(&seg_queue);
		break;

	default:
		BT_DBG("bad state %1.1x", pi->mode);
		err = -EBADFD;
	}

done:
	release_sock(sk);
	return err;
}

static inline int l2cap_rmem_available(struct sock *sk)
{
	BT_DBG("sk_rmem_alloc %d, sk_rcvbuf %d",
		atomic_read(&sk->sk_rmem_alloc), sk->sk_rcvbuf);
	return atomic_read(&sk->sk_rmem_alloc) < sk->sk_rcvbuf / 3;
}

static inline int l2cap_rmem_full(struct sock *sk)
{
	BT_DBG("sk_rmem_alloc %d, sk_rcvbuf %d",
		atomic_read(&sk->sk_rmem_alloc), sk->sk_rcvbuf);
	return atomic_read(&sk->sk_rmem_alloc) > (2 * sk->sk_rcvbuf) / 3;
}

static int l2cap_sock_recvmsg(struct kiocb *iocb, struct socket *sock, struct msghdr *msg, size_t len, int flags)
{
	struct sock *sk = sock->sk;
	int err;

	lock_sock(sk);

	if (sk->sk_state == BT_CONNECT2 && bt_sk(sk)->defer_setup) {
		struct l2cap_conn_rsp rsp;
		struct l2cap_conn *conn = l2cap_pi(sk)->conn;
		u8 buf[128];

		sk->sk_state = BT_CONFIG;

		rsp.scid   = cpu_to_le16(l2cap_pi(sk)->dcid);
		rsp.dcid   = cpu_to_le16(l2cap_pi(sk)->scid);
		rsp.result = cpu_to_le16(L2CAP_CR_SUCCESS);
		rsp.status = cpu_to_le16(L2CAP_CS_NO_INFO);
		l2cap_send_cmd(l2cap_pi(sk)->conn, l2cap_pi(sk)->ident,
					L2CAP_CONN_RSP, sizeof(rsp), &rsp);

		if (l2cap_pi(sk)->conf_state & L2CAP_CONF_REQ_SENT) {
			release_sock(sk);
			return 0;
		}

		l2cap_pi(sk)->conf_state |= L2CAP_CONF_REQ_SENT;
		l2cap_send_cmd(conn, l2cap_get_ident(conn), L2CAP_CONF_REQ,
				l2cap_build_conf_req(sk, buf), buf);
		l2cap_pi(sk)->num_conf_req++;

		release_sock(sk);
		return 0;
	}

	release_sock(sk);

	if (sock->type == SOCK_STREAM)
		err = bt_sock_stream_recvmsg(iocb, sock, msg, len, flags);
	else
		err = bt_sock_recvmsg(iocb, sock, msg, len, flags);

	lock_sock(sk);

	/* Consume any queued incoming frames and update local busy status */
	if (l2cap_pi(sk)->rx_state == L2CAP_ERTM_RX_STATE_SREJ_SENT &&
		l2cap_ertm_rx_queued_iframes(sk))
		l2cap_send_disconn_req(l2cap_pi(sk)->conn, sk, ECONNRESET);
	else if ((l2cap_pi(sk)->conn_state & L2CAP_CONN_LOCAL_BUSY) &&
		l2cap_rmem_available(sk))
		l2cap_ertm_tx(sk, 0, 0, L2CAP_ERTM_EVENT_LOCAL_BUSY_CLEAR);

	release_sock(sk);

	return err;
}

static void l2cap_amp_move_init(struct sock *sk)
{
	BT_DBG("sk %p", sk);

	if (!l2cap_pi(sk)->conn)
		return;

	if (!(l2cap_pi(sk)->conn->fc_mask & L2CAP_FC_A2MP))
		return;

	if (l2cap_pi(sk)->amp_id == 0) {
		if (l2cap_pi(sk)->amp_pref != BT_AMP_POLICY_PREFER_AMP)
			return;
		l2cap_pi(sk)->amp_move_role = L2CAP_AMP_MOVE_INITIATOR;
		l2cap_pi(sk)->amp_move_state = L2CAP_AMP_STATE_WAIT_PREPARE;
		amp_create_physical(l2cap_pi(sk)->conn, sk);
	} else {
		l2cap_pi(sk)->amp_move_role = L2CAP_AMP_MOVE_INITIATOR;
		l2cap_pi(sk)->amp_move_state =
					L2CAP_AMP_STATE_WAIT_MOVE_RSP_SUCCESS;
		l2cap_pi(sk)->amp_move_id = 0;
		l2cap_amp_move_setup(sk);
		l2cap_send_move_chan_req(l2cap_pi(sk)->conn,
					l2cap_pi(sk), l2cap_pi(sk)->scid, 0);
		l2cap_sock_set_timer(sk, L2CAP_MOVE_TIMEOUT);
	}
}

static int l2cap_sock_setsockopt_old(struct socket *sock, int optname, char __user *optval, unsigned int optlen)
{
	struct sock *sk = sock->sk;
	struct l2cap_options opts;
	int len, err = 0;
	u32 opt;

	BT_DBG("sk %p", sk);

	lock_sock(sk);

	switch (optname) {
	case L2CAP_OPTIONS:
		if (sk->sk_state == BT_CONNECTED) {
			err = -EINVAL;
			break;
		}

		opts.imtu     = l2cap_pi(sk)->imtu;
		opts.omtu     = l2cap_pi(sk)->omtu;
		opts.flush_to = l2cap_pi(sk)->flush_to;
		opts.mode     = l2cap_pi(sk)->mode;
		opts.fcs      = l2cap_pi(sk)->fcs;
		opts.max_tx   = l2cap_pi(sk)->max_tx;
		opts.txwin_size = l2cap_pi(sk)->tx_win;

		len = min_t(unsigned int, sizeof(opts), optlen);
		if (copy_from_user((char *) &opts, optval, len)) {
			err = -EFAULT;
			break;
		}

		if (opts.txwin_size < 1 ||
			opts.txwin_size > L2CAP_TX_WIN_MAX_EXTENDED) {
			err = -EINVAL;
			break;
		}

		l2cap_pi(sk)->mode = opts.mode;
		switch (l2cap_pi(sk)->mode) {
		case L2CAP_MODE_BASIC:
			l2cap_pi(sk)->conf_state &= ~L2CAP_CONF_STATE2_DEVICE;
			break;
		case L2CAP_MODE_ERTM:
		case L2CAP_MODE_STREAMING:
			if (!disable_ertm)
				break;
			/* fall through */
		default:
			err = -EINVAL;
			break;
		}

		l2cap_pi(sk)->imtu = opts.imtu;
		l2cap_pi(sk)->omtu = opts.omtu;
		l2cap_pi(sk)->fcs  = opts.fcs;
		l2cap_pi(sk)->max_tx = opts.max_tx;
		l2cap_pi(sk)->tx_win = opts.txwin_size;
		break;

	case L2CAP_LM:
		if (get_user(opt, (u32 __user *) optval)) {
			err = -EFAULT;
			break;
		}

		if (opt & L2CAP_LM_AUTH)
			l2cap_pi(sk)->sec_level = BT_SECURITY_LOW;
		if (opt & L2CAP_LM_ENCRYPT)
			l2cap_pi(sk)->sec_level = BT_SECURITY_MEDIUM;
		if (opt & L2CAP_LM_SECURE)
			l2cap_pi(sk)->sec_level = BT_SECURITY_HIGH;

		l2cap_pi(sk)->role_switch    = (opt & L2CAP_LM_MASTER);
		l2cap_pi(sk)->force_reliable = (opt & L2CAP_LM_RELIABLE);
		l2cap_pi(sk)->flushable = (opt & L2CAP_LM_FLUSHABLE);
		break;

	default:
		err = -ENOPROTOOPT;
		break;
	}

	release_sock(sk);
	return err;
}

static int l2cap_sock_setsockopt(struct socket *sock, int level, int optname, char __user *optval, unsigned int optlen)
{
	struct sock *sk = sock->sk;
	struct bt_security sec;
	struct bt_power pwr;
	int len, err = 0;
	u32 opt;

	BT_DBG("sk %p", sk);

	if (level == SOL_L2CAP)
		return l2cap_sock_setsockopt_old(sock, optname, optval, optlen);

	if (level != SOL_BLUETOOTH)
		return -ENOPROTOOPT;

	lock_sock(sk);

	switch (optname) {
	case BT_SECURITY:
		if (sk->sk_type != SOCK_SEQPACKET && sk->sk_type != SOCK_STREAM
				&& sk->sk_type != SOCK_RAW) {
			err = -EINVAL;
			break;
		}

		sec.level = BT_SECURITY_LOW;

		len = min_t(unsigned int, sizeof(sec), optlen);
		if (copy_from_user((char *) &sec, optval, len)) {
			err = -EFAULT;
			break;
		}

		if (sec.level < BT_SECURITY_LOW ||
					sec.level > BT_SECURITY_HIGH) {
			err = -EINVAL;
			break;
		}

		l2cap_pi(sk)->sec_level = sec.level;
		break;

	case BT_DEFER_SETUP:
		if (sk->sk_state != BT_BOUND && sk->sk_state != BT_LISTEN) {
			err = -EINVAL;
			break;
		}

		if (get_user(opt, (u32 __user *) optval)) {
			err = -EFAULT;
			break;
		}

		bt_sk(sk)->defer_setup = opt;
		break;
    case BT_POWER:
		if (sk->sk_type != SOCK_SEQPACKET && sk->sk_type != SOCK_STREAM
			&& sk->sk_type != SOCK_RAW) {
			err = -EINVAL;
			break;
		}

		pwr.force_active = 1;

		len = min_t(unsigned int, sizeof(pwr), optlen);
		if (copy_from_user((char *) &pwr, optval, len)) {
			err = -EFAULT;
			break;
		}
		BT_DBG("BT_POWER setting %d", pwr.force_active);
		l2cap_pi(sk)->force_active = pwr.force_active;
		break;

	case BT_AMP_POLICY:
		if (get_user(opt, (u32 __user *) optval)) {
			err = -EFAULT;
			break;
		}

		if ((opt > BT_AMP_POLICY_PREFER_BR_EDR) ||
			((l2cap_pi(sk)->mode != L2CAP_MODE_ERTM) &&
			 (l2cap_pi(sk)->mode != L2CAP_MODE_STREAMING))) {
			err = -EINVAL;
			break;
		}

		l2cap_pi(sk)->amp_pref = (u8) opt;
		BT_DBG("BT_AMP_POLICY now %d", opt);

		if ((sk->sk_state == BT_CONNECTED) &&
			(l2cap_pi(sk)->amp_move_role == L2CAP_AMP_MOVE_NONE) &&
			(l2cap_pi(sk)->conn->fc_mask & L2CAP_FC_A2MP))
			l2cap_amp_move_init(sk);
		else
			l2cap_pi(sk)->conn_state |= L2CAP_CONN_MOVE_PENDING;

		break;

	default:
		err = -ENOPROTOOPT;
		break;
	}

	release_sock(sk);
	return err;
}

static int l2cap_sock_getsockopt_old(struct socket *sock, int optname, char __user *optval, int __user *optlen)
{
	struct sock *sk = sock->sk;
	struct l2cap_options opts;
	struct l2cap_conninfo cinfo;
	int len, err = 0;
	u32 opt;

	BT_DBG("sk %p", sk);

	if (get_user(len, optlen))
		return -EFAULT;

	lock_sock(sk);

	switch (optname) {
	case L2CAP_OPTIONS:
		opts.imtu     = l2cap_pi(sk)->imtu;
		opts.omtu     = l2cap_pi(sk)->omtu;
		opts.flush_to = l2cap_pi(sk)->flush_to;
		opts.mode     = l2cap_pi(sk)->mode;
		opts.fcs      = l2cap_pi(sk)->fcs;
		opts.max_tx   = l2cap_pi(sk)->max_tx;
		opts.txwin_size = l2cap_pi(sk)->tx_win;

		len = min_t(unsigned int, len, sizeof(opts));
		if (copy_to_user(optval, (char *) &opts, len))
			err = -EFAULT;

		break;

	case L2CAP_LM:
		switch (l2cap_pi(sk)->sec_level) {
		case BT_SECURITY_LOW:
			opt = L2CAP_LM_AUTH;
			break;
		case BT_SECURITY_MEDIUM:
			opt = L2CAP_LM_AUTH | L2CAP_LM_ENCRYPT;
			break;
		case BT_SECURITY_HIGH:
			opt = L2CAP_LM_AUTH | L2CAP_LM_ENCRYPT |
							L2CAP_LM_SECURE;
			break;
		default:
			opt = 0;
			break;
		}

		if (l2cap_pi(sk)->role_switch)
			opt |= L2CAP_LM_MASTER;

		if (l2cap_pi(sk)->force_reliable)
			opt |= L2CAP_LM_RELIABLE;

		if (l2cap_pi(sk)->flushable)
			opt |= L2CAP_LM_FLUSHABLE;

		if (put_user(opt, (u32 __user *) optval))
			err = -EFAULT;
		break;

	case L2CAP_CONNINFO:
		if (sk->sk_state != BT_CONNECTED &&
					!(sk->sk_state == BT_CONNECT2 &&
						bt_sk(sk)->defer_setup)) {
			err = -ENOTCONN;
			break;
		}

		cinfo.hci_handle = l2cap_pi(sk)->conn->hcon->handle;
		memcpy(cinfo.dev_class, l2cap_pi(sk)->conn->hcon->dev_class, 3);

		len = min_t(unsigned int, len, sizeof(cinfo));
		if (copy_to_user(optval, (char *) &cinfo, len))
			err = -EFAULT;

		break;

	default:
		err = -ENOPROTOOPT;
		break;
	}

	release_sock(sk);
	return err;
}

static int l2cap_sock_getsockopt(struct socket *sock, int level, int optname, char __user *optval, int __user *optlen)
{
	struct sock *sk = sock->sk;
	struct bt_security sec;
	struct bt_power pwr;
	int len, err = 0;

	BT_DBG("sk %p", sk);

	if (level == SOL_L2CAP)
		return l2cap_sock_getsockopt_old(sock, optname, optval, optlen);

	if (level != SOL_BLUETOOTH)
		return -ENOPROTOOPT;

	if (get_user(len, optlen))
		return -EFAULT;

	lock_sock(sk);

	switch (optname) {
	case BT_SECURITY:
		if (sk->sk_type != SOCK_SEQPACKET && sk->sk_type != SOCK_STREAM
				&& sk->sk_type != SOCK_RAW) {
			err = -EINVAL;
			break;
		}

		sec.level = l2cap_pi(sk)->sec_level;

		len = min_t(unsigned int, len, sizeof(sec));
		if (copy_to_user(optval, (char *) &sec, len))
			err = -EFAULT;

		break;

	case BT_DEFER_SETUP:
		if (sk->sk_state != BT_BOUND && sk->sk_state != BT_LISTEN) {
			err = -EINVAL;
			break;
		}

		if (put_user(bt_sk(sk)->defer_setup, (u32 __user *) optval))
			err = -EFAULT;

		break;
	case BT_POWER:
		if (sk->sk_type != SOCK_SEQPACKET && sk->sk_type != SOCK_STREAM
				&& sk->sk_type != SOCK_RAW) {
			err = -EINVAL;
			break;
		}

		pwr.force_active = l2cap_pi(sk)->force_active;

		len = min_t(unsigned int, len, sizeof(pwr));
		if (copy_to_user(optval, (char *) &pwr, len))
			err = -EFAULT;

		break;
	case BT_AMP_POLICY:

		if (put_user(l2cap_pi(sk)->amp_pref, (u32 __user *) optval))
			err = -EFAULT;
		break;

	default:
		err = -ENOPROTOOPT;
		break;
	}

	release_sock(sk);
	return err;
}

static int l2cap_sock_shutdown(struct socket *sock, int how)
{
	struct sock *sk = sock->sk;
	int err = 0;

	BT_DBG("sock %p, sk %p", sock, sk);

	if (!sk)
		return 0;

	lock_sock(sk);
	if (!sk->sk_shutdown) {

		if (l2cap_pi(sk)->mode == L2CAP_MODE_ERTM) {
			err = __l2cap_wait_ack(sk);

			l2cap_ertm_stop_ack_timer(l2cap_pi(sk));
			l2cap_ertm_stop_retrans_timer(l2cap_pi(sk));
			l2cap_ertm_stop_monitor_timer(l2cap_pi(sk));
		}

		sk->sk_shutdown = SHUTDOWN_MASK;
		l2cap_sock_clear_timer(sk);
		__l2cap_sock_close(sk, 0);

		if (sock_flag(sk, SOCK_LINGER) && sk->sk_lingertime)
			err = bt_sock_wait_state(sk, BT_CLOSED,
							sk->sk_lingertime);
	}

	if (!err && sk->sk_err)
		err = -sk->sk_err;

	release_sock(sk);
	return err;
}

static int l2cap_sock_release(struct socket *sock)
{
	struct sock *sk = sock->sk;
	int err;

	BT_DBG("sock %p, sk %p", sock, sk);

	if (!sk)
		return 0;

	err = l2cap_sock_shutdown(sock, 2);

	sock_orphan(sk);
	l2cap_sock_kill(sk);
	return err;
}

static void l2cap_chan_ready(struct sock *sk)
{
	struct sock *parent = bt_sk(sk)->parent;

	BT_DBG("sk %p, parent %p", sk, parent);

	l2cap_pi(sk)->conf_state = 0;
	l2cap_sock_clear_timer(sk);

	if (!parent) {
		/* Outgoing channel.
		 * Wake up socket sleeping on connect.
		 */
		sk->sk_state = BT_CONNECTED;
		sk->sk_state_change(sk);
	} else {
		/* Incoming channel.
		 * Wake up socket sleeping on accept.
		 */
		parent->sk_data_ready(parent, 0);
	}

	if (l2cap_pi(sk)->conn_state & L2CAP_CONN_MOVE_PENDING) {
		l2cap_pi(sk)->conn_state &= ~L2CAP_CONN_MOVE_PENDING;
		l2cap_amp_move_init(sk);
	}
}

/* Copy frame to all raw sockets on that connection */
static void l2cap_raw_recv(struct l2cap_conn *conn, struct sk_buff *skb)
{
	struct l2cap_chan_list *l = &conn->chan_list;
	struct sk_buff *nskb;
	struct sock *sk;

	BT_DBG("conn %p", conn);

	read_lock(&l->lock);
	for (sk = l->head; sk; sk = l2cap_pi(sk)->next_c) {
		if (sk->sk_type != SOCK_RAW)
			continue;

		/* Don't send frame to the socket it came from */
		if (skb->sk == sk)
			continue;
		nskb = skb_clone(skb, GFP_ATOMIC);
		if (!nskb)
			continue;

		if (sock_queue_rcv_skb(sk, nskb))
			kfree_skb(nskb);
	}
	read_unlock(&l->lock);
}

/* ---- L2CAP signalling commands ---- */
static struct sk_buff *l2cap_build_cmd(struct l2cap_conn *conn,
				u8 code, u8 ident, u16 dlen, void *data)
{
	struct sk_buff *skb, **frag;
	struct l2cap_cmd_hdr *cmd;
	struct l2cap_hdr *lh;
	int len, count;
	unsigned int mtu = conn->hcon->hdev->acl_mtu;

	BT_DBG("conn %p, code 0x%2.2x, ident 0x%2.2x, len %d",
			conn, code, ident, dlen);

	len = L2CAP_HDR_SIZE + L2CAP_CMD_HDR_SIZE + dlen;
	count = min_t(unsigned int, mtu, len);

	skb = bt_skb_alloc(count, GFP_ATOMIC);
	if (!skb)
		return NULL;

	lh = (struct l2cap_hdr *) skb_put(skb, L2CAP_HDR_SIZE);
	lh->len = cpu_to_le16(L2CAP_CMD_HDR_SIZE + dlen);
	lh->cid = cpu_to_le16(L2CAP_CID_SIGNALING);

	cmd = (struct l2cap_cmd_hdr *) skb_put(skb, L2CAP_CMD_HDR_SIZE);
	cmd->code  = code;
	cmd->ident = ident;
	cmd->len   = cpu_to_le16(dlen);

	if (dlen) {
		count -= L2CAP_HDR_SIZE + L2CAP_CMD_HDR_SIZE;
		memcpy(skb_put(skb, count), data, count);
		data += count;
	}

	len -= skb->len;

	/* Continuation fragments (no L2CAP header) */
	frag = &skb_shinfo(skb)->frag_list;
	while (len) {
		count = min_t(unsigned int, mtu, len);

		*frag = bt_skb_alloc(count, GFP_ATOMIC);
		if (!*frag)
			goto fail;

		memcpy(skb_put(*frag, count), data, count);

		len  -= count;
		data += count;

		frag = &(*frag)->next;
	}

	return skb;

fail:
	kfree_skb(skb);
	return NULL;
}

static inline int l2cap_get_conf_opt(void **ptr, int *type, int *olen, unsigned long *val)
{
	struct l2cap_conf_opt *opt = *ptr;
	int len;

	len = L2CAP_CONF_OPT_SIZE + opt->len;
	*ptr += len;

	*type = opt->type;
	*olen = opt->len;

	switch (opt->len) {
	case 1:
		*val = *((u8 *) opt->val);
		break;

	case 2:
		*val = __le16_to_cpu(*((__le16 *) opt->val));
		break;

	case 4:
		*val = __le32_to_cpu(*((__le32 *) opt->val));
		break;

	default:
		*val = (unsigned long) opt->val;
		break;
	}

	BT_DBG("type 0x%2.2x len %d val 0x%lx", *type, opt->len, *val);
	return len;
}

static void l2cap_add_conf_opt(void **ptr, u8 type, u8 len, unsigned long val)
{
	struct l2cap_conf_opt *opt = *ptr;

	BT_DBG("type 0x%2.2x len %d val 0x%lx", type, len, val);

	opt->type = type;
	opt->len  = len;

	switch (len) {
	case 1:
		*((u8 *) opt->val)  = val;
		break;

	case 2:
		*((__le16 *) opt->val) = cpu_to_le16(val);
		break;

	case 4:
		*((__le32 *) opt->val) = cpu_to_le32(val);
		break;

	default:
		memcpy(opt->val, (void *) val, len);
		break;
	}

	*ptr += L2CAP_CONF_OPT_SIZE + len;
}

static void l2cap_ertm_ack_timeout(struct work_struct *work)
{
	struct delayed_work *delayed =
		container_of(work, struct delayed_work, work);
	struct l2cap_pinfo *pi =
		container_of(delayed, struct l2cap_pinfo, ack_work);
	struct sock *sk = (struct sock *)pi;
	u16 frames_to_ack;

	BT_DBG("sk %p", sk);

	if (!sk)
		return;

	lock_sock(sk);

	if (!l2cap_pi(sk)->conn) {
		release_sock(sk);
		return;
	}

	frames_to_ack = __delta_seq(l2cap_pi(sk)->buffer_seq,
				    l2cap_pi(sk)->last_acked_seq,
				    l2cap_pi(sk));

	if (frames_to_ack)
		l2cap_ertm_send_rr_or_rnr(sk, 0);

	release_sock(sk);
}

static void l2cap_ertm_retrans_timeout(struct work_struct *work)
{
	struct delayed_work *delayed =
		container_of(work, struct delayed_work, work);
	struct l2cap_pinfo *pi =
		container_of(delayed, struct l2cap_pinfo, retrans_work);
	struct sock *sk = (struct sock *)pi;

	BT_DBG("sk %p", sk);

	if (!sk)
		return;

	lock_sock(sk);

	if (!l2cap_pi(sk)->conn) {
		release_sock(sk);
		return;
	}

	l2cap_ertm_tx(sk, 0, 0, L2CAP_ERTM_EVENT_RETRANS_TIMER_EXPIRES);
	release_sock(sk);
}

static void l2cap_ertm_monitor_timeout(struct work_struct *work)
{
	struct delayed_work *delayed =
		container_of(work, struct delayed_work, work);
	struct l2cap_pinfo *pi =
		container_of(delayed, struct l2cap_pinfo, monitor_work);
	struct sock *sk = (struct sock *)pi;

	BT_DBG("sk %p", sk);

	if (!sk)
		return;

	lock_sock(sk);

	if (!l2cap_pi(sk)->conn) {
		release_sock(sk);
		return;
	}

	l2cap_ertm_tx(sk, 0, 0, L2CAP_ERTM_EVENT_MONITOR_TIMER_EXPIRES);

	release_sock(sk);
}

static inline void l2cap_ertm_init(struct sock *sk)
{
	l2cap_pi(sk)->next_tx_seq = 0;
	l2cap_pi(sk)->expected_tx_seq = 0;
	l2cap_pi(sk)->expected_ack_seq = 0;
	l2cap_pi(sk)->unacked_frames = 0;
	l2cap_pi(sk)->buffer_seq = 0;
	l2cap_pi(sk)->frames_sent = 0;
	l2cap_pi(sk)->last_acked_seq = 0;
	l2cap_pi(sk)->sdu = NULL;
	l2cap_pi(sk)->sdu_last_frag = NULL;
	l2cap_pi(sk)->sdu_len = 0;
	atomic_set(&l2cap_pi(sk)->ertm_queued, 0);

	l2cap_pi(sk)->rx_state = L2CAP_ERTM_RX_STATE_RECV;
	l2cap_pi(sk)->tx_state = L2CAP_ERTM_TX_STATE_XMIT;

	BT_DBG("tx_state 0x2.2%x rx_state 0x2.2%x", l2cap_pi(sk)->tx_state,
		l2cap_pi(sk)->rx_state);

	l2cap_pi(sk)->amp_id = 0;
	l2cap_pi(sk)->amp_move_state = L2CAP_AMP_STATE_STABLE;
	l2cap_pi(sk)->amp_move_role = L2CAP_AMP_MOVE_NONE;
	l2cap_pi(sk)->amp_move_reqseq = 0;
	l2cap_pi(sk)->amp_move_event = 0;

	INIT_DELAYED_WORK(&l2cap_pi(sk)->ack_work, l2cap_ertm_ack_timeout);
	INIT_DELAYED_WORK(&l2cap_pi(sk)->retrans_work,
			l2cap_ertm_retrans_timeout);
	INIT_DELAYED_WORK(&l2cap_pi(sk)->monitor_work,
			l2cap_ertm_monitor_timeout);
	INIT_WORK(&l2cap_pi(sk)->tx_work, l2cap_ertm_tx_worker);
	skb_queue_head_init(SREJ_QUEUE(sk));
	skb_queue_head_init(TX_QUEUE(sk));

	l2cap_seq_list_init(&l2cap_pi(sk)->srej_list, l2cap_pi(sk)->tx_win);
	l2cap_seq_list_init(&l2cap_pi(sk)->retrans_list,
			l2cap_pi(sk)->remote_tx_win);
}

static inline __u8 l2cap_select_mode(__u8 mode, __u16 remote_feat_mask)
{
	switch (mode) {
	case L2CAP_MODE_STREAMING:
	case L2CAP_MODE_ERTM:
		if (l2cap_mode_supported(mode, remote_feat_mask))
			return mode;
		/* fall through */
	default:
		return L2CAP_MODE_BASIC;
	}
}

static void l2cap_setup_txwin(struct l2cap_pinfo *pi)
{
	if (pi->tx_win > L2CAP_TX_WIN_MAX_ENHANCED &&
		(pi->conn->feat_mask & L2CAP_FEAT_EXT_WINDOW)) {
		pi->tx_win_max = L2CAP_TX_WIN_MAX_EXTENDED;
		pi->extended_control = 1;
	} else {
		if (pi->tx_win > L2CAP_TX_WIN_MAX_ENHANCED)
			pi->tx_win = L2CAP_TX_WIN_MAX_ENHANCED;

		pi->tx_win_max = L2CAP_TX_WIN_MAX_ENHANCED;
		pi->extended_control = 0;
	}
}

static int l2cap_build_conf_req(struct sock *sk, void *data)
{
	struct l2cap_pinfo *pi = l2cap_pi(sk);
	struct l2cap_conf_req *req = data;
	struct l2cap_conf_rfc rfc = { .mode = pi->mode };
	void *ptr = req->data;

	BT_DBG("sk %p", sk);

	if (pi->num_conf_req || pi->num_conf_rsp)
		goto done;

	switch (pi->mode) {
	case L2CAP_MODE_STREAMING:
	case L2CAP_MODE_ERTM:
		if (pi->conf_state & L2CAP_CONF_STATE2_DEVICE)
			break;

		/* fall through */
	default:
		pi->mode = l2cap_select_mode(rfc.mode, pi->conn->feat_mask);
		break;
	}

done:
	if (pi->imtu != L2CAP_DEFAULT_MTU)
		l2cap_add_conf_opt(&ptr, L2CAP_CONF_MTU, 2, pi->imtu);

	switch (pi->mode) {
	case L2CAP_MODE_BASIC:
		if (!(pi->conn->feat_mask & L2CAP_FEAT_ERTM) &&
				!(pi->conn->feat_mask & L2CAP_FEAT_STREAMING))
			break;

		rfc.txwin_size      = 0;
		rfc.max_transmit    = 0;
		rfc.retrans_timeout = 0;
		rfc.monitor_timeout = 0;
		rfc.max_pdu_size    = 0;

		l2cap_add_conf_opt(&ptr, L2CAP_CONF_RFC, sizeof(rfc),
							(unsigned long) &rfc);
		break;

	case L2CAP_MODE_ERTM:
		l2cap_setup_txwin(pi);
		if (pi->tx_win > L2CAP_TX_WIN_MAX_ENHANCED)
			rfc.txwin_size = L2CAP_TX_WIN_MAX_ENHANCED;
		else
			rfc.txwin_size = pi->tx_win;
		rfc.max_transmit    = pi->max_tx;
		rfc.retrans_timeout = cpu_to_le16(L2CAP_DEFAULT_RETRANS_TO);
		rfc.monitor_timeout = cpu_to_le16(L2CAP_DEFAULT_MONITOR_TO);
		rfc.max_pdu_size    = cpu_to_le16(L2CAP_DEFAULT_MAX_PDU_SIZE);
		if (L2CAP_DEFAULT_MAX_PDU_SIZE > pi->imtu)
			rfc.max_pdu_size = cpu_to_le16(pi->imtu);

		l2cap_add_conf_opt(&ptr, L2CAP_CONF_RFC, sizeof(rfc),
							(unsigned long) &rfc);

		if ((pi->conn->feat_mask & L2CAP_FEAT_EXT_WINDOW) &&
			pi->extended_control) {
			l2cap_add_conf_opt(&ptr, L2CAP_CONF_EXT_WINDOW, 2,
					pi->tx_win);
		}

		if (!(pi->conn->feat_mask & L2CAP_FEAT_FCS))
			break;

		if (pi->fcs == L2CAP_FCS_NONE ||
				pi->conf_state & L2CAP_CONF_NO_FCS_RECV) {
			pi->fcs = L2CAP_FCS_NONE;
			l2cap_add_conf_opt(&ptr, L2CAP_CONF_FCS, 1, pi->fcs);
		}
		break;

	case L2CAP_MODE_STREAMING:
		rfc.txwin_size      = 0;
		rfc.max_transmit    = 0;
		rfc.retrans_timeout = 0;
		rfc.monitor_timeout = 0;
		rfc.max_pdu_size    = cpu_to_le16(L2CAP_DEFAULT_MAX_PDU_SIZE);
		if (L2CAP_DEFAULT_MAX_PDU_SIZE > pi->imtu)
			rfc.max_pdu_size = cpu_to_le16(pi->imtu);

		l2cap_add_conf_opt(&ptr, L2CAP_CONF_RFC, sizeof(rfc),
							(unsigned long) &rfc);

		if ((pi->conn->feat_mask & L2CAP_FEAT_EXT_WINDOW) &&
			pi->extended_control) {
			l2cap_add_conf_opt(&ptr, L2CAP_CONF_EXT_WINDOW, 2, 0);
		}

		if (!(pi->conn->feat_mask & L2CAP_FEAT_FCS))
			break;

		if (pi->fcs == L2CAP_FCS_NONE ||
				pi->conf_state & L2CAP_CONF_NO_FCS_RECV) {
			pi->fcs = L2CAP_FCS_NONE;
			l2cap_add_conf_opt(&ptr, L2CAP_CONF_FCS, 1, pi->fcs);
		}
		break;
	}

	/* FIXME: Need actual value of the flush timeout.  Also, if
	 *        the BR/EDR baseband is configured for anything other than
	 *        an infinite flush timeout, L2CAP signal requests
	 *        (connect, info, config, channel moves, etc) must all
	 *        implement RTX timeouts.
	 */
	//if (flush_to != L2CAP_DEFAULT_FLUSH_TO)
	//   l2cap_add_conf_opt(&ptr, L2CAP_CONF_FLUSH_TO, 2, pi->flush_to);

	req->dcid  = cpu_to_le16(pi->dcid);
	req->flags = cpu_to_le16(0);

	return ptr - data;
}


static int l2cap_build_amp_reconf_req(struct sock *sk, void *data)
{
	struct l2cap_pinfo *pi = l2cap_pi(sk);
	struct l2cap_conf_req *req = data;
	struct l2cap_conf_rfc rfc = { .mode = pi->mode };
	void *ptr = req->data;
	u32 be_flush_to;

	BT_DBG("sk %p", sk);

	/* convert to milliseconds, round up */
	be_flush_to = (pi->conn->hcon->hdev->amp_be_flush_to + 999) / 1000;

	switch (pi->mode) {
	case L2CAP_MODE_ERTM:
		rfc.mode            = L2CAP_MODE_ERTM;
		rfc.txwin_size      = pi->tx_win;
		rfc.max_transmit    = pi->max_tx;
		if (pi->amp_move_id) {
			rfc.retrans_timeout =
					cpu_to_le16((3 * be_flush_to) + 500);
			rfc.monitor_timeout =
					cpu_to_le16((3 * be_flush_to) + 500);
		} else {
			rfc.retrans_timeout =
					cpu_to_le16(L2CAP_DEFAULT_RETRANS_TO);
			rfc.monitor_timeout =
					cpu_to_le16(L2CAP_DEFAULT_MONITOR_TO);
		}
		rfc.max_pdu_size    = cpu_to_le16(L2CAP_DEFAULT_MAX_PDU_SIZE);
		if (L2CAP_DEFAULT_MAX_PDU_SIZE > pi->imtu)
			rfc.max_pdu_size = cpu_to_le16(pi->imtu);

		break;

	default:
		return -ECONNREFUSED;
	}

	l2cap_add_conf_opt(&ptr, L2CAP_CONF_RFC, sizeof(rfc),
						(unsigned long) &rfc);

	if (pi->conn->feat_mask & L2CAP_FEAT_FCS) {

		/* TODO assign fcs for br/edr based on socket config option */
		if (pi->amp_move_id)
			pi->local_conf.fcs = L2CAP_FCS_NONE;
		else
			pi->local_conf.fcs = L2CAP_FCS_CRC16;

			l2cap_add_conf_opt(&ptr, L2CAP_CONF_FCS, 1,
						pi->local_conf.fcs);

			pi->fcs = pi->local_conf.fcs | pi->remote_conf.fcs;
	}

	req->dcid  = cpu_to_le16(pi->dcid);
	req->flags = cpu_to_le16(0);

	return ptr - data;
}

static int l2cap_parse_conf_req(struct sock *sk, void *data)
{
	struct l2cap_pinfo *pi = l2cap_pi(sk);
	struct l2cap_conf_rsp *rsp = data;
	void *ptr = rsp->data;
	void *req = pi->conf_req;
	int len = pi->conf_len;
	int type, hint, olen;
	unsigned long val;
	struct l2cap_conf_rfc rfc = { .mode = L2CAP_MODE_BASIC };
	u16 mtu = L2CAP_DEFAULT_MTU;
	u16 result = L2CAP_CONF_SUCCESS;

	BT_DBG("sk %p", sk);

	while (len >= L2CAP_CONF_OPT_SIZE) {
		len -= l2cap_get_conf_opt(&req, &type, &olen, &val);

		hint  = type & L2CAP_CONF_HINT;
		type &= L2CAP_CONF_MASK;

		switch (type) {
		case L2CAP_CONF_MTU:
			mtu = val;
			break;

		case L2CAP_CONF_FLUSH_TO:
			pi->flush_to = val;
			pi->remote_conf.flush_to = val;
			break;

		case L2CAP_CONF_QOS:
			break;

		case L2CAP_CONF_RFC:
			if (olen == sizeof(rfc))
				memcpy(&rfc, (void *) val, olen);
			break;

		case L2CAP_CONF_FCS:
			if (val == L2CAP_FCS_NONE)
				pi->conf_state |= L2CAP_CONF_NO_FCS_RECV;
			pi->remote_conf.fcs = val;
			break;

		case L2CAP_CONF_EXT_WINDOW:
			pi->extended_control = 1;
			pi->remote_tx_win = val;
			pi->tx_win_max = L2CAP_TX_WIN_MAX_EXTENDED;
			pi->conf_state |= L2CAP_CONF_EXT_WIN_RECV;
			break;

		default:
			if (hint)
				break;

			result = L2CAP_CONF_UNKNOWN;
			*((u8 *) ptr++) = type;
			break;
		}
	}

	if (pi->num_conf_rsp || pi->num_conf_req > 1)
		goto done;

	switch (pi->mode) {
	case L2CAP_MODE_STREAMING:
	case L2CAP_MODE_ERTM:
		if (!(pi->conf_state & L2CAP_CONF_STATE2_DEVICE)) {
			pi->mode = l2cap_select_mode(rfc.mode,
					pi->conn->feat_mask);
			break;
		}

		if (pi->mode != rfc.mode)
			return -ECONNREFUSED;

		break;
	}

done:
	if (pi->mode != rfc.mode) {
		result = L2CAP_CONF_UNACCEPT;
		rfc.mode = pi->mode;

		if (pi->num_conf_rsp == 1)
			return -ECONNREFUSED;

		l2cap_add_conf_opt(&ptr, L2CAP_CONF_RFC,
					sizeof(rfc), (unsigned long) &rfc);
	}


	if (result == L2CAP_CONF_SUCCESS) {
		/* Configure output options and let the other side know
		 * which ones we don't like. */

		if (mtu < L2CAP_DEFAULT_MIN_MTU) {
			result = L2CAP_CONF_UNACCEPT;
			pi->omtu = L2CAP_DEFAULT_MIN_MTU;
		}
		else {
			pi->omtu = mtu;
			pi->conf_state |= L2CAP_CONF_MTU_DONE;
		}
		l2cap_add_conf_opt(&ptr, L2CAP_CONF_MTU, 2, pi->omtu);

		switch (rfc.mode) {
		case L2CAP_MODE_BASIC:
			pi->fcs = L2CAP_FCS_NONE;
			pi->conf_state |= L2CAP_CONF_MODE_DONE;
			break;

		case L2CAP_MODE_ERTM:
			if (!(pi->conf_state & L2CAP_CONF_EXT_WIN_RECV))
				pi->remote_tx_win = rfc.txwin_size;

			pi->remote_max_tx = rfc.max_transmit;

			pi->remote_mps = le16_to_cpu(rfc.max_pdu_size);

			rfc.retrans_timeout =
				cpu_to_le16(L2CAP_DEFAULT_RETRANS_TO);
			rfc.monitor_timeout =
				cpu_to_le16(L2CAP_DEFAULT_MONITOR_TO);

			pi->conf_state |= L2CAP_CONF_MODE_DONE;

			l2cap_add_conf_opt(&ptr, L2CAP_CONF_RFC,
					sizeof(rfc), (unsigned long) &rfc);

			break;

		case L2CAP_MODE_STREAMING:
			pi->remote_mps = le16_to_cpu(rfc.max_pdu_size);

			pi->conf_state |= L2CAP_CONF_MODE_DONE;

			l2cap_add_conf_opt(&ptr, L2CAP_CONF_RFC,
					sizeof(rfc), (unsigned long) &rfc);

			break;

		default:
			result = L2CAP_CONF_UNACCEPT;

			memset(&rfc, 0, sizeof(rfc));
			rfc.mode = pi->mode;
		}

		if (result == L2CAP_CONF_SUCCESS)
			pi->conf_state |= L2CAP_CONF_OUTPUT_DONE;
	}
	rsp->scid   = cpu_to_le16(pi->dcid);
	rsp->result = cpu_to_le16(result);
	rsp->flags  = cpu_to_le16(0x0000);

	return ptr - data;
}

static int l2cap_parse_amp_move_reconf_req(struct sock *sk, void *data)
{
	struct l2cap_pinfo *pi = l2cap_pi(sk);
	struct l2cap_conf_rsp *rsp = data;
	void *ptr = rsp->data;
	void *req = pi->conf_req;
	int len = pi->conf_len;
	int type, hint, olen;
	unsigned long val;
	struct l2cap_conf_rfc rfc = { .mode = L2CAP_MODE_BASIC };
	struct l2cap_conf_ext_fs fs;
	u16 mtu = pi->omtu;
	u16 tx_win = pi->remote_tx_win;
	u16 result = L2CAP_CONF_SUCCESS;

	BT_DBG("sk %p", sk);

	while (len >= L2CAP_CONF_OPT_SIZE) {
		len -= l2cap_get_conf_opt(&req, &type, &olen, &val);

		hint  = type & L2CAP_CONF_HINT;
		type &= L2CAP_CONF_MASK;

		switch (type) {
		case L2CAP_CONF_MTU:
			mtu = val;
			break;

		case L2CAP_CONF_FLUSH_TO:
			if (pi->amp_move_id)
				result = L2CAP_CONF_UNACCEPT;
			else
				pi->remote_conf.flush_to = val;
			break;

		case L2CAP_CONF_QOS:
			if (pi->amp_move_id)
				result = L2CAP_CONF_UNACCEPT;
			break;

		case L2CAP_CONF_RFC:
			if (olen == sizeof(rfc))
				memcpy(&rfc, (void *) val, olen);
				if (pi->mode != rfc.mode ||
					rfc.mode == L2CAP_MODE_BASIC)
					result = L2CAP_CONF_UNACCEPT;
			break;

		case L2CAP_CONF_FCS:
			pi->remote_conf.fcs = val;
			break;

		case L2CAP_CONF_EXT_FS:
			if (olen == sizeof(fs)) {
				memcpy(&fs, (void *) val, olen);
				if (fs.type != L2CAP_SERVICE_BEST_EFFORT)
					result = L2CAP_CONF_FLOW_SPEC_REJECT;
				else {
					pi->remote_conf.flush_to =
						le32_to_cpu(fs.flush_to);
				}
			}
			break;

		case L2CAP_CONF_EXT_WINDOW:
			tx_win = val;
			break;

		default:
			if (hint)
				break;

			result = L2CAP_CONF_UNKNOWN;
			*((u8 *) ptr++) = type;
			break;
			}
	}

	BT_DBG("result 0x%2.2x cur mode 0x%2.2x req  mode 0x%2.2x",
		result, pi->mode, rfc.mode);

	if (result == L2CAP_CONF_SUCCESS) {
		/* Configure output options and let the other side know
		 * which ones we don't like. */

		/* Don't allow mtu to decrease. */
		if (mtu < pi->omtu)
			result = L2CAP_CONF_UNACCEPT;

		BT_DBG("mtu %d omtu %d", mtu, pi->omtu);

		l2cap_add_conf_opt(&ptr, L2CAP_CONF_MTU, 2, pi->omtu);

		/* Don't allow extended transmit window to change. */
		if (tx_win != pi->remote_tx_win) {
			result = L2CAP_CONF_UNACCEPT;
			l2cap_add_conf_opt(&ptr, L2CAP_CONF_EXT_WINDOW, 2,
					pi->remote_tx_win);
		}

		if (rfc.mode == L2CAP_MODE_ERTM) {
			pi->remote_conf.retrans_timeout =
				le16_to_cpu(rfc.retrans_timeout);
			pi->remote_conf.monitor_timeout =
				le16_to_cpu(rfc.monitor_timeout);

			BT_DBG("remote conf monitor timeout %d",
					pi->remote_conf.monitor_timeout);

			l2cap_add_conf_opt(&ptr, L2CAP_CONF_RFC,
					sizeof(rfc), (unsigned long) &rfc);
		}

	}

	if (result != L2CAP_CONF_SUCCESS)
		goto done;

	pi->fcs = pi->remote_conf.fcs | pi->local_conf.fcs ;

	if (pi->rx_state == L2CAP_ERTM_RX_STATE_WAIT_F_FLAG) {
		pi->flush_to = pi->remote_conf.flush_to;
		pi->retrans_timeout = pi->remote_conf.retrans_timeout;

		if (pi->amp_move_id)
			pi->monitor_timeout = pi->remote_conf.monitor_timeout;
		else
			pi->monitor_timeout = L2CAP_DEFAULT_MONITOR_TO;
		BT_DBG("mode %d monitor timeout %d",
			pi->mode, pi->monitor_timeout);

	}

done:
	rsp->scid   = cpu_to_le16(pi->dcid);
	rsp->result = cpu_to_le16(result);
	rsp->flags  = cpu_to_le16(0x0000);

	return ptr - data;
}

static int l2cap_parse_conf_rsp(struct sock *sk, void *rsp, int len, void *data, u16 *result)
{
	struct l2cap_pinfo *pi = l2cap_pi(sk);
	struct l2cap_conf_req *req = data;
	void *ptr = req->data;
	int type, olen;
	unsigned long val;
	struct l2cap_conf_rfc rfc;

	BT_DBG("sk %p, rsp %p, len %d, req %p", sk, rsp, len, data);

	while (len >= L2CAP_CONF_OPT_SIZE) {
		len -= l2cap_get_conf_opt(&rsp, &type, &olen, &val);

		switch (type) {
		case L2CAP_CONF_MTU:
			if (val < L2CAP_DEFAULT_MIN_MTU) {
				*result = L2CAP_CONF_UNACCEPT;
				pi->imtu = L2CAP_DEFAULT_MIN_MTU;
			} else
				pi->imtu = val;
			l2cap_add_conf_opt(&ptr, L2CAP_CONF_MTU, 2, pi->imtu);
			break;

		case L2CAP_CONF_FLUSH_TO:
			pi->flush_to = val;
			l2cap_add_conf_opt(&ptr, L2CAP_CONF_FLUSH_TO,
							2, pi->flush_to);
			break;

		case L2CAP_CONF_RFC:
			if (olen == sizeof(rfc))
				memcpy(&rfc, (void *)val, olen);

			if ((pi->conf_state & L2CAP_CONF_STATE2_DEVICE) &&
							rfc.mode != pi->mode)
				return -ECONNREFUSED;

			pi->fcs = 0;

			l2cap_add_conf_opt(&ptr, L2CAP_CONF_RFC,
					sizeof(rfc), (unsigned long) &rfc);
			break;

		case L2CAP_CONF_EXT_WINDOW:
			pi->tx_win = val;

			if (pi->tx_win > L2CAP_TX_WIN_MAX_ENHANCED)
				pi->tx_win = L2CAP_TX_WIN_MAX_ENHANCED;

			l2cap_add_conf_opt(&ptr, L2CAP_CONF_EXT_WINDOW,
					2, pi->tx_win);
			break;

		default:
			break;
		}
	}

	if (pi->mode == L2CAP_MODE_BASIC && pi->mode != rfc.mode)
		return -ECONNREFUSED;

	pi->mode = rfc.mode;

	if (*result == L2CAP_CONF_SUCCESS) {
		switch (rfc.mode) {
		case L2CAP_MODE_ERTM:
			pi->retrans_timeout = le16_to_cpu(rfc.retrans_timeout);
			pi->monitor_timeout = le16_to_cpu(rfc.monitor_timeout);
			pi->mps    = le16_to_cpu(rfc.max_pdu_size);
			break;
		case L2CAP_MODE_STREAMING:
			pi->mps    = le16_to_cpu(rfc.max_pdu_size);
		}
	}

	req->dcid   = cpu_to_le16(pi->dcid);
	req->flags  = cpu_to_le16(0x0000);

	return ptr - data;
}

static int l2cap_build_conf_rsp(struct sock *sk, void *data, u16 result, u16 flags)
{
	struct l2cap_conf_rsp *rsp = data;
	void *ptr = rsp->data;

	BT_DBG("sk %p", sk);

	rsp->scid   = cpu_to_le16(l2cap_pi(sk)->dcid);
	rsp->result = cpu_to_le16(result);
	rsp->flags  = cpu_to_le16(flags);

	return ptr - data;
}

static void l2cap_conf_rfc_get(struct sock *sk, void *rsp, int len)
{
	struct l2cap_pinfo *pi = l2cap_pi(sk);
	int type, olen;
	unsigned long val;
	struct l2cap_conf_rfc rfc;

	BT_DBG("sk %p, rsp %p, len %d", sk, rsp, len);

	if ((pi->mode != L2CAP_MODE_ERTM) && (pi->mode != L2CAP_MODE_STREAMING))
		return;

	while (len >= L2CAP_CONF_OPT_SIZE) {
		len -= l2cap_get_conf_opt(&rsp, &type, &olen, &val);

		switch (type) {
		case L2CAP_CONF_RFC:
			if (olen == sizeof(rfc))
				memcpy(&rfc, (void *)val, olen);
			goto done;
		}
	}

done:
	switch (rfc.mode) {
	case L2CAP_MODE_ERTM:
		pi->retrans_timeout = le16_to_cpu(rfc.retrans_timeout);
		pi->monitor_timeout = le16_to_cpu(rfc.monitor_timeout);
		pi->mps    = le16_to_cpu(rfc.max_pdu_size);
		break;
	case L2CAP_MODE_STREAMING:
		pi->mps    = le16_to_cpu(rfc.max_pdu_size);
	}
}

static int l2cap_finish_amp_move(struct sock *sk)
{
	struct l2cap_pinfo *pi;
	int err;

	BT_DBG("sk %p", sk);

	pi = l2cap_pi(sk);

	pi->amp_move_role = L2CAP_AMP_MOVE_NONE;
	pi->rx_state = L2CAP_ERTM_RX_STATE_RECV;

	if (pi->ampcon)
		pi->conn->mtu = pi->ampcon->hdev->acl_mtu;
	else
		pi->conn->mtu = pi->conn->hcon->hdev->acl_mtu;

	err = l2cap_setup_resegment(sk);

	return err;
}

static int l2cap_amp_move_reconf_rsp(struct sock *sk, void *rsp, int len,
					u16 result)
{
	int err = 0;
	struct l2cap_conf_rfc rfc = {.mode = L2CAP_MODE_BASIC};
	struct l2cap_pinfo *pi = l2cap_pi(sk);

	BT_DBG("sk %p, rsp %p, len %d, res 0x%2.2x", sk, rsp, len, result);

	if (pi->reconf_state == L2CAP_RECONF_NONE)
		return -ECONNREFUSED;

	if (result == L2CAP_CONF_SUCCESS) {
		while (len >= L2CAP_CONF_OPT_SIZE) {
			int type, olen;
			unsigned long val;

			len -= l2cap_get_conf_opt(&rsp, &type, &olen, &val);

			if (type == L2CAP_CONF_RFC) {
				if (olen == sizeof(rfc))
					memcpy(&rfc, (void *)val, olen);
				if (rfc.mode != pi->mode &&
					rfc.mode != L2CAP_MODE_ERTM) {
					err = -ECONNREFUSED;
					goto done;
				}
				break;
			}
		}
	}

done:
	l2cap_ertm_stop_ack_timer(pi);
	l2cap_ertm_stop_retrans_timer(pi);
	l2cap_ertm_stop_monitor_timer(pi);

	if (l2cap_pi(sk)->reconf_state == L2CAP_RECONF_ACC) {
		l2cap_pi(sk)->reconf_state = L2CAP_RECONF_NONE;

		/* Respond to poll */
		err = l2cap_answer_move_poll(sk);

	} else if (l2cap_pi(sk)->reconf_state == L2CAP_RECONF_INT) {

		/* If moving to BR/EDR, use default timeout defined by
		 * the spec */
		if (pi->amp_move_id == 0)
			pi->monitor_timeout = L2CAP_DEFAULT_MONITOR_TO;

		if (pi->mode == L2CAP_MODE_ERTM) {
			l2cap_ertm_tx(sk, NULL, NULL,
					L2CAP_ERTM_EVENT_EXPLICIT_POLL);
			pi->rx_state = L2CAP_ERTM_RX_STATE_WAIT_F_FLAG;
		}
	}

	return err;
}


static inline int l2cap_command_rej(struct l2cap_conn *conn, struct l2cap_cmd_hdr *cmd, u8 *data)
{
	struct l2cap_cmd_rej *rej = (struct l2cap_cmd_rej *) data;

	if (rej->reason != 0x0000)
		return 0;

	if ((conn->info_state & L2CAP_INFO_FEAT_MASK_REQ_SENT) &&
					cmd->ident == conn->info_ident) {
		del_timer(&conn->info_timer);

		conn->info_state |= L2CAP_INFO_FEAT_MASK_REQ_DONE;
		conn->info_ident = 0;

		l2cap_conn_start(conn);
	}

	return 0;
}

static inline int l2cap_connect_req(struct l2cap_conn *conn, struct l2cap_cmd_hdr *cmd, u8 *data)
{
	struct l2cap_chan_list *list = &conn->chan_list;
	struct l2cap_conn_req *req = (struct l2cap_conn_req *) data;
	struct l2cap_conn_rsp rsp;
	struct sock *parent, *sk = NULL;
	int result, status = L2CAP_CS_NO_INFO;

	u16 dcid = 0, scid = __le16_to_cpu(req->scid);
	__le16 psm = req->psm;

	BT_DBG("psm 0x%2.2x scid 0x%4.4x", psm, scid);

	/* Check if we have socket listening on psm */
	parent = l2cap_get_sock_by_psm(BT_LISTEN, psm, conn->src);
	if (!parent) {
		result = L2CAP_CR_BAD_PSM;
		goto sendresp;
	}

	/* Check if the ACL is secure enough (if not SDP) */
	if (psm != cpu_to_le16(0x0001) &&
				!hci_conn_check_link_mode(conn->hcon)) {
		conn->disc_reason = 0x05;
		result = L2CAP_CR_SEC_BLOCK;
		goto response;
	}

	result = L2CAP_CR_NO_MEM;

	/* Check for backlog size */
	if (sk_acceptq_is_full(parent)) {
		BT_DBG("backlog full %d", parent->sk_ack_backlog);
		goto response;
	}

	sk = l2cap_sock_alloc(sock_net(parent), NULL, BTPROTO_L2CAP, GFP_ATOMIC);
	if (!sk)
		goto response;

	write_lock_bh(&list->lock);

	/* Check if we already have channel with that dcid */
	if (__l2cap_get_chan_by_dcid(list, scid)) {
		write_unlock_bh(&list->lock);
		sock_set_flag(sk, SOCK_ZAPPED);
		l2cap_sock_kill(sk);
		goto response;
	}

	hci_conn_hold(conn->hcon);

	l2cap_sock_init(sk, parent);
	bacpy(&bt_sk(sk)->src, conn->src);
	bacpy(&bt_sk(sk)->dst, conn->dst);
	l2cap_pi(sk)->psm  = psm;
	l2cap_pi(sk)->dcid = scid;

	__l2cap_chan_add(conn, sk, parent);
	dcid = l2cap_pi(sk)->scid;

	l2cap_sock_set_timer(sk, sk->sk_sndtimeo);

	l2cap_pi(sk)->ident = cmd->ident;

	if (conn->info_state & L2CAP_INFO_FEAT_MASK_REQ_DONE) {
		if (l2cap_check_security(sk)) {
			if (bt_sk(sk)->defer_setup) {
				sk->sk_state = BT_CONNECT2;
				result = L2CAP_CR_PEND;
				status = L2CAP_CS_AUTHOR_PEND;
				parent->sk_data_ready(parent, 0);
			} else {
				sk->sk_state = BT_CONFIG;
				result = L2CAP_CR_SUCCESS;
				status = L2CAP_CS_NO_INFO;
			}
		} else {
			sk->sk_state = BT_CONNECT2;
			result = L2CAP_CR_PEND;
			status = L2CAP_CS_AUTHEN_PEND;
		}
	} else {
		sk->sk_state = BT_CONNECT2;
		result = L2CAP_CR_PEND;
		status = L2CAP_CS_NO_INFO;
	}

	write_unlock_bh(&list->lock);

response:
	bh_unlock_sock(parent);

sendresp:
	rsp.scid   = cpu_to_le16(scid);
	rsp.dcid   = cpu_to_le16(dcid);
	rsp.result = cpu_to_le16(result);
	rsp.status = cpu_to_le16(status);
	l2cap_send_cmd(conn, cmd->ident, L2CAP_CONN_RSP, sizeof(rsp), &rsp);

	if (result == L2CAP_CR_PEND && status == L2CAP_CS_NO_INFO) {
		struct l2cap_info_req info;
		info.type = cpu_to_le16(L2CAP_IT_FEAT_MASK);

		conn->info_state |= L2CAP_INFO_FEAT_MASK_REQ_SENT;
		conn->info_ident = l2cap_get_ident(conn);

		mod_timer(&conn->info_timer, jiffies +
					msecs_to_jiffies(L2CAP_INFO_TIMEOUT));

		l2cap_send_cmd(conn, conn->info_ident,
					L2CAP_INFO_REQ, sizeof(info), &info);
	}

	if (sk && !(l2cap_pi(sk)->conf_state & L2CAP_CONF_REQ_SENT) &&
				result == L2CAP_CR_SUCCESS) {
		u8 buf[128];
		l2cap_pi(sk)->conf_state |= L2CAP_CONF_REQ_SENT;
		l2cap_send_cmd(conn, l2cap_get_ident(conn), L2CAP_CONF_REQ,
					l2cap_build_conf_req(sk, buf), buf);
		l2cap_pi(sk)->num_conf_req++;
	}

	return 0;
}

static inline int l2cap_connect_rsp(struct l2cap_conn *conn, struct l2cap_cmd_hdr *cmd, u8 *data)
{
	struct l2cap_conn_rsp *rsp = (struct l2cap_conn_rsp *) data;
	u16 scid, dcid, result, status;
	struct sock *sk;
	u8 req[128];

	scid   = __le16_to_cpu(rsp->scid);
	dcid   = __le16_to_cpu(rsp->dcid);
	result = __le16_to_cpu(rsp->result);
	status = __le16_to_cpu(rsp->status);

	BT_DBG("dcid 0x%4.4x scid 0x%4.4x result 0x%2.2x status 0x%2.2x", dcid, scid, result, status);

	if (scid) {
		sk = l2cap_get_chan_by_scid(&conn->chan_list, scid);
		if (!sk)
			return -EFAULT;
	} else {
		sk = l2cap_get_chan_by_ident(&conn->chan_list, cmd->ident);
		if (!sk)
			return -EFAULT;
	}

	switch (result) {
	case L2CAP_CR_SUCCESS:
		sk->sk_state = BT_CONFIG;
		l2cap_pi(sk)->ident = 0;
		l2cap_pi(sk)->dcid = dcid;
		l2cap_pi(sk)->conf_state &= ~L2CAP_CONF_CONNECT_PEND;

		if (l2cap_pi(sk)->conf_state & L2CAP_CONF_REQ_SENT)
			break;

		l2cap_pi(sk)->conf_state |= L2CAP_CONF_REQ_SENT;

		l2cap_send_cmd(conn, l2cap_get_ident(conn), L2CAP_CONF_REQ,
					l2cap_build_conf_req(sk, req), req);
		l2cap_pi(sk)->num_conf_req++;
		break;

	case L2CAP_CR_PEND:
		l2cap_pi(sk)->conf_state |= L2CAP_CONF_CONNECT_PEND;
		break;

	default:
		l2cap_chan_del(sk, ECONNREFUSED);
		break;
	}

	bh_unlock_sock(sk);
	return 0;
}

static inline void set_default_fcs(struct l2cap_pinfo *pi)
{
	/* FCS is enabled only in ERTM or streaming mode, if one or both
	 * sides request it.
	 */
	if (pi->mode != L2CAP_MODE_ERTM && pi->mode != L2CAP_MODE_STREAMING)
		pi->fcs = L2CAP_FCS_NONE;
	else if (!(pi->conf_state & L2CAP_CONF_NO_FCS_RECV))
		pi->fcs = L2CAP_FCS_CRC16;
}

static inline int l2cap_config_req(struct l2cap_conn *conn, struct l2cap_cmd_hdr *cmd, u16 cmd_len, u8 *data)
{
	struct l2cap_conf_req *req = (struct l2cap_conf_req *) data;
	u16 dcid, flags;
	u8 rsp[64];
	struct sock *sk;
	int len;
	u8 amp_move_reconf = 0;

	dcid  = __le16_to_cpu(req->dcid);
	flags = __le16_to_cpu(req->flags);

	BT_DBG("dcid 0x%4.4x flags 0x%2.2x", dcid, flags);

	sk = l2cap_get_chan_by_scid(&conn->chan_list, dcid);
	if (!sk)
		return -ENOENT;

	BT_DBG("sk_state 0x%2.2x rx_state 0x%2.2x "
		"reconf_state 0x%2.2x amp_id 0x%2.2x amp_move_id 0x%2.2x",
		sk->sk_state, l2cap_pi(sk)->rx_state,
		l2cap_pi(sk)->reconf_state, l2cap_pi(sk)->amp_id,
		l2cap_pi(sk)->amp_move_id);

	/* Detect a reconfig request due to channel move between
	 * BR/EDR and AMP
	 */
	if (sk->sk_state == BT_CONNECTED &&
		l2cap_pi(sk)->rx_state ==
			L2CAP_ERTM_RX_STATE_WAIT_P_FLAG_RECONFIGURE)
		l2cap_pi(sk)->reconf_state = L2CAP_RECONF_ACC;

	if (l2cap_pi(sk)->reconf_state != L2CAP_RECONF_NONE)
		amp_move_reconf = 1;

	if (sk->sk_state == BT_DISCONN)
		goto unlock;

	/* Reject if config buffer is too small. */
	len = cmd_len - sizeof(*req);
	if (l2cap_pi(sk)->conf_len + len > sizeof(l2cap_pi(sk)->conf_req)) {
		l2cap_send_cmd(conn, cmd->ident, L2CAP_CONF_RSP,
				l2cap_build_conf_rsp(sk, rsp,
					L2CAP_CONF_REJECT, flags), rsp);
		goto unlock;
	}

	/* Store config. */
	memcpy(l2cap_pi(sk)->conf_req + l2cap_pi(sk)->conf_len, req->data, len);
	l2cap_pi(sk)->conf_len += len;

	if (flags & 0x0001) {
		/* Incomplete config. Send empty response. */
		l2cap_send_cmd(conn, cmd->ident, L2CAP_CONF_RSP,
				l2cap_build_conf_rsp(sk, rsp,
					L2CAP_CONF_SUCCESS, 0x0001), rsp);
		goto unlock;
	}

	/* Complete config. */
	if (!amp_move_reconf)
		len = l2cap_parse_conf_req(sk, rsp);
	else
		len = l2cap_parse_amp_move_reconf_req(sk, rsp);

	if (len < 0) {
		l2cap_send_disconn_req(conn, sk, ECONNRESET);
		goto unlock;
	}

	l2cap_send_cmd(conn, cmd->ident, L2CAP_CONF_RSP, len, rsp);

	/* Reset config buffer. */
	l2cap_pi(sk)->conf_len = 0;

	if (amp_move_reconf)
		goto unlock;

	l2cap_pi(sk)->num_conf_rsp++;

	if (!(l2cap_pi(sk)->conf_state & L2CAP_CONF_OUTPUT_DONE))
		goto unlock;

	if (l2cap_pi(sk)->conf_state & L2CAP_CONF_INPUT_DONE) {
		set_default_fcs(l2cap_pi(sk));

		sk->sk_state = BT_CONNECTED;

		if (l2cap_pi(sk)->mode == L2CAP_MODE_ERTM ||
			l2cap_pi(sk)->mode == L2CAP_MODE_STREAMING)
			l2cap_ertm_init(sk);

		l2cap_chan_ready(sk);
		goto unlock;
	}

	if (!(l2cap_pi(sk)->conf_state & L2CAP_CONF_REQ_SENT)) {
		u8 buf[64];
		l2cap_pi(sk)->conf_state |= L2CAP_CONF_REQ_SENT;
		l2cap_send_cmd(conn, l2cap_get_ident(conn), L2CAP_CONF_REQ,
					l2cap_build_conf_req(sk, buf), buf);
		l2cap_pi(sk)->num_conf_req++;
	}

unlock:
	bh_unlock_sock(sk);
	return 0;
}

static inline int l2cap_config_rsp(struct l2cap_conn *conn, struct l2cap_cmd_hdr *cmd, u8 *data)
{
	struct l2cap_conf_rsp *rsp = (struct l2cap_conf_rsp *)data;
	u16 scid, flags, result;
	struct sock *sk;
	int len = cmd->len - sizeof(*rsp);

	scid   = __le16_to_cpu(rsp->scid);
	flags  = __le16_to_cpu(rsp->flags);
	result = __le16_to_cpu(rsp->result);

	BT_DBG("scid 0x%4.4x flags 0x%2.2x result 0x%2.2x",
			scid, flags, result);

	sk = l2cap_get_chan_by_scid(&conn->chan_list, scid);
	if (!sk)
		return 0;

	if (l2cap_pi(sk)->reconf_state != L2CAP_RECONF_NONE)  {
		l2cap_amp_move_reconf_rsp(sk, rsp->data, len, result);
		goto done;
	}

	switch (result) {
	case L2CAP_CONF_SUCCESS:
		l2cap_conf_rfc_get(sk, rsp->data, len);
		break;

	case L2CAP_CONF_UNACCEPT:
		if (l2cap_pi(sk)->num_conf_rsp <= L2CAP_CONF_MAX_CONF_RSP) {
			char req[64];

			if (len > sizeof(req) - sizeof(struct l2cap_conf_req)) {
				l2cap_send_disconn_req(conn, sk, ECONNRESET);
				goto done;
			}

			/* throw out any old stored conf requests */
			result = L2CAP_CONF_SUCCESS;
			len = l2cap_parse_conf_rsp(sk, rsp->data,
							len, req, &result);
			if (len < 0) {
				l2cap_send_disconn_req(conn, sk, ECONNRESET);
				goto done;
			}

			l2cap_send_cmd(conn, l2cap_get_ident(conn),
						L2CAP_CONF_REQ, len, req);
			l2cap_pi(sk)->num_conf_req++;
			if (result != L2CAP_CONF_SUCCESS)
				goto done;
			break;
		}

	default:
		sk->sk_err = ECONNRESET;
		l2cap_sock_set_timer(sk, HZ * 5);
		l2cap_send_disconn_req(conn, sk, ECONNRESET);
		goto done;
	}

	if (flags & 0x01)
		goto done;

	l2cap_pi(sk)->conf_state |= L2CAP_CONF_INPUT_DONE;

	if (l2cap_pi(sk)->conf_state & L2CAP_CONF_OUTPUT_DONE) {
		set_default_fcs(l2cap_pi(sk));

		sk->sk_state = BT_CONNECTED;

		if (l2cap_pi(sk)->mode == L2CAP_MODE_ERTM ||
			l2cap_pi(sk)->mode == L2CAP_MODE_STREAMING)
			l2cap_ertm_init(sk);

		l2cap_chan_ready(sk);
	}

done:
	bh_unlock_sock(sk);
	return 0;
}

static inline int l2cap_disconnect_req(struct l2cap_conn *conn, struct l2cap_cmd_hdr *cmd, u8 *data)
{
	struct l2cap_disconn_req *req = (struct l2cap_disconn_req *) data;
	struct l2cap_disconn_rsp rsp;
	u16 dcid, scid;
	struct sock *sk;
	int err = 0;

	scid = __le16_to_cpu(req->scid);
	dcid = __le16_to_cpu(req->dcid);

	BT_DBG("scid 0x%4.4x dcid 0x%4.4x", scid, dcid);

	sk = l2cap_get_chan_by_scid(&conn->chan_list, dcid);
	if (!sk)
		return 0;

	rsp.dcid = cpu_to_le16(l2cap_pi(sk)->scid);
	rsp.scid = cpu_to_le16(l2cap_pi(sk)->dcid);
	l2cap_send_cmd(conn, cmd->ident, L2CAP_DISCONN_RSP, sizeof(rsp), &rsp);

	/* Only do cleanup if a disconnect request was not sent already */
	if (sk->sk_state != BT_DISCONN) {
		sk->sk_shutdown = SHUTDOWN_MASK;

		skb_queue_purge(TX_QUEUE(sk));

		if (l2cap_pi(sk)->mode == L2CAP_MODE_ERTM) {
			skb_queue_purge(SREJ_QUEUE(sk));

			__cancel_delayed_work(&l2cap_pi(sk)->ack_work);
			__cancel_delayed_work(&l2cap_pi(sk)->retrans_work);
			__cancel_delayed_work(&l2cap_pi(sk)->monitor_work);
		}
		err = ECONNRESET;
	}
	l2cap_chan_del(sk, err);
	bh_unlock_sock(sk);

	l2cap_sock_kill(sk);
	return 0;
}

static inline int l2cap_disconnect_rsp(struct l2cap_conn *conn, struct l2cap_cmd_hdr *cmd, u8 *data)
{
	struct l2cap_disconn_rsp *rsp = (struct l2cap_disconn_rsp *) data;
	u16 dcid, scid;
	struct sock *sk;

	scid = __le16_to_cpu(rsp->scid);
	dcid = __le16_to_cpu(rsp->dcid);

	BT_DBG("dcid 0x%4.4x scid 0x%4.4x", dcid, scid);

	sk = l2cap_get_chan_by_scid(&conn->chan_list, scid);
	if (!sk)
		return 0;

	l2cap_chan_del(sk, 0);
	bh_unlock_sock(sk);

	l2cap_sock_kill(sk);
	return 0;
}

static inline int l2cap_information_req(struct l2cap_conn *conn, struct l2cap_cmd_hdr *cmd, u8 *data)
{
	struct l2cap_info_req *req = (struct l2cap_info_req *) data;
	u16 type;

	type = __le16_to_cpu(req->type);

	BT_DBG("type 0x%4.4x", type);

	if (type == L2CAP_IT_FEAT_MASK) {
		u8 buf[8];
		u32 feat_mask = l2cap_feat_mask;
		struct l2cap_info_rsp *rsp = (struct l2cap_info_rsp *) buf;
		rsp->type   = cpu_to_le16(L2CAP_IT_FEAT_MASK);
		rsp->result = cpu_to_le16(L2CAP_IR_SUCCESS);
		if (!disable_ertm)
			feat_mask |= L2CAP_FEAT_ERTM | L2CAP_FEAT_STREAMING
				| L2CAP_FEAT_FCS | L2CAP_FEAT_EXT_WINDOW;
		put_unaligned_le32(feat_mask, rsp->data);
		l2cap_send_cmd(conn, cmd->ident,
					L2CAP_INFO_RSP, sizeof(buf), buf);
	} else if (type == L2CAP_IT_FIXED_CHAN) {
		u8 buf[12];
		struct l2cap_info_rsp *rsp = (struct l2cap_info_rsp *) buf;
		rsp->type   = cpu_to_le16(L2CAP_IT_FIXED_CHAN);
		rsp->result = cpu_to_le16(L2CAP_IR_SUCCESS);
		memcpy(buf + 4, l2cap_fixed_chan, 8);
		l2cap_send_cmd(conn, cmd->ident,
					L2CAP_INFO_RSP, sizeof(buf), buf);
	} else {
		struct l2cap_info_rsp rsp;
		rsp.type   = cpu_to_le16(type);
		rsp.result = cpu_to_le16(L2CAP_IR_NOTSUPP);
		l2cap_send_cmd(conn, cmd->ident,
					L2CAP_INFO_RSP, sizeof(rsp), &rsp);
	}

	return 0;
}

static inline int l2cap_information_rsp(struct l2cap_conn *conn, struct l2cap_cmd_hdr *cmd, u8 *data)
{
	struct l2cap_info_rsp *rsp = (struct l2cap_info_rsp *) data;
	u16 type, result;

	type   = __le16_to_cpu(rsp->type);
	result = __le16_to_cpu(rsp->result);

	BT_DBG("type 0x%4.4x result 0x%2.2x", type, result);

	del_timer(&conn->info_timer);

	if (result != L2CAP_IR_SUCCESS) {
		conn->info_state |= L2CAP_INFO_FEAT_MASK_REQ_DONE;
		conn->info_ident = 0;

		l2cap_conn_start(conn);

		return 0;
	}

	if (type == L2CAP_IT_FEAT_MASK) {
		conn->feat_mask = get_unaligned_le32(rsp->data);

		if (conn->feat_mask & L2CAP_FEAT_FIXED_CHAN) {
			struct l2cap_info_req req;
			req.type = cpu_to_le16(L2CAP_IT_FIXED_CHAN);

			conn->info_ident = l2cap_get_ident(conn);

			l2cap_send_cmd(conn, conn->info_ident,
					L2CAP_INFO_REQ, sizeof(req), &req);
		} else {
			conn->info_state |= L2CAP_INFO_FEAT_MASK_REQ_DONE;
			conn->info_ident = 0;

			l2cap_conn_start(conn);
		}
	} else if (type == L2CAP_IT_FIXED_CHAN) {
		conn->fc_mask = rsp->data[0];
		conn->info_state |= L2CAP_INFO_FEAT_MASK_REQ_DONE;
		conn->info_ident = 0;

		l2cap_conn_start(conn);
	}

	return 0;
}

static void l2cap_send_move_chan_req(struct l2cap_conn *conn,
			struct l2cap_pinfo *pi, u16 icid, u8 dest_amp_id)
{
	struct l2cap_move_chan_req req;
	u8 ident;

	BT_DBG("pi %p, icid %d, dest_amp_id %d", pi, (int) icid,
		(int) dest_amp_id);

	ident = l2cap_get_ident(conn);
	if (pi)
		pi->ident = ident;

	req.icid = cpu_to_le16(icid);
	req.dest_amp_id = dest_amp_id;

	l2cap_send_cmd(conn, ident, L2CAP_MOVE_CHAN_REQ, sizeof(req), &req);
}

static void l2cap_send_move_chan_rsp(struct l2cap_conn *conn, u8 ident,
				u16 icid, u16 result)
{
	struct l2cap_move_chan_rsp rsp;

	BT_DBG("icid %d, result %d", (int) icid, (int) result);

	rsp.icid = cpu_to_le16(icid);
	rsp.result = cpu_to_le16(result);

	l2cap_send_cmd(conn, ident, L2CAP_MOVE_CHAN_RSP, sizeof(rsp), &rsp);
}

static void l2cap_send_move_chan_cfm(struct l2cap_conn *conn,
				struct l2cap_pinfo *pi, u16 icid, u16 result)
{
	struct l2cap_move_chan_cfm cfm;
	u8 ident;

	BT_DBG("icid %d, result %d", (int) icid, (int) result);

	ident = l2cap_get_ident(conn);
	if (pi)
		pi->ident = ident;

	cfm.icid = cpu_to_le16(icid);
	cfm.result = cpu_to_le16(result);

	l2cap_send_cmd(conn, ident, L2CAP_MOVE_CHAN_CFM, sizeof(cfm), &cfm);
}

static void l2cap_send_move_chan_cfm_rsp(struct l2cap_conn *conn, u8 ident,
					u16 icid)
{
	struct l2cap_move_chan_cfm_rsp rsp;

	BT_DBG("icid %d", (int) icid);

	rsp.icid = cpu_to_le16(icid);
	l2cap_send_cmd(conn, ident, L2CAP_MOVE_CHAN_CFM_RSP, sizeof(rsp), &rsp);
}

static inline int l2cap_create_channel_req(struct l2cap_conn *conn,
					struct l2cap_cmd_hdr *cmd, u8 *data)
{
	struct l2cap_create_chan_req *req =
		(struct l2cap_create_chan_req *) data;
	struct l2cap_create_chan_rsp rsp;
	u16 psm, scid;

	psm = le16_to_cpu(req->psm);
	scid = le16_to_cpu(req->scid);

	BT_DBG("psm %d, scid %d, amp_id %d", (int) psm, (int) scid,
		(int) req->amp_id);

	/* Refuse all requests */

	rsp.dcid = 0;
	rsp.scid = cpu_to_le16(scid);
	rsp.result = L2CAP_CREATE_CHAN_REFUSED_RESOURCES;
	rsp.status = L2CAP_CREATE_CHAN_STATUS_NONE;

	l2cap_send_cmd(conn, cmd->ident, L2CAP_CREATE_CHAN_RSP,
			sizeof(rsp), &rsp);

	return 0;
}

static inline int l2cap_create_channel_rsp(struct l2cap_conn *conn,
					struct l2cap_cmd_hdr *cmd, u8 *data)
{
	struct l2cap_create_chan_rsp *rsp =
		(struct l2cap_create_chan_rsp *) data;
	u16 dcid, scid, result, status;

	dcid = le16_to_cpu(rsp->dcid);
	scid = le16_to_cpu(rsp->scid);
	result = le16_to_cpu(rsp->result);
	status = le16_to_cpu(rsp->status);

	BT_DBG("dcid %d, scid %d, result %d, status %d", (int) dcid,
		(int) scid, (int) result, (int) status);

	/* We never send create channel requests, so no responses expected. */

	return 0;
}

static inline int l2cap_move_channel_req(struct l2cap_conn *conn,
					struct l2cap_cmd_hdr *cmd, u8 *data)
{
	struct l2cap_move_chan_req *req = (struct l2cap_move_chan_req *) data;
	struct sock *sk;
	struct l2cap_pinfo *pi;
	u16 icid = 0;
	u16 result = L2CAP_MOVE_CHAN_REFUSED_NOT_ALLOWED;

	icid = le16_to_cpu(req->icid);

	BT_DBG("icid %d, dest_amp_id %d", (int) icid, (int) req->dest_amp_id);

	read_lock(&conn->chan_list.lock);
	sk = __l2cap_get_chan_by_dcid(&conn->chan_list, icid);
	read_unlock(&conn->chan_list.lock);

	if (!sk)
		goto send_move_response;

	lock_sock(sk);
	pi = l2cap_pi(sk);

	if (pi->scid < L2CAP_CID_DYN_START ||
		(pi->mode != L2CAP_MODE_ERTM &&
		 pi->mode != L2CAP_MODE_STREAMING)) {
		goto send_move_response;
	}

	if (pi->amp_id == req->dest_amp_id) {
		result = L2CAP_MOVE_CHAN_REFUSED_SAME_ID;
		goto send_move_response;
	}

	if ((pi->amp_move_state != L2CAP_AMP_STATE_STABLE ||
		pi->amp_move_role != L2CAP_AMP_MOVE_NONE) &&
		bacmp(conn->src, conn->dst) > 0) {
		result = L2CAP_MOVE_CHAN_REFUSED_COLLISION;
		goto send_move_response;
	}

	if (pi->amp_pref == BT_AMP_POLICY_REQUIRE_BR_EDR) {
		result = L2CAP_MOVE_CHAN_REFUSED_NOT_ALLOWED;
		goto send_move_response;
	}

	pi->amp_move_cmd_ident = cmd->ident;
	pi->amp_move_role = L2CAP_AMP_MOVE_RESPONDER;
	l2cap_amp_move_setup(sk);
	pi->amp_move_id = req->dest_amp_id;
	icid = pi->dcid;

	if (req->dest_amp_id == 0) {
		/* Moving to BR/EDR */
		if (pi->conn_state & L2CAP_CONN_LOCAL_BUSY) {
			pi->amp_move_state = L2CAP_AMP_STATE_WAIT_LOCAL_BUSY;
			result = L2CAP_MOVE_CHAN_PENDING;
		} else {
			pi->amp_move_state = L2CAP_AMP_STATE_WAIT_MOVE_CONFIRM;
			result = L2CAP_MOVE_CHAN_SUCCESS;
		}
	} else {
		pi->amp_move_state = L2CAP_AMP_STATE_WAIT_PREPARE;
		amp_accept_physical(pi->conn, req->dest_amp_id, sk);
		result = L2CAP_MOVE_CHAN_PENDING;
	}

send_move_response:
	l2cap_send_move_chan_rsp(conn, cmd->ident, icid, result);

	if (sk)
		release_sock(sk);

	return 0;
}

static inline int l2cap_move_channel_rsp(struct l2cap_conn *conn,
					struct l2cap_cmd_hdr *cmd, u8 *data)
{
	struct l2cap_move_chan_rsp *rsp = (struct l2cap_move_chan_rsp *) data;
	struct sock *sk;
	struct l2cap_pinfo *pi;
	u16 icid, result;

	icid = le16_to_cpu(rsp->icid);
	result = le16_to_cpu(rsp->result);

	BT_DBG("icid %d, result %d", (int) icid, (int) result);

	switch (result) {
	case L2CAP_MOVE_CHAN_SUCCESS:
	case L2CAP_MOVE_CHAN_PENDING:
		read_lock(&conn->chan_list.lock);
		sk = __l2cap_get_chan_by_scid(&conn->chan_list, icid);
		read_unlock(&conn->chan_list.lock);

		if (!sk) {
			l2cap_send_move_chan_cfm(conn, NULL, icid,
						L2CAP_MOVE_CHAN_UNCONFIRMED);
			break;
		}

		lock_sock(sk);
		pi = l2cap_pi(sk);

		l2cap_sock_clear_timer(sk);
		if (result == L2CAP_MOVE_CHAN_PENDING)
			l2cap_sock_set_timer(sk, L2CAP_MOVE_ERTX_TIMEOUT);

		if (pi->amp_move_state ==
				L2CAP_AMP_STATE_WAIT_LOGICAL_COMPLETE) {
			/* Move confirm will be sent when logical link
			 * is complete.
			 */
			pi->amp_move_state =
				L2CAP_AMP_STATE_WAIT_LOGICAL_CONFIRM;
		} else if (result == L2CAP_MOVE_CHAN_SUCCESS &&
			pi->amp_move_state ==
				L2CAP_AMP_STATE_WAIT_MOVE_RSP_SUCCESS) {
			/* Logical link is up or moving to BR/EDR,
			 * proceed with move */
			if (pi->conn_state & L2CAP_CONN_LOCAL_BUSY) {
				pi->amp_move_state =
					L2CAP_AMP_STATE_WAIT_LOCAL_BUSY;
			} else {
				pi->amp_move_state =
					L2CAP_AMP_STATE_WAIT_MOVE_CONFIRM_RSP;
				l2cap_send_move_chan_cfm(conn, pi, pi->scid,
						L2CAP_MOVE_CHAN_CONFIRMED);
				l2cap_sock_set_timer(sk, L2CAP_MOVE_TIMEOUT);
			}
		} else if (pi->amp_move_state ==
				L2CAP_AMP_STATE_WAIT_MOVE_RSP) {
			struct hci_chan *chan;
			/* Moving to AMP */
			if (result == L2CAP_MOVE_CHAN_SUCCESS) {
				/* Remote is ready, send confirm immediately
				 * after logical link is ready
				 */
				pi->amp_move_state =
					L2CAP_AMP_STATE_WAIT_LOGICAL_CONFIRM;
			} else {
				/* Both logical link and move success
				 * are required to confirm
				 */
				pi->amp_move_state =
					L2CAP_AMP_STATE_WAIT_LOGICAL_COMPLETE;
			}
			chan = hci_chan_create(A2MP_HCI_ID(pi->amp_move_id),
						pi->conn->dst);
			if (!chan) {
				/* Logical link not available */
				l2cap_send_move_chan_cfm(conn, pi, pi->scid,
						L2CAP_MOVE_CHAN_UNCONFIRMED);
				break;
			}
			if (chan->state == BT_CONNECTED) {
				/* Logical link is already ready to go */
				pi->ampchan = chan;
				pi->ampcon = chan->conn;
				pi->ampcon->l2cap_data = pi->conn;
				if (result == L2CAP_MOVE_CHAN_SUCCESS) {
					/* Can confirm now */
					l2cap_send_move_chan_cfm(conn, pi,
						pi->scid,
						L2CAP_MOVE_CHAN_CONFIRMED);
				} else {
					/* Now only need move success
					 * required to confirm
					 */
					pi->amp_move_state =
					L2CAP_AMP_STATE_WAIT_MOVE_RSP_SUCCESS;
				}
			} else
				chan->l2cap_sk = sk;
		} else {
			/* Any other amp move state means the move failed. */
			l2cap_send_move_chan_cfm(conn, pi, pi->scid,
						L2CAP_MOVE_CHAN_UNCONFIRMED);
			l2cap_sock_set_timer(sk, L2CAP_MOVE_TIMEOUT);
		}
		break;
	default:
		/* Failed (including collision case) */
		read_lock(&conn->chan_list.lock);
		sk = __l2cap_get_chan_by_ident(&conn->chan_list, cmd->ident);
		read_unlock(&conn->chan_list.lock);

		if (!sk) {
			/* Could not locate channel, icid is best guess */
			l2cap_send_move_chan_cfm(conn, NULL, icid,
						L2CAP_MOVE_CHAN_UNCONFIRMED);
			break;
		}

		lock_sock(sk);
		pi = l2cap_pi(sk);

		l2cap_sock_clear_timer(sk);

		if (pi->amp_move_role == L2CAP_AMP_MOVE_INITIATOR) {
			if (result == L2CAP_MOVE_CHAN_REFUSED_COLLISION)
				pi->amp_move_role = L2CAP_AMP_MOVE_RESPONDER;
			else {
				/* Cleanup - cancel move */
				pi->amp_move_id = pi->amp_id;
				pi->amp_move_state = L2CAP_AMP_STATE_STABLE;
				l2cap_amp_move_revert(sk);
				pi->amp_move_role = L2CAP_AMP_MOVE_NONE;
			}
		} else {
			/* State is STABLE so the confirm response is
			 * ignored.
			 */
			pi->amp_move_state = L2CAP_AMP_STATE_STABLE;
		}

		l2cap_send_move_chan_cfm(conn, pi, pi->scid,
					L2CAP_MOVE_CHAN_UNCONFIRMED);
		l2cap_sock_set_timer(sk, L2CAP_MOVE_TIMEOUT);
		break;
	}

	if (sk)
		release_sock(sk);

	return 0;
}

static inline int l2cap_move_channel_confirm(struct l2cap_conn *conn,
					struct l2cap_cmd_hdr *cmd, u8 *data)
{
	struct l2cap_move_chan_cfm *cfm = (struct l2cap_move_chan_cfm *) data;
	struct sock *sk;
	u16 icid, result;

	icid = le16_to_cpu(cfm->icid);
	result = le16_to_cpu(cfm->result);

	BT_DBG("icid %d, result %d", (int) icid, (int) result);

	read_lock(&conn->chan_list.lock);
	sk = __l2cap_get_chan_by_dcid(&conn->chan_list, icid);
	read_unlock(&conn->chan_list.lock);

	if (!sk) {
		BT_DBG("Bad channel (%d)", (int) icid);
		goto send_move_confirm_response;
	}

	lock_sock(sk);

	if (l2cap_pi(sk)->amp_move_state == L2CAP_AMP_STATE_WAIT_MOVE_CONFIRM) {
		l2cap_pi(sk)->amp_move_state = L2CAP_AMP_STATE_STABLE;
		if (result == L2CAP_MOVE_CHAN_CONFIRMED) {
			l2cap_pi(sk)->amp_id = l2cap_pi(sk)->amp_move_id;
			if (!l2cap_pi(sk)->amp_id) {
				/* Have moved off of AMP, free the channel */
				if (l2cap_pi(sk)->ampchan)
					hci_chan_put(l2cap_pi(sk)->ampchan);
				l2cap_pi(sk)->ampchan = NULL;
				l2cap_pi(sk)->ampcon = NULL;
			}
			l2cap_amp_move_success(sk);
		} else {
			l2cap_pi(sk)->amp_move_id = l2cap_pi(sk)->amp_id;
			l2cap_amp_move_revert(sk);
		}
		l2cap_pi(sk)->amp_move_role = L2CAP_AMP_MOVE_NONE;
	} else if (l2cap_pi(sk)->amp_move_state ==
			L2CAP_AMP_STATE_WAIT_LOGICAL_CONFIRM) {
		BT_DBG("Bad AMP_MOVE_STATE (%d)", l2cap_pi(sk)->amp_move_state);
	}

send_move_confirm_response:
	l2cap_send_move_chan_cfm_rsp(conn, cmd->ident, icid);

	if (sk)
		release_sock(sk);

	return 0;
}

static inline int l2cap_move_channel_confirm_rsp(struct l2cap_conn *conn,
					struct l2cap_cmd_hdr *cmd, u8 *data)
{
	struct l2cap_move_chan_cfm_rsp *rsp =
		(struct l2cap_move_chan_cfm_rsp *) data;
	struct sock *sk;
	u16 icid;

	icid = le16_to_cpu(rsp->icid);

	BT_DBG("icid %d", (int) icid);

	read_lock(&conn->chan_list.lock);
	sk = __l2cap_get_chan_by_scid(&conn->chan_list, icid);
	read_unlock(&conn->chan_list.lock);

	if (!sk)
		return 0;

	lock_sock(sk);

	l2cap_sock_clear_timer(sk);

	if (l2cap_pi(sk)->amp_move_state ==
			L2CAP_AMP_STATE_WAIT_MOVE_CONFIRM_RSP) {
		l2cap_pi(sk)->amp_move_state = L2CAP_AMP_STATE_STABLE;
		l2cap_pi(sk)->amp_id = l2cap_pi(sk)->amp_move_id;

		if (!l2cap_pi(sk)->amp_id) {
			/* Have moved off of AMP, free the channel */
			l2cap_pi(sk)->ampcon = NULL;
			if (l2cap_pi(sk)->ampchan)
				hci_chan_put(l2cap_pi(sk)->ampchan);
			l2cap_pi(sk)->ampchan = NULL;
		}

		l2cap_amp_move_success(sk);

		l2cap_pi(sk)->amp_move_role = L2CAP_AMP_MOVE_NONE;
	}

	release_sock(sk);

	return 0;
}

static void l2cap_amp_signal_worker(struct work_struct *work)
{
	int err = 0;
	struct l2cap_amp_signal_work *ampwork =
		container_of(work, struct l2cap_amp_signal_work, work);

	switch (ampwork->cmd.code) {
	case L2CAP_CREATE_CHAN_REQ:
		err = l2cap_create_channel_req(ampwork->conn, &ampwork->cmd,
						ampwork->data);
		break;

	case L2CAP_CREATE_CHAN_RSP:
		err = l2cap_create_channel_rsp(ampwork->conn, &ampwork->cmd,
						ampwork->data);
		break;

	case L2CAP_MOVE_CHAN_REQ:
		err = l2cap_move_channel_req(ampwork->conn, &ampwork->cmd,
						ampwork->data);
		break;

	case L2CAP_MOVE_CHAN_RSP:
		err = l2cap_move_channel_rsp(ampwork->conn, &ampwork->cmd,
						ampwork->data);
		break;

	case L2CAP_MOVE_CHAN_CFM:
		err = l2cap_move_channel_confirm(ampwork->conn, &ampwork->cmd,
						ampwork->data);
		break;

	case L2CAP_MOVE_CHAN_CFM_RSP:
		err = l2cap_move_channel_confirm_rsp(ampwork->conn,
						&ampwork->cmd, ampwork->data);
		break;

	default:
		BT_ERR("Unknown signaling command 0x%2.2x", ampwork->cmd.code);
		err = -EINVAL;
		break;
	}

	if (err) {
		struct l2cap_cmd_rej rej;
		BT_DBG("error %d", err);

		/* In this context, commands are only rejected with
		 * "command not understood", code 0.
		 */
		rej.reason = cpu_to_le16(0);
		l2cap_send_cmd(ampwork->conn, ampwork->cmd.ident,
				L2CAP_COMMAND_REJ, sizeof(rej), &rej);
	}

	kfree_skb(ampwork->skb);
	kfree(ampwork);
}

void l2cap_amp_physical_complete(int result, u8 local_id, u8 remote_id,
				struct sock *sk)
{
	struct l2cap_pinfo *pi;

	BT_DBG("result %d, local_id %d, remote_id %d, sk %p", result,
		(int) local_id, (int) remote_id, sk);

	lock_sock(sk);

	if (sk->sk_state != BT_CONNECTED) {
		release_sock(sk);
		return;
	}

	pi = l2cap_pi(sk);

	if (result == L2CAP_MOVE_CHAN_SUCCESS &&
		pi->amp_move_role == L2CAP_AMP_MOVE_INITIATOR) {
		l2cap_amp_move_setup(sk);
		pi->amp_move_id = local_id;
		pi->amp_move_state = L2CAP_AMP_STATE_WAIT_MOVE_RSP;

		l2cap_send_move_chan_req(pi->conn, pi, pi->scid, remote_id);
		l2cap_sock_set_timer(sk, L2CAP_MOVE_TIMEOUT);
	} else if (result == L2CAP_MOVE_CHAN_SUCCESS &&
		pi->amp_move_role == L2CAP_AMP_MOVE_RESPONDER) {
		struct hci_chan *chan;
		chan = hci_chan_accept(A2MP_HCI_ID(local_id), pi->conn->dst);
		if (chan) {
			if (chan->state == BT_CONNECTED) {
				/* Logical link is ready to go */
				pi->ampchan = chan;
				pi->ampcon = chan->conn;
				pi->ampcon->l2cap_data = pi->conn;
				pi->amp_move_state =
					L2CAP_AMP_STATE_WAIT_MOVE_CONFIRM;
				l2cap_send_move_chan_rsp(pi->conn,
					pi->amp_move_cmd_ident, pi->dcid,
					L2CAP_MOVE_CHAN_SUCCESS);
			} else {
				/* Wait for logical link to be ready */
				chan->l2cap_sk = sk;
				pi->amp_move_state =
					L2CAP_AMP_STATE_WAIT_LOGICAL_CONFIRM;
			}
		} else {
			/* Logical link not available */
			l2cap_send_move_chan_rsp(pi->conn,
				pi->amp_move_cmd_ident, pi->dcid,
				L2CAP_MOVE_CHAN_REFUSED_NOT_ALLOWED);
		}
	} else {
		BT_DBG("result %d, role %d, local_busy %d", result,
			(int) pi->amp_move_role,
			(int) ((pi->conn_state & L2CAP_CONN_LOCAL_BUSY) != 0));

		if (pi->amp_move_role == L2CAP_AMP_MOVE_RESPONDER) {
			if (result == -EINVAL)
				l2cap_send_move_chan_rsp(pi->conn,
					pi->amp_move_cmd_ident, pi->dcid,
					L2CAP_MOVE_CHAN_REFUSED_CONTROLLER);
			else
				l2cap_send_move_chan_rsp(pi->conn,
					pi->amp_move_cmd_ident, pi->dcid,
					L2CAP_MOVE_CHAN_REFUSED_NOT_ALLOWED);
		}

		pi->amp_move_role = L2CAP_AMP_MOVE_NONE;
		pi->amp_move_state = L2CAP_AMP_STATE_STABLE;

		if ((l2cap_pi(sk)->conn_state & L2CAP_CONN_LOCAL_BUSY) &&
			l2cap_rmem_available(sk))
			l2cap_ertm_tx(sk, 0, 0,
					L2CAP_ERTM_EVENT_LOCAL_BUSY_CLEAR);

		/* Restart data transmission */
		l2cap_ertm_send_txq(sk);
	}

	release_sock(sk);
}

int l2cap_logical_link_complete(struct hci_chan *chan, u8 status)
{
	struct l2cap_pinfo *pi;
	struct sock *sk;

	BT_DBG("status %d, chan %p, conn %p", (int) status, chan, chan->conn);

	sk = chan->l2cap_sk;

	BT_DBG("sk %p", sk);

	lock_sock(sk);

	if (sk->sk_state != BT_CONNECTED) {
		release_sock(sk);
		return 0;
	}

	pi = l2cap_pi(sk);

	if ((!status) && (chan != NULL)) {
		pi->ampchan = chan;
		pi->ampcon = chan->conn;
		pi->ampcon->l2cap_data = pi->conn;

		if (pi->amp_move_state ==
				L2CAP_AMP_STATE_WAIT_LOGICAL_COMPLETE) {
			/* Move confirm will be sent after a success
			 * response is received
			 */
			pi->amp_move_state =
				L2CAP_AMP_STATE_WAIT_MOVE_RSP_SUCCESS;
		} else if (pi->amp_move_state ==
				L2CAP_AMP_STATE_WAIT_LOGICAL_CONFIRM) {
			if (pi->conn_state & L2CAP_CONN_LOCAL_BUSY)
				pi->amp_move_state =
					L2CAP_AMP_STATE_WAIT_LOCAL_BUSY;
			else if (pi->amp_move_role ==
					L2CAP_AMP_MOVE_INITIATOR) {
				pi->amp_move_state =
					L2CAP_AMP_STATE_WAIT_MOVE_CONFIRM_RSP;
				l2cap_send_move_chan_cfm(pi->conn, pi, pi->scid,
					L2CAP_MOVE_CHAN_SUCCESS);
				l2cap_sock_set_timer(sk, L2CAP_MOVE_TIMEOUT);
			} else if (pi->amp_move_role ==
					L2CAP_AMP_MOVE_RESPONDER) {
				pi->amp_move_state =
					L2CAP_AMP_STATE_WAIT_MOVE_CONFIRM;
				l2cap_send_move_chan_rsp(pi->conn,
					pi->amp_move_cmd_ident, pi->dcid,
					L2CAP_MOVE_CHAN_SUCCESS);
			}
		} else {
			/* Move was not in expected state, free the
			 * logical link
			 */
			hci_chan_put(pi->ampchan);
			pi->ampcon = NULL;
			pi->ampchan = NULL;
		}
	} else {
		/* Logical link setup failed.  May be initiator or
		 * responder.  If initiator, may have already had a
		 * pending, success, or refused response.  If
		 * responder, send refused response and clean up.
		 */
		if (pi->amp_move_role == L2CAP_AMP_MOVE_RESPONDER) {
			l2cap_amp_move_revert(sk);
			l2cap_pi(sk)->amp_move_role = L2CAP_AMP_MOVE_NONE;
			pi->amp_move_state = L2CAP_AMP_STATE_STABLE;
			l2cap_send_move_chan_rsp(pi->conn,
					pi->amp_move_cmd_ident, pi->dcid,
					L2CAP_MOVE_CHAN_REFUSED_CONFIG);
		} else if (pi->amp_move_role == L2CAP_AMP_MOVE_INITIATOR) {
			if ((pi->amp_move_state ==
				L2CAP_AMP_STATE_WAIT_LOGICAL_COMPLETE) ||
				(pi->amp_move_state ==
				    L2CAP_AMP_STATE_WAIT_LOGICAL_CONFIRM)) {
				/* Remote has only sent pending or
				 * success responses, clean up
				 */
				l2cap_amp_move_revert(sk);
				l2cap_pi(sk)->amp_move_role =
					L2CAP_AMP_MOVE_NONE;
				pi->amp_move_state = L2CAP_AMP_STATE_STABLE;
			}

			/* Other amp move states imply that the move
			 * has already aborted
			 */
			l2cap_send_move_chan_cfm(pi->conn, pi, pi->scid,
						L2CAP_MOVE_CHAN_UNCONFIRMED);
			l2cap_sock_set_timer(sk, L2CAP_MOVE_TIMEOUT);
		}

		pi->ampcon = NULL;
		pi->ampchan = NULL;
	}

	release_sock(sk);
	return 0;
}

static void l2cap_logical_link_worker(struct work_struct *work)
{
	struct l2cap_logical_link_work *log_link_work =
		container_of(work, struct l2cap_logical_link_work, work);

	l2cap_logical_link_complete(log_link_work->chan, log_link_work->status);
	kfree(log_link_work);
}

int l2cap_create_cfm(struct hci_chan *chan, u8 status)
{
	struct l2cap_logical_link_work *amp_work;

	amp_work = kzalloc(sizeof(*amp_work), GFP_ATOMIC);
	if (!amp_work)
		return -ENOMEM;

	INIT_WORK(&amp_work->work, l2cap_logical_link_worker);
	amp_work->chan = chan;
	amp_work->status = status;
	if (!queue_work(_l2cap_wq, &amp_work->work)) {
		kfree(amp_work);
		return -ENOMEM;
	}

	return 0;
}

int l2cap_destroy_cfm(struct hci_chan *chan, u8 reason)
{
	struct l2cap_chan_list *l;
	struct l2cap_conn *conn = chan->conn->l2cap_data;
	struct sock *sk;

	BT_DBG("chan %p conn %p", chan, conn);

	if (!conn)
		return 0;

	l = &conn->chan_list;

	read_lock(&l->lock);

	for (sk = l->head; sk; sk = l2cap_pi(sk)->next_c) {
		bh_lock_sock(sk);
		/* TODO MM/PK - What to do if connection is LOCAL_BUSY?  */
		if (l2cap_pi(sk)->ampchan == chan) {
			l2cap_pi(sk)->ampchan = NULL;
			l2cap_amp_move_init(sk);
		}
		bh_unlock_sock(sk);
	}

	read_unlock(&l->lock);

	return 0;


}

static int l2cap_sig_amp(struct l2cap_conn *conn, struct l2cap_cmd_hdr *cmd,
			u8 *data, struct sk_buff *skb)
{
	struct l2cap_amp_signal_work *amp_work;

	amp_work = kzalloc(sizeof(*amp_work), GFP_ATOMIC);
	if (!amp_work)
		return -ENOMEM;

	INIT_WORK(&amp_work->work, l2cap_amp_signal_worker);
	amp_work->conn = conn;
	amp_work->cmd = *cmd;
	amp_work->data = data;
	amp_work->skb = skb_clone(skb, GFP_ATOMIC);
	if (!amp_work->skb) {
		kfree(amp_work);
		return -ENOMEM;
	}

	if (!queue_work(_l2cap_wq, &amp_work->work)) {
		kfree_skb(amp_work->skb);
		kfree(amp_work);
		return -ENOMEM;
	}

	return 0;
}

static inline void l2cap_sig_channel(struct l2cap_conn *conn, struct sk_buff *skb)
{
	u8 *data = skb->data;
	int len = skb->len;
	struct l2cap_cmd_hdr cmd;
	int err = 0;

	l2cap_raw_recv(conn, skb);

	while (len >= L2CAP_CMD_HDR_SIZE) {
		u16 cmd_len;
		memcpy(&cmd, data, L2CAP_CMD_HDR_SIZE);
		data += L2CAP_CMD_HDR_SIZE;
		len  -= L2CAP_CMD_HDR_SIZE;

		cmd_len = le16_to_cpu(cmd.len);

		BT_DBG("code 0x%2.2x len %d id 0x%2.2x", cmd.code, cmd_len, cmd.ident);

		if (cmd_len > len || !cmd.ident) {
			BT_DBG("corrupted command");
			break;
		}

		switch (cmd.code) {
		case L2CAP_COMMAND_REJ:
			l2cap_command_rej(conn, &cmd, data);
			break;

		case L2CAP_CONN_REQ:
			err = l2cap_connect_req(conn, &cmd, data);
			break;

		case L2CAP_CONN_RSP:
			err = l2cap_connect_rsp(conn, &cmd, data);
			break;

		case L2CAP_CONF_REQ:
			err = l2cap_config_req(conn, &cmd, cmd_len, data);
			break;

		case L2CAP_CONF_RSP:
			err = l2cap_config_rsp(conn, &cmd, data);
			break;

		case L2CAP_DISCONN_REQ:
			err = l2cap_disconnect_req(conn, &cmd, data);
			break;

		case L2CAP_DISCONN_RSP:
			err = l2cap_disconnect_rsp(conn, &cmd, data);
			break;

		case L2CAP_ECHO_REQ:
			l2cap_send_cmd(conn, cmd.ident, L2CAP_ECHO_RSP, cmd_len, data);
			break;

		case L2CAP_ECHO_RSP:
			break;

		case L2CAP_INFO_REQ:
			err = l2cap_information_req(conn, &cmd, data);
			break;

		case L2CAP_INFO_RSP:
			err = l2cap_information_rsp(conn, &cmd, data);
			break;

		case L2CAP_CREATE_CHAN_REQ:
		case L2CAP_CREATE_CHAN_RSP:
		case L2CAP_MOVE_CHAN_REQ:
		case L2CAP_MOVE_CHAN_RSP:
		case L2CAP_MOVE_CHAN_CFM:
		case L2CAP_MOVE_CHAN_CFM_RSP:
			err = l2cap_sig_amp(conn, &cmd, data, skb);
			break;

		default:
			BT_ERR("Unknown signaling command 0x%2.2x", cmd.code);
			err = -EINVAL;
			break;
		}

		if (err) {
			struct l2cap_cmd_rej rej;
			BT_DBG("error %d", err);

			/* FIXME: Map err to a valid reason */
			rej.reason = cpu_to_le16(0);
			l2cap_send_cmd(conn, cmd.ident, L2CAP_COMMAND_REJ, sizeof(rej), &rej);
		}

		data += cmd_len;
		len  -= cmd_len;
	}

	kfree_skb(skb);
}

static int l2cap_check_fcs(struct l2cap_pinfo *pi,  struct sk_buff *skb)
{
	u16 our_fcs, rcv_fcs;
	int hdr_size;

	if (pi->extended_control)
		hdr_size = L2CAP_EXTENDED_HDR_SIZE;
	else
		hdr_size = L2CAP_ENHANCED_HDR_SIZE;

	if (pi->fcs == L2CAP_FCS_CRC16) {
		skb_trim(skb, skb->len - L2CAP_FCS_SIZE);
		rcv_fcs = get_unaligned_le16(skb->data + skb->len);
		our_fcs = crc16(0, skb->data - hdr_size, skb->len + hdr_size);

		if (our_fcs != rcv_fcs) {
			BT_DBG("Bad FCS");
			return -EBADMSG;
		}
	}
	return 0;
}

static void l2cap_ertm_pass_to_tx(struct sock *sk,
				struct bt_l2cap_control *control)
{
	BT_DBG("sk %p, control %p", sk, control);
	l2cap_ertm_tx(sk, control, 0, L2CAP_ERTM_EVENT_RECV_REQSEQ_AND_FBIT);
}

static void l2cap_ertm_pass_to_tx_fbit(struct sock *sk,
				struct bt_l2cap_control *control)
{
	BT_DBG("sk %p, control %p", sk, control);
	l2cap_ertm_tx(sk, control, 0, L2CAP_ERTM_EVENT_RECV_FBIT);
}

static void l2cap_ertm_resend(struct sock *sk)
{
	struct bt_l2cap_control control;
	struct l2cap_pinfo *pi;
	struct sk_buff *skb;
	struct sk_buff *tx_skb;
	u16 seq;

	BT_DBG("sk %p", sk);

	pi = l2cap_pi(sk);

	if (pi->conn_state & L2CAP_CONN_REMOTE_BUSY)
		return;

	if (pi->amp_move_state != L2CAP_AMP_STATE_STABLE)
		return;

	while (pi->retrans_list.head != L2CAP_SEQ_LIST_CLEAR) {
		seq = l2cap_seq_list_pop(&pi->retrans_list);

		skb = l2cap_ertm_seq_in_queue(TX_QUEUE(sk), seq);
		if (!skb) {
			BT_DBG("Error: Can't retransmit seq %d, frame missing",
				(int) seq);
			continue;
		}

		bt_cb(skb)->retries += 1;
		control = bt_cb(skb)->control;

		if ((pi->max_tx != 0) && (bt_cb(skb)->retries > pi->max_tx)) {
			BT_DBG("Retry limit exceeded (%d)", (int) pi->max_tx);
			l2cap_send_disconn_req(pi->conn, sk, ECONNRESET);
			l2cap_seq_list_clear(&pi->retrans_list);
			break;
		}

		control.reqseq = pi->buffer_seq;
		if (pi->conn_state & L2CAP_CONN_SEND_FBIT) {
			control.final = 1;
			pi->conn_state &= ~L2CAP_CONN_SEND_FBIT;
		} else {
			control.final = 0;
		}

		if (skb_cloned(skb)) {
			/* Cloned sk_buffs are read-only, so we need a
			 * writeable copy
			 */
			tx_skb = skb_copy(skb, GFP_ATOMIC);
		} else {
			tx_skb = skb_clone(skb, GFP_ATOMIC);
		}

		/* Update skb contents */
		if (pi->extended_control) {
			put_unaligned_le32(__pack_extended_control(&control),
					tx_skb->data + L2CAP_HDR_SIZE);
		} else {
			put_unaligned_le16(__pack_enhanced_control(&control),
					tx_skb->data + L2CAP_HDR_SIZE);
		}

		if (pi->fcs == L2CAP_FCS_CRC16)
			apply_fcs(tx_skb);

		tx_skb->sk = sk;
		tx_skb->destructor = l2cap_skb_destructor;
		atomic_inc(&pi->ertm_queued);

		l2cap_do_send(sk, tx_skb);

		BT_DBG("Resent txseq %d", (int)control.txseq);

		pi->last_acked_seq = pi->buffer_seq;
	}
}

static inline void l2cap_ertm_retransmit(struct sock *sk,
					struct bt_l2cap_control *control)
{
	BT_DBG("sk %p, control %p", sk, control);

	l2cap_seq_list_append(&l2cap_pi(sk)->retrans_list, control->reqseq);
	l2cap_ertm_resend(sk);
}

static void l2cap_ertm_retransmit_all(struct sock *sk,
				struct bt_l2cap_control *control)
{
	struct l2cap_pinfo *pi;
	struct sk_buff *skb;

	BT_DBG("sk %p, control %p", sk, control);

	pi = l2cap_pi(sk);

	if (control->poll)
		pi->conn_state |= L2CAP_CONN_SEND_FBIT;

	l2cap_seq_list_clear(&pi->retrans_list);

	if (pi->conn_state & L2CAP_CONN_REMOTE_BUSY)
		return;

	if (pi->unacked_frames) {
		skb_queue_walk(TX_QUEUE(sk), skb) {
			if ((bt_cb(skb)->control.txseq == control->reqseq) ||
				skb == sk->sk_send_head)
				break;
		}

		skb_queue_walk_from(TX_QUEUE(sk), skb) {
			if (skb == sk->sk_send_head)
				break;

			l2cap_seq_list_append(&pi->retrans_list,
					bt_cb(skb)->control.txseq);
		}

		l2cap_ertm_resend(sk);
	}
}

static inline void append_skb_frag(struct sk_buff *skb,
			struct sk_buff *new_frag, struct sk_buff **last_frag)
{
	/* skb->len reflects data in skb as well as all fragments
	   skb->data_len reflects only data in fragments
	 */
	BT_DBG("skb %p, new_frag %p, *last_frag %p", skb, new_frag, *last_frag);

	if (!skb_has_frags(skb))
		skb_shinfo(skb)->frag_list = new_frag;

	new_frag->next = NULL;

	(*last_frag)->next = new_frag;
	*last_frag = new_frag;

	skb->len += new_frag->len;
	skb->data_len += new_frag->len;
	skb->truesize += new_frag->truesize;
}

static int l2cap_ertm_rx_expected_iframe(struct sock *sk,
			struct bt_l2cap_control *control, struct sk_buff *skb)
{
	struct l2cap_pinfo *pi;
	int err = -EINVAL;

	BT_DBG("sk %p, control %p, skb %p len %d truesize %d", sk, control,
		skb, skb->len, skb->truesize);

	if (!control)
		return err;

	pi = l2cap_pi(sk);

	BT_DBG("type %c, sar %d, txseq %d, reqseq %d, final %d",
		control->frame_type, control->sar, control->txseq,
		control->reqseq, control->final);

	switch (control->sar) {
	case L2CAP_SAR_UNSEGMENTED:
		if (pi->sdu) {
			BT_DBG("Unexpected unsegmented PDU during reassembly");
			kfree_skb(pi->sdu);
			pi->sdu = NULL;
			pi->sdu_last_frag = NULL;
			pi->sdu_len = 0;
		}

		BT_DBG("Unsegmented");
		err = sock_queue_rcv_skb(sk, skb);
		break;

	case L2CAP_SAR_START:
		if (pi->sdu) {
			BT_DBG("Unexpected start PDU during reassembly");
			kfree_skb(pi->sdu);
		}

		pi->sdu_len = get_unaligned_le16(skb->data);
		skb_pull(skb, 2);

		if (pi->sdu_len > pi->imtu) {
			err = -EMSGSIZE;
			break;
		}

		if (skb->len >= pi->sdu_len)
			break;

		pi->sdu = skb;
		pi->sdu_last_frag = skb;

		BT_DBG("Start");

		skb = NULL;
		err = 0;
		break;

	case L2CAP_SAR_CONTINUE:
		if (!pi->sdu)
			break;

		append_skb_frag(pi->sdu, skb,
				&pi->sdu_last_frag);
		skb = NULL;

		if (pi->sdu->len >= pi->sdu_len)
			break;

		BT_DBG("Continue, reassembled %d", pi->sdu->len);

		err = 0;
		break;

	case L2CAP_SAR_END:
		if (!pi->sdu)
			break;

		append_skb_frag(pi->sdu, skb,
				&pi->sdu_last_frag);
		skb = NULL;

		if (pi->sdu->len != pi->sdu_len)
			break;

		BT_DBG("End, reassembled %d", pi->sdu->len);
		/* If the sender used tiny PDUs, the rcv queuing could fail.
		 * Applications that have issues here should use a larger
		 * sk_rcvbuf.
		 */
		err = sock_queue_rcv_skb(sk, pi->sdu);

		if (!err) {
			/* Reassembly complete */
			pi->sdu = NULL;
			pi->sdu_last_frag = NULL;
			pi->sdu_len = 0;
		}
		break;

	default:
		BT_DBG("Bad SAR value");
		break;
	}

	if (err) {
		BT_DBG("Reassembly error %d, sk_rcvbuf %d, sk_rmem_alloc %d",
			err, sk->sk_rcvbuf, atomic_read(&sk->sk_rmem_alloc));
		if (pi->sdu) {
			kfree_skb(pi->sdu);
			pi->sdu = NULL;
		}
		pi->sdu_last_frag = NULL;
		pi->sdu_len = 0;
		if (skb)
			kfree_skb(skb);
	}

	/* Update local busy state */
	if (!(pi->conn_state & L2CAP_CONN_LOCAL_BUSY) && l2cap_rmem_full(sk))
		l2cap_ertm_tx(sk, 0, 0, L2CAP_ERTM_EVENT_LOCAL_BUSY_DETECTED);

	return err;
}

static int l2cap_ertm_rx_queued_iframes(struct sock *sk)
{
	int err = 0;
	/* Pass sequential frames to l2cap_ertm_rx_expected_iframe()
	 * until a gap is encountered.
	 */

	struct l2cap_pinfo *pi;

	BT_DBG("sk %p", sk);
	pi = l2cap_pi(sk);

	while (l2cap_rmem_available(sk)) {
		struct sk_buff *skb;
		BT_DBG("Searching for skb with txseq %d (queue len %d)",
			(int) pi->buffer_seq, skb_queue_len(SREJ_QUEUE(sk)));

		skb = l2cap_ertm_seq_in_queue(SREJ_QUEUE(sk), pi->buffer_seq);

		if (!skb)
			break;

		skb_unlink(skb, SREJ_QUEUE(sk));
		pi->buffer_seq = __next_seq(pi->buffer_seq, pi);
		err = l2cap_ertm_rx_expected_iframe(sk,
						&bt_cb(skb)->control, skb);
		if (err)
			break;
	}

	if (skb_queue_empty(SREJ_QUEUE(sk))) {
		pi->rx_state = L2CAP_ERTM_RX_STATE_RECV;
		l2cap_ertm_send_ack(sk);
	}

	return err;
}

static void l2cap_ertm_handle_srej(struct sock *sk,
				struct bt_l2cap_control *control)
{
	struct l2cap_pinfo *pi;
	struct sk_buff *skb;

	BT_DBG("sk %p, control %p", sk, control);

	pi = l2cap_pi(sk);

	if (control->reqseq == pi->next_tx_seq) {
		BT_DBG("Invalid reqseq %d, disconnecting",
			(int) control->reqseq);
		l2cap_send_disconn_req(pi->conn, sk, ECONNRESET);
		return;
	}

	skb = l2cap_ertm_seq_in_queue(TX_QUEUE(sk), control->reqseq);

	if (skb == NULL) {
		BT_DBG("Seq %d not available for retransmission",
			(int) control->reqseq);
		return;
	}

	if ((pi->max_tx != 0) && (bt_cb(skb)->retries >= pi->max_tx)) {
		BT_DBG("Retry limit exceeded (%d)", (int) pi->max_tx);
		l2cap_send_disconn_req(pi->conn, sk, ECONNRESET);
		return;
	}

	pi->conn_state &= ~L2CAP_CONN_REMOTE_BUSY;

	if (control->poll) {
		l2cap_ertm_pass_to_tx(sk, control);

		pi->conn_state |= L2CAP_CONN_SEND_FBIT;
		l2cap_ertm_retransmit(sk, control);
		l2cap_ertm_send_txq(sk);

		if (pi->tx_state == L2CAP_ERTM_TX_STATE_WAIT_F) {
			pi->conn_state |= L2CAP_CONN_SREJ_ACT;
			pi->srej_save_reqseq = control->reqseq;
		}
	} else {
		l2cap_ertm_pass_to_tx_fbit(sk, control);

		if (control->final) {
			if ((pi->conn_state & L2CAP_CONN_SREJ_ACT) &&
				(pi->srej_save_reqseq == control->reqseq)) {
				pi->conn_state &= ~L2CAP_CONN_SREJ_ACT;
			} else {
				l2cap_ertm_retransmit(sk, control);
			}
		} else {
			l2cap_ertm_retransmit(sk, control);
			if (pi->tx_state == L2CAP_ERTM_TX_STATE_WAIT_F) {
				pi->conn_state |= L2CAP_CONN_SREJ_ACT;
				pi->srej_save_reqseq = control->reqseq;
			}
		}
	}
}

static void l2cap_ertm_handle_rej(struct sock *sk,
				struct bt_l2cap_control *control)
{
	struct l2cap_pinfo *pi;
	struct sk_buff *skb;

	BT_DBG("sk %p, control %p", sk, control);

	pi = l2cap_pi(sk);

	if (control->reqseq == pi->next_tx_seq) {
		BT_DBG("Invalid reqseq %d, disconnecting",
			(int) control->reqseq);
		l2cap_send_disconn_req(pi->conn, sk, ECONNRESET);
		return;
	}

	skb = l2cap_ertm_seq_in_queue(TX_QUEUE(sk), control->reqseq);

	if (pi->max_tx && skb && bt_cb(skb)->retries >= pi->max_tx) {
		BT_DBG("Retry limit exceeded (%d)", (int) pi->max_tx);
		l2cap_send_disconn_req(pi->conn, sk, ECONNRESET);
		return;
	}

	pi->conn_state &= ~L2CAP_CONN_REMOTE_BUSY;

	l2cap_ertm_pass_to_tx(sk, control);

	if (control->final) {
		if (pi->conn_state & L2CAP_CONN_REJ_ACT)
			pi->conn_state &= ~L2CAP_CONN_REJ_ACT;
		else
			l2cap_ertm_retransmit_all(sk, control);
	} else {
		l2cap_ertm_retransmit_all(sk, control);
		l2cap_ertm_send_txq(sk);
		if (pi->tx_state == L2CAP_ERTM_TX_STATE_WAIT_F)
			pi->conn_state |= L2CAP_CONN_REJ_ACT;
	}
}

static u8 l2cap_ertm_classify_txseq(struct sock *sk, u16 txseq)
{
	struct l2cap_pinfo *pi;

	BT_DBG("sk %p, txseq %d", sk, (int)txseq);
	pi = l2cap_pi(sk);

	BT_DBG("last_acked_seq %d, expected_tx_seq %d", (int)pi->last_acked_seq,
		(int)pi->expected_tx_seq);

	if (pi->rx_state == L2CAP_ERTM_RX_STATE_SREJ_SENT) {
		if (__delta_seq(txseq, pi->last_acked_seq, pi) >= pi->tx_win) {
			/* See notes below regarding "double poll" and
			 * invalid packets.
			 */
			if (pi->tx_win <= ((pi->tx_win_max + 1) >> 1)) {
				BT_DBG("Invalid/Ignore - txseq outside "
					"tx window after SREJ sent");
				return L2CAP_ERTM_TXSEQ_INVALID_IGNORE;
			} else {
				BT_DBG("Invalid - bad txseq within tx "
					"window after SREJ sent");
				return L2CAP_ERTM_TXSEQ_INVALID;
			}
		}

		if (pi->srej_list.head == txseq) {
			BT_DBG("Expected SREJ");
			return L2CAP_ERTM_TXSEQ_EXPECTED_SREJ;
		}

		if (l2cap_ertm_seq_in_queue(SREJ_QUEUE(sk), txseq)) {
			BT_DBG("Duplicate SREJ - txseq already stored");
			return L2CAP_ERTM_TXSEQ_DUPLICATE_SREJ;
		}

		if (l2cap_seq_list_contains(&pi->srej_list, txseq)) {
			BT_DBG("Unexpected SREJ - txseq not requested "
				"with SREJ");
			return L2CAP_ERTM_TXSEQ_UNEXPECTED_SREJ;
		}
	}

	if (pi->expected_tx_seq == txseq) {
		if (__delta_seq(txseq, pi->last_acked_seq, pi) >= pi->tx_win) {
			BT_DBG("Invalid - txseq outside tx window");
			return L2CAP_ERTM_TXSEQ_INVALID;
		} else {
			BT_DBG("Expected");
			return L2CAP_ERTM_TXSEQ_EXPECTED;
		}
	}

	if (__delta_seq(txseq, pi->last_acked_seq, pi) <
		__delta_seq(pi->expected_tx_seq, pi->last_acked_seq, pi)) {
		BT_DBG("Duplicate - expected_tx_seq later than txseq");
		return L2CAP_ERTM_TXSEQ_DUPLICATE;
	}

	if (__delta_seq(txseq, pi->last_acked_seq, pi) >= pi->tx_win) {
		/* A source of invalid packets is a "double poll" condition,
		 * where delays cause us to send multiple poll packets.  If
		 * the remote stack receives and processes both polls,
		 * sequence numbers can wrap around in such a way that a
		 * resent frame has a sequence number that looks like new data
		 * with a sequence gap.  This would trigger an erroneous SREJ
		 * request.
		 *
		 * Fortunately, this is impossible with a tx window that's
		 * less than half of the maximum sequence number, which allows
		 * invalid frames to be safely ignored.
		 *
		 * With tx window sizes greater than half of the tx window
		 * maximum, the frame is invalid and cannot be ignored.  This
		 * causes a disconnect.
		 */

		if (pi->tx_win <= ((pi->tx_win_max + 1) >> 1)) {
			BT_DBG("Invalid/Ignore - txseq outside tx window");
			return L2CAP_ERTM_TXSEQ_INVALID_IGNORE;
		} else {
			BT_DBG("Invalid - txseq outside tx window");
			return L2CAP_ERTM_TXSEQ_INVALID;
		}
	} else {
		BT_DBG("Unexpected - txseq indicates missing frames");
		return L2CAP_ERTM_TXSEQ_UNEXPECTED;
	}
}

static int l2cap_ertm_rx_state_recv(struct sock *sk,
				struct bt_l2cap_control *control,
				struct sk_buff *skb, u8 event)
{
	struct l2cap_pinfo *pi;
	int err = 0;
	bool skb_in_use = 0;

	BT_DBG("sk %p, control %p, skb %p, event %d", sk, control, skb,
		(int)event);
	pi = l2cap_pi(sk);

	switch (event) {
	case L2CAP_ERTM_EVENT_RECV_IFRAME:
		switch (l2cap_ertm_classify_txseq(sk, control->txseq)) {
		case L2CAP_ERTM_TXSEQ_EXPECTED:
			l2cap_ertm_pass_to_tx(sk, control);

			if (pi->conn_state & L2CAP_CONN_LOCAL_BUSY) {
				BT_DBG("Busy, discarding expected seq %d",
					control->txseq);
				break;
			}

			pi->expected_tx_seq = __next_seq(control->txseq, pi);
			pi->buffer_seq = pi->expected_tx_seq;
			skb_in_use = 1;

			err = l2cap_ertm_rx_expected_iframe(sk, control, skb);
			if (err)
				break;

			if (control->final) {
				if (pi->conn_state & L2CAP_CONN_REJ_ACT)
					pi->conn_state &= ~L2CAP_CONN_REJ_ACT;
				else {
					control->final = 0;
					l2cap_ertm_retransmit_all(sk, control);
					l2cap_ertm_send_txq(sk);
				}
			}

			if (!(pi->conn_state & L2CAP_CONN_LOCAL_BUSY))
				l2cap_ertm_send_ack(sk);
			break;
		case L2CAP_ERTM_TXSEQ_UNEXPECTED:
			l2cap_ertm_pass_to_tx(sk, control);

			/* Can't issue SREJ frames in the local busy state.
			 * Drop this frame, it will be seen as missing
			 * when local busy is exited.
			 */
			if (pi->conn_state & L2CAP_CONN_LOCAL_BUSY) {
				BT_DBG("Busy, discarding unexpected seq %d",
					control->txseq);
				break;
			}

			/* There was a gap in the sequence, so an SREJ
			 * must be sent for each missing frame.  The
			 * current frame is stored for later use.
			 */
			skb_queue_tail(SREJ_QUEUE(sk), skb);
			skb_in_use = 1;
			BT_DBG("Queued %p (queue len %d)", skb,
			       skb_queue_len(SREJ_QUEUE(sk)));

			pi->conn_state &= ~L2CAP_CONN_SREJ_ACT;
			l2cap_seq_list_clear(&pi->srej_list);
			l2cap_ertm_send_srej(sk, control->txseq);

			pi->rx_state = L2CAP_ERTM_RX_STATE_SREJ_SENT;
			break;
		case L2CAP_ERTM_TXSEQ_DUPLICATE:
			l2cap_ertm_pass_to_tx(sk, control);
			break;
		case L2CAP_ERTM_TXSEQ_INVALID_IGNORE:
			break;
		case L2CAP_ERTM_TXSEQ_INVALID:
		default:
			l2cap_send_disconn_req(l2cap_pi(sk)->conn, sk,
					ECONNRESET);
			break;
		}
		break;
	case L2CAP_ERTM_EVENT_RECV_RR:
		l2cap_ertm_pass_to_tx(sk, control);
		if (control->final) {
			pi->conn_state &= ~L2CAP_CONN_REMOTE_BUSY;

			if (pi->conn_state & L2CAP_CONN_REJ_ACT)
				pi->conn_state &= ~L2CAP_CONN_REJ_ACT;
			else if (pi->amp_move_state == L2CAP_AMP_STATE_STABLE) {
				control->final = 0;
				l2cap_ertm_retransmit_all(sk, control);
			}

			l2cap_ertm_send_txq(sk);
		} else if (control->poll) {
			l2cap_ertm_send_i_or_rr_or_rnr(sk);
		} else {
			if ((pi->conn_state & L2CAP_CONN_REMOTE_BUSY) &&
				pi->unacked_frames)
				l2cap_ertm_start_retrans_timer(pi);
			pi->conn_state &= ~L2CAP_CONN_REMOTE_BUSY;
			l2cap_ertm_send_txq(sk);
		}
		break;
	case L2CAP_ERTM_EVENT_RECV_RNR:
		pi->conn_state |= L2CAP_CONN_REMOTE_BUSY;
		l2cap_ertm_pass_to_tx(sk, control);
		if (control && control->poll) {
			pi->conn_state |= L2CAP_CONN_SEND_FBIT;
			l2cap_ertm_send_rr_or_rnr(sk, 0);
		}
		l2cap_ertm_stop_retrans_timer(pi);
		l2cap_seq_list_clear(&pi->retrans_list);
		break;
	case L2CAP_ERTM_EVENT_RECV_REJ:
		l2cap_ertm_handle_rej(sk, control);
		break;
	case L2CAP_ERTM_EVENT_RECV_SREJ:
		l2cap_ertm_handle_srej(sk, control);
		break;
	default:
		break;
	}

	if (skb && !skb_in_use) {
		BT_DBG("Freeing %p", skb);
		kfree_skb(skb);
	}

	return err;
}

static int l2cap_ertm_rx_state_srej_sent(struct sock *sk,
					struct bt_l2cap_control *control,
					struct sk_buff *skb, u8 event)
{
	struct l2cap_pinfo *pi;
	int err = 0;
	u16 txseq = control->txseq;
	bool skb_in_use = 0;

	BT_DBG("sk %p, control %p, skb %p, event %d", sk, control, skb,
		(int)event);
	pi = l2cap_pi(sk);

	switch (event) {
	case L2CAP_ERTM_EVENT_RECV_IFRAME:
		switch (l2cap_ertm_classify_txseq(sk, txseq)) {
		case L2CAP_ERTM_TXSEQ_EXPECTED:
			/* Keep frame for reassembly later */
			l2cap_ertm_pass_to_tx(sk, control);
			skb_queue_tail(SREJ_QUEUE(sk), skb);
			skb_in_use = 1;
			BT_DBG("Queued %p (queue len %d)", skb,
			       skb_queue_len(SREJ_QUEUE(sk)));

			pi->expected_tx_seq = __next_seq(txseq, pi);
			break;
		case L2CAP_ERTM_TXSEQ_EXPECTED_SREJ:
			l2cap_seq_list_pop(&pi->srej_list);

			l2cap_ertm_pass_to_tx(sk, control);
			skb_queue_tail(SREJ_QUEUE(sk), skb);
			skb_in_use = 1;
			BT_DBG("Queued %p (queue len %d)", skb,
			       skb_queue_len(SREJ_QUEUE(sk)));

			err = l2cap_ertm_rx_queued_iframes(sk);
			if (err)
				break;

			break;
		case L2CAP_ERTM_TXSEQ_UNEXPECTED:
			/* Got a frame that can't be reassembled yet.
			 * Save it for later, and send SREJs to cover
			 * the missing frames.
			 */
			skb_queue_tail(SREJ_QUEUE(sk), skb);
			skb_in_use = 1;
			BT_DBG("Queued %p (queue len %d)", skb,
			       skb_queue_len(SREJ_QUEUE(sk)));

			l2cap_ertm_pass_to_tx(sk, control);
			l2cap_ertm_send_srej(sk, control->txseq);
			break;
		case L2CAP_ERTM_TXSEQ_UNEXPECTED_SREJ:
			/* This frame was requested with an SREJ, but
			 * some expected retransmitted frames are
			 * missing.  Request retransmission of missing
			 * SREJ'd frames.
			 */
			skb_queue_tail(SREJ_QUEUE(sk), skb);
			skb_in_use = 1;
			BT_DBG("Queued %p (queue len %d)", skb,
			       skb_queue_len(SREJ_QUEUE(sk)));

			l2cap_ertm_pass_to_tx(sk, control);
			l2cap_ertm_send_srej_list(sk, control->txseq);
			break;
		case L2CAP_ERTM_TXSEQ_DUPLICATE_SREJ:
			/* We've already queued this frame.  Drop this copy. */
			l2cap_ertm_pass_to_tx(sk, control);
			break;
		case L2CAP_ERTM_TXSEQ_DUPLICATE:
			/* Expecting a later sequence number, so this frame
			 * was already received.  Ignore it completely.
			 */
			break;
		case L2CAP_ERTM_TXSEQ_INVALID_IGNORE:
			break;
		case L2CAP_ERTM_TXSEQ_INVALID:
		default:
			l2cap_send_disconn_req(l2cap_pi(sk)->conn, sk,
					ECONNRESET);
			break;
		}
		break;
	case L2CAP_ERTM_EVENT_RECV_RR:
		l2cap_ertm_pass_to_tx(sk, control);
		if (control->final) {
			pi->conn_state &= ~L2CAP_CONN_REMOTE_BUSY;

			if (pi->conn_state & L2CAP_CONN_REJ_ACT)
				pi->conn_state &= ~L2CAP_CONN_REJ_ACT;
			else {
				control->final = 0;
				l2cap_ertm_retransmit_all(sk, control);
			}

			l2cap_ertm_send_txq(sk);
		} else if (control->poll) {
			if ((pi->conn_state & L2CAP_CONN_REMOTE_BUSY) &&
				pi->unacked_frames) {
				l2cap_ertm_start_retrans_timer(pi);
			}
			pi->conn_state &= ~L2CAP_CONN_REMOTE_BUSY;
			pi->conn_state |= L2CAP_CONN_SEND_FBIT;
			l2cap_ertm_send_srej_tail(sk);
		} else {
			if ((pi->conn_state & L2CAP_CONN_REMOTE_BUSY) &&
				pi->unacked_frames) {
				l2cap_ertm_start_retrans_timer(pi);
			}
			pi->conn_state &= ~L2CAP_CONN_REMOTE_BUSY;
			l2cap_ertm_send_ack(sk);
		}
		break;
	case L2CAP_ERTM_EVENT_RECV_RNR:
		pi->conn_state |= L2CAP_CONN_REMOTE_BUSY;
		l2cap_ertm_pass_to_tx(sk, control);
		if (control->poll)
			l2cap_ertm_send_srej_tail(sk);
		else {
			struct bt_l2cap_control rr_control;
			memset(&rr_control, 0, sizeof(rr_control));
			rr_control.frame_type = 's';
			rr_control.super = L2CAP_SFRAME_RR;
			rr_control.reqseq = pi->buffer_seq;
			l2cap_ertm_send_sframe(sk, &rr_control);
		}

		break;
	case L2CAP_ERTM_EVENT_RECV_REJ:
		l2cap_ertm_handle_rej(sk, control);
		break;
	case L2CAP_ERTM_EVENT_RECV_SREJ:
		l2cap_ertm_handle_srej(sk, control);
		break;
	}

	if (skb && !skb_in_use) {
		BT_DBG("Freeing %p", skb);
		kfree_skb(skb);
	}

	return err;
}

static int l2cap_ertm_rx_state_amp_move(struct sock *sk,
					struct bt_l2cap_control *control,
					struct sk_buff *skb, u8 event)
{
	struct l2cap_pinfo *pi;
	int err = 0;
	bool skb_in_use = 0;

	BT_DBG("sk %p, control %p, skb %p, event %d", sk, control, skb,
		(int)event);
	pi = l2cap_pi(sk);

	/* Only handle expected frames, to avoid state changes. */

	switch (event) {
	case L2CAP_ERTM_EVENT_RECV_IFRAME:
		if (l2cap_ertm_classify_txseq(sk, control->txseq) ==
				L2CAP_ERTM_TXSEQ_EXPECTED) {
			l2cap_ertm_pass_to_tx(sk, control);

			if (pi->conn_state & L2CAP_CONN_LOCAL_BUSY) {
				BT_DBG("Busy, discarding expected seq %d",
					control->txseq);
				break;
			}

			pi->expected_tx_seq = __next_seq(control->txseq, pi);
			pi->buffer_seq = pi->expected_tx_seq;
			skb_in_use = 1;

			err = l2cap_ertm_rx_expected_iframe(sk, control, skb);
			if (err)
				break;

			if (control->final) {
				if (pi->conn_state & L2CAP_CONN_REJ_ACT)
					pi->conn_state &= ~L2CAP_CONN_REJ_ACT;
				else
					control->final = 0;
			}
		}
		break;
	case L2CAP_ERTM_EVENT_RECV_RR:
	case L2CAP_ERTM_EVENT_RECV_RNR:
	case L2CAP_ERTM_EVENT_RECV_REJ:
		l2cap_ertm_process_reqseq(sk, control->reqseq);
		break;
	case L2CAP_ERTM_EVENT_RECV_SREJ:
		/* Ignore */
		break;
	default:
		break;
	}

	if (skb && !skb_in_use) {
		BT_DBG("Freeing %p", skb);
		kfree_skb(skb);
	}

	return err;
}

static int l2cap_answer_move_poll(struct sock *sk)
{
	struct l2cap_pinfo *pi;
	struct bt_l2cap_control control;
	int err = 0;

	BT_DBG("sk %p", sk);

	pi = l2cap_pi(sk);

	l2cap_ertm_process_reqseq(sk, pi->amp_move_reqseq);

	if (!skb_queue_empty(TX_QUEUE(sk)))
		sk->sk_send_head = skb_peek(TX_QUEUE(sk));
	else
		sk->sk_send_head = NULL;

	/* Rewind next_tx_seq to the point expected
	 * by the receiver.
	 */
	pi->next_tx_seq = pi->amp_move_reqseq;
	pi->unacked_frames = 0;

	err = l2cap_finish_amp_move(sk);

	if (err)
		return err;

	pi->conn_state |= L2CAP_CONN_SEND_FBIT;
	l2cap_ertm_send_i_or_rr_or_rnr(sk);

	memset(&control, 0, sizeof(control));
	control.reqseq = pi->amp_move_reqseq;

	if (pi->amp_move_event == L2CAP_ERTM_EVENT_RECV_IFRAME)
		err = -EPROTO;
	else
		err = l2cap_ertm_rx_state_recv(sk, &control, NULL,
					pi->amp_move_event);

	return err;
}

static void l2cap_amp_move_setup(struct sock *sk)
{
	struct l2cap_pinfo *pi;
	struct sk_buff *skb;

	BT_DBG("sk %p", sk);

	pi = l2cap_pi(sk);

	l2cap_ertm_stop_ack_timer(pi);
	l2cap_ertm_stop_retrans_timer(pi);
	l2cap_ertm_stop_monitor_timer(pi);

	pi->retry_count = 0;
	skb_queue_walk(TX_QUEUE(sk), skb) {
		if (bt_cb(skb)->retries)
			bt_cb(skb)->retries = 1;
		else
			break;
	}

	pi->expected_tx_seq = pi->buffer_seq;

	pi->conn_state &= ~(L2CAP_CONN_REJ_ACT | L2CAP_CONN_SREJ_ACT);
	l2cap_seq_list_clear(&pi->retrans_list);
	l2cap_seq_list_clear(&l2cap_pi(sk)->srej_list);
	skb_queue_purge(SREJ_QUEUE(sk));

	pi->tx_state = L2CAP_ERTM_TX_STATE_XMIT;
	pi->rx_state = L2CAP_ERTM_RX_STATE_AMP_MOVE;

	BT_DBG("tx_state 0x2.2%x rx_state  0x2.2%x", pi->tx_state,
		pi->rx_state);

	pi->conn_state |= L2CAP_CONN_REMOTE_BUSY;
}

static void l2cap_amp_move_revert(struct sock *sk)
{
	struct l2cap_pinfo *pi;

	BT_DBG("sk %p", sk);

	pi = l2cap_pi(sk);

	if (pi->amp_move_role == L2CAP_AMP_MOVE_INITIATOR) {
		l2cap_ertm_tx(sk, NULL, NULL, L2CAP_ERTM_EVENT_EXPLICIT_POLL);
		pi->rx_state = L2CAP_ERTM_RX_STATE_WAIT_F_FLAG;
	} else if (pi->amp_move_role == L2CAP_AMP_MOVE_RESPONDER)
		pi->rx_state = L2CAP_ERTM_RX_STATE_WAIT_P_FLAG;
}

static int l2cap_amp_move_reconf(struct sock *sk)
{
	struct l2cap_pinfo *pi;
	u8 buf[64];
	int err = 0;

	BT_DBG("sk %p", sk);

	pi = l2cap_pi(sk);

	l2cap_send_cmd(pi->conn, l2cap_get_ident(pi->conn), L2CAP_CONF_REQ,
				l2cap_build_amp_reconf_req(sk, buf), buf);
	return err;
}

static void l2cap_amp_move_success(struct sock *sk)
{
	struct l2cap_pinfo *pi;

	BT_DBG("sk %p", sk);

	pi = l2cap_pi(sk);

	if (pi->amp_move_role == L2CAP_AMP_MOVE_INITIATOR) {
		int err = 0;
		/* Send reconfigure request */
		if (pi->mode == L2CAP_MODE_ERTM) {
			pi->reconf_state = L2CAP_RECONF_INT;
			err = l2cap_amp_move_reconf(sk);

			if (err) {
				pi->reconf_state = L2CAP_RECONF_NONE;
				l2cap_ertm_tx(sk, NULL, NULL,
						L2CAP_ERTM_EVENT_EXPLICIT_POLL);
				pi->rx_state = L2CAP_ERTM_RX_STATE_WAIT_F_FLAG;
			}
		} else
			pi->rx_state = L2CAP_ERTM_RX_STATE_RECV;
	} else if (pi->amp_move_role == L2CAP_AMP_MOVE_RESPONDER) {
		if (pi->mode == L2CAP_MODE_ERTM)
			pi->rx_state =
				L2CAP_ERTM_RX_STATE_WAIT_P_FLAG_RECONFIGURE;
		else
			pi->rx_state = L2CAP_ERTM_RX_STATE_RECV;
	}
}

static inline bool __valid_reqseq(struct l2cap_pinfo *pi, u16 reqseq)
{
	/* Make sure reqseq is for a packet that has been sent but not acked */
	u16 unacked = __delta_seq(pi->next_tx_seq, pi->expected_ack_seq, pi);
	return __delta_seq(pi->next_tx_seq, reqseq, pi) <= unacked;
}

static int l2cap_strm_rx(struct sock *sk, struct bt_l2cap_control *control,
			struct sk_buff *skb)
{
	struct l2cap_pinfo *pi;
	int err = 0;

	BT_DBG("sk %p, control %p, skb %p, state %d",
		sk, control, skb, l2cap_pi(sk)->rx_state);

	pi = l2cap_pi(sk);

	if (l2cap_ertm_classify_txseq(sk, control->txseq) ==
		L2CAP_ERTM_TXSEQ_EXPECTED) {
		l2cap_ertm_pass_to_tx(sk, control);

		BT_DBG("buffer_seq %d->%d", pi->buffer_seq,
			   __next_seq(pi->buffer_seq, pi));

		pi->buffer_seq = __next_seq(pi->buffer_seq, pi);

		l2cap_ertm_rx_expected_iframe(sk, control, skb);
	} else {
		if (pi->sdu) {
			kfree_skb(pi->sdu);
			pi->sdu = NULL;
		}
		pi->sdu_last_frag = NULL;
		pi->sdu_len = 0;

		if (skb) {
			BT_DBG("Freeing %p", skb);
			kfree_skb(skb);
		}
	}

	pi->last_acked_seq = control->txseq;
	pi->expected_tx_seq = __next_seq(control->txseq, pi);

	return err;
}

static int l2cap_ertm_rx(struct sock *sk, struct bt_l2cap_control *control,
			struct sk_buff *skb, u8 event)
{
	struct l2cap_pinfo *pi;
	int err = 0;

	BT_DBG("sk %p, control %p, skb %p, event %d, state %d",
		sk, control, skb, (int)event, l2cap_pi(sk)->rx_state);

	pi = l2cap_pi(sk);

	if (__valid_reqseq(pi, control->reqseq)) {
		switch (pi->rx_state) {
		case L2CAP_ERTM_RX_STATE_RECV:
			err = l2cap_ertm_rx_state_recv(sk, control, skb, event);
			break;
		case L2CAP_ERTM_RX_STATE_SREJ_SENT:
			err = l2cap_ertm_rx_state_srej_sent(sk, control, skb,
							event);
			break;
		case L2CAP_ERTM_RX_STATE_AMP_MOVE:
			err = l2cap_ertm_rx_state_amp_move(sk, control, skb,
							event);
			break;
		case L2CAP_ERTM_RX_STATE_WAIT_F_FLAG:
			if (control->final) {
				pi->conn_state &= ~L2CAP_CONN_REMOTE_BUSY;
				pi->amp_move_role = L2CAP_AMP_MOVE_NONE;

				pi->rx_state = L2CAP_ERTM_RX_STATE_RECV;
				l2cap_ertm_process_reqseq(sk, control->reqseq);

				if (!skb_queue_empty(TX_QUEUE(sk)))
					sk->sk_send_head =
						skb_peek(TX_QUEUE(sk));
				else
					sk->sk_send_head = NULL;

				/* Rewind next_tx_seq to the point expected
				 * by the receiver.
				 */
				pi->next_tx_seq = control->reqseq;
				pi->unacked_frames = 0;

				if (pi->ampcon)
					pi->conn->mtu =
						pi->ampcon->hdev->acl_mtu;
				else
					pi->conn->mtu =
						pi->conn->hcon->hdev->acl_mtu;

				err = l2cap_setup_resegment(sk);

				if (err)
					break;

				err = l2cap_ertm_rx_state_recv(sk, control, skb,
							event);
			}
			break;
		case L2CAP_ERTM_RX_STATE_WAIT_P_FLAG:
			if (control->poll) {
				pi->amp_move_reqseq = control->reqseq;
				pi->amp_move_event = event;
				err = l2cap_answer_move_poll(sk);
			}
			break;
		case L2CAP_ERTM_RX_STATE_WAIT_P_FLAG_RECONFIGURE:
			if (control->poll) {
				pi->amp_move_reqseq = control->reqseq;
				pi->amp_move_event = event;

				BT_DBG("amp_move_role 0x%2.2x, "
					"reconf_state 0x%2.2x",
					pi->amp_move_role, pi->reconf_state);

				if (pi->reconf_state == L2CAP_RECONF_ACC)
					err = l2cap_amp_move_reconf(sk);
				else
					err = l2cap_answer_move_poll(sk);
			}
			break;
		default:
			/* shut it down */
			break;
		}
	} else {
		BT_DBG("Invalid reqseq %d (next_tx_seq %d, expected_ack_seq %d",
			control->reqseq, pi->next_tx_seq, pi->expected_ack_seq);
		l2cap_send_disconn_req(pi->conn, sk, ECONNRESET);
	}

	return err;
}

void l2cap_fixed_channel_config(struct sock *sk, struct l2cap_options *opt,
				u16 cid, u16 mps)
{
	lock_sock(sk);

	l2cap_pi(sk)->fixed_channel = 1;

	l2cap_pi(sk)->dcid = cid;
	l2cap_pi(sk)->scid = cid;

	l2cap_pi(sk)->imtu = opt->imtu;
	l2cap_pi(sk)->omtu = opt->omtu;
	l2cap_pi(sk)->remote_mps = mps;
	l2cap_pi(sk)->mps = mps;
	l2cap_pi(sk)->flush_to = opt->flush_to;
	l2cap_pi(sk)->mode = opt->mode;
	l2cap_pi(sk)->fcs = opt->fcs;
	l2cap_pi(sk)->max_tx = opt->max_tx;
	l2cap_pi(sk)->remote_max_tx = opt->max_tx;
	l2cap_pi(sk)->tx_win = opt->txwin_size;
	l2cap_pi(sk)->remote_tx_win = opt->txwin_size;
	l2cap_pi(sk)->retrans_timeout = L2CAP_DEFAULT_RETRANS_TO;
	l2cap_pi(sk)->monitor_timeout = L2CAP_DEFAULT_MONITOR_TO;

	if (opt->mode == L2CAP_MODE_ERTM ||
		l2cap_pi(sk)->mode == L2CAP_MODE_STREAMING)
		l2cap_ertm_init(sk);

	release_sock(sk);

	return;
}

static const u8 l2cap_ertm_rx_func_to_event[4] = {
	L2CAP_ERTM_EVENT_RECV_RR, L2CAP_ERTM_EVENT_RECV_REJ,
	L2CAP_ERTM_EVENT_RECV_RNR, L2CAP_ERTM_EVENT_RECV_SREJ
};

static int l2cap_data_channel(struct sock *sk, struct sk_buff *skb)
{
	struct l2cap_pinfo *pi;
	struct bt_l2cap_control *control;
	u16 len;
	u8 event;
	pi = l2cap_pi(sk);

	BT_DBG("sk %p, len %d, mode %d", sk, skb->len, pi->mode);

	if (sk->sk_state != BT_CONNECTED)
		goto drop;

	switch (pi->mode) {
	case L2CAP_MODE_BASIC:
		/* If socket recv buffers overflows we drop data here
		 * which is *bad* because L2CAP has to be reliable.
		 * But we don't have any other choice. L2CAP doesn't
		 * provide flow control mechanism. */

		if (pi->imtu < skb->len)
			goto drop;

		if (!sock_queue_rcv_skb(sk, skb))
			goto done;
		break;

	case L2CAP_MODE_ERTM:
	case L2CAP_MODE_STREAMING:
		control = &bt_cb(skb)->control;
		if (pi->extended_control) {
			__get_extended_control(get_unaligned_le32(skb->data),
						control);
			skb_pull(skb, 4);
		} else {
			__get_enhanced_control(get_unaligned_le16(skb->data),
						control);
			skb_pull(skb, 2);
		}

		len = skb->len;

		if (l2cap_check_fcs(pi, skb))
			goto drop;

		if ((control->frame_type == 'i') &&
			(control->sar == L2CAP_SAR_START))
			len -= 2;

		if (pi->fcs == L2CAP_FCS_CRC16)
			len -= 2;

		/*
		 * We can just drop the corrupted I-frame here.
		 * Receiver will miss it and start proper recovery
		 * procedures and ask for retransmission.
		 */
		if (len > pi->mps) {
			l2cap_send_disconn_req(pi->conn, sk, ECONNRESET);
			goto drop;
		}

		if (control->frame_type == 'i') {

			int err;

			BT_DBG("iframe sar %d, reqseq %d, final %d, txseq %d",
				control->sar, control->reqseq, control->final,
				control->txseq);

			/* Validate F-bit - F=0 always valid, F=1 only
			 * valid in TX WAIT_F
			 */
			if (control->final && (pi->tx_state !=
					L2CAP_ERTM_TX_STATE_WAIT_F))
				goto drop;

			if (pi->mode != L2CAP_MODE_STREAMING) {
				event = L2CAP_ERTM_EVENT_RECV_IFRAME;
				err = l2cap_ertm_rx(sk, control, skb, event);
			} else
				err = l2cap_strm_rx(sk, control, skb);
			if (err)
				l2cap_send_disconn_req(pi->conn, sk,
						ECONNRESET);
		} else {
			/* Only I-frames are expected in streaming mode */
			if (pi->mode == L2CAP_MODE_STREAMING)
				goto drop;

			BT_DBG("sframe reqseq %d, final %d, poll %d, super %d",
				control->reqseq, control->final, control->poll,
				control->super);

			if (len != 0) {
				l2cap_send_disconn_req(pi->conn, sk,
						ECONNRESET);
				goto drop;
			}

			/* Validate F and P bits */
			if (control->final &&
				((pi->tx_state != L2CAP_ERTM_TX_STATE_WAIT_F)
					|| control->poll))
				goto drop;

			event = l2cap_ertm_rx_func_to_event[control->super];
			if (l2cap_ertm_rx(sk, control, skb, event))
				l2cap_send_disconn_req(pi->conn, sk,
						ECONNRESET);
		}

		goto done;

	default:
		BT_DBG("sk %p: bad mode 0x%2.2x", sk, pi->mode);
		break;
	}

drop:
	kfree_skb(skb);

done:
	return 0;
}

void l2cap_recv_deferred_frame(struct sock *sk, struct sk_buff *skb)
{
	lock_sock(sk);
	l2cap_data_channel(sk, skb);
	release_sock(sk);
}

static inline int l2cap_conless_channel(struct l2cap_conn *conn, __le16 psm, struct sk_buff *skb)
{
	struct sock *sk;

	sk = l2cap_get_sock_by_psm(0, psm, conn->src);
	if (!sk)
		goto drop;

	BT_DBG("sk %p, len %d", sk, skb->len);

	if (sk->sk_state != BT_BOUND && sk->sk_state != BT_CONNECTED)
		goto drop;

	if (l2cap_pi(sk)->imtu < skb->len)
		goto drop;

	if (!sock_queue_rcv_skb(sk, skb))
		goto done;

drop:
	kfree_skb(skb);

done:
	if (sk)
		bh_unlock_sock(sk);
	return 0;
}

static void l2cap_recv_frame(struct l2cap_conn *conn, struct sk_buff *skb)
{
	struct l2cap_hdr *lh = (void *) skb->data;
	struct sock *sk;
	u16 cid, len;
	__le16 psm;

	skb_pull(skb, L2CAP_HDR_SIZE);
	cid = __le16_to_cpu(lh->cid);
	len = __le16_to_cpu(lh->len);

	if (len != skb->len) {
		kfree_skb(skb);
		return;
	}

	BT_DBG("len %d, cid 0x%4.4x", len, cid);

	switch (cid) {
	case L2CAP_CID_SIGNALING:
		l2cap_sig_channel(conn, skb);
		break;

	case L2CAP_CID_CONN_LESS:
		psm = get_unaligned_le16(skb->data);
		skb_pull(skb, 2);
		l2cap_conless_channel(conn, psm, skb);
		break;

	default:
		sk = l2cap_get_chan_by_scid(&conn->chan_list, cid);
		if (sk) {
			if (sock_owned_by_user(sk)) {
				BT_DBG("backlog sk %p", sk);
				if (sk_add_backlog(sk, skb))
					kfree_skb(skb);
			} else
				l2cap_data_channel(sk, skb);

			bh_unlock_sock(sk);
		} else if (cid == L2CAP_CID_A2MP) {
			BT_DBG("A2MP");
			amp_conn_ind(conn, skb);
		} else {
			BT_DBG("unknown cid 0x%4.4x", cid);
			kfree_skb(skb);
		}

		break;
	}
}

/* ---- L2CAP interface with lower layer (HCI) ---- */

static int l2cap_connect_ind(struct hci_dev *hdev, bdaddr_t *bdaddr, u8 type)
{
	int exact = 0, lm1 = 0, lm2 = 0;
	register struct sock *sk;
	struct hlist_node *node;

	if (type != ACL_LINK)
		return -EINVAL;

	BT_DBG("hdev %s, bdaddr %s", hdev->name, batostr(bdaddr));

	/* Find listening sockets and check their link_mode */
	read_lock(&l2cap_sk_list.lock);
	sk_for_each(sk, node, &l2cap_sk_list.head) {
		if (sk->sk_state != BT_LISTEN)
			continue;

		if (!bacmp(&bt_sk(sk)->src, &hdev->bdaddr)) {
			lm1 |= HCI_LM_ACCEPT;
			if (l2cap_pi(sk)->role_switch)
				lm1 |= HCI_LM_MASTER;
			exact++;
		} else if (!bacmp(&bt_sk(sk)->src, BDADDR_ANY)) {
			lm2 |= HCI_LM_ACCEPT;
			if (l2cap_pi(sk)->role_switch)
				lm2 |= HCI_LM_MASTER;
		}
	}
	read_unlock(&l2cap_sk_list.lock);

	return exact ? lm1 : lm2;
}

static int l2cap_connect_cfm(struct hci_conn *hcon, u8 status)
{
	struct l2cap_conn *conn;

	BT_DBG("hcon %p bdaddr %s status %d", hcon, batostr(&hcon->dst), status);

	if (hcon->type != ACL_LINK)
		return -EINVAL;

	if (!status) {
		conn = l2cap_conn_add(hcon, status);
		if (conn)
			l2cap_conn_ready(conn);
	} else
		l2cap_conn_del(hcon, bt_err(status));

	return 0;
}

static int l2cap_disconn_ind(struct hci_conn *hcon)
{
	struct l2cap_conn *conn = hcon->l2cap_data;

	BT_DBG("hcon %p", hcon);

	if (hcon->type != ACL_LINK || !conn)
		return 0x13;

	return conn->disc_reason;
}

static int l2cap_disconn_cfm(struct hci_conn *hcon, u8 reason)
{
	BT_DBG("hcon %p reason %d", hcon, reason);

	if (hcon->type != ACL_LINK)
		return -EINVAL;

	l2cap_conn_del(hcon, bt_err(reason));

	return 0;
}

static inline void l2cap_check_encryption(struct sock *sk, u8 encrypt)
{
	if (sk->sk_type != SOCK_SEQPACKET && sk->sk_type != SOCK_STREAM)
		return;

	if (encrypt == 0x00) {
		if (l2cap_pi(sk)->sec_level == BT_SECURITY_MEDIUM) {
			l2cap_sock_clear_timer(sk);
			l2cap_sock_set_timer(sk, HZ * 5);
		} else if (l2cap_pi(sk)->sec_level == BT_SECURITY_HIGH)
			__l2cap_sock_close(sk, ECONNREFUSED);
	} else {
		if (l2cap_pi(sk)->sec_level == BT_SECURITY_MEDIUM)
			l2cap_sock_clear_timer(sk);
	}
}

static int l2cap_security_cfm(struct hci_conn *hcon, u8 status, u8 encrypt)
{
	struct l2cap_chan_list *l;
	struct l2cap_conn *conn = hcon->l2cap_data;
	struct sock *sk;

	if (!conn)
		return 0;

	l = &conn->chan_list;

	BT_DBG("conn %p", conn);

	read_lock(&l->lock);

	for (sk = l->head; sk; sk = l2cap_pi(sk)->next_c) {
		bh_lock_sock(sk);

		if (l2cap_pi(sk)->conf_state & L2CAP_CONF_CONNECT_PEND) {
			bh_unlock_sock(sk);
			continue;
		}

		if (!status && (sk->sk_state == BT_CONNECTED ||
						sk->sk_state == BT_CONFIG)) {
			l2cap_check_encryption(sk, encrypt);
			bh_unlock_sock(sk);
			continue;
		}

		if (sk->sk_state == BT_CONNECT) {
			if (!status) {
				struct l2cap_conn_req req;
				req.scid = cpu_to_le16(l2cap_pi(sk)->scid);
				req.psm  = l2cap_pi(sk)->psm;

				l2cap_pi(sk)->ident = l2cap_get_ident(conn);
				l2cap_pi(sk)->conf_state |= L2CAP_CONF_CONNECT_PEND;

				l2cap_send_cmd(conn, l2cap_pi(sk)->ident,
					L2CAP_CONN_REQ, sizeof(req), &req);
			} else {
				l2cap_sock_clear_timer(sk);
				l2cap_sock_set_timer(sk, HZ / 10);
			}
		} else if (sk->sk_state == BT_CONNECT2) {
			struct l2cap_conn_rsp rsp;
			__u16 result;

			if (!status) {
				sk->sk_state = BT_CONFIG;
				result = L2CAP_CR_SUCCESS;
			} else {
				sk->sk_state = BT_DISCONN;
				l2cap_sock_set_timer(sk, HZ / 10);
				result = L2CAP_CR_SEC_BLOCK;
			}

			rsp.scid   = cpu_to_le16(l2cap_pi(sk)->dcid);
			rsp.dcid   = cpu_to_le16(l2cap_pi(sk)->scid);
			rsp.result = cpu_to_le16(result);
			rsp.status = cpu_to_le16(L2CAP_CS_NO_INFO);
			l2cap_send_cmd(conn, l2cap_pi(sk)->ident,
					L2CAP_CONN_RSP, sizeof(rsp), &rsp);
		}

		bh_unlock_sock(sk);
	}

	read_unlock(&l->lock);

	return 0;
}

static int l2cap_recv_acldata(struct hci_conn *hcon, struct sk_buff *skb, u16 flags)
{
	struct l2cap_conn *conn = hcon->l2cap_data;

	if (!conn && hcon->hdev->dev_type != HCI_BREDR)
		goto drop;

	if (!conn && !(conn = l2cap_conn_add(hcon, 0)))
		goto drop;

	BT_DBG("conn %p len %d flags 0x%x", conn, skb->len, flags);

	if (flags & ACL_START) {
		struct l2cap_hdr *hdr;
		int len;

		if (conn->rx_len) {
			BT_ERR("Unexpected start frame (len %d)", skb->len);
			kfree_skb(conn->rx_skb);
			conn->rx_skb = NULL;
			conn->rx_len = 0;
			l2cap_conn_unreliable(conn, ECOMM);
		}

		if (skb->len < 2) {
			BT_ERR("Frame is too short (len %d)", skb->len);
			l2cap_conn_unreliable(conn, ECOMM);
			goto drop;
		}

		hdr = (struct l2cap_hdr *) skb->data;
		len = __le16_to_cpu(hdr->len) + L2CAP_HDR_SIZE;

		if (len == skb->len) {
			/* Complete frame received */
			l2cap_recv_frame(conn, skb);
			return 0;
		}

		if (flags & ACL_CONT) {
			BT_ERR("Complete frame is incomplete "
				"(len %d, expected len %d)",
				skb->len, len);
			l2cap_conn_unreliable(conn, ECOMM);
			goto drop;
		}

		BT_DBG("Start: total len %d, frag len %d", len, skb->len);

		if (skb->len > len) {
			BT_ERR("Frame is too long (len %d, expected len %d)",
				skb->len, len);
			l2cap_conn_unreliable(conn, ECOMM);
			goto drop;
		}

		/* Allocate skb for the complete frame (with header) */
		conn->rx_skb = bt_skb_alloc(len, GFP_ATOMIC);
		if (!conn->rx_skb)
			goto drop;

		skb_copy_from_linear_data(skb, skb_put(conn->rx_skb, skb->len),
								skb->len);
		conn->rx_len = len - skb->len;
	} else {
		BT_DBG("Cont: frag len %d (expecting %d)", skb->len, conn->rx_len);

		if (!conn->rx_len) {
			BT_ERR("Unexpected continuation frame (len %d)", skb->len);
			l2cap_conn_unreliable(conn, ECOMM);
			goto drop;
		}

		if (skb->len > conn->rx_len) {
			BT_ERR("Fragment is too long (len %d, expected %d)",
					skb->len, conn->rx_len);
			kfree_skb(conn->rx_skb);
			conn->rx_skb = NULL;
			conn->rx_len = 0;
			l2cap_conn_unreliable(conn, ECOMM);
			goto drop;
		}

		skb_copy_from_linear_data(skb, skb_put(conn->rx_skb, skb->len),
								skb->len);
		conn->rx_len -= skb->len;

		if (!conn->rx_len) {
			/* Complete frame received */
			l2cap_recv_frame(conn, conn->rx_skb);
			conn->rx_skb = NULL;
		}
	}

drop:
	kfree_skb(skb);
	return 0;
}

static int l2cap_debugfs_show(struct seq_file *f, void *p)
{
	struct sock *sk;
	struct hlist_node *node;

	read_lock_bh(&l2cap_sk_list.lock);

	sk_for_each(sk, node, &l2cap_sk_list.head) {
		struct l2cap_pinfo *pi = l2cap_pi(sk);

		seq_printf(f, "%s %s %d %d 0x%4.4x 0x%4.4x %d %d %d\n",
					batostr(&bt_sk(sk)->src),
					batostr(&bt_sk(sk)->dst),
					sk->sk_state, __le16_to_cpu(pi->psm),
					pi->scid, pi->dcid,
					pi->imtu, pi->omtu, pi->sec_level);
	}

	read_unlock_bh(&l2cap_sk_list.lock);

	return 0;
}

static int l2cap_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, l2cap_debugfs_show, inode->i_private);
}

static const struct file_operations l2cap_debugfs_fops = {
	.open		= l2cap_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static struct dentry *l2cap_debugfs;

static const struct proto_ops l2cap_sock_ops = {
	.family		= PF_BLUETOOTH,
	.owner		= THIS_MODULE,
	.release	= l2cap_sock_release,
	.bind		= l2cap_sock_bind,
	.connect	= l2cap_sock_connect,
	.listen		= l2cap_sock_listen,
	.accept		= l2cap_sock_accept,
	.getname	= l2cap_sock_getname,
	.sendmsg	= l2cap_sock_sendmsg,
	.recvmsg	= l2cap_sock_recvmsg,
	.poll		= bt_sock_poll,
	.ioctl		= bt_sock_ioctl,
	.mmap		= sock_no_mmap,
	.socketpair	= sock_no_socketpair,
	.shutdown	= l2cap_sock_shutdown,
	.setsockopt	= l2cap_sock_setsockopt,
	.getsockopt	= l2cap_sock_getsockopt
};

static const struct net_proto_family l2cap_sock_family_ops = {
	.family	= PF_BLUETOOTH,
	.owner	= THIS_MODULE,
	.create	= l2cap_sock_create,
};

static struct hci_proto l2cap_hci_proto = {
	.name		= "L2CAP",
	.id		= HCI_PROTO_L2CAP,
	.connect_ind	= l2cap_connect_ind,
	.connect_cfm	= l2cap_connect_cfm,
	.disconn_ind	= l2cap_disconn_ind,
	.disconn_cfm	= l2cap_disconn_cfm,
	.security_cfm	= l2cap_security_cfm,
	.recv_acldata	= l2cap_recv_acldata,
	.create_cfm		= l2cap_create_cfm,
	.destroy_cfm	= l2cap_destroy_cfm,
};

static int __init l2cap_init(void)
{
	int err;

	err = proto_register(&l2cap_proto, 0);
	if (err < 0)
		return err;

	_l2cap_wq = create_singlethread_workqueue("l2cap");
	if (!_l2cap_wq)
		goto error;

	err = bt_sock_register(BTPROTO_L2CAP, &l2cap_sock_family_ops);
	if (err < 0) {
		BT_ERR("L2CAP socket registration failed");
		goto error;
	}

	err = hci_register_proto(&l2cap_hci_proto);
	if (err < 0) {
		BT_ERR("L2CAP protocol registration failed");
		bt_sock_unregister(BTPROTO_L2CAP);
		goto error;
	}

	if (bt_debugfs) {
		l2cap_debugfs = debugfs_create_file("l2cap", 0444,
					bt_debugfs, NULL, &l2cap_debugfs_fops);
		if (!l2cap_debugfs)
			BT_ERR("Failed to create L2CAP debug file");
	}

	if (amp_init() < 0) {
		BT_ERR("AMP Manager initialization failed");
		goto error;
	}

	BT_INFO("L2CAP ver %s", VERSION);
	BT_INFO("L2CAP socket layer initialized");

	return 0;

error:
	proto_unregister(&l2cap_proto);
	return err;
}

static void __exit l2cap_exit(void)
{
	amp_exit();

	debugfs_remove(l2cap_debugfs);

	flush_workqueue(_l2cap_wq);
	destroy_workqueue(_l2cap_wq);

	if (bt_sock_unregister(BTPROTO_L2CAP) < 0)
		BT_ERR("L2CAP socket unregistration failed");

	if (hci_unregister_proto(&l2cap_hci_proto) < 0)
		BT_ERR("L2CAP protocol unregistration failed");

	proto_unregister(&l2cap_proto);
}

void l2cap_load(void)
{
	/* Dummy function to trigger automatic L2CAP module loading by
	 * other modules that use L2CAP sockets but don't use any other
	 * symbols from it. */
}
EXPORT_SYMBOL(l2cap_load);

module_init(l2cap_init);
module_exit(l2cap_exit);

module_param(disable_ertm, bool, 0644);
MODULE_PARM_DESC(disable_ertm, "Disable enhanced retransmission mode");

MODULE_AUTHOR("Marcel Holtmann <marcel@holtmann.org>");
MODULE_DESCRIPTION("Bluetooth L2CAP ver " VERSION);
MODULE_VERSION(VERSION);
MODULE_LICENSE("GPL");
MODULE_ALIAS("bt-proto-0");
