/* Qualcomm Crypto driver
 *
 * Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#include <linux/clk.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/crypto.h>
#include <linux/kernel.h>
#include <linux/rtnetlink.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/debugfs.h>

#include <crypto/ctr.h>
#include <crypto/des.h>
#include <crypto/aes.h>
#include <crypto/sha.h>
#include <crypto/hash.h>
#include <crypto/algapi.h>
#include <crypto/aead.h>
#include <crypto/authenc.h>
#include <crypto/scatterwalk.h>
#include <crypto/internal/hash.h>

#include <mach/scm.h>
#include <mach/board.h>
#include "inc/qce.h"


#define MAX_CRYPTO_DEVICE 3
#define DEBUG_MAX_FNAME  16
#define DEBUG_MAX_RW_BUF 1024

struct crypto_stat {
	u32 aead_sha1_aes_enc;
	u32 aead_sha1_aes_dec;
	u32 aead_sha1_des_enc;
	u32 aead_sha1_des_dec;
	u32 aead_sha1_3des_enc;
	u32 aead_sha1_3des_dec;
	u32 aead_op_success;
	u32 aead_op_fail;
	u32 ablk_cipher_aes_enc;
	u32 ablk_cipher_aes_dec;
	u32 ablk_cipher_des_enc;
	u32 ablk_cipher_des_dec;
	u32 ablk_cipher_3des_enc;
	u32 ablk_cipher_3des_dec;
	u32 ablk_cipher_op_success;
	u32 ablk_cipher_op_fail;
	u32 sha1_digest;
	u32 sha256_digest;
	u32 sha_op_success;
	u32 sha_op_fail;
};
static struct crypto_stat _qcrypto_stat[MAX_CRYPTO_DEVICE];
static struct dentry *_debug_dent;
static char _debug_read_buf[DEBUG_MAX_RW_BUF];

struct crypto_priv {

	struct msm_ce_hw_support ce_hw_support;

	/* the lock protects queue and req*/
	spinlock_t lock;

	/* qce handle */
	void *qce;

	/* list of  registered algorithms */
	struct list_head alg_list;

	/* platform device */
	struct platform_device *pdev;

	/* current active request */
	struct crypto_async_request *req;
	int res;

	/* request queue */
	struct crypto_queue queue;

	uint32_t ce_lock_count;

	struct work_struct unlock_ce_ws;

	struct tasklet_struct done_tasklet;
};


/*-------------------------------------------------------------------------
* Resource Locking Service
* ------------------------------------------------------------------------*/
#define QCRYPTO_CMD_ID				1
#define QCRYPTO_CE_LOCK_CMD			1
#define QCRYPTO_CE_UNLOCK_CMD			0
#define NUM_RETRY				1000
#define CE_BUSY				        55

static int qcrypto_scm_cmd(int resource, int cmd, int *response)
{
#ifdef CONFIG_MSM_SCM

	struct {
		int resource;
		int cmd;
	} cmd_buf;

	cmd_buf.resource = resource;
	cmd_buf.cmd = cmd;

	return scm_call(SCM_SVC_TZ, QCRYPTO_CMD_ID, &cmd_buf,
		sizeof(cmd_buf), response, sizeof(*response));

#else
	return 0;
#endif
}

static void qcrypto_unlock_ce(struct work_struct *work)
{
	int response = 0;
	unsigned long flags;
	struct crypto_priv *cp = container_of(work, struct crypto_priv,
							unlock_ce_ws);
	if (cp->ce_lock_count == 1)
		BUG_ON(qcrypto_scm_cmd(cp->ce_hw_support.shared_ce_resource,
				QCRYPTO_CE_UNLOCK_CMD, &response) != 0);
	spin_lock_irqsave(&cp->lock, flags);
	cp->ce_lock_count--;
	spin_unlock_irqrestore(&cp->lock, flags);
}

static int qcrypto_lock_ce(struct crypto_priv *cp)
{
	unsigned long flags;
	int response = -CE_BUSY;
	int i = 0;

	if (cp->ce_lock_count == 0) {
		do {
			if (qcrypto_scm_cmd(
				cp->ce_hw_support.shared_ce_resource,
				QCRYPTO_CE_LOCK_CMD, &response)) {
				response = -EINVAL;
				break;
			}
		} while ((response == -CE_BUSY) && (i++ < NUM_RETRY));

		if ((response == -CE_BUSY) && (i >= NUM_RETRY))
			return -EUSERS;
		if (response < 0)
			return -EINVAL;
	}
	spin_lock_irqsave(&cp->lock, flags);
	cp->ce_lock_count++;
	spin_unlock_irqrestore(&cp->lock, flags);


	return 0;
}

enum qcrypto_alg_type {
	QCRYPTO_ALG_CIPHER	= 0,
	QCRYPTO_ALG_SHA	= 1,
	QCRYPTO_ALG_LAST
};

struct qcrypto_alg {
	struct list_head entry;
	struct crypto_alg cipher_alg;
	struct ahash_alg sha_alg;
	enum qcrypto_alg_type alg_type;
	struct crypto_priv *cp;
};

#define QCRYPTO_MAX_KEY_SIZE	64
/* max of AES_BLOCK_SIZE, DES3_EDE_BLOCK_SIZE */
#define QCRYPTO_MAX_IV_LENGTH	16

struct qcrypto_cipher_ctx {
	u8 auth_key[QCRYPTO_MAX_KEY_SIZE];
	u8 iv[QCRYPTO_MAX_IV_LENGTH];

	u8 enc_key[QCRYPTO_MAX_KEY_SIZE];
	unsigned int enc_key_len;

	unsigned int authsize;
	unsigned int auth_key_len;

	struct crypto_priv *cp;
};

struct qcrypto_cipher_req_ctx {
	u8 *iv;
	unsigned int ivsize;
	int  aead;
	enum qce_cipher_alg_enum alg;
	enum qce_cipher_dir_enum dir;
	enum qce_cipher_mode_enum mode;
};

#define SHA_MAX_BLOCK_SIZE      SHA256_BLOCK_SIZE
#define SHA_MAX_STATE_SIZE	(SHA256_DIGEST_SIZE / sizeof(u32))
#define SHA_MAX_DIGEST_SIZE	 SHA256_DIGEST_SIZE

static uint8_t  _std_init_vector_sha1_uint8[] =   {
	0x67, 0x45, 0x23, 0x01, 0xEF, 0xCD, 0xAB, 0x89,
	0x98, 0xBA, 0xDC, 0xFE, 0x10, 0x32, 0x54, 0x76,
	0xC3, 0xD2, 0xE1, 0xF0
};

/* standard initialization vector for SHA-256, source: FIPS 180-2 */
static uint8_t _std_init_vector_sha256_uint8[] = {
	0x6A, 0x09, 0xE6, 0x67, 0xBB, 0x67, 0xAE, 0x85,
	0x3C, 0x6E, 0xF3, 0x72, 0xA5, 0x4F, 0xF5, 0x3A,
	0x51, 0x0E, 0x52, 0x7F, 0x9B, 0x05, 0x68, 0x8C,
	0x1F, 0x83, 0xD9, 0xAB, 0x5B, 0xE0, 0xCD, 0x19
};

struct qcrypto_sha_ctx {
	enum qce_hash_alg_enum  alg;
	uint32_t		byte_count[2];
	uint8_t			digest[SHA_MAX_DIGEST_SIZE];
	uint32_t		diglen;
	uint8_t			*tmp_buf;
	uint8_t			trailing_buf[SHA_MAX_BLOCK_SIZE];
	uint32_t		trailing_buf_len;
	uint8_t			first_blk;
	uint8_t			last_blk;
	struct scatterlist *sg;
	struct crypto_priv *cp;
};

struct qcrypto_sha_req_ctx {
	union {
		struct sha1_state sha1_state_ctx;
		struct sha256_state sha256_state_ctx;
	};
	struct scatterlist *src;
	uint32_t nbytes;
};

static void _byte_stream_to_words(uint32_t *iv, unsigned char *b,
		unsigned int len)
{
	unsigned n;

	n = len  / sizeof(uint32_t) ;
	for (; n > 0; n--) {
		*iv =  ((*b << 24)      & 0xff000000) |
				(((*(b+1)) << 16) & 0xff0000)   |
				(((*(b+2)) << 8) & 0xff00)     |
				(*(b+3)          & 0xff);
		b += sizeof(uint32_t);
		iv++;
	}

	n = len %  sizeof(uint32_t);
	if (n == 3) {
		*iv = ((*b << 24) & 0xff000000) |
				(((*(b+1)) << 16) & 0xff0000)   |
				(((*(b+2)) << 8) & 0xff00)     ;
	} else if (n == 2) {
		*iv = ((*b << 24) & 0xff000000) |
				(((*(b+1)) << 16) & 0xff0000)   ;
	} else if (n == 1) {
		*iv = ((*b << 24) & 0xff000000) ;
	}
}

static void _words_to_byte_stream(uint32_t *iv, unsigned char *b,
		unsigned int len)
{
	unsigned n = len  / sizeof(uint32_t);

	for (; n > 0; n--) {
		*b++ = (unsigned char) ((*iv >> 24)   & 0xff);
		*b++ = (unsigned char) ((*iv >> 16)   & 0xff);
		*b++ = (unsigned char) ((*iv >> 8)    & 0xff);
		*b++ = (unsigned char) (*iv           & 0xff);
		iv++;
	}
	n = len % sizeof(uint32_t);
	if (n == 3) {
		*b++ = (unsigned char) ((*iv >> 24)   & 0xff);
		*b++ = (unsigned char) ((*iv >> 16)   & 0xff);
		*b =   (unsigned char) ((*iv >> 8)    & 0xff);
	} else if (n == 2) {
		*b++ = (unsigned char) ((*iv >> 24)   & 0xff);
		*b =   (unsigned char) ((*iv >> 16)   & 0xff);
	} else if (n == 1) {
		*b =   (unsigned char) ((*iv >> 24)   & 0xff);
	}
}

static void _start_qcrypto_process(struct crypto_priv *cp);

static struct qcrypto_alg *_qcrypto_sha_alg_alloc(struct crypto_priv *cp,
		struct ahash_alg *template)
{
	struct qcrypto_alg *q_alg;
	q_alg = kzalloc(sizeof(struct qcrypto_alg), GFP_KERNEL);
	if (!q_alg) {
		pr_err("qcrypto Memory allocation of q_alg FAIL, error %ld\n",
				PTR_ERR(q_alg));
		return ERR_PTR(-ENOMEM);
	}

	q_alg->alg_type = QCRYPTO_ALG_SHA;
	q_alg->sha_alg = *template;
	q_alg->cp = cp;

	return q_alg;
};

static struct qcrypto_alg *_qcrypto_cipher_alg_alloc(struct crypto_priv *cp,
		struct crypto_alg *template)
{
	struct qcrypto_alg *q_alg;

	q_alg = kzalloc(sizeof(struct qcrypto_alg), GFP_KERNEL);
	if (!q_alg) {
		pr_err("qcrypto Memory allocation of q_alg FAIL, error %ld\n",
				PTR_ERR(q_alg));
		return ERR_PTR(-ENOMEM);
	}

	q_alg->alg_type = QCRYPTO_ALG_CIPHER;
	q_alg->cipher_alg = *template;
	q_alg->cp = cp;

	return q_alg;
};

static int _qcrypto_cipher_cra_init(struct crypto_tfm *tfm)
{
	struct crypto_alg *alg = tfm->__crt_alg;
	struct qcrypto_alg *q_alg;
	struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(tfm);

	q_alg = container_of(alg, struct qcrypto_alg, cipher_alg);

	/* update context with ptr to cp */
	ctx->cp = q_alg->cp;

	/* random first IV */
	get_random_bytes(ctx->iv, QCRYPTO_MAX_IV_LENGTH);

	return 0;
};

static int _qcrypto_ahash_cra_init(struct crypto_tfm *tfm)
{
	struct crypto_ahash *ahash = __crypto_ahash_cast(tfm);
	struct qcrypto_sha_ctx *sha_ctx = crypto_tfm_ctx(tfm);
	struct ahash_alg *alg =	container_of(crypto_hash_alg_common(ahash),
						struct ahash_alg, halg);
	struct qcrypto_alg *q_alg = container_of(alg, struct qcrypto_alg,
								sha_alg);

	crypto_ahash_set_reqsize(ahash, sizeof(struct qcrypto_sha_req_ctx));
	/* update context with ptr to cp */
	sha_ctx->cp = q_alg->cp;
	sha_ctx->sg = NULL;
	sha_ctx->tmp_buf = kzalloc(SHA256_BLOCK_SIZE, GFP_KERNEL);
	if (sha_ctx->tmp_buf == NULL) {
		pr_err("qcrypto Can't Allocate mem: sha_ctx->tmp_buf, error %ld\n",
			PTR_ERR(sha_ctx->tmp_buf));
		return -ENOMEM;
	}
	return 0;
};

static void _qcrypto_ahash_cra_exit(struct crypto_tfm *tfm)
{
	struct qcrypto_sha_ctx *sha_ctx = crypto_tfm_ctx(tfm);

	kfree(sha_ctx->tmp_buf);
	if (sha_ctx->sg != NULL)
		kfree(sha_ctx->sg);
};

static int _qcrypto_cra_ablkcipher_init(struct crypto_tfm *tfm)
{
	tfm->crt_ablkcipher.reqsize = sizeof(struct qcrypto_cipher_req_ctx);
	return _qcrypto_cipher_cra_init(tfm);
};

static int _qcrypto_cra_aead_init(struct crypto_tfm *tfm)
{
	tfm->crt_aead.reqsize = sizeof(struct qcrypto_cipher_req_ctx);
	return _qcrypto_cipher_cra_init(tfm);
};

static int _disp_stats(int id)
{
	struct crypto_stat *pstat;
	int len = 0;

	pstat = &_qcrypto_stat[id];
	len = snprintf(_debug_read_buf, DEBUG_MAX_RW_BUF - 1,
			"\nQualcomm crypto accelerator %d Statistics:\n",
				id + 1);

	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   ABLK AES CIPHER encryption   : %d\n",
					pstat->ablk_cipher_aes_enc);
	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   ABLK AES CIPHER decryption   : %d\n",
					pstat->ablk_cipher_aes_dec);

	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   ABLK DES CIPHER encryption   : %d\n",
					pstat->ablk_cipher_des_enc);
	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   ABLK DES CIPHER decryption   : %d\n",
					pstat->ablk_cipher_des_dec);

	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   ABLK 3DES CIPHER encryption  : %d\n",
					pstat->ablk_cipher_3des_enc);

	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   ABLK 3DES CIPHER decryption  : %d\n",
					pstat->ablk_cipher_3des_dec);

	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   ABLK CIPHER operation success: %d\n",
					pstat->ablk_cipher_op_success);
	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   ABLK CIPHER operation fail   : %d\n",
					pstat->ablk_cipher_op_fail);

	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   AEAD SHA1-AES encryption      : %d\n",
					pstat->aead_sha1_aes_enc);
	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   AEAD SHA1-AES decryption      : %d\n",
					pstat->aead_sha1_aes_dec);

	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   AEAD SHA1-DES encryption      : %d\n",
					pstat->aead_sha1_des_enc);
	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   AEAD SHA1-DES decryption      : %d\n",
					pstat->aead_sha1_des_dec);

	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   AEAD SHA1-3DES encryption     : %d\n",
					pstat->aead_sha1_3des_enc);
	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   AEAD SHA1-3DES decryption     : %d\n",
					pstat->aead_sha1_3des_dec);

	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   AEAD operation success       : %d\n",
					pstat->aead_op_success);
	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   AEAD operation fail          : %d\n",
					pstat->aead_op_fail);
	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   SHA1 digest			 : %d\n",
					pstat->sha1_digest);
	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   SHA256 digest		 : %d\n",
					pstat->sha256_digest);
	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   SHA  operation fail          : %d\n",
					pstat->sha_op_fail);
	len += snprintf(_debug_read_buf + len, DEBUG_MAX_RW_BUF - len - 1,
			"   SHA  operation success          : %d\n",
					pstat->sha_op_success);
	return len;
}

static int _qcrypto_remove(struct platform_device *pdev)
{
	struct crypto_priv *cp;
	struct qcrypto_alg *q_alg;
	struct qcrypto_alg *n;

	cp = platform_get_drvdata(pdev);

	if (!cp)
		return 0;

	list_for_each_entry_safe(q_alg, n, &cp->alg_list, entry) {
		if (q_alg->alg_type == QCRYPTO_ALG_CIPHER)
			crypto_unregister_alg(&q_alg->cipher_alg);
		if (q_alg->alg_type == QCRYPTO_ALG_SHA)
			crypto_unregister_ahash(&q_alg->sha_alg);
		list_del(&q_alg->entry);
		kfree(q_alg);
	}

	if (cp->qce)
		qce_close(cp->qce);
	tasklet_kill(&cp->done_tasklet);
	kfree(cp);
	return 0;
};

static int _qcrypto_setkey_aes(struct crypto_ablkcipher *cipher, const u8 *key,
		unsigned int len)
{
	struct crypto_tfm *tfm = crypto_ablkcipher_tfm(cipher);
	struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(tfm);

	switch (len) {
	case AES_KEYSIZE_128:
	case AES_KEYSIZE_192:
	case AES_KEYSIZE_256:
		break;
	default:
		crypto_ablkcipher_set_flags(cipher, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	};
	ctx->enc_key_len = len;
	memcpy(ctx->enc_key, key, len);
	return 0;
};

static int _qcrypto_setkey_des(struct crypto_ablkcipher *cipher, const u8 *key,
		unsigned int len)
{
	struct crypto_tfm *tfm = crypto_ablkcipher_tfm(cipher);
	struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(tfm);
	u32 tmp[DES_EXPKEY_WORDS];
	int ret = des_ekey(tmp, key);

	if (len != DES_KEY_SIZE) {
		crypto_ablkcipher_set_flags(cipher, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	};

	if (unlikely(ret == 0) && (tfm->crt_flags & CRYPTO_TFM_REQ_WEAK_KEY)) {
		tfm->crt_flags |= CRYPTO_TFM_RES_WEAK_KEY;
		return -EINVAL;
	}

	ctx->enc_key_len = len;
	memcpy(ctx->enc_key, key, len);
	return 0;
};

static int _qcrypto_setkey_3des(struct crypto_ablkcipher *cipher, const u8 *key,
		unsigned int len)
{
	struct crypto_tfm *tfm = crypto_ablkcipher_tfm(cipher);
	struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(tfm);

	if (len != DES3_EDE_KEY_SIZE) {
		crypto_ablkcipher_set_flags(cipher, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	};
	ctx->enc_key_len = len;
	memcpy(ctx->enc_key, key, len);
	return 0;
};

static void req_done(unsigned long data)
{
	struct crypto_async_request *areq;
	struct crypto_priv *cp = (struct crypto_priv *)data;
	unsigned long flags;

	spin_lock_irqsave(&cp->lock, flags);
	areq = cp->req;
	cp->req = NULL;
	spin_unlock_irqrestore(&cp->lock, flags);

	if (areq)
		areq->complete(areq, cp->res);
	_start_qcrypto_process(cp);
};

static void _update_sha1_ctx(struct ahash_request  *req)
{
	struct qcrypto_sha_req_ctx *rctx = ahash_request_ctx(req);
	struct sha1_state *sha_state_ctx = &rctx->sha1_state_ctx;
	struct qcrypto_sha_ctx *sha_ctx = crypto_tfm_ctx(req->base.tfm);

	memset(sha_state_ctx->buffer, 0x00, SHA1_BLOCK_SIZE);
	if (sha_ctx->last_blk == 0)
		memcpy(sha_state_ctx->buffer, sha_ctx->trailing_buf,
						sha_ctx->trailing_buf_len);
	else {
		memset(sha_ctx->trailing_buf, 0x00, SHA1_BLOCK_SIZE);
		memset(&sha_ctx->digest[0], 0x00, SHA1_DIGESTSIZE);
		sha_ctx->trailing_buf_len  = 0;
		sha_ctx->first_blk = 0;
		sha_ctx->last_blk = 0;
		sha_ctx->diglen = 0;
		sha_ctx->byte_count[0] = 0;
		sha_ctx->byte_count[1] = 0;
		sha_state_ctx->count = 0;
	}
	_byte_stream_to_words(sha_state_ctx->state , sha_ctx->digest,
							sha_ctx->diglen);

	return;
}

static void _update_sha256_ctx(struct ahash_request  *req)
{
	struct qcrypto_sha_req_ctx *rctx = ahash_request_ctx(req);
	struct sha256_state *sha_state_ctx = &rctx->sha256_state_ctx;
	struct qcrypto_sha_ctx *sha_ctx = crypto_tfm_ctx(req->base.tfm);

	memset(sha_state_ctx->buf, 0x00, SHA256_BLOCK_SIZE);
	if (sha_ctx->last_blk == 0)
		memcpy(sha_state_ctx->buf, sha_ctx->trailing_buf,
						sha_ctx->trailing_buf_len);
	else {
		memset(sha_ctx->trailing_buf, 0x00, SHA256_BLOCK_SIZE);
		memset(&sha_ctx->digest[0], 0x00, SHA256_DIGESTSIZE);
		sha_ctx->trailing_buf_len  = 0;
		sha_ctx->first_blk = 0;
		sha_ctx->last_blk = 0;
		sha_ctx->diglen = 0;
		sha_ctx->byte_count[0] = 0;
		sha_ctx->byte_count[1] = 0;
		sha_state_ctx->count = 0;
	}
	_byte_stream_to_words(sha_state_ctx->state, sha_ctx->digest,
							sha_ctx->diglen);

	return;
}

static void _qce_ahash_complete(void *cookie, unsigned char *digest,
		unsigned char *authdata, int ret)
{
	struct ahash_request *areq = (struct ahash_request *) cookie;
	struct crypto_ahash *ahash = crypto_ahash_reqtfm(areq);
	struct qcrypto_sha_ctx *sha_ctx = crypto_tfm_ctx(areq->base.tfm);
	struct qcrypto_sha_req_ctx *rctx = ahash_request_ctx(areq);
	struct crypto_priv *cp = sha_ctx->cp;
	struct crypto_stat *pstat;
	uint32_t diglen = crypto_ahash_digestsize(ahash);
	uint32_t *auth32 = (uint32_t *)authdata;

	pstat = &_qcrypto_stat[cp->pdev->id];

#ifdef QCRYPTO_DEBUG
	dev_info(&cp->pdev->dev, "_qce_ahash_complete: %p ret %d\n",
				areq, ret);
#endif

	if (digest) {
		memcpy(sha_ctx->digest, digest, diglen);
		memcpy(areq->result, digest, diglen);
	}
	if (authdata) {
		sha_ctx->byte_count[0] = auth32[0];
		sha_ctx->byte_count[1] = auth32[1];
	}
	areq->src = rctx->src;
	areq->nbytes = rctx->nbytes;

	if (sha_ctx->sg != NULL) {
		kfree(sha_ctx->sg);
		sha_ctx->sg = NULL;
	}

	if (sha_ctx->alg == QCE_HASH_SHA1)
		_update_sha1_ctx(areq);
	if (sha_ctx->alg == QCE_HASH_SHA256)
		_update_sha256_ctx(areq);

	sha_ctx->last_blk = 0;
	sha_ctx->first_blk = 0;

	if (ret) {
		cp->res = -ENXIO;
		pstat->sha_op_fail++;
	} else {
		cp->res = 0;
		pstat->sha_op_success++;
	}

	if (cp->ce_hw_support.ce_shared)
		schedule_work(&cp->unlock_ce_ws);
	tasklet_schedule(&cp->done_tasklet);
};

static void _qce_ablk_cipher_complete(void *cookie, unsigned char *icb,
		unsigned char *iv, int ret)
{
	struct ablkcipher_request *areq = (struct ablkcipher_request *) cookie;
	struct crypto_ablkcipher *ablk = crypto_ablkcipher_reqtfm(areq);
	struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(areq->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

#ifdef QCRYPTO_DEBUG
	dev_info(&cp->pdev->dev, "_qce_ablk_cipher_complete: %p ret %d\n",
				areq, ret);
#endif
	if (iv)
		memcpy(ctx->iv, iv, crypto_ablkcipher_ivsize(ablk));

	if (ret) {
		cp->res = -ENXIO;
		pstat->ablk_cipher_op_fail++;
	} else {
		cp->res = 0;
		pstat->ablk_cipher_op_success++;
	}
	if (cp->ce_hw_support.ce_shared)
		schedule_work(&cp->unlock_ce_ws);
	tasklet_schedule(&cp->done_tasklet);
};


static void _qce_aead_complete(void *cookie, unsigned char *icv,
				unsigned char *iv, int ret)
{
	struct aead_request *areq = (struct aead_request *) cookie;
	struct crypto_aead *aead = crypto_aead_reqtfm(areq);
	struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(areq->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct qcrypto_cipher_req_ctx *rctx;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	if (ret == 0) {
		rctx = aead_request_ctx(areq);
		if (rctx->dir  == QCE_ENCRYPT) {
			/* copy the icv to dst */
			scatterwalk_map_and_copy(icv, areq->dst,
					areq->cryptlen,
					ctx->authsize, 1);

		} else {
			unsigned char tmp[SHA256_DIGESTSIZE];

			/* compare icv from src */
			scatterwalk_map_and_copy(tmp,
					areq->src, areq->cryptlen -
					ctx->authsize, ctx->authsize,
					0);
			ret = memcmp(icv, tmp, ctx->authsize);
			if (ret != 0)
				ret = -EBADMSG;

		}
	} else {
		ret = -ENXIO;
	}

	if (iv)
		memcpy(ctx->iv, iv, crypto_aead_ivsize(aead));

	if (ret)
		pstat->aead_op_fail++;
	else
		pstat->aead_op_success++;

	if (cp->ce_hw_support.ce_shared)
		schedule_work(&cp->unlock_ce_ws);
	tasklet_schedule(&cp->done_tasklet);
}

static void _start_qcrypto_process(struct crypto_priv *cp)
{
	struct crypto_async_request *async_req = NULL;
	struct crypto_async_request *backlog = NULL;
	unsigned long flags;
	u32 type;
	struct qce_req qreq;
	int ret;
	struct qcrypto_cipher_req_ctx *rctx;
	struct qcrypto_cipher_ctx *cipher_ctx;
	struct qcrypto_sha_ctx *sha_ctx;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

again:
	spin_lock_irqsave(&cp->lock, flags);
	if (cp->req == NULL) {
		backlog = crypto_get_backlog(&cp->queue);
		async_req = crypto_dequeue_request(&cp->queue);
		cp->req = async_req;
	}
	spin_unlock_irqrestore(&cp->lock, flags);
	if (!async_req)
		return;
	if (backlog)
		backlog->complete(backlog, -EINPROGRESS);
	type = crypto_tfm_alg_type(async_req->tfm);

	if (type == CRYPTO_ALG_TYPE_ABLKCIPHER) {
		struct ablkcipher_request *req;
		struct crypto_ablkcipher *tfm;

		req = container_of(async_req, struct ablkcipher_request, base);
		cipher_ctx = crypto_tfm_ctx(async_req->tfm);
		rctx = ablkcipher_request_ctx(req);
		tfm = crypto_ablkcipher_reqtfm(req);

		qreq.op = QCE_REQ_ABLK_CIPHER;
		qreq.qce_cb = _qce_ablk_cipher_complete;
		qreq.areq = req;
		qreq.alg = rctx->alg;
		qreq.dir = rctx->dir;
		qreq.mode = rctx->mode;
		qreq.enckey = cipher_ctx->enc_key;
		qreq.encklen = cipher_ctx->enc_key_len;
		qreq.iv = req->info;
		qreq.ivsize = crypto_ablkcipher_ivsize(tfm);
		qreq.cryptlen = req->nbytes;
		qreq.use_pmem = 0;

		if ((cipher_ctx->enc_key_len == 0) &&
				(cp->ce_hw_support.hw_key_support == 0))
			ret = -EINVAL;
		else
			ret =  qce_ablk_cipher_req(cp->qce, &qreq);
	} else {
		if (type == CRYPTO_ALG_TYPE_AHASH) {

			struct ahash_request *req;
			struct qce_sha_req sreq;

			req = container_of(async_req,
						struct ahash_request, base);
			sha_ctx = crypto_tfm_ctx(async_req->tfm);

			sreq.qce_cb = _qce_ahash_complete;
			sreq.alg = sha_ctx->alg;
			sreq.digest =  &sha_ctx->digest[0];
			sreq.src = req->src;
			sreq.auth_data[0] = sha_ctx->byte_count[0];
			sreq.auth_data[1] = sha_ctx->byte_count[1];
			sreq.first_blk = sha_ctx->first_blk;
			sreq.last_blk = sha_ctx->last_blk;
			sreq.size = req->nbytes;
			sreq.areq = req;

			ret =  qce_process_sha_req(cp->qce, &sreq);

		} else {
			struct aead_request *req;

			req = container_of(async_req, struct aead_request,
									base);
			rctx = aead_request_ctx(req);
			cipher_ctx = crypto_tfm_ctx(async_req->tfm);

			qreq.op = QCE_REQ_AEAD;
			qreq.qce_cb = _qce_aead_complete;

			qreq.areq = req;
			qreq.alg = rctx->alg;
			qreq.dir = rctx->dir;
			qreq.mode = rctx->mode;
			qreq.iv = rctx->iv;

			qreq.enckey = cipher_ctx->enc_key;
			qreq.encklen = cipher_ctx->enc_key_len;
			qreq.authkey = cipher_ctx->auth_key;
			qreq.authklen = cipher_ctx->auth_key_len;

			ret =  qce_aead_req(cp->qce, &qreq);
		}
	};

	if (ret) {

		spin_lock_irqsave(&cp->lock, flags);
		cp->req = NULL;
		spin_unlock_irqrestore(&cp->lock, flags);

		if (type == CRYPTO_ALG_TYPE_ABLKCIPHER)
			pstat->ablk_cipher_op_fail++;
		else
			if (type == CRYPTO_ALG_TYPE_AHASH)
				pstat->sha_op_fail++;
			else
				pstat->aead_op_fail++;

		async_req->complete(async_req, ret);
		goto again;
	};
};

static int _qcrypto_queue_req(struct crypto_priv *cp,
				struct crypto_async_request *req)
{
	int ret;
	unsigned long flags;

	if (cp->ce_hw_support.ce_shared) {
		ret = qcrypto_lock_ce(cp);
		if (ret)
			return ret;
	}

	spin_lock_irqsave(&cp->lock, flags);
	ret = crypto_enqueue_request(&cp->queue, req);
	spin_unlock_irqrestore(&cp->lock, flags);
	_start_qcrypto_process(cp);

	return ret;
}

static int _qcrypto_enc_aes_ecb(struct ablkcipher_request *req)
{
	struct qcrypto_cipher_req_ctx *rctx;
	struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	BUG_ON(crypto_tfm_alg_type(req->base.tfm) !=
					CRYPTO_ALG_TYPE_ABLKCIPHER);
#ifdef QCRYPTO_DEBUG
	dev_info(&cp->pdev->dev, "_qcrypto_enc_aes_ecb: %p\n", req);
#endif
	rctx = ablkcipher_request_ctx(req);
	rctx->aead = 0;
	rctx->alg = CIPHER_ALG_AES;
	rctx->dir = QCE_ENCRYPT;
	rctx->mode = QCE_MODE_ECB;

	pstat->ablk_cipher_aes_enc++;
	return _qcrypto_queue_req(cp, &req->base);
};

static int _qcrypto_enc_aes_cbc(struct ablkcipher_request *req)
{
	struct qcrypto_cipher_req_ctx *rctx;
	struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	BUG_ON(crypto_tfm_alg_type(req->base.tfm) !=
					CRYPTO_ALG_TYPE_ABLKCIPHER);
#ifdef QCRYPTO_DEBUG
	dev_info(&cp->pdev->dev, "_qcrypto_enc_aes_cbc: %p\n", req);
#endif
	rctx = ablkcipher_request_ctx(req);
	rctx->aead = 0;
	rctx->alg = CIPHER_ALG_AES;
	rctx->dir = QCE_ENCRYPT;
	rctx->mode = QCE_MODE_CBC;

	pstat->ablk_cipher_aes_enc++;
	return _qcrypto_queue_req(cp, &req->base);
};

static int _qcrypto_enc_aes_ctr(struct ablkcipher_request *req)
{
	struct qcrypto_cipher_req_ctx *rctx;
	struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	BUG_ON(crypto_tfm_alg_type(req->base.tfm) !=
				CRYPTO_ALG_TYPE_ABLKCIPHER);
#ifdef QCRYPTO_DEBUG
	dev_info(&cp->pdev->dev, "_qcrypto_enc_aes_ctr: %p\n", req);
#endif
	rctx = ablkcipher_request_ctx(req);
	rctx->aead = 0;
	rctx->alg = CIPHER_ALG_AES;
	rctx->dir = QCE_ENCRYPT;
	rctx->mode = QCE_MODE_CTR;

	pstat->ablk_cipher_aes_enc++;
	return _qcrypto_queue_req(cp, &req->base);
};

static int _qcrypto_enc_des_ecb(struct ablkcipher_request *req)
{
	struct qcrypto_cipher_req_ctx *rctx;
	struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	BUG_ON(crypto_tfm_alg_type(req->base.tfm) !=
					CRYPTO_ALG_TYPE_ABLKCIPHER);
	rctx = ablkcipher_request_ctx(req);
	rctx->aead = 0;
	rctx->alg = CIPHER_ALG_DES;
	rctx->dir = QCE_ENCRYPT;
	rctx->mode = QCE_MODE_ECB;

	pstat->ablk_cipher_des_enc++;
	return _qcrypto_queue_req(cp, &req->base);
};

static int _qcrypto_enc_des_cbc(struct ablkcipher_request *req)
{
	struct qcrypto_cipher_req_ctx *rctx;
	struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	BUG_ON(crypto_tfm_alg_type(req->base.tfm) !=
					CRYPTO_ALG_TYPE_ABLKCIPHER);
	rctx = ablkcipher_request_ctx(req);
	rctx->aead = 0;
	rctx->alg = CIPHER_ALG_DES;
	rctx->dir = QCE_ENCRYPT;
	rctx->mode = QCE_MODE_CBC;

	pstat->ablk_cipher_des_enc++;
	return _qcrypto_queue_req(cp, &req->base);
};

static int _qcrypto_enc_3des_ecb(struct ablkcipher_request *req)
{
	struct qcrypto_cipher_req_ctx *rctx;
	struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	BUG_ON(crypto_tfm_alg_type(req->base.tfm) !=
					CRYPTO_ALG_TYPE_ABLKCIPHER);
	rctx = ablkcipher_request_ctx(req);
	rctx->aead = 0;
	rctx->alg = CIPHER_ALG_3DES;
	rctx->dir = QCE_ENCRYPT;
	rctx->mode = QCE_MODE_ECB;

	pstat->ablk_cipher_3des_enc++;
	return _qcrypto_queue_req(cp, &req->base);
};

static int _qcrypto_enc_3des_cbc(struct ablkcipher_request *req)
{
	struct qcrypto_cipher_req_ctx *rctx;
	struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	BUG_ON(crypto_tfm_alg_type(req->base.tfm) !=
					CRYPTO_ALG_TYPE_ABLKCIPHER);
	rctx = ablkcipher_request_ctx(req);
	rctx->aead = 0;
	rctx->alg = CIPHER_ALG_3DES;
	rctx->dir = QCE_ENCRYPT;
	rctx->mode = QCE_MODE_CBC;

	pstat->ablk_cipher_3des_enc++;
	return _qcrypto_queue_req(cp, &req->base);
};

static int _qcrypto_dec_aes_ecb(struct ablkcipher_request *req)
{
	struct qcrypto_cipher_req_ctx *rctx;
	struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	BUG_ON(crypto_tfm_alg_type(req->base.tfm) !=
				CRYPTO_ALG_TYPE_ABLKCIPHER);
#ifdef QCRYPTO_DEBUG
	dev_info(&cp->pdev->dev, "_qcrypto_dec_aes_ecb: %p\n", req);
#endif
	rctx = ablkcipher_request_ctx(req);
	rctx->aead = 0;
	rctx->alg = CIPHER_ALG_AES;
	rctx->dir = QCE_DECRYPT;
	rctx->mode = QCE_MODE_ECB;

	pstat->ablk_cipher_aes_dec++;
	return _qcrypto_queue_req(cp, &req->base);
};

static int _qcrypto_dec_aes_cbc(struct ablkcipher_request *req)
{
	struct qcrypto_cipher_req_ctx *rctx;
	struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	BUG_ON(crypto_tfm_alg_type(req->base.tfm) !=
				CRYPTO_ALG_TYPE_ABLKCIPHER);
#ifdef QCRYPTO_DEBUG
	dev_info(&cp->pdev->dev, "_qcrypto_dec_aes_cbc: %p\n", req);
#endif

	rctx = ablkcipher_request_ctx(req);
	rctx->aead = 0;
	rctx->alg = CIPHER_ALG_AES;
	rctx->dir = QCE_DECRYPT;
	rctx->mode = QCE_MODE_CBC;

	pstat->ablk_cipher_aes_dec++;
	return _qcrypto_queue_req(cp, &req->base);
};

static int _qcrypto_dec_aes_ctr(struct ablkcipher_request *req)
{
	struct qcrypto_cipher_req_ctx *rctx;
	struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	BUG_ON(crypto_tfm_alg_type(req->base.tfm) !=
					CRYPTO_ALG_TYPE_ABLKCIPHER);
#ifdef QCRYPTO_DEBUG
	dev_info(&cp->pdev->dev, "_qcrypto_dec_aes_ctr: %p\n", req);
#endif
	rctx = ablkcipher_request_ctx(req);
	rctx->aead = 0;
	rctx->alg = CIPHER_ALG_AES;
	rctx->mode = QCE_MODE_CTR;

	/* Note. There is no such thing as aes/counter mode, decrypt */
	rctx->dir = QCE_ENCRYPT;

	pstat->ablk_cipher_aes_dec++;
	return _qcrypto_queue_req(cp, &req->base);
};

static int _qcrypto_dec_des_ecb(struct ablkcipher_request *req)
{
	struct qcrypto_cipher_req_ctx *rctx;
	struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	BUG_ON(crypto_tfm_alg_type(req->base.tfm) !=
					CRYPTO_ALG_TYPE_ABLKCIPHER);
	rctx = ablkcipher_request_ctx(req);
	rctx->aead = 0;
	rctx->alg = CIPHER_ALG_DES;
	rctx->dir = QCE_DECRYPT;
	rctx->mode = QCE_MODE_ECB;

	pstat->ablk_cipher_des_dec++;
	return _qcrypto_queue_req(cp, &req->base);
};

static int _qcrypto_dec_des_cbc(struct ablkcipher_request *req)
{
	struct qcrypto_cipher_req_ctx *rctx;
	struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	BUG_ON(crypto_tfm_alg_type(req->base.tfm) !=
					CRYPTO_ALG_TYPE_ABLKCIPHER);
	rctx = ablkcipher_request_ctx(req);
	rctx->aead = 0;
	rctx->alg = CIPHER_ALG_DES;
	rctx->dir = QCE_DECRYPT;
	rctx->mode = QCE_MODE_CBC;

	pstat->ablk_cipher_des_dec++;
	return _qcrypto_queue_req(cp, &req->base);
};

static int _qcrypto_dec_3des_ecb(struct ablkcipher_request *req)
{
	struct qcrypto_cipher_req_ctx *rctx;
	struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	BUG_ON(crypto_tfm_alg_type(req->base.tfm) !=
					CRYPTO_ALG_TYPE_ABLKCIPHER);
	rctx = ablkcipher_request_ctx(req);
	rctx->aead = 0;
	rctx->alg = CIPHER_ALG_3DES;
	rctx->dir = QCE_DECRYPT;
	rctx->mode = QCE_MODE_ECB;

	pstat->ablk_cipher_3des_dec++;
	return _qcrypto_queue_req(cp, &req->base);
};

static int _qcrypto_dec_3des_cbc(struct ablkcipher_request *req)
{
	struct qcrypto_cipher_req_ctx *rctx;
	struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	BUG_ON(crypto_tfm_alg_type(req->base.tfm) !=
					CRYPTO_ALG_TYPE_ABLKCIPHER);
	rctx = ablkcipher_request_ctx(req);
	rctx->aead = 0;
	rctx->alg = CIPHER_ALG_3DES;
	rctx->dir = QCE_DECRYPT;
	rctx->mode = QCE_MODE_CBC;

	pstat->ablk_cipher_3des_dec++;
	return _qcrypto_queue_req(cp, &req->base);
};


static int _qcrypto_aead_setauthsize(struct crypto_aead *authenc,
				unsigned int authsize)
{
	struct qcrypto_cipher_ctx *ctx = crypto_aead_ctx(authenc);

	ctx->authsize = authsize;
	return 0;
}

static int _qcrypto_aead_setkey(struct crypto_aead *tfm, const u8 *key,
			unsigned int keylen)
{
	struct qcrypto_cipher_ctx *ctx = crypto_aead_ctx(tfm);
	struct rtattr *rta = (struct rtattr *)key;
	struct crypto_authenc_key_param *param;

	if (!RTA_OK(rta, keylen))
		goto badkey;
	if (rta->rta_type != CRYPTO_AUTHENC_KEYA_PARAM)
		goto badkey;
	if (RTA_PAYLOAD(rta) < sizeof(*param))
		goto badkey;

	param = RTA_DATA(rta);
	ctx->enc_key_len = be32_to_cpu(param->enckeylen);

	key += RTA_ALIGN(rta->rta_len);
	keylen -= RTA_ALIGN(rta->rta_len);

	if (keylen < ctx->enc_key_len)
		goto badkey;

	ctx->auth_key_len = keylen - ctx->enc_key_len;
	if (ctx->enc_key_len >= QCRYPTO_MAX_KEY_SIZE ||
				ctx->auth_key_len >= QCRYPTO_MAX_KEY_SIZE)
		goto badkey;
	memset(ctx->auth_key, 0, QCRYPTO_MAX_KEY_SIZE);
	memcpy(ctx->enc_key, key + ctx->auth_key_len, ctx->enc_key_len);
	memcpy(ctx->auth_key, key, ctx->auth_key_len);

	return 0;
badkey:
	ctx->enc_key_len = 0;
	crypto_aead_set_flags(tfm, CRYPTO_TFM_RES_BAD_KEY_LEN);
	return -EINVAL;
}

static int _qcrypto_aead_encrypt_aes_cbc(struct aead_request *req)
{
	struct qcrypto_cipher_req_ctx *rctx;
	struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

#ifdef QCRYPTO_DEBUG
	dev_info(&cp->pdev->dev, "_qcrypto_aead_encrypt_aes_cbc: %p\n", req);
#endif

	rctx = aead_request_ctx(req);
	rctx->aead = 1;
	rctx->alg = CIPHER_ALG_AES;
	rctx->dir = QCE_ENCRYPT;
	rctx->mode = QCE_MODE_CBC;
	rctx->iv = req->iv;

	pstat->aead_sha1_aes_enc++;
	return _qcrypto_queue_req(cp, &req->base);
}

static int _qcrypto_aead_decrypt_aes_cbc(struct aead_request *req)
{
	struct qcrypto_cipher_req_ctx *rctx;
	struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

#ifdef QCRYPTO_DEBUG
	dev_info(&cp->pdev->dev, "_qcrypto_aead_decrypt_aes_cbc: %p\n", req);
#endif
	rctx = aead_request_ctx(req);
	rctx->aead = 1;
	rctx->alg = CIPHER_ALG_AES;
	rctx->dir = QCE_DECRYPT;
	rctx->mode = QCE_MODE_CBC;
	rctx->iv = req->iv;

	pstat->aead_sha1_aes_dec++;
	return _qcrypto_queue_req(cp, &req->base);
}

static int _qcrypto_aead_givencrypt_aes_cbc(struct aead_givcrypt_request *req)
{
	struct aead_request *areq = &req->areq;
	struct crypto_aead *authenc = crypto_aead_reqtfm(areq);
	struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(areq->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct qcrypto_cipher_req_ctx *rctx;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	rctx = aead_request_ctx(areq);
	rctx->aead = 1;
	rctx->alg = CIPHER_ALG_AES;
	rctx->dir = QCE_ENCRYPT;
	rctx->mode = QCE_MODE_CBC;
	rctx->iv = req->giv;	/* generated iv */

	memcpy(req->giv, ctx->iv, crypto_aead_ivsize(authenc));
	 /* avoid consecutive packets going out with same IV */
	*(__be64 *)req->giv ^= cpu_to_be64(req->seq);
	pstat->aead_sha1_aes_enc++;
	return _qcrypto_queue_req(cp, &areq->base);
}

#ifdef QCRYPTO_AEAD_AES_CTR
static int _qcrypto_aead_encrypt_aes_ctr(struct aead_request *req)
{
	struct qcrypto_cipher_req_ctx *rctx;
	struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	rctx = aead_request_ctx(req);
	rctx->aead = 1;
	rctx->alg = CIPHER_ALG_AES;
	rctx->dir = QCE_ENCRYPT;
	rctx->mode = QCE_MODE_CTR;
	rctx->iv = req->iv;

	pstat->aead_sha1_aes_enc++;
	return _qcrypto_queue_req(cp, &req->base);
}

static int _qcrypto_aead_decrypt_aes_ctr(struct aead_request *req)
{
	struct qcrypto_cipher_req_ctx *rctx;
	struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	rctx = aead_request_ctx(req);
	rctx->aead = 1;
	rctx->alg = CIPHER_ALG_AES;

	/* Note. There is no such thing as aes/counter mode, decrypt */
	rctx->dir = QCE_ENCRYPT;

	rctx->mode = QCE_MODE_CTR;
	rctx->iv = req->iv;

	pstat->aead_sha1_aes_dec++;
	return _qcrypto_queue_req(cp, &req->base);
}

static int _qcrypto_aead_givencrypt_aes_ctr(struct aead_givcrypt_request *req)
{
	struct aead_request *areq = &req->areq;
	struct crypto_aead *authenc = crypto_aead_reqtfm(areq);
	struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(areq->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct qcrypto_cipher_req_ctx *rctx;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	rctx = aead_request_ctx(areq);
	rctx->aead = 1;
	rctx->alg = CIPHER_ALG_AES;
	rctx->dir = QCE_ENCRYPT;
	rctx->mode = QCE_MODE_CTR;
	rctx->iv = req->giv;	/* generated iv */

	memcpy(req->giv, ctx->iv, crypto_aead_ivsize(authenc));
	 /* avoid consecutive packets going out with same IV */
	*(__be64 *)req->giv ^= cpu_to_be64(req->seq);
	pstat->aead_sha1_aes_enc++;
	return _qcrypto_queue_req(cp, &areq->base);
};
#endif /* QCRYPTO_AEAD_AES_CTR */

static int _qcrypto_aead_encrypt_des_cbc(struct aead_request *req)
{
	struct qcrypto_cipher_req_ctx *rctx;
	struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	rctx = aead_request_ctx(req);
	rctx->aead = 1;
	rctx->alg = CIPHER_ALG_DES;
	rctx->dir = QCE_ENCRYPT;
	rctx->mode = QCE_MODE_CBC;
	rctx->iv = req->iv;

	pstat->aead_sha1_des_enc++;
	return _qcrypto_queue_req(cp, &req->base);
}

static int _qcrypto_aead_decrypt_des_cbc(struct aead_request *req)
{
	struct qcrypto_cipher_req_ctx *rctx;
	struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	rctx = aead_request_ctx(req);
	rctx->aead = 1;
	rctx->alg = CIPHER_ALG_DES;
	rctx->dir = QCE_DECRYPT;
	rctx->mode = QCE_MODE_CBC;
	rctx->iv = req->iv;

	pstat->aead_sha1_des_dec++;
	return _qcrypto_queue_req(cp, &req->base);
}

static int _qcrypto_aead_givencrypt_des_cbc(struct aead_givcrypt_request *req)
{
	struct aead_request *areq = &req->areq;
	struct crypto_aead *authenc = crypto_aead_reqtfm(areq);
	struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(areq->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct qcrypto_cipher_req_ctx *rctx;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	rctx = aead_request_ctx(areq);
	rctx->aead = 1;
	rctx->alg = CIPHER_ALG_DES;
	rctx->dir = QCE_ENCRYPT;
	rctx->mode = QCE_MODE_CBC;
	rctx->iv = req->giv;	/* generated iv */

	memcpy(req->giv, ctx->iv, crypto_aead_ivsize(authenc));
	 /* avoid consecutive packets going out with same IV */
	*(__be64 *)req->giv ^= cpu_to_be64(req->seq);
	pstat->aead_sha1_des_enc++;
	return _qcrypto_queue_req(cp, &areq->base);
}

static int _qcrypto_aead_encrypt_3des_cbc(struct aead_request *req)
{
	struct qcrypto_cipher_req_ctx *rctx;
	struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	rctx = aead_request_ctx(req);
	rctx->aead = 1;
	rctx->alg = CIPHER_ALG_3DES;
	rctx->dir = QCE_ENCRYPT;
	rctx->mode = QCE_MODE_CBC;
	rctx->iv = req->iv;

	pstat->aead_sha1_3des_enc++;
	return _qcrypto_queue_req(cp, &req->base);
}

static int _qcrypto_aead_decrypt_3des_cbc(struct aead_request *req)
{
	struct qcrypto_cipher_req_ctx *rctx;
	struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	rctx = aead_request_ctx(req);
	rctx->aead = 1;
	rctx->alg = CIPHER_ALG_3DES;
	rctx->dir = QCE_DECRYPT;
	rctx->mode = QCE_MODE_CBC;
	rctx->iv = req->iv;

	pstat->aead_sha1_3des_dec++;
	return _qcrypto_queue_req(cp, &req->base);
}

static int _qcrypto_aead_givencrypt_3des_cbc(struct aead_givcrypt_request *req)
{
	struct aead_request *areq = &req->areq;
	struct crypto_aead *authenc = crypto_aead_reqtfm(areq);
	struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(areq->base.tfm);
	struct crypto_priv *cp = ctx->cp;
	struct qcrypto_cipher_req_ctx *rctx;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	rctx = aead_request_ctx(areq);
	rctx->aead = 1;
	rctx->alg = CIPHER_ALG_3DES;
	rctx->dir = QCE_ENCRYPT;
	rctx->mode = QCE_MODE_CBC;
	rctx->iv = req->giv;	/* generated iv */

	memcpy(req->giv, ctx->iv, crypto_aead_ivsize(authenc));
	 /* avoid consecutive packets going out with same IV */
	*(__be64 *)req->giv ^= cpu_to_be64(req->seq);
	pstat->aead_sha1_3des_enc++;
	return _qcrypto_queue_req(cp, &areq->base);
}

static int qcrypto_count_sg(struct scatterlist *sg, int nbytes)
{
	int i;

	for (i = 0; nbytes > 0; i++, sg = sg_next(sg))
		nbytes -= sg->length;

	return i;
}

static int _sha_init(struct ahash_request *req, struct qcrypto_sha_ctx *ctx)
{
	ctx->first_blk = 1;
	ctx->last_blk = 0;
	ctx->diglen = 0;
	ctx->byte_count[0] = 0;
	ctx->byte_count[1] = 0;
	ctx->trailing_buf_len = 0;

	return 0;
};

static int _sha1_init(struct ahash_request *req)
{
	struct qcrypto_sha_ctx *sha_ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = sha_ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	_sha_init(req, sha_ctx);
	sha_ctx->alg = QCE_HASH_SHA1;

	memset(&sha_ctx->trailing_buf[0], 0x00, SHA1_BLOCK_SIZE);
	memcpy(&sha_ctx->digest[0], &_std_init_vector_sha1_uint8[0],
						SHA1_DIGEST_SIZE);
	sha_ctx->diglen = SHA1_DIGEST_SIZE;
	_update_sha1_ctx(req);

	memset(sha_ctx->tmp_buf, 0x00, SHA1_BLOCK_SIZE);
	pstat->sha1_digest++;
	return 0;
};

static int _sha256_init(struct ahash_request *req)
{
	struct qcrypto_sha_ctx *sha_ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = sha_ctx->cp;
	struct crypto_stat *pstat;

	pstat = &_qcrypto_stat[cp->pdev->id];

	_sha_init(req, sha_ctx);
	sha_ctx->alg = QCE_HASH_SHA256;

	memset(&sha_ctx->trailing_buf[0], 0x00, SHA256_BLOCK_SIZE);
	memcpy(&sha_ctx->digest[0], &_std_init_vector_sha256_uint8[0],
						SHA256_DIGEST_SIZE);
	sha_ctx->diglen = SHA256_DIGEST_SIZE;
	_update_sha256_ctx(req);

	memset(sha_ctx->tmp_buf, 0x00, SHA256_BLOCK_SIZE);
	pstat->sha256_digest++;
	return 0;
};


static int _sha1_export(struct ahash_request  *req, void *out)
{
	struct qcrypto_sha_req_ctx *rctx = ahash_request_ctx(req);
	struct sha1_state *sha_state_ctx = &rctx->sha1_state_ctx;
	struct sha1_state *out_ctx = (struct sha1_state *)out;

	out_ctx->count = sha_state_ctx->count;
	memcpy(out_ctx->state, sha_state_ctx->state, sizeof(out_ctx->state));
	memcpy(out_ctx->buffer, sha_state_ctx->buffer, SHA1_BLOCK_SIZE);

	return 0;
};

static int _sha1_import(struct ahash_request  *req, const void *in)
{
	struct qcrypto_sha_ctx *sha_ctx = crypto_tfm_ctx(req->base.tfm);
	struct qcrypto_sha_req_ctx *rctx = ahash_request_ctx(req);
	struct sha1_state *sha_state_ctx = &rctx->sha1_state_ctx;
	struct sha1_state *in_ctx = (struct sha1_state *)in;

	sha_state_ctx->count = in_ctx->count;
	memcpy(sha_state_ctx->state, in_ctx->state, sizeof(in_ctx->state));
	memcpy(sha_state_ctx->buffer, in_ctx->buffer, SHA1_BLOCK_SIZE);
	memcpy(sha_ctx->trailing_buf, in_ctx->buffer, SHA1_BLOCK_SIZE);

	sha_ctx->byte_count[0] =  (uint32_t)(in_ctx->count & 0xFFFFFFC0);
	sha_ctx->byte_count[1] =  (uint32_t)(in_ctx->count >> 32);
	_words_to_byte_stream(in_ctx->state, sha_ctx->digest, sha_ctx->diglen);

	sha_ctx->trailing_buf_len = (uint32_t)(in_ctx->count &
						(SHA1_BLOCK_SIZE-1));

	if (!(in_ctx->count))
		sha_ctx->first_blk = 1;
	else
		sha_ctx->first_blk = 0;

	return 0;
}
static int _sha256_export(struct ahash_request  *req, void *out)
{
	struct qcrypto_sha_req_ctx *rctx = ahash_request_ctx(req);
	struct sha256_state *sha_state_ctx = &rctx->sha256_state_ctx;
	struct sha256_state *out_ctx = (struct sha256_state *)out;

	out_ctx->count = sha_state_ctx->count;
	memcpy(out_ctx->state, sha_state_ctx->state, sizeof(out_ctx->state));
	memcpy(out_ctx->buf, sha_state_ctx->buf, SHA256_BLOCK_SIZE);

	return 0;
};

static int _sha256_import(struct ahash_request  *req, const void *in)
{
	struct qcrypto_sha_ctx *sha_ctx = crypto_tfm_ctx(req->base.tfm);
	struct qcrypto_sha_req_ctx *rctx = ahash_request_ctx(req);
	struct sha256_state *sha_state_ctx = &rctx->sha256_state_ctx;
	struct sha256_state *in_ctx = (struct sha256_state *)in;

	sha_state_ctx->count = in_ctx->count;
	memcpy(sha_state_ctx->state, in_ctx->state, sizeof(in_ctx->state));
	memcpy(sha_state_ctx->buf, in_ctx->buf, SHA256_BLOCK_SIZE);
	memcpy(sha_ctx->trailing_buf, in_ctx->buf, SHA256_BLOCK_SIZE);

	sha_ctx->byte_count[0] =  (uint32_t)(in_ctx->count & 0xFFFFFFC0);
	sha_ctx->byte_count[1] =  (uint32_t)(in_ctx->count >> 32);
	_words_to_byte_stream(in_ctx->state, sha_ctx->digest, sha_ctx->diglen);

	sha_ctx->trailing_buf_len = (uint32_t)(in_ctx->count &
						(SHA256_BLOCK_SIZE-1));

	if (!(in_ctx->count))
		sha_ctx->first_blk = 1;
	else
		sha_ctx->first_blk = 0;

	return 0;
}


static int _sha_update(struct ahash_request  *req, uint32_t sha_block_size)
{
	struct qcrypto_sha_ctx *sha_ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = sha_ctx->cp;
	struct qcrypto_sha_req_ctx *rctx = ahash_request_ctx(req);
	uint32_t total, len, i, num_sg;
	uint8_t *k_src = NULL;
	uint32_t sha_pad_len = 0;
	uint32_t end_src = 0;
	uint32_t trailing_buf_len = 0;
	uint32_t nbytes;
	int ret = 0;

	/* check for trailing buffer from previous updates and append it */
	total = req->nbytes + sha_ctx->trailing_buf_len;
	len = req->nbytes;

	if (total <= sha_block_size) {
		i = 0;

		k_src = &sha_ctx->trailing_buf[sha_ctx->trailing_buf_len];
		while (len > 0) {
			memcpy(k_src, sg_virt(&req->src[i]),
							req->src[i].length);
			len -= req->src[i].length;
			k_src += req->src[i].length;
			i++;
		}
		sha_ctx->trailing_buf_len = total;
		if (sha_ctx->alg == QCE_HASH_SHA1)
			_update_sha1_ctx(req);
		if (sha_ctx->alg == QCE_HASH_SHA256)
			_update_sha256_ctx(req);
		return 0;
	}

	/* save the original req structure fields*/
	rctx->src = req->src;
	rctx->nbytes = req->nbytes;

	memcpy(sha_ctx->tmp_buf, sha_ctx->trailing_buf,
					sha_ctx->trailing_buf_len);
	k_src = &sha_ctx->trailing_buf[0];
	/*  get new trailing buffer */
	sha_pad_len = ALIGN(total, sha_block_size) - total;
	trailing_buf_len =  sha_block_size - sha_pad_len;
	nbytes = total - trailing_buf_len;
	num_sg = qcrypto_count_sg(req->src, req->nbytes);

	len = sha_ctx->trailing_buf_len;
	i = 0;

	while (len < nbytes) {
		if ((len + req->src[i].length) > nbytes)
			break;
		len += req->src[i].length;
		i++;
	}

	end_src = i;
	if (len < nbytes) {
		uint32_t remnant = (nbytes - len);
		memcpy(k_src, (sg_virt(&req->src[i]) + remnant),
				(req->src[i].length - remnant));
		k_src += (req->src[i].length - remnant);
		req->src[i].length = remnant;
		i++;
	}

	while (i < num_sg) {
		memcpy(k_src, sg_virt(&req->src[i]), req->src[i].length);
		k_src += req->src[i].length;
		i++;
	}

	if (sha_ctx->trailing_buf_len) {
		num_sg = end_src + 2;
		sha_ctx->sg = kzalloc(num_sg * (sizeof(struct scatterlist)),
								GFP_KERNEL);
		if (sha_ctx->sg == NULL) {
			pr_err("qcrypto Can't Allocate mem: sha_ctx->sg, error %ld\n",
				PTR_ERR(sha_ctx->sg));
			return -ENOMEM;
		}

		sg_set_buf(&sha_ctx->sg[0],  sha_ctx->tmp_buf,
						sha_ctx->trailing_buf_len);
		for (i = 1; i < num_sg; i++)
			sg_set_buf(&sha_ctx->sg[i], sg_virt(&req->src[i-1]),
							req->src[i-1].length);

		req->src = sha_ctx->sg;
		sg_mark_end(&sha_ctx->sg[num_sg - 1]);
	} else
		sg_mark_end(&req->src[end_src]);

	req->nbytes = nbytes;
	sha_ctx->trailing_buf_len = trailing_buf_len;

	ret =  _qcrypto_queue_req(cp, &req->base);

	return ret;
};

static int _sha1_update(struct ahash_request  *req)
{
	struct qcrypto_sha_req_ctx *rctx = ahash_request_ctx(req);
	struct sha1_state *sha_state_ctx = &rctx->sha1_state_ctx;

	sha_state_ctx->count += req->nbytes;
	return _sha_update(req, SHA1_BLOCK_SIZE);
}

static int _sha256_update(struct ahash_request  *req)
{
	struct qcrypto_sha_req_ctx *rctx = ahash_request_ctx(req);
	struct sha256_state *sha_state_ctx = &rctx->sha256_state_ctx;

	sha_state_ctx->count += req->nbytes;
	return _sha_update(req, SHA256_BLOCK_SIZE);
}

static int _sha_final(struct ahash_request *req, uint32_t sha_block_size)
{
	struct qcrypto_sha_ctx *sha_ctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_priv *cp = sha_ctx->cp;
	struct qcrypto_sha_req_ctx *rctx = ahash_request_ctx(req);
	int ret = 0;

	sha_ctx->last_blk = 1;

	/* save the original req structure fields*/
	rctx->src = req->src;
	rctx->nbytes = req->nbytes;

	memcpy(sha_ctx->tmp_buf, &sha_ctx->trailing_buf[0],
						sha_ctx->trailing_buf_len);

	sha_ctx->sg = kzalloc(sizeof(struct scatterlist), GFP_KERNEL);
	if (sha_ctx->sg == NULL) {
		pr_err("qcrypto Can't Allocate mem: sha_ctx->sg, error %ld\n",
			PTR_ERR(sha_ctx->sg));
		return -ENOMEM;
	}
	sg_set_buf(sha_ctx->sg, sha_ctx->tmp_buf, sha_ctx->trailing_buf_len);
	sg_mark_end(sha_ctx->sg);

	req->src = sha_ctx->sg;
	req->nbytes = sha_ctx->trailing_buf_len;

	ret =  _qcrypto_queue_req(cp, &req->base);

	return ret;
};

static int _sha1_final(struct ahash_request  *req)
{
	return _sha_final(req, SHA1_BLOCK_SIZE);
}

static int _sha256_final(struct ahash_request  *req)
{
	return _sha_final(req, SHA256_BLOCK_SIZE);
}

static int _sha_digest(struct ahash_request *req)
{
	struct qcrypto_sha_ctx *sha_ctx = crypto_tfm_ctx(req->base.tfm);
	struct qcrypto_sha_req_ctx *rctx = ahash_request_ctx(req);
	struct crypto_priv *cp = sha_ctx->cp;
	int ret = 0;

	/* save the original req structure fields*/
	rctx->src = req->src;
	rctx->nbytes = req->nbytes;

	sha_ctx->last_blk = 1;
	ret =  _qcrypto_queue_req(cp, &req->base);

	return ret;
}

static int _sha1_digest(struct ahash_request *req)
{
	_sha1_init(req);
	return _sha_digest(req);
}

static int _sha256_digest(struct ahash_request *req)
{
	_sha256_init(req);
	return _sha_digest(req);
}

static struct ahash_alg _qcrypto_sha_algos[] = {
	{
		.init		=	_sha1_init,
		.update		=	_sha1_update,
		.final		=	_sha1_final,
		.export		=	_sha1_export,
		.import		=	_sha1_import,
		.digest		=	_sha1_digest,
		.halg		= {
			.digestsize	= SHA1_DIGEST_SIZE,
			.statesize	= sizeof(struct sha1_state),
			.base	= {
				.cra_name	 = "sha1",
				.cra_driver_name = "qcrypto-sha1",
				.cra_priority	 = 300,
				.cra_flags	 = CRYPTO_ALG_TYPE_AHASH |
							 CRYPTO_ALG_ASYNC,
				.cra_blocksize	 = SHA1_BLOCK_SIZE,
				.cra_ctxsize	 =
						sizeof(struct qcrypto_sha_ctx),
				.cra_alignmask	 = 0,
				.cra_type	 = &crypto_ahash_type,
				.cra_module	 = THIS_MODULE,
				.cra_init	 = _qcrypto_ahash_cra_init,
				.cra_exit	 = _qcrypto_ahash_cra_exit,
			},
		},
	},
	{
		.init		=	_sha256_init,
		.update		=	_sha256_update,
		.final		=	_sha256_final,
		.export		=	_sha256_export,
		.import		=	_sha256_import,
		.digest		=	_sha256_digest,
		.halg		= {
			.digestsize	= SHA256_DIGEST_SIZE,
			.statesize	= sizeof(struct sha256_state),
			.base		= {
				.cra_name	 = "sha256",
				.cra_driver_name = "qcrypto-sha256",
				.cra_priority	 = 300,
				.cra_flags	 = CRYPTO_ALG_TYPE_AHASH |
							CRYPTO_ALG_ASYNC,
				.cra_blocksize	 = SHA256_BLOCK_SIZE,
				.cra_ctxsize	 =
						sizeof(struct qcrypto_sha_ctx),
				.cra_alignmask	 = 0,
				.cra_type	 = &crypto_ahash_type,
				.cra_module	 = THIS_MODULE,
				.cra_init	 = _qcrypto_ahash_cra_init,
				.cra_exit	 = _qcrypto_ahash_cra_exit,
			},
		},
	},
};

static struct crypto_alg _qcrypto_algos[] = {
	{
		.cra_name		= "ecb(aes)",
		.cra_driver_name	= "qcrypto-ecb-aes",
		.cra_priority	= 300,
		.cra_flags	= CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize	= AES_BLOCK_SIZE,
		.cra_ctxsize	= sizeof(struct qcrypto_cipher_ctx),
		.cra_alignmask	= 0,
		.cra_type	= &crypto_ablkcipher_type,
		.cra_module	= THIS_MODULE,
		.cra_init	= _qcrypto_cra_ablkcipher_init,
		.cra_u		= {
			.ablkcipher = {
				.min_keysize	= AES_MIN_KEY_SIZE,
				.max_keysize	= AES_MAX_KEY_SIZE,
				.setkey		= _qcrypto_setkey_aes,
				.encrypt	= _qcrypto_enc_aes_ecb,
				.decrypt	= _qcrypto_dec_aes_ecb,
			},
		},
	},
	{
		.cra_name	= "cbc(aes)",
		.cra_driver_name = "qcrypto-cbc-aes",
		.cra_priority	= 300,
		.cra_flags	= CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize	= AES_BLOCK_SIZE,
		.cra_ctxsize	= sizeof(struct qcrypto_cipher_ctx),
		.cra_alignmask	= 0,
		.cra_type	= &crypto_ablkcipher_type,
		.cra_module	= THIS_MODULE,
		.cra_init	= _qcrypto_cra_ablkcipher_init,
		.cra_u		= {
			.ablkcipher = {
				.ivsize		= AES_BLOCK_SIZE,
				.min_keysize	= AES_MIN_KEY_SIZE,
				.max_keysize	= AES_MAX_KEY_SIZE,
				.setkey		= _qcrypto_setkey_aes,
				.encrypt	= _qcrypto_enc_aes_cbc,
				.decrypt	= _qcrypto_dec_aes_cbc,
			},
		},
	},
	{
		.cra_name	= "ctr(aes)",
		.cra_driver_name = "qcrypto-ctr-aes",
		.cra_priority	= 300,
		.cra_flags	= CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize	= AES_BLOCK_SIZE,
		.cra_ctxsize	= sizeof(struct qcrypto_cipher_ctx),
		.cra_alignmask	= 0,
		.cra_type	= &crypto_ablkcipher_type,
		.cra_module	= THIS_MODULE,
		.cra_init	= _qcrypto_cra_ablkcipher_init,
		.cra_u		= {
			.ablkcipher = {
				.ivsize		= AES_BLOCK_SIZE,
				.min_keysize	= AES_MIN_KEY_SIZE,
				.max_keysize	= AES_MAX_KEY_SIZE,
				.setkey		= _qcrypto_setkey_aes,
				.encrypt	= _qcrypto_enc_aes_ctr,
				.decrypt	= _qcrypto_dec_aes_ctr,
			},
		},
	},
	{
		.cra_name		= "ecb(des)",
		.cra_driver_name	= "qcrypto-ecb-des",
		.cra_priority	= 300,
		.cra_flags	= CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize	= DES_BLOCK_SIZE,
		.cra_ctxsize	= sizeof(struct qcrypto_cipher_ctx),
		.cra_alignmask	= 0,
		.cra_type	= &crypto_ablkcipher_type,
		.cra_module	= THIS_MODULE,
		.cra_init	= _qcrypto_cra_ablkcipher_init,
		.cra_u		= {
			.ablkcipher = {
				.min_keysize	= DES_KEY_SIZE,
				.max_keysize	= DES_KEY_SIZE,
				.setkey		= _qcrypto_setkey_des,
				.encrypt	= _qcrypto_enc_des_ecb,
				.decrypt	= _qcrypto_dec_des_ecb,
			},
		},
	},
	{
		.cra_name	= "cbc(des)",
		.cra_driver_name = "qcrypto-cbc-des",
		.cra_priority	= 300,
		.cra_flags	= CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize	= DES_BLOCK_SIZE,
		.cra_ctxsize	= sizeof(struct qcrypto_cipher_ctx),
		.cra_alignmask	= 0,
		.cra_type	= &crypto_ablkcipher_type,
		.cra_module	= THIS_MODULE,
		.cra_init	= _qcrypto_cra_ablkcipher_init,
		.cra_u		= {
			.ablkcipher = {
				.ivsize		= DES_BLOCK_SIZE,
				.min_keysize	= DES_KEY_SIZE,
				.max_keysize	= DES_KEY_SIZE,
				.setkey		= _qcrypto_setkey_des,
				.encrypt	= _qcrypto_enc_des_cbc,
				.decrypt	= _qcrypto_dec_des_cbc,
			},
		},
	},
	{
		.cra_name		= "ecb(des3_ede)",
		.cra_driver_name	= "qcrypto-ecb-3des",
		.cra_priority	= 300,
		.cra_flags	= CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize	= DES3_EDE_BLOCK_SIZE,
		.cra_ctxsize	= sizeof(struct qcrypto_cipher_ctx),
		.cra_alignmask	= 0,
		.cra_type	= &crypto_ablkcipher_type,
		.cra_module	= THIS_MODULE,
		.cra_init	= _qcrypto_cra_ablkcipher_init,
		.cra_u		= {
			.ablkcipher = {
				.min_keysize	= DES3_EDE_KEY_SIZE,
				.max_keysize	= DES3_EDE_KEY_SIZE,
				.setkey		= _qcrypto_setkey_3des,
				.encrypt	= _qcrypto_enc_3des_ecb,
				.decrypt	= _qcrypto_dec_3des_ecb,
			},
		},
	},
	{
		.cra_name	= "cbc(des3_ede)",
		.cra_driver_name = "qcrypto-cbc-3des",
		.cra_priority	= 300,
		.cra_flags	= CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize	= DES3_EDE_BLOCK_SIZE,
		.cra_ctxsize	= sizeof(struct qcrypto_cipher_ctx),
		.cra_alignmask	= 0,
		.cra_type	= &crypto_ablkcipher_type,
		.cra_module	= THIS_MODULE,
		.cra_init	= _qcrypto_cra_ablkcipher_init,
		.cra_u		= {
			.ablkcipher = {
				.ivsize		= DES3_EDE_BLOCK_SIZE,
				.min_keysize	= DES3_EDE_KEY_SIZE,
				.max_keysize	= DES3_EDE_KEY_SIZE,
				.setkey		= _qcrypto_setkey_3des,
				.encrypt	= _qcrypto_enc_3des_cbc,
				.decrypt	= _qcrypto_dec_3des_cbc,
			},
		},
	},
	{
		.cra_name	= "authenc(hmac(sha1),cbc(aes))",
		.cra_driver_name = "qcrypto-aead-hmac-sha1-cbc-aes",
		.cra_priority	= 300,
		.cra_flags	= CRYPTO_ALG_TYPE_AEAD | CRYPTO_ALG_ASYNC,
		.cra_blocksize  = AES_BLOCK_SIZE,
		.cra_ctxsize	= sizeof(struct qcrypto_cipher_ctx),
		.cra_alignmask	= 0,
		.cra_type	= &crypto_aead_type,
		.cra_module	= THIS_MODULE,
		.cra_init	= _qcrypto_cra_aead_init,
		.cra_u		= {
			.aead = {
				.ivsize         = AES_BLOCK_SIZE,
				.maxauthsize    = SHA1_DIGEST_SIZE,
				.setkey = _qcrypto_aead_setkey,
				.setauthsize = _qcrypto_aead_setauthsize,
				.encrypt = _qcrypto_aead_encrypt_aes_cbc,
				.decrypt = _qcrypto_aead_decrypt_aes_cbc,
				.givencrypt = _qcrypto_aead_givencrypt_aes_cbc,
				.geniv = "<built-in>",
			}
		}
	},

#ifdef QCRYPTO_AEAD_AES_CTR
	{
		.cra_name	= "authenc(hmac(sha1),ctr(aes))",
		.cra_driver_name = "qcrypto-aead-hmac-sha1-ctr-aes",
		.cra_priority	= 300,
		.cra_flags	= CRYPTO_ALG_TYPE_AEAD | CRYPTO_ALG_ASYNC,
		.cra_blocksize  = AES_BLOCK_SIZE,
		.cra_ctxsize	= sizeof(struct qcrypto_cipher_ctx),
		.cra_alignmask	= 0,
		.cra_type	= &crypto_aead_type,
		.cra_module	= THIS_MODULE,
		.cra_init	= _qcrypto_cra_aead_init,
		.cra_u		= {
			.aead = {
				.ivsize         = AES_BLOCK_SIZE,
				.maxauthsize    = SHA1_DIGEST_SIZE,
				.setkey = _qcrypto_aead_setkey,
				.setauthsize = _qcrypto_aead_setauthsize,
				.encrypt = _qcrypto_aead_encrypt_aes_ctr,
				.decrypt = _qcrypto_aead_decrypt_aes_ctr,
				.givencrypt = _qcrypto_aead_givencrypt_aes_ctr,
				.geniv = "<built-in>",
			}
		}
	},
#endif /* QCRYPTO_AEAD_AES_CTR */
	{
		.cra_name	= "authenc(hmac(sha1),cbc(des))",
		.cra_driver_name = "qcrypto-aead-hmac-sha1-cbc-des",
		.cra_priority	= 300,
		.cra_flags	= CRYPTO_ALG_TYPE_AEAD | CRYPTO_ALG_ASYNC,
		.cra_blocksize  = DES_BLOCK_SIZE,
		.cra_ctxsize	= sizeof(struct qcrypto_cipher_ctx),
		.cra_alignmask	= 0,
		.cra_type	= &crypto_aead_type,
		.cra_module	= THIS_MODULE,
		.cra_init	= _qcrypto_cra_aead_init,
		.cra_u		= {
			.aead = {
				.ivsize         = DES_BLOCK_SIZE,
				.maxauthsize    = SHA1_DIGEST_SIZE,
				.setkey = _qcrypto_aead_setkey,
				.setauthsize = _qcrypto_aead_setauthsize,
				.encrypt = _qcrypto_aead_encrypt_des_cbc,
				.decrypt = _qcrypto_aead_decrypt_des_cbc,
				.givencrypt = _qcrypto_aead_givencrypt_des_cbc,
				.geniv = "<built-in>",
			}
		}
	},
	{
		.cra_name	= "authenc(hmac(sha1),cbc(des3_ede))",
		.cra_driver_name = "qcrypto-aead-hmac-sha1-cbc-3des",
		.cra_priority	= 300,
		.cra_flags	= CRYPTO_ALG_TYPE_AEAD | CRYPTO_ALG_ASYNC,
		.cra_blocksize  = DES3_EDE_BLOCK_SIZE,
		.cra_ctxsize	= sizeof(struct qcrypto_cipher_ctx),
		.cra_alignmask	= 0,
		.cra_type	= &crypto_aead_type,
		.cra_module	= THIS_MODULE,
		.cra_init	= _qcrypto_cra_aead_init,
		.cra_u		= {
			.aead = {
				.ivsize         = DES3_EDE_BLOCK_SIZE,
				.maxauthsize    = SHA1_DIGEST_SIZE,
				.setkey = _qcrypto_aead_setkey,
				.setauthsize = _qcrypto_aead_setauthsize,
				.encrypt = _qcrypto_aead_encrypt_3des_cbc,
				.decrypt = _qcrypto_aead_decrypt_3des_cbc,
				.givencrypt = _qcrypto_aead_givencrypt_3des_cbc,
				.geniv = "<built-in>",
			}
		}
	},
};

static int  _qcrypto_probe(struct platform_device *pdev)
{
	int rc = 0;
	void *handle;
	struct crypto_priv *cp;
	int i;
	struct msm_ce_hw_support *ce_hw_support;

	if (pdev->id >= MAX_CRYPTO_DEVICE) {
		printk(KERN_ERR "%s: device id %d  exceeds allowed %d\n",
				__func__, pdev->id, MAX_CRYPTO_DEVICE);
		return -ENOENT;
	}

	cp = kzalloc(sizeof(*cp), GFP_KERNEL);
	if (!cp) {
		pr_err("qcrypto Memory allocation of q_alg FAIL, error %ld\n",
				PTR_ERR(cp));
		return -ENOMEM;
	}

	/* open qce */
	handle = qce_open(pdev, &rc);
	if (handle == NULL) {
		kfree(cp);
		platform_set_drvdata(pdev, NULL);
		return rc;
	}

	INIT_LIST_HEAD(&cp->alg_list);
	platform_set_drvdata(pdev, cp);
	spin_lock_init(&cp->lock);
	tasklet_init(&cp->done_tasklet, req_done, (unsigned long)cp);
	crypto_init_queue(&cp->queue, 50);
	cp->qce = handle;
	cp->pdev = pdev;

	ce_hw_support = (struct msm_ce_hw_support *)pdev->dev.platform_data;
	cp->ce_hw_support.ce_shared = ce_hw_support->ce_shared;
	cp->ce_hw_support.shared_ce_resource =
				ce_hw_support->shared_ce_resource;
	cp->ce_hw_support.hw_key_support =
				ce_hw_support->hw_key_support;
	cp->ce_lock_count = 0;
	if (cp->ce_hw_support.ce_shared)
		INIT_WORK(&cp->unlock_ce_ws, qcrypto_unlock_ce);
	/* register crypto cipher algorithms the device supports */
	for (i = 0; i < ARRAY_SIZE(_qcrypto_algos); i++) {
		struct qcrypto_alg *q_alg;

		q_alg = _qcrypto_cipher_alg_alloc(cp, &_qcrypto_algos[i]);
		if (IS_ERR(q_alg)) {
			rc = PTR_ERR(q_alg);
			goto err;
		}

		if (((q_alg->cipher_alg.cra_flags & CRYPTO_ALG_TYPE_MASK) ==
				CRYPTO_ALG_TYPE_AEAD) &&
					qce_hmac_support(handle) <= 0) {
			kfree(q_alg);
			continue;
		}

		rc = crypto_register_alg(&q_alg->cipher_alg);
		if (rc) {
			dev_err(&pdev->dev, "%s alg registration failed\n",
					q_alg->cipher_alg.cra_driver_name);
			kfree(q_alg);
		} else {
			list_add_tail(&q_alg->entry, &cp->alg_list);
			dev_info(&pdev->dev, "%s\n",
					q_alg->cipher_alg.cra_driver_name);
		}
	}

	/* register crypto hash algorithms the device supports */
	for (i = 0; i < ARRAY_SIZE(_qcrypto_sha_algos); i++) {
		struct qcrypto_alg *q_alg = NULL;

		q_alg = _qcrypto_sha_alg_alloc(cp, &_qcrypto_sha_algos[i]);

		if (IS_ERR(q_alg)) {
			rc = PTR_ERR(q_alg);
			goto err;
		}

		rc = crypto_register_ahash(&q_alg->sha_alg);
		if (rc) {
			dev_err(&pdev->dev, "%s alg registration failed\n",
				q_alg->sha_alg.halg.base.cra_driver_name);
			kfree(q_alg);
		} else {
			list_add_tail(&q_alg->entry, &cp->alg_list);
			dev_info(&pdev->dev, "%s\n",
				q_alg->sha_alg.halg.base.cra_driver_name);
		}
	}

	return 0;
err:
	_qcrypto_remove(pdev);
	return rc;
};

static struct platform_driver _qualcomm_crypto = {
	.probe          = _qcrypto_probe,
	.remove         = _qcrypto_remove,
	.driver         = {
		.owner  = THIS_MODULE,
		.name   = "qcrypto",
	},
};

static int _debug_qcrypto[MAX_CRYPTO_DEVICE];

static int _debug_stats_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t _debug_stats_read(struct file *file, char __user *buf,
			size_t count, loff_t *ppos)
{
	int rc = -EINVAL;
	int qcrypto = *((int *) file->private_data);
	int len;

	len = _disp_stats(qcrypto);

	rc = simple_read_from_buffer((void __user *) buf, len,
			ppos, (void *) _debug_read_buf, len);

	return rc;
}

static ssize_t _debug_stats_write(struct file *file, const char __user *buf,
			size_t count, loff_t *ppos)
{

	int qcrypto = *((int *) file->private_data);

	memset((char *)&_qcrypto_stat[qcrypto], 0, sizeof(struct crypto_stat));
	return count;
};

static const struct file_operations _debug_stats_ops = {
	.open =         _debug_stats_open,
	.read =         _debug_stats_read,
	.write =        _debug_stats_write,
};

static int _qcrypto_debug_init(void)
{
	int rc;
	char name[DEBUG_MAX_FNAME];
	int i;
	struct dentry *dent;

	_debug_dent = debugfs_create_dir("qcrypto", NULL);
	if (IS_ERR(_debug_dent)) {
		pr_err("qcrypto debugfs_create_dir fail, error %ld\n",
				PTR_ERR(_debug_dent));
		return PTR_ERR(_debug_dent);
	}

	for (i = 0; i < MAX_CRYPTO_DEVICE; i++) {
		snprintf(name, DEBUG_MAX_FNAME-1, "stats-%d", i+1);
		_debug_qcrypto[i] = i;
		dent = debugfs_create_file(name, 0644, _debug_dent,
				&_debug_qcrypto[i], &_debug_stats_ops);
		if (dent == NULL) {
			pr_err("qcrypto debugfs_create_file fail, error %ld\n",
					PTR_ERR(dent));
			rc = PTR_ERR(dent);
			goto err;
		}
	}
	return 0;
err:
	debugfs_remove_recursive(_debug_dent);
	return rc;
}

static int __init _qcrypto_init(void)
{
	int rc;

	rc = _qcrypto_debug_init();
	if (rc)
		return rc;

	return platform_driver_register(&_qualcomm_crypto);
}

static void __exit _qcrypto_exit(void)
{
	printk(KERN_ALERT "%s Unregister QCRYPTO\n", __func__);
	debugfs_remove_recursive(_debug_dent);
	platform_driver_unregister(&_qualcomm_crypto);
}

module_init(_qcrypto_init);
module_exit(_qcrypto_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Mona Hossain <mhossain@codeaurora.org>");
MODULE_DESCRIPTION("Qualcomm Crypto driver");
MODULE_VERSION("1.07");
