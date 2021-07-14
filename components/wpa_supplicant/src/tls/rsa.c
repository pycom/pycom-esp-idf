/*
 * RSA
 * Copyright (c) 2006, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#include "utils/includes.h"

#include "utils/common.h"
#include "tls/asn1.h"
#include "tls/bignum.h"
#include "tls/rsa.h"

#ifdef SE050_ENABLED
#include "libtommath.h"
#include <ex_sss.h>
#include <ex_sss_auth.h>
#include <ex_sss_boot.h>
#include <fsl_sss_se05x_apis.h>
#include <nxEnsure.h>
#include <nxLog_App.h>
#include <se05x_APDU_apis.h>
#include <se05x_enums.h>

#define EX_SSS_BOOT_PCONTEXT (&gex_sss_get_timestamp_boot_ctx)
//#define EX_SSS_BOOT_DO_ERASE 1
#define EX_SSS_BOOT_EXPOSE_ARGC_ARGV 0
//#define DIGEST_LENGTH 32

/* Taken from the reference key parameters */
uint32_t se050_obj_id = 0;
uint8_t item[256];
uint32_t p1 = 0;
uint32_t coef = 0;

static ex_sss_boot_ctx_t gex_sss_get_timestamp_boot_ctx;

sss_status_t status;
extern uint8_t signature[512] = {0};
size_t signatureLen=0;
sss_algorithm_t algorithm;
sss_mode_t mode;
size_t keylen=0;
sss_object_t key;
sss_asymmetric_t ctx_asymm = {0};
uint32_t keyId = 0; //RSA Injected Key
#endif

struct crypto_rsa_key {
	int private_key; /* whether private key is set */
	struct bignum *n; /* modulus (p * q) */
	struct bignum *e; /* public exponent */
	/* The following parameters are available only if private_key is set */
	struct bignum *d; /* private exponent */
	struct bignum *p; /* prime p (factor of n) */
	struct bignum *q; /* prime q (factor of n) */
	struct bignum *dmp1; /* d mod (p - 1); CRT exponent */
	struct bignum *dmq1; /* d mod (q - 1); CRT exponent */
	struct bignum *iqmp; /* 1 / q mod p; CRT coefficient */
};

#ifdef SE050_ENABLED
sss_status_t init_sss_open_session()
{
    mode           = kMode_SSS_Sign;
    signatureLen   = sizeof(signature);
	algorithm      = kAlgorithm_SSS_RSASSA_PKCS1_V1_5_SHA256;
    keylen  = 2048;//4096
	const char *portName;

	LOG_I("init_sss_open_session ...");
	if (ctx_asymm.session != NULL)
	{
		LOG_I("init_sss_open_session ... already open");
		return kStatus_SSS_Success;
	}

    memset((EX_SSS_BOOT_PCONTEXT), 0, sizeof(*(EX_SSS_BOOT_PCONTEXT)));

	LOG_I("ex_sss_boot_connectstring ...");
    status = ex_sss_boot_connectstring(0, NULL, &portName);
    if (kStatus_SSS_Success != status) {
        LOG_E("ex_sss_boot_connectstring Failed");
        goto cleanup;
    }
	LOG_I("ex_sss_boot_isHelp ...");
    if (ex_sss_boot_isHelp(portName)) {
        memset(EX_SSS_BOOT_PCONTEXT, 0, sizeof(*EX_SSS_BOOT_PCONTEXT));
        goto before_ex_sss_entry;
    }

    status = ex_sss_boot_open(EX_SSS_BOOT_PCONTEXT, portName);
    if (kStatus_SSS_Success != status) {
        LOG_E("ex_sss_session_open Failed");
        goto cleanup;
    }

    /* Do Not try to delete the pre-provisioned Key/Certificates */
    #ifndef USE_DEFAULT_RSA_KEY
    #ifdef EX_SSS_BOOT_DO_ERASE
        status = ex_sss_boot_factory_reset((EX_SSS_BOOT_PCONTEXT));
    #endif   
    #endif 
	LOG_I("ex_sss_kestore_and_object_init ...");
    status = ex_sss_kestore_and_object_init((EX_SSS_BOOT_PCONTEXT));
    if (kStatus_SSS_Success != status) {
        LOG_E("ex_sss_kestore_and_object_init Failed");
        goto cleanup;
    }
	LOG_I("ex_sss_boot_open_host_session ...");
    ex_sss_boot_open_host_session((EX_SSS_BOOT_PCONTEXT));
            
    //main execution
    before_ex_sss_entry:

    cleanup:
    if (kStatus_SSS_Success == status) {
        LOG_I("init_sss_open_session OK...");
    }
    else {
        LOG_E("init_sss_open_session Failed !!!...");
    }	
	return status;
}

sss_status_t se050_init_key(ex_sss_boot_ctx_t *pCtx)
{
    status = sss_key_object_init(&key, &pCtx->ks);
    ENSURE_OR_GO_CLEANUP(status == kStatus_SSS_Success);

    status = sss_key_object_get_handle(&key, keyId);
    ENSURE_OR_GO_CLEANUP(status == kStatus_SSS_Success);

	cleanup:
    if (kStatus_SSS_Success == status) {
        LOG_I("se050_init_key OK...");
    }
    else {
        LOG_E("se050_init_key Failed !!!...");
    }	
	return status;	
}

void se050_free()
{
	LOG_I("sss_asymmetric_context_free ...");	
	if (ctx_asymm.session != NULL)
	{
		sss_asymmetric_context_free(&ctx_asymm);
		ctx_asymm.session = NULL;
	}
}

void se050_close_session()
{
	LOG_I("ex_sss_session_close ...");	
    ex_sss_session_close((EX_SSS_BOOT_PCONTEXT));
}

sss_status_t se050_rsa_encrypt(ex_sss_boot_ctx_t *pCtx, u8 *inData, size_t inDataLen, u8 *outData, size_t *outDataLen)
{
	LOG_I("[se050_rsa_encrypt] inDataLen=%d outDataLen=%d", inDataLen, *outDataLen);
    status = sss_asymmetric_context_init(&ctx_asymm, &pCtx->session, &key, kAlgorithm_SSS_RSASSA_NO_PADDING, kMode_SSS_Encrypt);
    ENSURE_OR_GO_CLEANUP(status == kStatus_SSS_Success);

	/* with RSA NO Padding the input buffer length is the same as the output */
	*outDataLen = inDataLen;
    status = sss_asymmetric_decrypt(&ctx_asymm, inData, inDataLen, outData, &outDataLen);
    ENSURE_OR_GO_CLEANUP(status == kStatus_SSS_Success);
    LOG_I("Encrypt successful !!!");
	cleanup:
    if (kStatus_SSS_Success == status) {
        LOG_I("se050_rsa_encrypt OK...");
    }
    else {
        LOG_E("se050_rsa_encrypt Failed !!!...");
	}
	return status;
}
 
#endif


static const u8 * crypto_rsa_parse_integer(const u8 *pos, const u8 *end,
					   struct bignum *num)
{
	struct asn1_hdr hdr;

	if (pos == NULL)
		return NULL;

	if (asn1_get_next(pos, end - pos, &hdr) < 0 ||
	    hdr.class != ASN1_CLASS_UNIVERSAL || hdr.tag != ASN1_TAG_INTEGER) {
		wpa_printf(MSG_DEBUG, "RSA: Expected INTEGER - found class %d "
			   "tag 0x%x", hdr.class, hdr.tag);
		return NULL;
	}

	if (bignum_set_unsigned_bin(num, hdr.payload, hdr.length) < 0) {
		wpa_printf(MSG_DEBUG, "RSA: Failed to parse INTEGER");
		return NULL;
	}

	return hdr.payload + hdr.length;
}


/**
 * crypto_rsa_import_public_key - Import an RSA public key
 * @buf: Key buffer (DER encoded RSA public key)
 * @len: Key buffer length in bytes
 * Returns: Pointer to the public key or %NULL on failure
 */
struct crypto_rsa_key *
crypto_rsa_import_public_key(const u8 *buf, size_t len)
{
	struct crypto_rsa_key *key;
	struct asn1_hdr hdr;
	const u8 *pos, *end;

	key = (struct crypto_rsa_key *)os_zalloc(sizeof(*key));
	if (key == NULL)
		return NULL;

	key->n = bignum_init();
	key->e = bignum_init();
	if (key->n == NULL || key->e == NULL) {
		crypto_rsa_free(key);
		return NULL;
	}

	/*
	 * PKCS #1, 7.1:
	 * RSAPublicKey ::= SEQUENCE {
	 *     modulus INTEGER, -- n
	 *     publicExponent INTEGER -- e
	 * }
	 */

	if (asn1_get_next(buf, len, &hdr) < 0 ||
	    hdr.class != ASN1_CLASS_UNIVERSAL ||
	    hdr.tag != ASN1_TAG_SEQUENCE) {
		wpa_printf(MSG_DEBUG, "RSA: Expected SEQUENCE "
			   "(public key) - found class %d tag 0x%x",
			   hdr.class, hdr.tag);
		goto error;
	}
	pos = hdr.payload;
	end = pos + hdr.length;

	pos = crypto_rsa_parse_integer(pos, end, key->n);
	pos = crypto_rsa_parse_integer(pos, end, key->e);

	if (pos == NULL)
		goto error;

	if (pos != end) {
		wpa_hexdump(MSG_DEBUG,
			    "RSA: Extra data in public key SEQUENCE",
			    pos, end - pos);
		goto error;
	}

	return key;

error:
	crypto_rsa_free(key);
	return NULL;
}

#ifdef SE050_DEBUG_ENABLED
int _wpa_snprintf_hex_ex(char *buf, size_t buf_size, const u8 *data, size_t len,
                  int uppercase, int whitespace)
{
    size_t i;
    char *pos = buf, *end = buf + buf_size;
    int ret;

    static const char *fmt_upper = "%02X";
    static const char *fmt_lower = "%02x";
    static const char *fmt_upper_ws = "%02X ";
    static const char *fmt_lower_ws = "%02x ";
    const char *fmt = uppercase ? (whitespace ? fmt_upper_ws : fmt_upper) :
                                  (whitespace ? fmt_lower_ws : fmt_lower);

    if (buf_size == 0)
        return 0;

    for (i = 0; i < len; i++) {
        ret = snprintf(pos, end - pos, fmt, data[i]);
        if (ret < 0 || ret >= end - pos) {
            end[-1] = '\0';
            return pos - buf;
        }
        pos += ret;
    }
    end[-1]='\0';
    return pos - buf;
}
void  wpa_hexdump_ex(int level, const char *title, const u8 *buf, size_t len)
{
	size_t i;
	char output[50];


	printf("%s - hexdump(len=%lu):", title, (unsigned long) len);
	if (buf == NULL) {
		printf(" [NULL]");
	} else {
		for (i = 0; i < len / 16; i++) {
			_wpa_snprintf_hex_ex(output, 50, buf + i * 16, 16, 0, 1);
			printf( "%s", output);
		}
		if (len % 16) {
			int bytes_printed = (len / 16) * 16;
			_wpa_snprintf_hex_ex(output, 50, buf + bytes_printed,
							  len - bytes_printed, 0, 1);
			printf("%s", output);
		}
	} 	
}
#endif

/**
 * crypto_rsa_import_private_key - Import an RSA private key
 * @buf: Key buffer (DER encoded RSA private key)
 * @len: Key buffer length in bytes
 * Returns: Pointer to the private key or %NULL on failure
 */
struct crypto_rsa_key *
crypto_rsa_import_private_key(const u8 *buf, size_t len)
{
	struct crypto_rsa_key *key;
	struct bignum *zero;
	struct asn1_hdr hdr;
	const u8 *pos, *end;

	key = (struct crypto_rsa_key *)os_zalloc(sizeof(*key));
	if (key == NULL)
		return NULL;

	key->private_key = 1;

	key->n = bignum_init();
	key->e = bignum_init();
	key->d = bignum_init();
	key->p = bignum_init();
	key->q = bignum_init();
	key->dmp1 = bignum_init();
	key->dmq1 = bignum_init();
	key->iqmp = bignum_init();

	if (key->n == NULL || key->e == NULL || key->d == NULL ||
	    key->p == NULL || key->q == NULL || key->dmp1 == NULL ||
	    key->dmq1 == NULL || key->iqmp == NULL) {
		crypto_rsa_free(key);
		return NULL;
	}

	/*
	 * PKCS #1, 7.2:
	 * RSAPrivateKey ::= SEQUENCE {
	 *    version Version,
	 *    modulus INTEGER, -- n
	 *    publicExponent INTEGER, -- e
	 *    privateExponent INTEGER, -- d
	 *    prime1 INTEGER, -- p
	 *    prime2 INTEGER, -- q
	 *    exponent1 INTEGER, -- d mod (p-1)
	 *    exponent2 INTEGER, -- d mod (q-1)
	 *    coefficient INTEGER -- (inverse of q) mod p
	 * }
	 *
	 * Version ::= INTEGER -- shall be 0 for this version of the standard
	 */
	if (asn1_get_next(buf, len, &hdr) < 0 ||
	    hdr.class != ASN1_CLASS_UNIVERSAL ||
	    hdr.tag != ASN1_TAG_SEQUENCE) {
		wpa_printf(MSG_DEBUG, "RSA: Expected SEQUENCE "
			   "(public key) - found class %d tag 0x%x",
			   hdr.class, hdr.tag);
		goto error;
	}
	pos = hdr.payload;
	end = pos + hdr.length;

	zero = bignum_init();
	if (zero == NULL)
		goto error;
	pos = crypto_rsa_parse_integer(pos, end, zero);
	if (pos == NULL || bignum_cmp_d(zero, 0) != 0) {
		wpa_printf(MSG_DEBUG, "RSA: Expected zero INTEGER in the "
			   "beginning of private key; not found");
		bignum_deinit(zero);
		goto error;
	}
	bignum_deinit(zero);

	pos = crypto_rsa_parse_integer(pos, end, key->n);
	pos = crypto_rsa_parse_integer(pos, end, key->e);
	pos = crypto_rsa_parse_integer(pos, end, key->d);
	pos = crypto_rsa_parse_integer(pos, end, key->p);
	pos = crypto_rsa_parse_integer(pos, end, key->q);
	pos = crypto_rsa_parse_integer(pos, end, key->dmp1);
	pos = crypto_rsa_parse_integer(pos, end, key->dmq1);
	pos = crypto_rsa_parse_integer(pos, end, key->iqmp);

	#ifdef SE050_ENABLED
		#ifdef SE050_DEBUG_ENABLED
		printf("crypto_rsa_import_private_key+++ \n");
		#endif
		p1 = 0;
		coef = 0;
		se050_obj_id = 0;

		size_t sz_p = bignum_get_unsigned_bin_len(key->p);
		size_t sz_q = bignum_get_unsigned_bin_len(key->q);
		size_t sz_iqmp = bignum_get_unsigned_bin_len(key->iqmp);

		#ifdef SE050_DEBUG_ENABLED
		printf("key->p %d \n", sz_p);
		printf("key->q %d \n", sz_q);
		printf("key->iqmp %d \n", sz_iqmp);
		#endif

		//size_t need = mp_unsigned_bin_size((mp_int *) key->p);
		memset(item, 0x00, sizeof(item));
		if (mp_to_unsigned_bin(key->p, item) != MP_OKAY) {
			printf("BIGNUM: %s failed\n", __func__ );
		}
		#ifdef SE050_DEBUG_ENABLED
			wpa_hexdump_ex(MSG_MSGDUMP, "\n>>>>>>>>p", item, sz_p);
		#endif
		/* get p1 component value */
		if (sz_p<5)
		{
			uint8_t c1 = item[3];
			uint8_t c2 = item[2];
			uint8_t c3 = item[1];
			uint8_t c4 = item[0];

			p1 = (c1 << 24) | (c2 << 16) | (c3 << 8) | (c4);
		}
		#ifdef SE050_DEBUG_ENABLED
		printf("new key->p %d \n", p1);
		#endif




		/* get p2 component value */
		memset(item, 0x00, sizeof(item));
		if (mp_to_unsigned_bin(key->q, item) != MP_OKAY) {
			printf("BIGNUM: %s failed\n", __func__ );
		}
		if (sz_q<5)
		{
			uint8_t c1 = item[3];
			uint8_t c2 = item[2];
			uint8_t c3 = item[1];
			uint8_t c4 = item[0];

			se050_obj_id = (c1 << 24) | (c2 << 16) | (c3 << 8) | (c4);
		}
		#ifdef SE050_DEBUG_ENABLED
		wpa_hexdump_ex(MSG_MSGDUMP, "\n>>>>>>>>>q", item, sz_q);
		printf("new key->q %d \n", se050_obj_id);
		#endif

		
		
		/* get iqmp (coefficient) component value */
		memset(item, 0x00, sizeof(item));
		if (mp_to_unsigned_bin(key->iqmp, item) != MP_OKAY) {
			printf("BIGNUM: %s failed\n", __func__ );
		}
		if (sz_iqmp<5)
		{
			uint8_t c1 = item[3];
			uint8_t c2 = item[2];
			uint8_t c3 = item[1];
			uint8_t c4 = item[0];

			coef = (c1 << 24) | (c2 << 16) | (c3 << 8) | (c4);
		}

		keyId = se050_obj_id;
		#ifdef SE050_DEBUG_ENABLED
		wpa_hexdump_ex(MSG_MSGDUMP, "\n>>>>>>>>>iqmp", item, sz_iqmp);
		printf("new key->iqmp %d %04X\n", coef, coef);
		printf("crypto_rsa_import_private_key--- \n");
		#endif
	#endif

	if (pos == NULL)
		goto error;

	if (pos != end) {
		wpa_hexdump(MSG_DEBUG,
			    "RSA: Extra data in public key SEQUENCE",
			    pos, end - pos);
		goto error;
	}

	return key;

error:
	crypto_rsa_free(key);
	return NULL;
}


/**
 * crypto_rsa_get_modulus_len - Get the modulus length of the RSA key
 * @key: RSA key
 * Returns: Modulus length of the key
 */
size_t crypto_rsa_get_modulus_len(struct crypto_rsa_key *key)
{
	return bignum_get_unsigned_bin_len(key->n);
}


/**
 * crypto_rsa_exptmod - RSA modular exponentiation
 * @in: Input data
 * @inlen: Input data length
 * @out: Buffer for output data
 * @outlen: Maximum size of the output buffer and used size on success
 * @key: RSA key
 * @use_private: 1 = Use RSA private key, 0 = Use RSA public key
 * Returns: 0 on success, -1 on failure
 */
int crypto_rsa_exptmod(const u8 *in, size_t inlen, u8 *out, size_t *outlen,
		       struct crypto_rsa_key *key, int use_private)
{
	struct bignum *tmp, *a = NULL, *b = NULL;
	int ret = -1;
	size_t modlen;

	#ifdef SE050_DEBUG_ENABLED
	wpa_printf(MSG_INFO, "(rsa.c)[crypto_rsa_exptmod] use_private %d", use_private);
	#endif

	if (use_private && !key->private_key)
	{
		#ifdef SE050_DEBUG_ENABLED
		LOG_I("[crypto_rsa_exptmod] Key Error !");
		#endif
		return -1;
	}
#ifdef SE050_ENABLED
	if (se050_obj_id != 0)
	{
		if (kStatus_SSS_Success != status)
		{ 
			status = init_sss_open_session();
			if (kStatus_SSS_Success == status) {
				LOG_I("SE050 Init OK...");
			}
			else {
				LOG_E("SE050 Init Failed !!!...");
			}

			status = se050_init_key((EX_SSS_BOOT_PCONTEXT));
				if (kStatus_SSS_Success == status) {
				LOG_I("SE050 Key Init OK...");
			}
			else {
				LOG_E("SE050 Key Init Failed !!!...");
			}
		} 
		#ifdef SE050_DEBUG_ENABLED
		wpa_printf(MSG_INFO, "(rsa.c)[crypto_rsa_exptmod] use_private %d", use_private);
		LOG_MAU8_I("Original In Data", in, inlen);
		#endif
	}
#endif
	*outlen = inlen;/* input data = output data */

	modlen = crypto_rsa_get_modulus_len(key);

#ifdef SE050_ENABLED
	if (se050_obj_id != 0)
	{
		if (use_private == 1) /* use private key from the SE */
		{
		LOG_I("[crypto_rsa_exptmod] priv key modlen = %d", modlen);

		signatureLen = modlen;
		LOG_I("Using SE050 Certificate...");

		//Encrypt using the Client Private Key stored in the SE
		se050_rsa_encrypt((EX_SSS_BOOT_PCONTEXT), in, inlen, signature, &signatureLen);
		LOG_MAU8_I("Out Encrypt SE", signature, signatureLen);
		}
	}
#endif

	tmp = bignum_init();
	if (tmp == NULL)
		return -1;

	if (bignum_set_unsigned_bin(tmp, in, inlen) < 0)
		goto error;
	if (bignum_cmp(key->n, tmp) < 0) {
		/* Too large input value for the RSA key modulus */
		goto error;
	}

	if (use_private) {
		#ifdef SE050_ENABLED
			if (se050_obj_id == 0)
			{
				LOG_I("Using Flash stored Certificate...");
		#endif
		/*
		 * Decrypt (or sign) using Chinese remainer theorem to speed
		 * up calculation. This is equivalent to tmp = tmp^d mod n
		 * (which would require more CPU to calculate directly).
		 *
		 * dmp1 = (1/e) mod (p-1)
		 * dmq1 = (1/e) mod (q-1)
		 * iqmp = (1/q) mod p, where p > q
		 * m1 = c^dmp1 mod p
		 * m2 = c^dmq1 mod q
		 * h = q^-1 (m1 - m2) mod p
		 * m = m2 + hq
		 */
		a = bignum_init();
		b = bignum_init();
		if (a == NULL || b == NULL)
			goto error;

		/* a = tmp^dmp1 mod p */
		if (bignum_exptmod(tmp, key->dmp1, key->p, a) < 0)
			goto error;

		/* b = tmp^dmq1 mod q */
		if (bignum_exptmod(tmp, key->dmq1, key->q, b) < 0)
			goto error;

		/* tmp = (a - b) * (1/q mod p) (mod p) */
		if (bignum_sub(a, b, tmp) < 0 ||
		    bignum_mulmod(tmp, key->iqmp, key->p, tmp) < 0)
			goto error;

		/* tmp = b + q * tmp */
		if (bignum_mul(tmp, key->q, tmp) < 0 ||
		    bignum_add(tmp, b, tmp) < 0)
			goto error;
		#ifdef SE050_ENABLED
			}
		#endif
	
	} else {
		/* Encrypt (or verify signature) */
		/* tmp = tmp^e mod N */
		if (bignum_exptmod(tmp, key->e, key->n, tmp) < 0)
			goto error;
	}

	modlen = crypto_rsa_get_modulus_len(key);
	if (modlen > *outlen) {
		*outlen = modlen;
		goto error;
	}

	if (bignum_get_unsigned_bin_len(tmp) > modlen)
		goto error; /* should never happen */

	*outlen = modlen;
	os_memset(out, 0, modlen);
	if (bignum_get_unsigned_bin(
		    tmp, out +
		    (modlen - bignum_get_unsigned_bin_len(tmp)), NULL) < 0)
		goto error;	


	#ifdef SE050_ENABLED
		LOG_MAU8_I("SW OUT", out, *outlen);
		if (1 == use_private)
		{
			LOG_I("SW OUT Signature Len= %d", signatureLen);
			os_memcpy(out, signature, signatureLen);
			for (int i=0;i<signatureLen;i++)
			{
				*out++ = signature[i];
			}
		}
		LOG_I("SW OUT Private= %d", use_private);
		LOG_MAU8_I("SW OUT 2", out, *outlen);
	#endif

	ret = 0;

error:
	if (ret != 0)
	{
		wpa_printf(MSG_INFO, "(rsa.c)[crypto_rsa_exptmod] ERROR %d", ret);
	}

	bignum_deinit(tmp);
	bignum_deinit(a);
	bignum_deinit(b);
	return ret;
}


/**
 * crypto_rsa_free - Free RSA key
 * @key: RSA key to be freed
 *
 * This function frees an RSA key imported with either
 * crypto_rsa_import_public_key() or crypto_rsa_import_private_key().
 */
void crypto_rsa_free(struct crypto_rsa_key *key)
{
	if (key) {
		bignum_deinit(key->n);
		bignum_deinit(key->e);
		bignum_deinit(key->d);
		bignum_deinit(key->p);
		bignum_deinit(key->q);
		bignum_deinit(key->dmp1);
		bignum_deinit(key->dmq1);
		bignum_deinit(key->iqmp);
		os_free(key);
	}
}