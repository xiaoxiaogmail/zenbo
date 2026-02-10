/*
 * Customer code to add GPIO control during WLAN start/stop
 *
 * $ Copyright Open Broadcom Corporation $
 *
 *
 * <<Broadcom-WL-IPTag/Open:>>
 *
 * $Id: dhd_custom_gpio.c 581723 2015-08-25 10:34:12Z $
 */

#include <typedefs.h>
#include <linuxver.h>
#include <osl.h>
#include <bcmutils.h>
#include <dngl_stats.h>
#include <dhd.h>
#include <dhd_linux.h>

#include <wlioctl.h>

#define WL_ERROR(x) printf x
#define WL_TRACE(x)

#if defined(CUSTOMER_HW2)

#if defined(PLATFORM_MPS)
int __attribute__ ((weak)) wifi_get_fw_nv_path(char *fw, char *nv) { return 0;};
#endif

#endif 

#if defined(OOB_INTR_ONLY)

#if defined(BCMLXSDMMC)
extern int sdioh_mmc_irq(int irq);
#endif /* (BCMLXSDMMC)  */

#if defined(CUSTOMER_HW3) || defined(PLATFORM_MPS)
#include <mach/gpio.h>
#endif

/* Customer specific Host GPIO defintion  */
static int dhd_oob_gpio_num = -1;

module_param(dhd_oob_gpio_num, int, 0644);
MODULE_PARM_DESC(dhd_oob_gpio_num, "DHD oob gpio number");

/* This function will return:
 *  1) return :  Host gpio interrupt number per customer platform
 *  2) irq_flags_ptr : Type of Host interrupt as Level or Edge
 *
 *  NOTE :
 *  Customer should check his platform definitions
 *  and his Host Interrupt spec
 *  to figure out the proper setting for his platform.
 *  Broadcom provides just reference settings as example.
 *
 */
int dhd_customer_oob_irq_map(void *adapter, unsigned long *irq_flags_ptr)
{
	int  host_oob_irq = 0;

#if defined(CUSTOMER_HW2) && !defined(PLATFORM_MPS)
	host_oob_irq = wifi_platform_get_irq_number(adapter, irq_flags_ptr);

#else
#if defined(CUSTOM_OOB_GPIO_NUM)
	if (dhd_oob_gpio_num < 0) {
		dhd_oob_gpio_num = CUSTOM_OOB_GPIO_NUM;
	}
#endif /* CUSTOMER_OOB_GPIO_NUM */

	if (dhd_oob_gpio_num < 0) {
		WL_ERROR(("%s: ERROR customer specific Host GPIO is NOT defined \n",
		__FUNCTION__));
		return (dhd_oob_gpio_num);
	}

	WL_ERROR(("%s: customer specific Host GPIO number is (%d)\n",
	         __FUNCTION__, dhd_oob_gpio_num));

#if defined CUSTOMER_HW3 || defined(PLATFORM_MPS)
	gpio_request(dhd_oob_gpio_num, "oob irq");
	host_oob_irq = gpio_to_irq(dhd_oob_gpio_num);
	gpio_direction_input(dhd_oob_gpio_num);
#endif /* defined CUSTOMER_HW3 || defined(PLATFORM_MPS) */
#endif 

	return (host_oob_irq);
}
#endif 

/* Customer function to control hw specific wlan gpios */
int
dhd_customer_gpio_wlan_ctrl(void *adapter, int onoff)
{
	int err = 0;

	return err;
}

#ifdef GET_CUSTOM_MAC_ENABLE
/* Function to get custom MAC address */
int
dhd_custom_get_mac_address(void *adapter, unsigned char *buf)
{
	int ret = 0;

	WL_TRACE(("%s Enter\n", __FUNCTION__));
	if (!buf)
		return -EINVAL;

	/* Customer access to MAC address stored outside of DHD driver */
#if (defined(CUSTOMER_HW2) || defined(CUSTOMER_HW10)) && (LINUX_VERSION_CODE >= \
	KERNEL_VERSION(2, 6, 35))
	ret = wifi_platform_get_mac_addr(adapter, buf);
#endif

#ifdef EXAMPLE_GET_MAC
	/* EXAMPLE code */
	{
		struct ether_addr ea_example = {{0x00, 0x11, 0x22, 0x33, 0x44, 0xFF}};
		bcopy((char *)&ea_example, buf, sizeof(struct ether_addr));
	}
#endif /* EXAMPLE_GET_MAC */

	return ret;
}
#endif /* GET_CUSTOM_MAC_ENABLE */

struct cntry_locales_custom {
	char iso_abbrev[WLC_CNTRY_BUF_SZ];	/* ISO 3166-1 country abbreviation */
	char custom_locale[WLC_CNTRY_BUF_SZ];	/* Custom firmware locale */
	int32 custom_locale_rev;		/* Custom local revisin default -1 */
};

/* Customized Locale table : OPTIONAL feature */
const struct cntry_locales_custom translate_custom_table[] = {
/* Table should be filled out based on custom platform regulatory requirement */
	/*1 2 3 4 5 6 7 8 9 10 11
	*Band 1、Band 2、Band 3、Band 4*/
	{"", "XV", 17},  /* Universal if Country code is unknown or empty */
	/* NA
	*1 2 3 4 5 6 7 8 9 10 11
	*Band 1、Band 2、Band 3、Band 4*/
	{"US", "CA", 31},
	{"CA", "CA", 31},
	/* EU
	*1 2 3 4 5 6 7 8 9 10 11 12 13
	*Band 1、Band 2、Band 3*/
	{"GB", "DE", 7},
	{"DE", "DE", 7},
	{"IT", "DE", 7},
	{"FR", "DE", 7},
	{"ES", "DE", 7},
	{"NL", "DE", 7},
	{"PT", "DE", 7},
	{"PL", "DE", 7},
	{"BE", "DE", 7},
	{"LU", "DE", 7},
	{"DK", "DE", 7},
	{"FI", "DE", 7},
	{"SE", "DE", 7},
	{"HU", "DE", 7},
	{"TR", "DE", 7},
	/* APAC1 & BR
	*1 2 3 4 5 6 7 8 9 10 11 12 13
	*Band 1、Band 2、Band 3、Band 4*/
	{"HK", "HK", 2},
	{"TH", "HK", 2},
	{"VN", "HK", 2},
	{"BR", "HK", 2},
	/* APAC2
	*1 2 3 4 5 6 7 8 9 10 11 12 13
	*Band 1、Band 2、Band 4*/
	{"SG", "SG", 4},
	{"MY", "SG", 4},
	{"CN", "SG", 4},
	{"IN", "SG", 4},
	/* TW
	*1 2 3 4 5 6 7 8 9 10 11 
	*Band 2(no 52)、Band 3、Band 4*/
	{"TW", "TW", 1},
	/* JP
	*1 2 3 4 5 6 7 8 9 10 11 12 13
	*Band 1、Band 2、Band 3*/
	{"JP", "JP", 45},
	/* KR
	*1 2 3 4 5 6 7 8 9 10 11 12 13
	*Band 1、Band 2、Band 3(no 128~)、Band 4(no 165)*/
	{"KR", "KR", 57},
	/* RU
	*1 2 3 4 5 6 7 8 9 10 11 12 13
	*Band 1、Band 2、Band 3(132~140)、Band 4*/
	{"RU", "RU", 13},
	/* ID
	*1 2 3 4 5 6 7 8 9 10 11 12 13
	*Band 4(149 ~ 161)*/
	{"ID", "ID", 1},
};


/* Customized Locale convertor
*  input : ISO 3166-1 country abbreviation
*  output: customized cspec
*/
#ifdef CUSTOM_COUNTRY_CODE
void get_customized_country_code(void *adapter, char *country_iso_code,
  wl_country_t *cspec, u32 flags)
#else
void get_customized_country_code(void *adapter, char *country_iso_code, wl_country_t *cspec)
#endif /* CUSTOM_COUNTRY_CODE */
{
#if 0 && defined(CUSTOMER_HW2) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 39))

	struct cntry_locales_custom *cloc_ptr;

	if (!cspec)
		return;
#ifdef CUSTOM_COUNTRY_CODE
	cloc_ptr = wifi_platform_get_country_code(adapter, country_iso_code,
	           flags);
#else
	cloc_ptr = wifi_platform_get_country_code(adapter, country_iso_code);
#endif /* CUSTOM_COUNTRY_CODE */
	if (cloc_ptr) {
		strlcpy(cspec->ccode, cloc_ptr->custom_locale, WLC_CNTRY_BUF_SZ);
		cspec->rev = cloc_ptr->custom_locale_rev;
	}
	return;
#else
	int size, i;

	size = ARRAYSIZE(translate_custom_table);

	if (cspec == 0)
		 return;

	if (size == 0)
		 return;

	for (i = 0; i < size; i++) {
		if (strcmp(country_iso_code, translate_custom_table[i].iso_abbrev) == 0) {
			memcpy(cspec->ccode,
				translate_custom_table[i].custom_locale, WLC_CNTRY_BUF_SZ);
			cspec->rev = translate_custom_table[i].custom_locale_rev;
			return;
		}
	}
//#ifdef EXAMPLE_TABLE
	/* if no country code matched return first universal code from translate_custom_table */
	memcpy(cspec->ccode, translate_custom_table[0].custom_locale, WLC_CNTRY_BUF_SZ);
	cspec->rev = translate_custom_table[0].custom_locale_rev;
//#endif /* EXMAPLE_TABLE */
	return;
#endif /* defined(CUSTOMER_HW2) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)) */
}
