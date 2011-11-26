/*
 * hostapd / WMM (Wi-Fi Multimedia)
 * Copyright 2002-2003, Instant802 Networks, Inc.
 * Copyright 2005-2006, Devicescape Software, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Alternatively, this software may be distributed under the terms of BSD
 * license.
 *
 * See README and COPYING for more details.
 */

#ifndef WME_H
#define WME_H

#ifdef __linux__
#include <endian.h>
#endif /* __linux__ */

#if defined(__FreeBSD__) || defined(__NetBSD__) || defined(__DragonFly__)
#include <sys/types.h>
#include <sys/endian.h>
#endif /* defined(__FreeBSD__) || defined(__NetBSD__) ||
	* defined(__DragonFly__) */


extern inline u16 tsinfo(int tag1d, int contention_based, int direction)
{
	return (tag1d << 11) | (contention_based << 7) | (direction << 5) |
	  (tag1d << 1);
}


struct wme_information_element {
	/* required fields for WME version 1 */
	u8 oui[3];
	u8 oui_type;
	u8 oui_subtype;
	u8 version;
	u8 acInfo;

} __attribute__ ((packed));

struct wme_ac_parameter {
#if __BYTE_ORDER == __LITTLE_ENDIAN
	/* byte 1 */
	u8 	aifsn:4,
		acm:1,
	 	aci:2,
	 	reserved:1;

	/* byte 2 */
	u8 	eCWmin:4,
	 	eCWmax:4;
#elif __BYTE_ORDER == __BIG_ENDIAN
	/* byte 1 */
	u8 	reserved:1,
	 	aci:2,
	 	acm:1,
	 	aifsn:4;

	/* byte 2 */
	u8 	eCWmax:4,
	 	eCWmin:4;
#else
#error	"Please fix <endian.h>"
#endif

	/* bytes 3 & 4 */
	le16 txopLimit;
} __attribute__ ((packed));

struct wme_parameter_element {
	/* required fields for WME version 1 */
	u8 oui[3];
	u8 oui_type;
	u8 oui_subtype;
	u8 version;
	u8 acInfo;
	u8 reserved;
	struct wme_ac_parameter ac[4];

} __attribute__ ((packed));

struct wme_tspec_info_element {
	u8 eid; /* 221 = 0xdd */
	u8 length; /* 6 + 55 = 61 */
	u8 oui[3]; /* 00:50:f2 */
	u8 oui_type; /* 2 */
	u8 oui_subtype; /* 2 */
	u8 version; /* 1 */
	/* WMM TSPEC body (55 octets): */
	u8 ts_info[3];
	le16 nominal_msdu_size;
	le16 maximum_msdu_size;
	le32 minimum_service_interval;
	le32 maximum_service_interval;
	le32 inactivity_interval;
	le32 suspension_interval;
	le32 service_start_time;
	le32 minimum_data_rate;
	le32 mean_data_rate;
	le32 peak_data_rate;
	le32 maximum_burst_size;
	le32 delay_bound;
	le32 minimum_phy_rate;
	le16 surplus_bandwidth_allowance;
	le16 medium_time;
} __attribute__ ((packed));


/* Access Categories */
enum {
	WME_AC_BK = 1,
	WME_AC_BE = 0,
	WME_AC_VI = 2,
	WME_AC_VO = 3
};

struct ieee80211_mgmt;

u8 * hostapd_eid_wme(struct hostapd_data *hapd, u8 *eid);
int hostapd_eid_wme_valid(struct hostapd_data *hapd, u8 *eid, size_t len);
int hostapd_wme_sta_config(struct hostapd_data *hapd, struct sta_info *sta);
void hostapd_wme_action(struct hostapd_data *hapd, struct ieee80211_mgmt *mgmt,
			size_t len);

#endif /* WME_H */
