#ifndef __MT76_TEST_H
#define __MT76_TEST_H

#include <stdbool.h>
#include <stdint.h>

#include <linux/nl80211.h>
#include <unl.h>

#include "../testmode.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))
#endif

#ifndef DIV_ROUND_UP
#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))
#endif

struct nl_msg;
struct nlattr;

struct tm_field {
	const char *name;
	const char *prefix;

	bool (*parse)(const struct tm_field *field, int idx, struct nl_msg *msg,
		      const char *val);
	void (*print)(const struct tm_field *field, struct nlattr *attr);

	union {
		struct {
			const char * const *enum_str;
			int enum_len;
		};
		struct {
			bool (*parse2)(const struct tm_field *field, int idx,
				       struct nl_msg *msg, const char *val);
			void (*print2)(const struct tm_field *field,
				       struct nlattr *attr);
		};
		struct {
			const struct tm_field *fields;
			struct nla_policy *policy;
			int len;
		};
	};
};

extern const struct tm_field msg_field;

#endif
