#ifndef _LOG_H_
#define _LOG_H_

#define LOG_VERBORSE		0
#define LOG_DEBUG			1
#define LOG_INFO			2
#define	LOG_NOTICE		3
#define LOG_WARNING		4
#define LOG_ERROR			5
#define LOG_CRIT			6

#define LOG_PRINT_LEVEL			LOG_INFO
#define FUNCTION_TRACE		0

#define CIF_LOG(level, fmt, ...) \
	do { \
		if (level <= LOG_INFO) { \
			printk(fmt "\n", ##__VA_ARGS__); \
		} \
	} while (0)
#if FUNCTION_TRACE
#define REPORT_FUNC()	\
	do { \
		printk("enter %s\n", __FUNCTION__); \
	} while (0)
#else
#define REPORT_FUNC()
#endif
#endif



