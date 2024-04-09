#ifndef __SYSTEM_LOG_H__
#define __SYSTEM_LOG_H__

enum {
    LOG_NOTHING,
    LOG_EMERG,
    LOG_ALERT,
    LOG_CRIT,
    LOG_ERR,
    LOG_WARNING,
    LOG_NOTICE,
    LOG_INFO,
    LOG_DEBUG,
    LOG_IRQ,
    LOG_MAX
};

int printk( const char *, ... );

//changeable logs
#define FW_LOG_LEVEL LOG_DEBUG

extern const char *const gdc_log_level[LOG_MAX];
extern unsigned int gdc_log;

#define FILE ( strrchr( __FILE__, '/' ) ? strrchr( __FILE__, '/' ) + 1 : __FILE__ )

#define LOG( level, fmt, ... ) \
    if ( ( level ) <= gdc_log ) printk( "%s: %s(%d) %s: " fmt /*"\n"*/, FILE, __func__, __LINE__, gdc_log_level[level], ##__VA_ARGS__ )

#endif // __SYSTEM_LOG_H__
