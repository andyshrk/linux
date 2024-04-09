#undef TRACE_SYSTEM
#define TRACE_SYSTEM rpmsg

#if !defined(_SE1000_RPMSG_TRACE_H_) || defined(TRACE_HEADER_MULTI_READ)
#define _SE1000_RPMSG_TRACE_H_

#include <linux/tracepoint.h>

TRACE_EVENT(se1000_ipc_rpmsg_ept_cb,
	TP_PROTO(char type, const char *name, uint32_t src, uint32_t dst, void *ptr),
	TP_ARGS(type, name, src, dst, ptr),
	TP_STRUCT__entry(
		__field(char, type)
		__string(name, name)
		__field(uint32_t, src)
		__field(uint32_t, dst)
		__field(void *,	ptr)
 	),
	TP_fast_assign(
		__entry->type = type;
		__assign_str(name, name);
		__entry->src = src;
		__entry->dst = dst;
		__entry->ptr = ptr;
	),
	TP_printk("%c|%s|0x%x|0x%x|%ps", __entry->type, __get_str(name), __entry->src, __entry->dst, __entry->ptr)
)

#endif

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH  ../../drivers/rpmsg
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE se1000-rpmsg-trace
#include <trace/define_trace.h>
