#undef TRACE_SYSTEM
#define TRACE_SYSTEM nano2d

#if !defined(_NANO2D_TRACE_H_) || defined(TRACE_HEADER_MULTI_READ)
#define _NANO2D_TRACE_H_

#include <linux/tracepoint.h>

TRACE_EVENT(tracing_mark_write,
    TP_PROTO(char type, const struct task_struct *task,
        const char *name, int value),
    TP_ARGS(type, task, name, value),
    TP_STRUCT__entry(
        __field(char, type)
        __field(int, pid)
        __string(name, name)
        __field(int, value)
    ),
    TP_fast_assign(
        __entry->type = type;
        __entry->pid = task ? task->tgid : 0;
        __assign_str(name, name);
        __entry->value = value;
    ),
    TP_printk("%c|%d|%s|%d", __entry->type, __entry->pid, __get_str(name), __entry->value)
)

#define NANO_TRACE_BEGIN(name)      trace_tracing_mark_write('B', current, name, 0)
#define NANO_TRACE_END(name)        trace_tracing_mark_write('E', current, name, 0)
#define NANO_TRACE_INT(name, value) trace_tracing_mark_write('C', current, name, value)
#endif

#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE nano2d_trace
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#include <trace/define_trace.h>
