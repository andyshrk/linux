#ifndef __SE1000_MAILBOX_IRQ_H__
#define __SE1000_MAILBOX_IRQ_H__

#include <dt-bindings/interrupt-controller/se1000-mailbox-irq-id.h>

#ifdef CONFIG_SE1000_MBOX
void *se1000_mailbox_find_channel_by_phandle(struct device_node *np);
void *se1000_mailbox_find_channel_by_phandle_index(struct device_node *np, int index);
int se1000_mailbox_raise_hwirq(void *chan, int hwirq);
int se1000_mailbox_chan_check(void *chan);
#else
inline void *se1000_mailbox_find_channel_by_phandle(struct device_node *np)
{
	return NULL;
}

inline void *se1000_mailbox_find_channel_by_phandle_index(struct device_node *np, int index)
{
	return NULL;
}

inline int se1000_mailbox_raise_hwirq(void *chan, int hwirq)
{
	return 0;
}

inline int se1000_mailbox_chan_check(void *chan)
{
	return 0;
}
#endif

#endif /* __SE1000_MAILBOX_IRQ_H__ */
