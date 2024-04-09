/*
 * (C) 2019-2021 Siengine, Inc. (www.siengine.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */


#ifndef __RFKILL_GPIO_H
#define __RFKILL_GPIO_H

#include <linux/types.h>
#include <linux/rfkill.h>
#include <linux/clk.h>

#define RFKILL_GPIO_NAME_SIZE   64

struct rfkill_gpio {
    int     io;
    char    name[RFKILL_GPIO_NAME_SIZE];
    int     enable; // disable = !enable
    struct  pinctrl_state    *gpio_state;
    struct  pinctrl_state    *default_state;
};

struct rfkill_irq {
    char                    name[RFKILL_GPIO_NAME_SIZE];
    struct rfkill_gpio   gpio;
    int                     irq;
};

/**
 * struct rfkill_platform_data - platform data for rfkill gpio device.
 * for unused gpio's, the expected value is -1.
 * @name:               name for the gpio rf kill instance
 * @reset_gpio:         GPIO which is used for reseting rfkill switch
 * @shutdown_gpio:      GPIO which is used for shutdown of rfkill switch
 */

struct rfkill_platform_data {
    char                    *name;
    enum rfkill_type        type;
    bool                    power_toggle;
    struct pinctrl          *pinctrl;
    struct rfkill_gpio   poweron_gpio;
    struct rfkill_gpio   reset_gpio;
    struct rfkill_gpio   wake_gpio;      // Host wake or sleep BT
    struct rfkill_irq    wake_host_irq;  // BT wakeup host
    struct rfkill_gpio   rts_gpio;
    struct clk              *ext_clk;
};

int rfkill_get_bt_power_state(int *power, bool *toggle);

#endif /* __RFKILL_GPIO_H */
