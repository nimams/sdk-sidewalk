#ifdef SX126x
#include <stdio.h>
#include "zephyr/sys/util.h"
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

uint32_t CLI_sleep_to_full_power_us = 440;

static int cmd_sx126x_sfp(const struct shell *shell, size_t argc, char **argv)
{
    if (argc == 2) {
	    sscanf(argv[1], "%u", &CLI_sleep_to_full_power_us);
    }
    shell_info(shell, "SFP: %u", CLI_sleep_to_full_power_us);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_services,
    SHELL_CMD_ARG(sfp, NULL, "get/set sleep_to_full_power_us", cmd_sx126x_sfp, 1, 2),
	SHELL_SUBCMD_SET_END);

// command, subcommands, help, handler
SHELL_CMD_REGISTER(sx126x, &sub_services, "SX126X testing CLI", NULL);

#else
#endif /* SX126x */

