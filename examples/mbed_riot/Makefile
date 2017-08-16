# If no BOARD is found in the environment, use this default:
BOARD ?= openmote-cc2538

# Select the app based on the subfolder name here
APP_SELECT ?= tests/hdlc_txvr

PROGRAMMER ?= jlink

USEMODULE += xtimer
FEATURES_REQUIRED += periph_timer

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

# add app directory, add module, and generate app name
DIRS += $(APP_SELECT)
USEMODULE += $(shell basename $(APP_SELECT))
APPLICATION = $(shell basename $(APP_SELECT))

# add top directory of this example into include path
INCLUDES += -I$(CURDIR)

# Include packages that pull up and auto-init the link layer.
# NOTE: 6LoWPAN will be included if IEEE802.15.4 devices are present
USEMODULE += gnrc_netdev_default
USEMODULE += auto_init_gnrc_netif
# Specify the mandatory networking modules for IPv6 and UDP
USEMODULE += gnrc_sock_udp
USEMODULE += gnrc_ipv6_default
# Include MQTT-SN
USEMODULE += emcute
USEMODULE += gnrc_ipv6_router_default
USEMODULE += gnrc_udp
USEMODULE += ps
USEMODULE += csma_sender
USEMODULE += hdlc
USEMODULE += shell
USEMODULE += shell_commands

# Additional networking modules that can be dropped if not needed
# USEMODULE += netstats_l2
# USEMODULE += netstats_ipv6

# Set a custom 802.15.4 channel if needed
DEFAULT_CHANNEL ?= 25
CFLAGS += -DDEFAULT_CHANNEL=$(DEFAULT_CHANNEL)
CFLAGS += -DRETRANSMIT_TIMEO_USEC=100000
CFLAGS += -DHDLC_MAX_PKT_SIZE=32

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
CFLAGS += -DDEVELHELP 
WERROR=0
# Comment this out to join RPL DODAGs even if DIOs do not contain
# DODAG Configuration Options (see the doc for more info)
# CFLAGS += -DGNRC_RPL_DODAG_CONF_OPTIONAL_ON_JOIN

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 1
include $(RIOTBASE)/Makefile.include

