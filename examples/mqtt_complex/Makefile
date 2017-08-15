# name of your application
APPLICATION = mqtt_complex

# If no BOARD is found in the environment, use this default:
BOARD ?= openmote-cc2538

PROGRAMMER ?= jlink
USEMODULE += csma_sender
USEMODULE += hdlc
USEMODULE += xtimer
FEATURES_REQUIRED += periph_timer

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

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
# This application dumps received packets to STDIO using the pktdump module
USEMODULE += gnrc_pktdump
# For testing we also include the ping6 command and some stats
USEMODULE += gnrc_icmpv6_echo
# Additional networking modules that can be dropped if not needed
# USEMODULE += netstats_l2
# USEMODULE += netstats_ipv6

# Set a custom 802.15.4 channel if needed
DEFAULT_CHANNEL ?= 19
#very important line of code 
CFLAGS += -DIEEE802154_DEFAULT_CHANNEL=$(DEFAULT_CHANNEL)

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
CFLAGS += -DDEVELHELP -Wno-error=int-conversion
WERROR=0
# Comment this out to join RPL DODAGs even if DIOs do not contain
# DODAG Configuration Options (see the doc for more info)
# CFLAGS += -DGNRC_RPL_DODAG_CONF_OPTIONAL_ON_JOIN

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 1
include $(RIOTBASE)/Makefile.include

