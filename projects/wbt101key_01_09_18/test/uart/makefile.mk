#
# Copyright 2016, Cypress Semiconductor
# All Rights Reserved.
#
# This is UNPUBLISHED PROPRIETARY SOURCE CODE of Cypress Semiconductor;
# the contents of this file may not be disclosed to third parties, copied
# or duplicated in any form, in whole or in part, without the prior
# written permission of Cypress Semiconductor.
#

########################################################################
# Application source files
########################################################################
APP_SRC =  uart.c
APP_SRC += wiced_bt_cfg.c

C_FLAGS += -DENABLE_HCI_TRACE -DWICED_BT_TRACE_ENABLE