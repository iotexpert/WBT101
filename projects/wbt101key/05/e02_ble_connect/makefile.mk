#
# Copyright 2014, Cypress Semiconductor
# All Rights Reserved.
#
# This is UNPUBLISHED PROPRIETARY SOURCE CODE of Cypress Semiconductor;
# the contents of this file may not be disclosed to third parties, copied
# or duplicated in any form, in whole or in part, without the prior
# written permission of Cypress Semiconductor.
#

########################################################################
# Add Application sources here.
########################################################################

APP_SRC += e02_ble_connect.c
APP_SRC  += wiced_bt_cfg.c 

########################################################################
# C flags
########################################################################
C_FLAGS += -DWICED_BT_TRACE_ENABLE

########################################################################
################ DO NOT MODIFY FILE BELOW THIS LINE ####################
########################################################################
