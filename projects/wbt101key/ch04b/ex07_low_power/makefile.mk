#
# Copyright 2015, Broadcom Corporation
# All Rights Reserved.
#
# This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
# the contents of this file may not be disclosed to third parties, copied
# or duplicated in any form, in whole or in part, without the prior
# written permission of Broadcom Corporation.
#

########################################################################
# Application sources.
########################################################################

$(NAME)_COMPONENTS += fw_upgrade_lib.a

APP_SRC += ecdsa256_pub.c
APP_SRC += wiced_app_cfg.c 
APP_SRC += lsm9ds1_acc_gyro_driver.c
APP_SRC += lsm9ds1_mag_driver.c
APP_SRC += ex07_low_power_hw.c 
APP_SRC += ex07_low_power_gatt_db.c 
APP_SRC += ex07_low_power_ble.c 
APP_SRC += ex07_low_power.c

C_FLAGS += -DWICED_BT_TRACE_ENABLE
