#
# This file has been automatically generated by the WICED 20719-B1 Designer.
#

APP_SRC =  key_SppConfirm.c
APP_SRC += key_SppConfirm_sdp_db.c
APP_SRC += wiced_bt_cfg.c
APP_SRC += spp.c

C_FLAGS += -DWICED_BT_TRACE_ENABLE

# If defined, HCI traces are sent over transport/WICED HCI interface
C_FLAGS += -DHCI_TRACE_OVER_TRANSPORT

$(NAME)_COMPONENTS := spp_lib.a

########################################################################
################ DO NOT MODIFY FILE BELOW THIS LINE ####################
########################################################################

