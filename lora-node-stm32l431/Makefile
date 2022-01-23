PROJECT = lora-node
BUILD_DIR = bin

OPT := -Os

# application settings
#CFLAGS += -DSWD_ENABLE=1
CFLAGS += -DDEEP_SLEEP_ENABLE=1

# lmic settings
CFLAGS += -DLMIC_X_DEBUG_LEVEL=0 -DUSE_ORIGINAL_AES=1 -DOSTICKS_PER_SEC=32768 -DCFG_eu868=1 -DCFG_sx1276_radio=1 -DHAS_ostick_conv=0 -DCFG_TxContinuousMode=0 -DLMIC_USE_INTERRUPTS=1 -DLMIC_PRId_ostime_t="PRId64" -DLMIC_ENABLE_DeviceTimeReq=1 -DLMIC_ENABLE_event_logging=1

CFILES = lora-node.c

LMIC_DIR = ../arduino-lmic/src/lmic
CFILES += lmic_as923.c lmic_au915.c lmic.c lmic_channelshuffle.c lmic_compliance.c lmic_eu868.c lmic_eu_like.c lmic_in866.c lmic_kr920.c lmic_us915.c lmic_us_like.c lmic_util.c oslmic.c radio.c

AES_DIR = ../arduino-lmic/src/aes
CFILES += lmic-aes.c other.c

HAL_DIR = ../common
CFILES += hal.c debug.c

DEVICE=stm32l431cb
OOCD_FILE = board/stm32f103c8_blue_pill.cfg

# You shouldn't have to edit anything below here.
VPATH += $(LMIC_DIR) $(AES_DIR) $(HAL_DIR)
INCLUDES += $(patsubst %,-I%, . $(LMIC_DIR) $(AES_DIR) $(HAL_DIR))
OPENCM3_DIR=../libopencm3

include $(OPENCM3_DIR)/mk/genlink-config.mk
include ../rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk
