ifneq ($(TARGET_SIMULATOR),true)

LOCAL_PATH := $(call my-dir)

#########################################################################
# Build mkdosfs
include $(CLEAR_VARS)

LOCAL_SRC_FILES := \
    src/mkdosfs.c \
    src/mntent.c

LOCAL_C_INCLUDES += $(LOCAL_PATH)/src
LOCAL_MODULE := mkdosfs
LOCAL_MODULE_TAGS := optional eng

LOCAL_FORCE_STATIC_EXECUTABLE := true

LOCAL_STATIC_LIBRARIES := \
	libc

include $(BUILD_EXECUTABLE)

endif
