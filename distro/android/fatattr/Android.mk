LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_SRC_FILES:=\
	fatattr.c

LOCAL_CFLAGS:=-O2 -g

LOCAL_MODULE:=fatattr

LOCAL_MODULE_TAGS := optional eng

include $(BUILD_EXECUTABLE)

