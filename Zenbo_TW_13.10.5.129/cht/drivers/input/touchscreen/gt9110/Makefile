ifeq ($(TARGET_BUILD_VARIANT), eng)
KBUILD_CFLAGS += -DFACTORY_IMAGE=1
endif

obj-$(CONFIG_TOUCHSCREEN_GT9110) += gt9xx.o gt9xx_update.o goodix_tool.o gt9xx_openshort.o
