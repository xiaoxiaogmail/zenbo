uvcvideo-objs  := uvc_driver.o uvc_queue.o uvc_v4l2.o uvc_video.o uvc_ctrl.o \
		  uvc_status.o uvc_isight.o uvc_debugfs.o

ifeq ($(TARGET_BUILD_VARIANT), eng)
KBUILD_CFLAGS += -DFACTORY_IMAGE=1
else
KBUILD_CFLAGS += -DFACTORY_IMAGE=0
endif

ifeq ($(CONFIG_MEDIA_CONTROLLER),y)
uvcvideo-objs  += uvc_entity.o
endif
obj-$(CONFIG_USB_VIDEO_CLASS) += uvcvideo.o
