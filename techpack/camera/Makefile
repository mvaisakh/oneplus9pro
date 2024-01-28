# SPDX-License-Identifier: GPL-2.0-only

# auto-detect subdirs
ifneq ($(CONFIG_ARCH_QTI_VM), y)
ifeq ($(CONFIG_ARCH_KONA), y)
ifeq ($(BUILTIN), 1)
include $(srctree)/techpack/camera/config/konabuiltincamera.conf
else
include $(srctree)/techpack/camera/config/konacamera.conf
endif
endif

ifeq ($(CONFIG_ARCH_LITO), y)
ifeq ($(BUILTIN), 1)
include $(srctree)/techpack/camera/config/litobuiltincamera.conf
else
include $(srctree)/techpack/camera/config/litocamera.conf
endif
endif

ifeq ($(CONFIG_ARCH_BENGAL), y)
ifeq ($(BUILTIN), 1)
include $(srctree)/techpack/camera/config/bengalbuiltincamera.conf
else
include $(srctree)/techpack/camera/config/bengalcamera.conf
endif
endif

ifeq ($(CONFIG_ARCH_MONACO), y)
ifeq ($(BUILTIN), 1)
include $(srctree)/techpack/camera/config/monacobuiltincamera.conf
else
include $(srctree)/techpack/camera/config/monacocamera.conf
endif
endif

ifeq ($(CONFIG_ARCH_LAHAINA), y)
ifeq ($(BUILTIN), 1)
include $(srctree)/techpack/camera/config/lahainabuiltincamera.conf
else
include $(srctree)/techpack/camera/config/lahainacamera.conf
endif
endif

ifeq ($(CONFIG_ARCH_HOLI), y)
ifeq ($(BUILTIN), 1)
include $(srctree)/techpack/camera/config/holibuiltincamera.conf
else
include $(srctree)/techpack/camera/config/holicamera.conf
endif
endif

ifeq ($(CONFIG_ARCH_BLAIR), y)
ifeq ($(BUILTIN), 1)
include $(srctree)/techpack/camera/config/blairbuiltincamera.conf
else
include $(srctree)/techpack/camera/config/blaircamera.conf
endif
endif

ifeq ($(CONFIG_ARCH_SHIMA), y)
ifeq ($(BUILTIN), 1)
include $(srctree)/techpack/camera/config/shimabuiltincamera.conf
else
include $(srctree)/techpack/camera/config/shimacamera.conf
endif
endif

ifeq ($(CONFIG_ARCH_YUPIK), y)
ifeq ($(BUILTIN), 1)
include $(srctree)/techpack/camera/config/yupikbuiltincamera.conf
else
include $(srctree)/techpack/camera/config/yupikcamera.conf
endif
endif

ifeq ($(CONFIG_ARCH_SM6150), y)
ifeq ($(BUILTIN), 1)
include $(srctree)/techpack/camera/config/sm6150builtincamera.conf
else
include $(srctree)/techpack/camera/config/sm6150camera.conf
endif
endif

ifeq ($(CONFIG_ARCH_KONA), y)
LINUXINCLUDE    += \
		-include $(srctree)/techpack/camera/config/konacameraconf.h
endif

ifeq ($(CONFIG_ARCH_LITO), y)
LINUXINCLUDE    += \
		-include $(srctree)/techpack/camera/config/litocameraconf.h
endif

ifeq ($(CONFIG_ARCH_BENGAL), y)
LINUXINCLUDE    += \
		-include $(srctree)/techpack/camera/config/bengalcameraconf.h
endif

ifeq ($(CONFIG_ARCH_MONACO), y)
LINUXINCLUDE    += \
		-include $(srctree)/techpack/camera/config/monacocameraconf.h
endif

ifeq ($(CONFIG_ARCH_LAHAINA), y)
LINUXINCLUDE    += \
		-include $(srctree)/techpack/camera/config/lahainacameraconf.h
endif

ifeq ($(CONFIG_ARCH_HOLI), y)
LINUXINCLUDE    += \
		-include $(srctree)/techpack/camera/config/holicameraconf.h
endif

ifeq ($(CONFIG_ARCH_BLAIR), y)
LINUXINCLUDE    += \
		-include $(srctree)/techpack/camera/config/blaircameraconf.h
endif

ifeq ($(CONFIG_ARCH_SHIMA), y)
LINUXINCLUDE    += \
		-include $(srctree)/techpack/camera/config/shimacameraconf.h
endif

ifeq ($(CONFIG_ARCH_YUPIK), y)
LINUXINCLUDE    += \
		-include $(srctree)/techpack/camera/config/yupikcameraconf.h
endif

ifeq ($(CONFIG_ARCH_SM6150), y)
LINUXINCLUDE    += \
		-include $(srctree)/techpack/camera/config/sm6150cameraconf.h
endif

endif

ifneq (,$(filter $(CONFIG_SPECTRA_CAMERA), y m))
# Use USERINCLUDE when you must reference the UAPI directories only.
USERINCLUDE     += \
                -I$(srctree)/techpack/camera/include/uapi

# Use LINUXINCLUDE when you must reference the include/ directory.
# Needed to be compatible with the O= option
LINUXINCLUDE    += \
                -I$(srctree)/techpack/camera/include/uapi \
                -I$(srctree)/techpack/camera/include
obj-y += drivers/
else
$(info Target not found)
endif
