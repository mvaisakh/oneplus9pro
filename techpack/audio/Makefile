# auto-detect subdirs
ifneq ($(CONFIG_ARCH_QTI_VM), y)
ifeq ($(CONFIG_QTI_QUIN_GVM), y)
ifeq ($(BUILTIN), 1)
include $(srctree)/techpack/audio/config/gvmbuiltinauto.conf
else
include $(srctree)/techpack/audio/config/gvmauto.conf
endif
endif
ifeq ($(CONFIG_ARCH_SDXPOORWILLS), y)
ifeq ($(BUILTIN), 1)
include $(srctree)/techpack/audio/config/sdxpoorwillsbuiltinauto.conf
else
include $(srctree)/techpack/audio/config/sdxpoorwillsauto.conf
endif
export
endif
ifeq ($(CONFIG_ARCH_SM8150), y)
ifeq ($(BUILTIN), 1)
include $(srctree)/techpack/audio/config/sm8150builtinauto.conf
else
include $(srctree)/techpack/audio/config/sm8150auto.conf
endif
export
endif
ifeq ($(CONFIG_ARCH_SDMSHRIKE), y)
ifeq ($(BUILTIN), 1)
include $(srctree)/techpack/audio/config/sm8150builtinauto.conf
else
include $(srctree)/techpack/audio/config/sm8150auto.conf
endif
export
endif
ifeq ($(CONFIG_ARCH_KONA), y)
ifeq ($(BUILTIN), 1)
include $(srctree)/techpack/audio/config/konabuiltinauto.conf
else
include $(srctree)/techpack/audio/config/konaauto.conf
endif
endif
ifeq ($(CONFIG_ARCH_LAHAINA), y)
ifeq ($(BUILTIN), 1)
include $(srctree)/techpack/audio/config/lahainabuiltinauto.conf
else
include $(srctree)/techpack/audio/config/lahainaauto.conf
endif
endif
ifeq ($(CONFIG_ARCH_HOLI), y)
ifeq ($(BUILTIN), 1)
include $(srctree)/techpack/audio/config/holibuiltinauto.conf
else
include $(srctree)/techpack/audio/config/holiauto.conf
endif
endif
ifeq ($(CONFIG_ARCH_SA8155), y)
ifeq ($(BUILTIN), 1)
include $(srctree)/techpack/audio/config/sa8155builtinauto.conf
else
include $(srctree)/techpack/audio/config/sa8155auto.conf
endif
endif
ifeq ($(CONFIG_ARCH_SA6155), y)
ifeq ($(BUILTIN), 1)
include $(srctree)/techpack/audio/config/sa6155builtinauto.conf
else
include $(srctree)/techpack/audio/config/sa6155auto.conf
endif
endif
ifeq ($(CONFIG_ARCH_SM6150), y)
ifeq ($(BUILTIN), 1)
include $(srctree)/techpack/audio/config/sm6150builtinauto.conf
else
include $(srctree)/techpack/audio/config/sm6150auto.conf
endif
endif
endif
# Use USERINCLUDE when you must reference the UAPI directories only.
USERINCLUDE     += \
                -I$(srctree)/techpack/audio/include/uapi/audio

# Use LINUXINCLUDE when you must reference the include/ directory.
# Needed to be compatible with the O= option
LINUXINCLUDE    += \
                -I$(srctree)/techpack/audio/include/uapi \
                -I$(srctree)/techpack/audio/include/uapi/audio \
                -I$(srctree)/techpack/audio/include/asoc \
                -I$(srctree)/techpack/audio/include

ifeq ($(CONFIG_QTI_QUIN_GVM), y)
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/gvmautoconf.h
endif
ifeq ($(CONFIG_ARCH_SDXPOORWILLS), y)
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/sdxpoorwillsautoconf.h
endif
ifeq ($(CONFIG_ARCH_SM8150), y)
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/sm8150autoconf.h
endif
ifeq ($(CONFIG_ARCH_SDMSHRIKE), y)
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/sm8150autoconf.h
endif
ifeq ($(CONFIG_ARCH_KONA), y)
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/konaautoconf.h
endif
ifeq ($(CONFIG_ARCH_LAHAINA), y)
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/lahainaautoconf.h
endif
ifeq ($(CONFIG_ARCH_HOLI), y)
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/holiautoconf.h
endif
ifeq ($(CONFIG_ARCH_SA8155), y)
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/sa8155autoconf.h
endif
ifeq ($(CONFIG_ARCH_SA6155), y)
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/sa6155autoconf.h
endif
ifeq ($(CONFIG_ARCH_SM6150), y)
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/config/sm6150autoconf.h
endif
obj-y += soc/
obj-y += dsp/
obj-y += ipc/
obj-y += asoc/
