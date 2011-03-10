#
# Makefile for msm-bus driver specific files
#
obj-$(CONFIG_MSM_BUS_SCALING)	+= msm_bus_core.o msm_bus_fabric.o msm_bus_board_8660.o
ifdef CONFIG_MSM_BUS_SCALING
obj-$(CONFIG_DEBUG_FS) += msm_bus_dbg.o
endif

