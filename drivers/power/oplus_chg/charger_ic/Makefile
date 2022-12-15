ifeq ($(CONFIG_OPLUS_MSM8953N_CHARGER),y)
	obj-y	+= oplus_battery_msm8953_N.o
else ifeq ($(CONFIG_OPLUS_MSM8953_CHARGER),y)
	obj-y	+= oplus_battery_msm8953.o
else ifeq ($(CONFIG_OPLUS_MSM8998_CHARGER),y)
	obj-y	+= oplus_battery_msm8998.o
else ifeq ($(CONFIG_OPLUS_MSM8998O_CHARGER),y)
	obj-y	+= oplus_battery_msm8998_O.o
else ifeq ($(CONFIG_OPLUS_SDM845_CHARGER),y)
	obj-y	+= oplus_battery_sdm845.o
	obj-y	+= oplus_bq25882.o oplus_da9313.o
else ifeq ($(CONFIG_OPLUS_SDM670_CHARGER),y)
	obj-y	+= oplus_battery_sdm670.o
	obj-y	+= oplus_bq25882.o oplus_da9313.o
else ifeq ($(CONFIG_OPLUS_SDM670P_CHARGER),y)
	obj-y	+= oplus_battery_sdm670P.o
	obj-y	+= oplus_bq25882.o oplus_da9313.o
else ifeq ($(CONFIG_OPLUS_SDM670Q_CHARGER),y)
	obj-y	+= oplus_battery_sdm670Q.o
	obj-y	+= oplus_bq25882.o oplus_da9313.o
else ifeq ($(CONFIG_OPLUS_SM8150_CHARGER),y)
	obj-y	+= oplus_bq25882.o
	obj-y	+= oplus_da9313.o
	obj-y	+= oplus_battery_msm8150.o
else ifeq ($(CONFIG_OPLUS_SM8150_PRO_CHARGER),y)
	obj-y	+= oplus_mp2650.o
	obj-y	+= oplus_da9313.o
	obj-y	+= oplus_battery_msm8150_pro.o
else ifeq ($(CONFIG_OPLUS_SM6125_CHARGER),y)
	obj-y   += oplus_battery_sm6125P.o
else ifeq ($(CONFIG_OPLUS_SM7150_CHARGER),y)
	obj-y   += oplus_battery_sm7150_P.o
else ifeq ($(CONFIG_OPLUS_SM7250_CHARGER),y)
	//obj-y	+= oplus_bq25882.o
	obj-y	+= oplus_da9313.o
	obj-y   += oplus_battery_msm7250_Q.o
	obj-y	+= oplus_mp2650.o
else ifeq ($(CONFIG_OPLUS_SM7250R_CHARGER),y)
	obj-y	+= oplus_da9313.o
	obj-y   += oplus_battery_msm7250_R.o
	obj-y	+= oplus_mp2650.o
	obj-y   += oplus_usbtemp.o
else ifeq ($(CONFIG_OPLUS_SM8250_CHARGER),y)
	obj-y	+= oplus_da9313.o
	obj-y	+= oplus_battery_msm8250.o
	obj-y	+= oplus_mp2650.o
	obj-y   += oplus_usbtemp.o

else ifeq ($(CONFIG_OPLUS_SM8150R_CHARGER),y)
	obj-y	+= oplus_da9313.o
	obj-y	+= oplus_mp2650.o
	obj-y	+= oplus_battery_msm8150Q.o
else
	obj-y	+= oplus_battery_msm8976.o
endif 

	obj-y   += oplus_short_ic.o
