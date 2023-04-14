ccflags-$(CONFIG_SND_ASOC_INGENIC_DEBUG) += -DDEBUG
ccflags-$(CONFIG_SND_ASOC_INGENIC_VERBOSE) += -DVERBOSE_DEBUG


obj-$(CONFIG_SND_ASOC_INGENIC_AS_FE) += snd-asoc-as-fe.o
snd-asoc-as-fe-objs 		:= as-dma.o as-dsp.o as-mixer.o as-fmtcov.o

ingenic_baic-y := as-baic.o
obj-$(CONFIG_SND_ASOC_INGENIC_AS_BAIC)  += ingenic_baic.o

ingenic_spdif-y := as-spdif.o
obj-$(CONFIG_SND_ASOC_INGENIC_AS_SPDIF) += ingenic_spdif.o

ingenic_dmic-y := as-dmic.o
obj-$(CONFIG_SND_ASOC_INGENIC_AS_DMIC) += ingenic_dmic.o

obj-$(CONFIG_SND_ASOC_INGENIC_AS_VIR_FE) += as-vir-fe.o
