#
# this (ugly) Makefile builds multiple kernel modules
# representing different SGX variants and flavours
#
# We currently need this since there are different compiler
# options and -D definitions needed for omap3430,omap3630,am33xx etc.
# so that we have to build multiple kernel modules from the
# same source code tree.
#
# Limitations:
#   always rebuilds even if nothing has changed
#
# There are two ideas how to get rid of this ugly code
#
# 1. add something to copy the sources into different subdirectories
#    before calling make to keept the .cmd and .o separate.
#    Can we set $(obj) to something different from$(src) for each
#    sub-make?
#    Most likely not since Makefile.build says: src := $(obj) and
#    KBUILD_OUTPUT (O= parameter to make) has a different meaning.
#
# 2. reduce to a single .ko that handles all architectures and variants
#    This means making the SoC glue code of the pvr generic (i.e.
#    get rid of e.g. SGX_CORE_REV or -DSGX530) and replace the
#    #ifdef SGX530 etc. in SGX errata and wherelse by dynamic decisions
#

$(obj)/pvr:
# should somehow keep the compiled .o separate...
	@$(kecho) "  MAKE    $(obj): omap3"
	$(Q)TI_PLATFORM=omap3 $(MAKE) $(build)=$(obj)/omap3
	@$(kecho) "  MAKE    $(obj): omap3630"
	$(Q)TI_PLATFORM=omap3630 $(MAKE) $(build)=$(obj)/omap3
	@$(kecho) "  MAKE    $(obj): ti335x"
	$(Q)TI_PLATFORM=ti335x $(MAKE) $(build)=$(obj)/omap3
# may add more SOC variants here, e.g.
# $(Q)CONFIG_SOC=generic $(MAKE) $(build)=$(obj)/omap3
# $(Q)CONFIG_SOC=omap5 $(MAKE) $(build)=$(obj)/omap5

ifneq ($(CONFIG_SGX_OMAP),)
always += pvr
clean-dirs := $(objtree)/omap3 $(objtree)/omap5
endif
