COMPILER = /opt/st/stm32cubeide_1.9.0/headless-build.sh
COMPILER_FLAGS = -import .
PROJNAME = steering
BUILDTYPE = Debug
CUBEIDE_VERSION = 1.9.0


.PHONY: build
build:
	/opt/st/stm32cubeide_$(CUBEIDE_VERSION)/headless-build.sh $(COMPILER_FLAGS) -cleanBuild $(PROJNAME)/$(BUILDTYPE)

