#
# Openthread Mesh Component Makefile
#
COMPONENT_ADD_INCLUDEDIRS := \
	src \
	src\openthread

COMPONENT_SRCDIRS := \
	src/api \
	src/coap \
	src/common \
	src/crypto \
	src/mac \
	src/meshcop \
	src/net \
	src/thread \
	src/utils \
	src/cli

CXXFLAGS += -DOPENTHREAD_FTD=1
#CXXFLAGS += -DOPENTHREAD_MTD=1
