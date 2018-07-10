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

#CFLAGS += -DOPENTHREAD_FTD=1
CXXFLAGS += -DOPENTHREAD_FTD=1 -DOPENTHREAD_ENABLE_DNS_CLIENT=1
#CXXFLAGS += -DOPENTHREAD_MTD=1 -DOPENTHREAD_ENABLE_DNS_CLIENT=1
