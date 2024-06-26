################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/protocolStack/mac/packet-scheduler/delay-edd-rule-downlink-packet-scheduler.cpp \
../src/protocolStack/mac/packet-scheduler/dl-exp-packet-scheduler.cpp \
../src/protocolStack/mac/packet-scheduler/dl-fls-packet-scheduler.cpp \
../src/protocolStack/mac/packet-scheduler/dl-mlwdf-packet-scheduler.cpp \
../src/protocolStack/mac/packet-scheduler/dl-pf-packet-scheduler.cpp \
../src/protocolStack/mac/packet-scheduler/downlink-packet-scheduler.cpp \
../src/protocolStack/mac/packet-scheduler/enhanced-uplink-packet-scheduler.cpp \
../src/protocolStack/mac/packet-scheduler/exp-rule-downlink-packet-scheduler.cpp \
../src/protocolStack/mac/packet-scheduler/log-rule-downlink-packet-scheduler.cpp \
../src/protocolStack/mac/packet-scheduler/mt-uplink-packet-scheduler.cpp \
../src/protocolStack/mac/packet-scheduler/mw-rule-downlink-packet-scheduler.cpp \
../src/protocolStack/mac/packet-scheduler/packet-scheduler.cpp \
../src/protocolStack/mac/packet-scheduler/roundrobin-uplink-packet-scheduler.cpp \
../src/protocolStack/mac/packet-scheduler/uplink-packet-scheduler.cpp \
../src/protocolStack/mac/packet-scheduler/downlink-nvs-scheduler.cpp \
../src/protocolStack/mac/packet-scheduler/downlink-transport-scheduler.cpp\
../src/protocolStack/mac/packet-scheduler/downlink-heterogenous-scheduler.cpp \
../src/protocolStack/mac/packet-scheduler/opt_maxcell_scheduler.cpp
# ../src/protocolStack/mac/packet-scheduler/maxflow.cpp \
# ../src/protocolStack/mac/packet-scheduler/downlink_maxflow_scheduler.cpp

OBJS += \
./src/protocolStack/mac/packet-scheduler/delay-edd-rule-downlink-packet-scheduler.o \
./src/protocolStack/mac/packet-scheduler/dl-exp-packet-scheduler.o \
./src/protocolStack/mac/packet-scheduler/dl-fls-packet-scheduler.o \
./src/protocolStack/mac/packet-scheduler/dl-mlwdf-packet-scheduler.o \
./src/protocolStack/mac/packet-scheduler/dl-pf-packet-scheduler.o \
./src/protocolStack/mac/packet-scheduler/downlink-packet-scheduler.o \
./src/protocolStack/mac/packet-scheduler/enhanced-uplink-packet-scheduler.o \
./src/protocolStack/mac/packet-scheduler/exp-rule-downlink-packet-scheduler.o \
./src/protocolStack/mac/packet-scheduler/log-rule-downlink-packet-scheduler.o \
./src/protocolStack/mac/packet-scheduler/mt-uplink-packet-scheduler.o \
./src/protocolStack/mac/packet-scheduler/mw-rule-downlink-packet-scheduler.o \
./src/protocolStack/mac/packet-scheduler/packet-scheduler.o \
./src/protocolStack/mac/packet-scheduler/roundrobin-uplink-packet-scheduler.o \
./src/protocolStack/mac/packet-scheduler/uplink-packet-scheduler.o \
./src/protocolStack/mac/packet-scheduler/downlink-nvs-scheduler.o \
./src/protocolStack/mac/packet-scheduler/downlink-transport-scheduler.o\
./src/protocolStack/mac/packet-scheduler/downlink-heterogenous-scheduler.o \
./src/protocolStack/mac/packet-scheduler/opt_maxcell_scheduler.o
# ./src/protocolStack/mac/packet-scheduler/maxflow.o \
# ../src/protocolStack/mac/packet-scheduler/downlink_maxflow_scheduler.o

CPP_DEPS += \
./src/protocolStack/mac/packet-scheduler/delay-edd-rule-downlink-packet-scheduler.d \
./src/protocolStack/mac/packet-scheduler/dl-exp-packet-scheduler.d \
./src/protocolStack/mac/packet-scheduler/dl-fls-packet-scheduler.d \
./src/protocolStack/mac/packet-scheduler/dl-mlwdf-packet-scheduler.d \
./src/protocolStack/mac/packet-scheduler/dl-pf-packet-scheduler.d \
./src/protocolStack/mac/packet-scheduler/downlink-packet-scheduler.d \
./src/protocolStack/mac/packet-scheduler/enhanced-uplink-packet-scheduler.d \
./src/protocolStack/mac/packet-scheduler/exp-rule-downlink-packet-scheduler.d \
./src/protocolStack/mac/packet-scheduler/log-rule-downlink-packet-scheduler.d \
./src/protocolStack/mac/packet-scheduler/mt-uplink-packet-scheduler.d \
./src/protocolStack/mac/packet-scheduler/mw-rule-downlink-packet-scheduler.d \
./src/protocolStack/mac/packet-scheduler/packet-scheduler.d \
./src/protocolStack/mac/packet-scheduler/roundrobin-uplink-packet-scheduler.d \
./src/protocolStack/mac/packet-scheduler/uplink-packet-scheduler.d \
./src/protocolStack/mac/packet-scheduler/downlink-nvs-scheduler.d \
./src/protocolStack/mac/packet-scheduler/downlink-transport-scheduler.d \
./src/protocolStack/mac/packet-scheduler/downlink-heterogenous-scheduler.d \
./src/protocolStack/mac/packet-scheduler/opt_maxcell_scheduler.d \
# ./src/protocolStack/mac/packet-scheduler/maxflow.d \
# ./src/protocolStack/mac/packet-scheduler/downlink_maxflow_scheduler.d


# Each subdirectory must supply rules for building sources it contributes
src/protocolStack/mac/packet-scheduler/%.o: ../src/protocolStack/mac/packet-scheduler/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/include/boost -O0 -g3 -Wall -Wno-unused-variable -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


