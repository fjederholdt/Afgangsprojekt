################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../DHParams.cpp \
../FileStorageClass.cpp \
../PC.cpp \
../TCPServer.cpp \
../main.cpp 

OBJS += \
./DHParams.o \
./FileStorageClass.o \
./PC.o \
./TCPServer.o \
./main.o 

CPP_DEPS += \
./DHParams.d \
./FileStorageClass.d \
./PC.d \
./TCPServer.d \
./main.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/include -I/usr/local/include -I/usr/include/vtk-7.1 -I/usr/include/pcl-1.10 -I/usr/include/eigen3 -O0 -g3 -Wall -c -fmessage-length=0 -fPIC -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


