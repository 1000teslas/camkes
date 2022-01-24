
# Copyright (c) 2022, University of New South Wales
#
# SPDX-License-Identifier: BSD-2-Clause
#

cmake_minimum_required(VERSION 3.7.2)

project(kitty C)

include(${LWIP_HELPERS})
AddLWIPConfiguration(${CMAKE_CURRENT_LIST_DIR}/lwip_include)

set(libs sel4utils sel4vka sel4allocman sel4vspace sel4simple sel4platsupport ethdrivers lwip usbdrivers)

DeclareCAmkESComponent(
	client
	SOURCES components/client/src/client.c
	INCLUDES include
	LIBS
	${libs}
)

DeclareCAmkESComponent(
	display
	SOURCES components/display/src/ipu.c components/display/src/textPrint.c components/display/src/display.c
	INCLUDES components/display/include include
	LIBS
	${libs}
)

DeclareCAmkESComponent(
	rfid
	SOURCES components/rfid/src/client.c
	INCLUDES include
	LIBS
	${libs}
)

DeclareCAmkESComponent(
	serial
	SOURCES components/serial/src/driver.c
	INCLUDES components/serial/include include
	LIBS
	${libs}
)

DeclareCAmkESComponent(
	timer
   	SOURCES components/timer/src/timer.c components/timer/src/epit.c
	INCLUDES components/timer/include
	LIBS
	${libs}
)

DeclareCAmkESComponent(
	usb
	SOURCES components/usb/src/usb.c
	INCLUDES include
	LIBS
	${libs}
)

DeclareCAmkESRootserver(kitty.camkes CPP_INCLUDES include)
