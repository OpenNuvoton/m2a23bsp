# M2A23 Series CMSIS BSP

This BSP folder

## .\Document\


- CMSIS.html<br>
	Introduction of CMSIS version 5.0. CMSIS components included CMSIS-CORE, CMSIS-Driver, CMSIS-DSP, etc.

- NuMicro M2A23 Series CMSIS BSP Revision History.pdf<br>
	The revision history of M2A23 Series BSP.

- NuMicro M2A23 Series Driver Reference Guide.chm<br>
	The usage of drivers in M2A23 Series BSP.

## .\Library\


- CMSIS<br>
	Cortex® Microcontroller Software Interface Standard (CMSIS) V5.0 definitions by ARM® Corp.

- Device<br>
	CMSIS compliant device header file.

- SpdhLib<br>
	Library of SPD5 Hub.

- StdDriver<br>
	All peripheral driver header and source files.


## .\Sample Code\


- Hard\_Fault\_Sample<br>
	Show hard fault information when hard fault happened. The hard fault handler show some information included program counter, which is the address where the processor was executing when the hard fault occur. The listing file (or map file) can show what function and instruction that was. It also shows the Link Register (LR), which contains the return address of the last function call. It can show the status where CPU comes from to get to this point.

- ISP<br>
	Sample codes for In-System-Programming.

- Semihost<br>
	Show how to print and get character through IDE console window.

- StdDriver<br>
	Demonstrate the usage of M2A23 series MCU peripheral driver APIs.

- Template<br>
	A project template for M2A23 series MCU.


# Licesne

**SPDX-License-Identifier: Apache-2.0**

Copyright in some of the content available in this BSP belongs to third parties.
Third parties license is specified in a file header or license file.
M2A23 BSP files are provided under the Apache-2.0 license.

