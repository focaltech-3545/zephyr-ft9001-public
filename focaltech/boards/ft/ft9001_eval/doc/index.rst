zephyr:board:: ft9001_eval

The FT9001-EVAL board is a hardware platform that enables design and debug
of the ARM Cortex-M4F High Performance MCU which can run upto 160-MHz, 
32KByte of ROM, 224KByte of RAM.

Hardware
  - 2x UARTs 
  - 2x SPIs support master mode and slave mode
  - 2x EPORTs including GINTs		
		- GINT13  eport1	GINT13	
		- GINT58  eport7	ISODAT2	only pulldown
		- GINT59  eport7	ISOCLK2	only pulldown
		- GINT60  eport7	ISORST2	only pulldown
		- GINT61  eport7	ISODAT1	


For more information about FT9001_QFN32-EVAL board:
  - `FT9001 QFN32 EVB 使用说明V1.1.pdf`

The Zephyr ft9001 board configuration supports the following hardware features:

+-----------+------------+-------------------------------------+
| Interface | Controller | Driver/Component                    |
+===========+============+=====================================+
| CLOCK     | on-chip    | clock control                       |
+-----------+------------+-------------------------------------+
| RESET     | on-chip    | reset                               |
+-----------+------------+-------------------------------------+
| GPIO      | on-chip    | gpio                                |
+-----------+------------+-------------------------------------+
| NVIC      | on-chip    | nested vector interrupt controller  |
+-----------+------------+-------------------------------------+
| SPI       | on-chip    | spi bus                             |
+-----------+------------+-------------------------------------+
| UART      | on-chip    | serial port-polling;                |
|           |            | serial port-interrupt               |
+-----------+------------+-------------------------------------+

#. Build the Zephyr kernel and the :zephyr:code-sample:`hello_world` sample application:
   zephyr-app-commands::
      :zephyr-app: samples/hello_world
      :board: ft9001_eval
      :goals: build
      :compact: