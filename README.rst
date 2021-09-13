.. _nrfcloud_gateway_controller:

nRF Cloud Gateway Controller
############################

This controller firmware combines two major functions:

1. USB-UART bridge

#. Bluetooth HCI controller

The USB-UART bridge acts as a serial adapter, exposing 2 UART pairs to a USB host as 2 CDC ACM devices.
This is used by the application running in the nRF9160 for logging and for management interface (shell) access.

It also exposes the Zephyr Bluetooth controller support over UART to the nRF9160 
using the H:4 HCI transport protocol (requires HW flow control from the UART).


Requirements
************

* The following board:

  * Apricity Gateway

* A USB host which can communicate with CDC ACM devices, like a Windows or Linux PC

Default UART settings
*********************

Two UARTs connect the nRF52840 running this firmware to the nRF9160:

1. UART1 - for logging and shell

   - Baudrate: 115200 bit/s
   - 8 bits, no parity, 1 stop bit
   - Hardware Flow Control (RTS/CTS) lines connected, but not currently enabled

#. UART2 - for Bluetooth HCI

   - Baudrate: 1 Mbit/s
   - 8 bits, no parity, 1 stop bit
   - Hardware Flow Control (RTS/CTS) enabled

Building and running
********************

In order to Flash the first firmware image to the Apricity Gateway, you will need one of the following connections:

- An nRF9160 DK with VDDIO set to 3V, a 10 pin ribbon connected to Debug out, and an adapter from that to a 6 pin Tag Connect connector.
- A Segger J-Link with an adapter to a 6 pin Tag Connect.
- For either method, connect the Tag Connect to ``NRF52:J1`` on the PCB.

Program nRF52840 Board Controller
---------------------------------

1. Checkout this repository.
#. Execute the following to pull down all other required repositories::

      west update 
 
#. Execute the following to build for the Apricity Gateway hardware::

      west build -d build -b apricity_gateway_nrf52840

#. Flash it::

      west flash -d build --erase --force


Testing
-------

After programming the sample to your board, test it by performing the following steps:

1. Connect the board to the host via a USB cable
#. Observe that the CDC ACM devices enumerate on the USB host (Typically COM ports on Windows, /dev/tty* on Linux)
#. Use a serial client on the USB host to communicate over the board's UART pins


Dependencies
************

This sample uses the following Zephyr subsystems:

* `zephyr:usb_api`_
* `zephyr:bluetooth_api`_

Debugging the controller
************************

The sample can be debugged using RTT since the UART is otherwise used by this
application. To enable debug over RTT the debug configuration file can be used.

.. code-block:: console

   west build -- -DOVERLAY_CONFIG='debug.conf'

Then attach RTT as described here: `Using Segger J-Link <Using Segger J-Link>`_

.. ### Local links.

.. _`zephyr:usb_api`: https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/zephyr/reference/usb/index.html#usb-api
.. _`zephyr:bluetooth_api`: https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/zephyr/reference/bluetooth/index.html#bluetooth-api
.. _`Using Segger J-Link <Using Segger J-Link>`: https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/zephyr/guides/flash_debug/probes.html#using-segger-j-link

