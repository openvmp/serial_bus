# remote_serial_bus

[![License](./apache20.svg)](./LICENSE.txt)

This package is an implementation of serial bus (like RS-485).

The core feature is multiplexing of request/response pairs with the assumption
that the responses could not be distinguished between themselves (for example,
using responder's leaf identifiers like it's done in the ModBus protocol).

### Examples

- Absolute encoders in OpenVMP

  [The OpenVMP platform](https://github.com/openvmp/openvmp) uses this module to multiplex requests to multiple
[AMT-21* encoders](https://github.com/openvmp/encoder_amt21)
sitting on the same RS-485 line.
