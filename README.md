# OpenVMP

[![License](./apache20.svg)](./LICENSE.txt)

This package is a part of [the OpenVMP project](https://github.com/openvmp/openvmp).
But it's designed to be universal and usable independently from the rest of OpenVMP or in a combination with select OpenVMP packages.

## Serial bus

This package is an implementation of serial bus (like RS-485).

The core feature is multiplexing of request/response pairs with the assumption
that the responses could not be distinguished between themselves (for example,
using responder's leaf identifiers like it's done in the ModBus protocol).

### Examples

- Absolute encoders in OpenVMP

  The OpenVMP platform uses this module to multiplex requests to multiple
[AMT-21](https://www.cuidevices.com/product/motion-and-control/rotary-encoders/absolute/modular/amt21-series)
encoders sitting on the same RS-485 line.
