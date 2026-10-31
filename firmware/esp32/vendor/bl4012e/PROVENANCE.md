# BL4012e 3in1 bus component

Source snapshot: `BL4012e/3in1/src/libs/joint_bus/joint_bus.c` and
`include/joint_bus.h`, copied from the user's working bench on 2026-10-31.
`upstream/` retains those files byte for byte; hashes are in `upstream/SHA256.json`.
No license was added or inferred for the user's source.

The firmware compiles **src/joint_bus.c**, not the reference snapshot. Its
local changes are limited to:

- Last-error classification (packet, TX, echo short/mismatch, reply short/invalid).
- TX snapshot before driving; echo/RX copies and stage timestamps after the pad is released.
- Rejection of an incomplete `uart_write_bytes` return, after waiting/releasing
  the pad. Upstream ignored that return value.
- C++ linkage declarations and diagnostics fields in the header.
- IDF4 compatibility spelling for APB clock (inactive in the pinned IDF5 build).

Drive/release functions, UART initialization, packet codec, checksums, reply
validation and stage waits are inherited: TX-done 5 ms, echo 2 ms, joint reply
2 ms, wheel reply 6 ms. No FIFO transmission, manual OEN selection, spinwait,
RMT pad capture, retry loop or resynchronizing parser is added.

`EspMotorBus` only validates the HCR-supported calls, delegates ping/read/write,
and maps replies/errors. In particular HCR rejects broadcast IDs: no reply
cannot constitute a successful six-motor batch. The backend still performs
coordinate conversion, freshness checks and ACK after both buses.

The bench application `leg.c`, its trajectory generator, wheel ramp/alternating
schedule and miss-count policy are **not** the bus component and are not copied.
Those differ from HCR absolute targets and explicit batch acknowledgements.

Review the local delta with:

```sh
diff -u upstream/joint_bus.c src/joint_bus.c
diff -u upstream/joint_bus.h include/joint_bus.h
```

From the repository root, run `python3 firmware/esp32/tests/check_inherited_bus.py`.
It compares upstream/candidate SDK call traces and golden packets, compiles the
actual C++ adapter against mocked SDK calls, injects faults, and checks that a
negative control with the pad attachment removed fails. The mock fixtures derive
from `BL4012e/tests/check_motor_bus.py`. This cannot prove physical UART timing.
