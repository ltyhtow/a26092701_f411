# Serial Library Sources

This directory vendors the C sources used by the serial transport:

- `third_party/lwrb`: LwRB v3.3.0, copied from `C:\Users\30496\Desktop\lwrb-develop`.
- `third_party/lwpkt`: LwPKT v2.0.0, copied from `C:\Users\30496\Desktop\lwpkt-develop`.

Both libraries are distributed under the MIT license. The original copyright and
license notices remain in each vendored source file. The project uses the
libraries through their native APIs; application code only supplies the UART
transport and maps validated packet payloads to FreeRTOS queues.
