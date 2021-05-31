# egm-control
EGM control for ABB's collaborative robot YuMi

## How to Setup

1. Add Transmission Protocol in Configuration--->Communication--->Transmission Protocol.

| Name  | Type  | Serial Port | Remote Address | Remote Port Number | Local Port Number |
| ----- | ----- | ----------- | -------------- | ------------------ | ----------------- |
| EGM_L | UDPUC | N/A         | 192.168.125.10 | 6510               | 0                 |
| EGM_R | UDPUC | N/A         | 192.168.125.10 | 6511               | 0                 |

2. Copy the code in the folder [server](./server) to YuMi controller.
3. Run the code in the folder [client](./client/client.py) in your external computer.

```bash
$ python client/client.py
```

