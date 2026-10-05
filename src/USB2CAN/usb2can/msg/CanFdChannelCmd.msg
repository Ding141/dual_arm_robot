# One CAN-FD channel command (Cmd + 64-byte payload).
# Layout in the on-wire frame: 1 byte cmd + 64 bytes payload = 65 bytes.
uint8 cmd
uint8[64] payload
