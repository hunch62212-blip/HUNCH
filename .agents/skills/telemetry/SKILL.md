---
name: telemetry-protocol
description: Telemetry and serial packet protocol rules for rover communication.
---
# Skill: Telemetry & Serial Protocol
- Frame Format: [0xAA][0x55][MSG_ID][LEN][PAYLOAD...][CRC16_H][CRC16_L][0x0A]
- MSG_ID 0x01: Velocity (float vx, float wz)
- MSG_ID 0x02: Arm angles (uint16_t j1, j2, j3)
- MSG_ID 0xFF: Emergency stop override
- Require CRC16 verification before executing commands.
