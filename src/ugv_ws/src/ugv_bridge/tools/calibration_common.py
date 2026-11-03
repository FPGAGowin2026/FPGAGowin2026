"""标定工具共用的 IMU 合同和序号连续性统计。"""

EXPECTED_WHO_AM_I = 0x70
EXPECTED_READY_FLAGS = 0x0F
GYRO_DPS_PER_LSB = 0.0175
UINT16_MODULUS = 1 << 16
UINT32_MODULUS = 1 << 32


def sequence_quality(values: list[int], modulus: int, *, backward_key: str = "backward_steps") -> dict[str, int]:
    result = {"transitions": max(0, len(values) - 1), "duplicates": 0,
              "gap_events": 0, "skipped_values": 0, backward_key: 0}
    for previous, current in zip(values, values[1:]):
        delta = (current - previous) % modulus
        if delta == 0:
            result["duplicates"] += 1
        elif delta > modulus // 2:
            result[backward_key] += 1
        elif delta > 1:
            result["gap_events"] += 1
            result["skipped_values"] += delta - 1
    return result
