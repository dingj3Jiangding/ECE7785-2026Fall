"""Source-time and steady receive-time watchdog, following the Lab 3 pattern.

Lab partners: Ding Jiang, Tongning Zhang.
"""

import math


class FreshnessGuard:
    def __init__(self, timeout=0.5, max_future_stamp=0.1):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout must be positive")
        if not math.isfinite(max_future_stamp) or not 0 <= max_future_stamp <= 0.1:
            raise ValueError("max_future_stamp must be in [0, 0.1]")
        self.timeout = timeout
        self.max_future_stamp = max_future_stamp
        self.stamp = None
        self.received_at = None
        self.watermark = None
        self.reason = "waiting for data"

    def invalidate(self, reason):
        # Keep the watermark: a rejected payload must not allow older replay.
        self.stamp = self.received_at = None
        self.reason = reason

    def receive(self, stamp, now_ros, now_steady):
        if not all(math.isfinite(v) for v in (stamp, now_ros, now_steady)) or stamp <= 0:
            self.invalidate("invalid or zero source timestamp")
            return False
        age = now_ros - stamp
        if age < -self.max_future_stamp or age > self.timeout:
            self.invalidate("source timestamp is stale or in the future")
            return False
        if self.watermark is not None and stamp <= self.watermark:
            self.invalidate("duplicate or out-of-order source timestamp")
            return False
        self.watermark = self.stamp = stamp
        self.received_at = now_steady
        self.reason = ""
        return True

    def fresh(self, now_ros, now_steady):
        if self.stamp is None or self.received_at is None:
            return False
        if not all(math.isfinite(v) for v in (now_ros, now_steady)):
            self.invalidate("invalid clock")
            return False
        source_age, receive_age = now_ros - self.stamp, now_steady - self.received_at
        if not (-self.max_future_stamp <= source_age <= self.timeout
                and 0 <= receive_age <= self.timeout):
            self.invalidate("sensor watchdog expired")
            return False
        return True
