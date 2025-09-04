import numpy as np
import random
from collections import deque
import math

# ----------------- Simple V2P channel -----------------
class V2PChannel:
    def __init__(self, latency_steps=2, drop_prob=0.1, range_m=50.0, pos_noise_std=0.2, speed_noise_std=0.1):
        self.latency_steps = max(0, int(latency_steps))
        self.drop_prob = float(drop_prob)
        self.range_m2 = float(range_m)**2
        self.pos_noise_std = pos_noise_std
        self.speed_noise_std = speed_noise_std
        self._q = deque([[] for _ in range(self.latency_steps + 1)], maxlen=self.latency_steps+1)
        self._outbox = []

    def broadcast(self, ped_id, payload):
        noisy = dict(payload)
        noisy["x"] += np.random.normal(0, self.pos_noise_std)
        noisy["y"] += np.random.normal(0, self.pos_noise_std)
        noisy["speed"] += np.random.normal(0, self.speed_noise_std)
        self._q[-1].append((ped_id, noisy))

    def tick(self):
        self._outbox = self._q.popleft()
        self._q.append([])

    def receive(self, ego_xy):
        rx, ry = ego_xy
        deliver = []
        for ped_id, msg in self._outbox:
            if random.random() < self.drop_prob:
                continue
            dx = msg["x"] - rx
            dy = msg["y"] - ry
            if dx*dx + dy*dy <= self.range_m2:
                m = dict(msg)
                m["rx"] = dx
                m["ry"] = dy
                deliver.append((ped_id, m))
        return deliver

