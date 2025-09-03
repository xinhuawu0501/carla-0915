import os
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecTransposeImage
from stable_baselines3.common.callbacks import CheckpointCallback
from lib.agents.sensor.AgentWithSensor import AgentWithSensor  # Make sure CarEnv returns semantic camera images as observations

# -----------------------------
# Paths
# -----------------------------
save_path = "carla_checkpoints"
os.makedirs(save_path, exist_ok=True)

# -----------------------------
# Checkpoint callback
# -----------------------------
checkpoint_callback = CheckpointCallback(
    save_freq=5000,
    save_path=save_path,
    name_prefix="ppo_carla_semantic"
)

# -----------------------------
# Environment
# -----------------------------
# Your CarEnv should output observations as a dict: {"camera": image_array, ...}
raw_env = AgentWithSensor()  # Make sure semantic camera is enabled
env = DummyVecEnv([lambda: raw_env])
env = VecTransposeImage(env)  # Channels last for images (HxWxC)

# -----------------------------
# PPO Model
# -----------------------------
model = PPO(
    "MultiInputPolicy",  # Handles both images and other vector observations
    env,
    n_steps=2048,
    batch_size=64,
    gamma=0.99,
    n_epochs=10,
    ent_coef=0.01,
    learning_rate=3e-4,
    verbose=1,
    device='cuda'
)

# -----------------------------
# Training
# -----------------------------
model.learn(total_timesteps=20_000, callback=checkpoint_callback)

# -----------------------------
# Save final model
# -----------------------------
model.save("ppo_carla_semantic_final")

env.close()
