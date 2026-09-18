import sys
from pathlib import Path
import torch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "out" / "build" / "x64-Release"))

from encoder import Layout, stateEncoder
from model import ActorCritic

import traffic_rl_native as native

config = native.get_config()


checkpoint = torch.load(Path(__file__).parent / "checkpoints" / "ppo_5tl.pt",weights_only=False)

layout = None
encoder = None
model = None
current_obs = None
current_mask = None



def send(data):
    global layout, encoder, model, current_obs, current_mask

    if data.done:
        print("Final rewards:", data.rewards)
        return

    if encoder is None:
        layout = Layout.from_states(data.states)
        encoder = stateEncoder(layout, config["max_cars"],config["steps"],config["green_steps"],)

        model = ActorCritic(feature_count=checkpoint["feature_count"],max_phases=checkpoint["max_phases"],)
        model.load_state_dict(checkpoint["model_state"])
        model.eval()

    current_obs, current_mask = encoder.encode_batch(data.states)


def receive():
    with torch.no_grad():
        logits, _ = model(current_obs, current_mask)
        probabilities = torch.softmax(logits, dim=-1)
        print(probabilities.tolist())

    actions = logits.argmax(dim=-1)
    print("actions:", actions.tolist())
    return actions.tolist()


native.prepare()
native.set_callbacks(send, receive)
native.start_output()

try:
    native.reset(0)
    native.run_episode()
finally:
    native.clear_callbacks()
    native.finish_output()