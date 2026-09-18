from pathlib import Path
import sys
import torch
from model import ActorCritic
import ppo
from ppo import rolloutBuffer
from pathlib import Path
from encoder import Layout, stateEncoder

root = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(root / "out" / "build" / "x64-Release"))

import traffic_rl_native as native

config = native.get_config()

encoder = None
current_obs = None
current_mask = None
model = None
optimizer = None
buffer = rolloutBuffer()

def send(data):
    global encoder, current_obs, current_mask

    if encoder is None:
       layout = Layout.from_states(data.states)
       encoder = stateEncoder(layout, config["max_cars"],config["steps"],config["green_steps"])

    if buffer.pending:
       buffer.close_transition(data.rewards, data.done)

    if data.done:
       return

    current_obs, current_mask = encoder.encode_batch(data.states)

def receive():
    global model, optimizer

    if model is None:
        model = ActorCritic(feature_count=current_obs.shape[1],max_phases=current_mask.shape[1])
        optimizer = torch.optim.Adam(model.parameters(),lr=1e-4,)

    with torch.no_grad():
        actions, log_probs, values = model.act(current_obs,current_mask)

    buffer.add_decision(current_obs,current_mask,actions,log_probs,values)   
    
    return actions.tolist()


native.prepare()
native.set_callbacks(send, receive)

try:
 for i in range(2000):
  if i > 0: 
   native.reset(i)
  native.run_episode()

  if (i+1)%8 == 0:  
   rollout = buffer.tensors()

   advantages, returns = ppo.compute_gae(rollout)

   metrics = ppo.ppo_update(model, optimizer, rollout, advantages, returns)  
    
   print("mean reward:", rollout["rewards"].mean().item(),"mean return:", returns.mean().item(),"actor:", metrics["actor_loss"],"critic:", metrics["critic_loss"],"entropy:", metrics["entropy"])
   buffer.clear()
finally:
    native.clear_callbacks()

Path("rl/traffic_rl/checkpoints").mkdir(parents=True, exist_ok=True)

checkpoint_path = Path(__file__).parent / "checkpoints" / "ppo_5tl.pt"
checkpoint_path.parent.mkdir(parents=True, exist_ok=True)

torch.save({"model_state": model.state_dict(),"feature_count": current_obs.shape[1],"max_phases": current_mask.shape[1],},checkpoint_path)

