import torch
from torch.distributions import Categorical
import torch.nn.functional as F

class rolloutBuffer:
    def __init__(self):
        self.clear()

    def clear(self):
        self.observations = []
        self.masks = []
        self.actions = []
        self.log_probs = []
        self.values = []
        self.rewards = []
        self.dones = []
        self.pending = False

    def add_decision(self, obs, mask, actions, log_probs, values):
        if self.pending:
            raise RuntimeError("No reward")

        self.observations.append(obs.detach().cpu().clone())
        self.masks.append(mask.detach().cpu().clone())
        self.actions.append(actions.detach().cpu().clone())
        self.log_probs.append(log_probs.detach().cpu().clone())
        self.values.append(values.detach().cpu().clone())

        self.pending = True

    def close_transition(self, rewards, done):
        if not self.pending:
            raise RuntimeError("No action.")

        self.rewards.append(
            torch.tensor(rewards, dtype=torch.float32)
        )
        self.dones.append(done)

        self.pending = False

    def tensors(self):
        if self.pending:
            raise RuntimeError("Ultima tranziție nu are reward.")

        return {
            "obs": torch.stack(self.observations),
            "masks": torch.stack(self.masks),
            "actions": torch.stack(self.actions),
            "log_probs": torch.stack(self.log_probs),
            "values": torch.stack(self.values),
            "rewards": torch.stack(self.rewards),
            "dones": torch.tensor(self.dones, dtype=torch.bool),
        }



@torch.no_grad()
def compute_gae(rollout, gamma=0.995, gae_lambda=0.95):
    rewards = rollout["rewards"]  
    values = rollout["values"]    
    dones = rollout["dones"]   

    advantages = torch.zeros_like(rewards)
    gae = torch.zeros_like(values[0])

    next_value = torch.zeros_like(values[0])

    for step in reversed(range(rewards.shape[0])):
        not_done = 1.0 - dones[step].float()

        delta = (rewards[step]+ gamma * next_value * not_done- values[step])

        gae = (delta+ gamma * gae_lambda * not_done * gae)

        advantages[step] = gae
        next_value = values[step]

    returns = advantages + values

    advantages = (advantages - advantages.mean()) / advantages.std(unbiased=False).clamp_min(1e-8)

    return advantages, returns

def ppo_update(
    model,
    optimizer,
    rollout,
    advantages,
    returns,
    clip_ratio=0.1,
    value_coef=0.5,
    entropy_coef=0.005,
    epochs=4,
    minibatch_size=512):
    device = next(model.parameters()).device

    obs = rollout["obs"].flatten(0, 1).to(device)
    masks = rollout["masks"].flatten(0, 1).to(device)
    actions = rollout["actions"].flatten(0, 1).to(device)
    old_log_probs = rollout["log_probs"].flatten(0, 1).to(device)

    advantages = advantages.flatten(0, 1).to(device)
    returns = returns.flatten(0, 1).to(device)

    total_samples = obs.shape[0]

    actor_losses = []
    critic_losses = []
    entropies = []

    model.train()

    for _ in range(epochs):
        order = torch.randperm(total_samples, device=device)

        for start in range(0, total_samples, minibatch_size):
            indices = order[start:start + minibatch_size]

            logits, values = model(obs[indices],masks[indices],)

            distribution = Categorical(logits=logits)
            new_log_probs = distribution.log_prob(actions[indices])

            ratio = torch.exp(new_log_probs - old_log_probs[indices])

            unclipped = ratio * advantages[indices]

            clipped = torch.clamp(ratio,1.0 - clip_ratio,1.0 + clip_ratio,) * advantages[indices]

            actor_loss = -torch.min(unclipped, clipped).mean()

            critic_loss = F.mse_loss(values,returns[indices],)

            entropy = distribution.entropy().mean()

            loss = (actor_loss+ value_coef * critic_loss- entropy_coef * entropy)

            optimizer.zero_grad()
            loss.backward()

            torch.nn.utils.clip_grad_norm_(model.parameters(),max_norm=0.5,)

            optimizer.step()

            actor_losses.append(actor_loss.item())
            critic_losses.append(critic_loss.item())
            entropies.append(entropy.item())

    return {"actor_loss": sum(actor_losses) / len(actor_losses),"critic_loss": sum(critic_losses) / len(critic_losses), "entropy": sum(entropies) / len(entropies) }