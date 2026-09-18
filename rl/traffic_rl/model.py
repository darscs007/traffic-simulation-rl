import torch
from torch import nn
from torch.distributions import Categorical


class ActorCritic(nn.Module):
    def __init__(self, feature_count, max_phases, hidden_size=128):
        super().__init__()

        self.backbone = nn.Sequential(nn.Linear(feature_count, hidden_size),nn.ReLU(), nn.Linear(hidden_size, hidden_size),nn.ReLU(),)
        self.actor = nn.Linear(hidden_size, max_phases)
        self.critic = nn.Linear(hidden_size, 1)

    def forward(self, obs, action_mask):
        hidden = self.backbone(obs)

        logits = self.actor(hidden)
        logits = logits.masked_fill(~action_mask, -1e9)

        values = self.critic(hidden).squeeze(-1)

        return logits, values

    def act(self, obs, action_mask):
        logits, values = self(obs, action_mask)

        distribution = Categorical(logits=logits)
        actions = distribution.sample()

        return actions, distribution.log_prob(actions), values