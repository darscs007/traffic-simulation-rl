import torch
import math
from dataclasses import dataclass


@dataclass
class Layout:
    max_in: int
    max_out: int
    max_capacity: int

    @classmethod
    def from_states(cls, states):
        roads = []

        for state in states:
           for road in state.incoming_roads:
              roads.append(road)

           for road in state.outgoing_roads:
              roads.append(road)

        return cls(max_in=max(len(s.incoming_roads) for s in states),max_out=max(len(s.outgoing_roads) for s in states),max_capacity=max((road.capacity for road in roads), default=1))


class stateEncoder:
   def __init__(self, layout, max_cars, episode_steps, green_steps):
      self.layout = layout
      self.max_cars = max_cars
      self.episode_steps = episode_steps
      self.max_phases = layout.max_in + 1
      self.green_steps=green_steps

   def encode_batch(self, states):
      rows=[]
      mask = torch.zeros( (len(states), self.max_phases), dtype=torch.bool)

      for i in range(len(states)):
        state = states[i]
        features =[]

        phase_count = 1 + len(state.incoming_roads)
        phase = [0.0] * self.max_phases
        phase[state.current_phase] = 1.0
        features.extend(phase)

        age_in_windows = state.time_since_phase / self.green_steps

        features.append(math.log1p(age_in_windows)/math.log1p(self.episode_steps / self.green_steps))

        features.append(state.external_queue_size/max(self.max_cars / len(states), 1))

        self.append_roads(features, state.incoming_roads, self.layout.max_in)
        self.append_roads(features, state.outgoing_roads, self.layout.max_out)

        self.append_matrix(features, state.road_change_matrix, states)

        self.append_neighbours(features, state)

        rows.append(features)

        for phase in range(phase_count):
            mask[i, phase] = True

      obs = torch.tensor(rows, dtype=torch.float32)
      return obs, mask

   def append_roads(self, features, roads, max_roads):
      for road in roads:
         occupancy = road.detected_vehicles/ max(road.capacity,1)
         normalized_capacity = road.capacity/ self.layout.max_capacity
         features.extend([occupancy, normalized_capacity])

      features.extend([0.0,0.0] * (max_roads - len(roads)))

   def append_matrix(self, features, matrix, states):
      max_cols = self.layout.max_out +1

      for row_index in range(self.max_phases):
         row=matrix[row_index] if row_index < len(matrix) else []
         row_sum = sum(row)

         for col_index in range(max_cols):
            value = row[col_index] if col_index < len(row) else 0
            features.append (value/ row_sum if row_sum else 0.0)

         features.append(math.log1p(row_sum)/math.log1p(max(self.max_cars / len(states), 1) * self.episode_steps))       

   def append_neighbours(self, features, state):
      for index in range(self.layout.max_out):
         if index < len(state.adj_tl_phases):
            features.append(float(state.adj_tl_phases[index]))
            features.append(math.log1p(state.adj_tl_times[index]/self.green_steps)/ math.log1p(self.episode_steps/self.green_steps))
         else:
            features.extend([0.0, 0.0])
