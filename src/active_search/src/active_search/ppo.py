import rospkg
import os
import torch as T
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import numpy as np

from pathlib import Path
from torch.distributions import Categorical

class Autoencoder(nn.Module):
    def __init__(self):
        super(Autoencoder, self).__init__()

        self.get_path()

        # Encoder layers
        self.encoder = nn.Sequential(

            nn.Conv3d(2, 32, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.MaxPool3d(kernel_size=2, stride=2),

            nn.Conv3d(32, 64, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.MaxPool3d(kernel_size=2, stride=2),

            nn.Conv3d(64, 128, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),

            nn.Flatten(),  # Flatten the 3D tensor into a 1D vector
            nn.Linear(128 * 10 * 10 * 10, 512)
        )

        # Decoder layers
        self.decoder = nn.Sequential(

            nn.Linear(512, 128 * 10 * 10 * 10),  # Map from the latent space back to the decoder input shape
            nn.Unflatten(1, (128, 10, 10, 10)),  # Reshape the tensor back to 4D (batch_size, channels, height, width, depth)
            nn.ReLU(),

            nn.Conv3d(128, 64, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.Upsample(scale_factor=2),

            nn.Conv3d(64, 32, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.Upsample(scale_factor=2),

            nn.Conv3d(32, 2, kernel_size=3, stride=1, padding=1),
            nn.Sigmoid()
        )

    def forward(self, x):
        encoded = self.encoder(x)
        decoded = self.decoder(encoded)
        return decoded 
    
    def get_path(self):
        rospack = rospkg.RosPack()
        pkg_root = Path(rospack.get_path("active_search"))
        self.model_path =  str(pkg_root)+"/models/autoencoder_weights.pth"


class GraspEval(nn.Module):
    def __init__(self):
        super(GraspEval, self).__init__()
        self.fc1 = nn.Linear(527, 256) 
        self.fc2 = nn.Linear(256, 128)  
        self.fc3 = nn.Linear(128, 2)

        self.optimizer = optim.Adam(self.parameters(), lr=0.00005)  
        self.device = T.device('cuda:0' if T.cuda.is_available() else 'cpu')
        self.to(self.device)
        self.checkpoint_file = 'tmp/ppo/grasp_actor_ppo'

    def forward(self, x):
        x = self.fc1(x)
        x = T.relu(x)
        x = self.fc2(x) 
        x = T.relu(x)    
        x = self.fc3(x)
        outputs = T.relu(x)        
        return outputs
    
    def save_checkpoint(self):
        T.save(self.state_dict(), self.checkpoint_file)

    def load_checkpoint(self):
        self.load_state_dict(T.load(self.checkpoint_file))


class ViewEval(nn.Module):
    def __init__(self):
        super(ViewEval, self).__init__()
        self.fc1 = nn.Linear(527, 256) 
        self.fc2 = nn.Linear(256, 128)  
        self.fc3 = nn.Linear(128, 2)   

        self.optimizer = optim.Adam(self.parameters(), lr=0.00005)
        self.device = T.device('cuda:0' if T.cuda.is_available() else 'cpu')
        self.to(self.device)
        self.checkpoint_file = 'tmp/ppo/view_actor_ppo'

    def forward(self, x):
        x = T.relu(self.fc1(x))    
        x = T.relu(self.fc2(x))    
        outputs = T.relu(self.fc3(x))        
        return outputs
    
    def save_checkpoint(self):
        T.save(self.state_dict(), self.checkpoint_file)

    def load_checkpoint(self):
        self.load_state_dict(T.load(self.checkpoint_file))
    
    
class PPOMemory:
    def __init__(self, batch_size):
        self.states = []
        self.probs = []
        self.vals = []
        self.actions = []
        self.reiwards = []
        self.dones = []

        self.batch_size = batch_size

    def generate_batches(self):
        n_states = len(self.states)
        batch_start = np.arange(0, n_states, self.batch_size)
        indices = np.arange(n_states, dtype=np.int64)
        np.random.shuffle(indices)
        batches = [indices[i:i+self.batch_size] for i in batch_start]

        return np.array(self.states),\
                np.array(self.actions),\
                np.array(self.probs),\
                np.array(self.vals),\
                np.array(self.rewards),\
                np.array(self.dones),\
                batches

    def store_memory(self, state, action, probs, vals, reward, done):
        self.states.append(state)
        self.actions.append(action)
        self.probs.append(probs)
        self.vals.append(vals)
        self.rewards.append(reward)
        self.dones.append(done)

    def clear_memory(self):
        self.states = []
        self.probs = []
        self.actions = []
        self.rewards = []
        self.dones = []
        self.vals = []

class GraspActorNetwork(nn.Module):
    def __init__(self, input_dims, alpha,
            fc1_dims=256, fc2_dims=256, chkpt_dir='tmp/ppo'):
        super(GraspActorNetwork, self).__init__()

        self.checkpoint_file = os.path.join(chkpt_dir, 'grasp_torch_ppo')
        self.actor = nn.Sequential(
                nn.Linear(*input_dims, fc1_dims),
                nn.ReLU(),
                nn.Linear(fc1_dims, fc2_dims),
                nn.ReLU(),
                nn.Linear(fc2_dims, 1),
                nn.Softmax(dim=-2)
        )

        self.optimizer = optim.Adam(self.parameters(), lr=alpha)
        self.device = T.device('cuda:-1' if T.cuda.is_available() else 'cpu')
        self.to(self.device)

    def forward(self, state):
        dist = self.actor(state)
        dist = Categorical(dist)
        
        return dist

    def save_checkpoint(self):
        T.save(self.state_dict(), self.checkpoint_file)

    def load_checkpoint(self):
        self.load_state_dict(T.load(self.checkpoint_file))

class ViewActorNetwork(nn.Module):
    def __init__(self, input_dims, alpha,
            fc1_dims=256, fc2_dims=256, chkpt_dir='tmp/ppo'):
        super(ViewActorNetwork, self).__init__()

        self.checkpoint_file = os.path.join(chkpt_dir, 'view_torch_ppo')
        self.actor = nn.Sequential(
                nn.Linear(*input_dims, fc1_dims),
                nn.ReLU(),
                nn.Linear(fc1_dims, fc2_dims),
                nn.ReLU(),
                nn.Linear(fc2_dims, 2),
                nn.Softmax(dim=-2)
        )

        self.optimizer = optim.Adam(self.parameters(), lr=alpha)
        self.device = T.device('cuda:-1' if T.cuda.is_available() else 'cpu')
        self.to(self.device)

    def forward(self, state):
        dist = self.actor(state)
        dist = Categorical(dist)
        
        return dist

    def save_checkpoint(self):
        T.save(self.state_dict(), self.checkpoint_file)

    def load_checkpoint(self):
        self.load_state_dict(T.load(self.checkpoint_file))

class CriticNetwork(nn.Module):
    def __init__(self, input_dims, alpha, fc1_dims=256, fc2_dims=256,
            chkpt_dir='tmp/ppo'):
        super(CriticNetwork, self).__init__()

        self.checkpoint_file = os.path.join(chkpt_dir, 'critic_torch_ppo')
        self.critic = nn.Sequential(
                nn.Linear(input_dims, fc1_dims),
                nn.ReLU(),
                nn.Linear(fc1_dims, fc2_dims),
                nn.ReLU(),
                nn.Linear(fc2_dims, 1)
        )

        self.optimizer = optim.Adam(self.parameters(), lr=alpha)
        self.device = T.device('cuda:0' if T.cuda.is_available() else 'cpu')
        self.to(self.device)

    def forward(self, state):
        value = self.critic(state)

        return value

    def save_checkpoint(self):
        T.save(self.state_dict(), self.checkpoint_file)

    def load_checkpoint(self):
        self.load_state_dict(T.load(self.checkpoint_file))

class Agent:
    def __init__(self, input_dims, gamma=0.99, alpha=0.0003, gae_lambda=0.95,
            policy_clip=0.2, batch_size=64, n_epochs=10):
        self.gamma = gamma
        self.policy_clip = policy_clip
        self.n_epochs = n_epochs
        self.gae_lambda = gae_lambda

        self.view_actor = ViewEval()
        self.grasp_actor = GraspEval()
        self.critic = CriticNetwork(input_dims, alpha)
        self.memory = PPOMemory(batch_size)
       
    def remember(self, state, action, probs, vals, reward, done):
        self.memory.store_memory(state, action, probs, vals, reward, done)

    def save_models(self):
        print('... saving models ...')
        self.actor.save_checkpoint()
        self.critic.save_checkpoint()

    def load_models(self):
        print('... loading models ...')
        self.actor.load_checkpoint()
        self.critic.load_checkpoint()

    def choose_action(self, state, q, grasps, qualities, views, gains):
        # state = T.tensor([observation], dtype=T.float).to(self.actor.device)

        # dist = self.actor(state)
        # value = self.critic(state)
        # action = dist.sample()

        # probs = T.squeeze(dist.log_prob(action)).item()
        # action = T.squeeze(action).item()
        # value = T.squeeze(value).item()

        grasp_q = []
        grasp_t = []
        for grasp, quality in zip(self.grasps, self.qualities):
            grasp_nn_input = T.cat((state, T.tensor([np.concatenate((q, grasp.pose.to_list(), [quality]), dtype=np.float32)]).to(self.device)), 1)
            # print(grasp_nn_input.shape)
            # From here evaluate the actions through the network
            grasp_val = self.grasp_actor(grasp_nn_input.squeeze())
            # print("grasp val", grasp_val)
            grasp_q.append(grasp_val[0])
            grasp_t.append(grasp_val[1]) 

        view_q = []
        view_t = []
        for view, info_gain in zip(views, gains):
            view_nn_input = T.cat((state, T.tensor([np.concatenate((q, view.to_list(), [info_gain]), dtype=np.float32)]).to(self.device)), 1)
            # print(view_nn_input.shape)
            # From here evaluate the actions through the network 
            view_val = self.view_actor(view_nn_input.squeeze())
            # print("view val", view_val)
            view_q.append(view_val[0])
            view_t.append(view_val[1])

        # output is a set of actions with values and estimated completion time

        # Combine the grasp and view probabilities
        if len(grasp_q) > 0:
            combined_probabilities = F.softmax(T.cat((T.stack(grasp_q), T.stack(view_q)), dim=0), dim=0)

        else:
            combined_probabilities = F.softmax(T.stack(view_q), dim=0)
        # print(combined_probabilities)
 
        action_dist = T.distributions.Categorical(combined_probabilities)
        selected_action_index = action_dist.sample()
        action_lprob = action_dist.log_prob(selected_action_index)

        print("Selected index", selected_action_index)

        print("Action prob", action_lprob)

        grasp, view = False, False
        # Based on the selected index, determine whether it's a grasp or a view
        if selected_action_index < len(self.grasps):
            print("Grasp")
            grasp = True
            selected_action = self.grasps[selected_action_index]
            time_est = grasp_t[selected_action_index]
        else:
            print("View")
            view = True
            selected_action = views[selected_action_index - len(self.grasps)]
            time_est = view_t[selected_action_index - len(self.grasps)]

        critic_input = T.cat((state, T.tensor(q).to(self.device)), 1)
        value = self.critic(critic_input.squeeze())

        return selected_action, action_lprob, value

    def learn(self):
        for _ in range(self.n_epochs):
            state_arr, action_arr, old_prob_arr, vals_arr,\
            reward_arr, dones_arr, batches = \
                    self.memory.generate_batches()

            values = vals_arr
            advantage = np.zeros(len(reward_arr), dtype=np.float32)

            for t in range(len(reward_arr)-1):
                discount = 1
                a_t = 0
                for k in range(t, len(reward_arr)-1):
                    a_t += discount*(reward_arr[k] + self.gamma*values[k+1]*\
                            (1-int(dones_arr[k])) - values[k])
                    discount *= self.gamma*self.gae_lambda
                advantage[t] = a_t
            advantage = T.tensor(advantage).to(self.actor.device)

            values = T.tensor(values).to(self.actor.device)
            for batch in batches:
                states = T.tensor(state_arr[batch], dtype=T.float).to(self.actor.device)
                old_probs = T.tensor(old_prob_arr[batch]).to(self.actor.device)
                actions = T.tensor(action_arr[batch]).to(self.actor.device)

                dist = self.actor(states)
                critic_value = self.critic(states)

                critic_value = T.squeeze(critic_value)

                new_probs = dist.log_prob(actions)
                prob_ratio = new_probs.exp() / old_probs.exp()
                #prob_ratio = (new_probs - old_probs).exp()
                weighted_probs = advantage[batch] * prob_ratio
                weighted_clipped_probs = T.clamp(prob_ratio, 1-self.policy_clip,
                        1+self.policy_clip)*advantage[batch]
                actor_loss = -T.min(weighted_probs, weighted_clipped_probs).mean()

                returns = advantage[batch] + values[batch]
                critic_loss = (returns-critic_value)**2
                critic_loss = critic_loss.mean()

                total_loss = actor_loss + 0.5*critic_loss
                self.actor.optimizer.zero_grad()
                self.critic.optimizer.zero_grad()
                total_loss.backward()
                self.actor.optimizer.step()
                self.critic.optimizer.step()

        self.memory.clear_memory()   