import pandas as pd
import torch 
import torch.nn as nn
import torch.optim as optim
import numpy as np

import struct
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import RegularGridInterpolator


device = torch.accelerator.current_accelerator().type if torch.accelerator.is_available() and torch.accelerator.current_accelerator() is not None else "cpu"

class SlipEstimator(nn.Module):
    def __init__(self, input_dim):
        super().__init__()
        self.flatten = nn.Flatten()

        self.lstm = nn.LSTM(input_dim, hidden_size=128, batch_first=True,dtype=torch.float32)
        self.mu_r_net = nn.Sequential(
            # nn.BatchNorm1d(input_dim),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 16),
            nn.ReLU(),
            nn.Linear(16, 1),
            nn.Sigmoid()
        )
        self.mu_l_net = nn.Sequential(
            # nn.BatchNorm1d(input_dim),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 16),
            nn.ReLU(),
            nn.Linear(16, 1),
            nn.Sigmoid()
        )

    def forward(self, x):
        output, (hidden_state, cell_state) = self.lstm(x)
        return self.mu_r_net(output), self.mu_l_net(output)

class SimDataset(torch.utils.data.Dataset):
    def __init__(self, file_path : str, seq_len = 500):
        self.df = pd.read_csv(file_path)
        self.seq_len = seq_len

        self.data = self.df.to_numpy()
        self.final_idx = len(self.data) - self.seq_len - 1
    
    def __len__(self):
        return self.final_idx
    
    def __getitem__(self, idx):
        X_window = self.data[idx : idx + self.seq_len]
        Y_window = self.data[idx + 1 : idx + self.seq_len + 1]

        X_tensor = torch.tensor(X_window, dtype=torch.float32)
        Y_tensor = torch.tensor(Y_window, dtype=torch.float32)

        state_delta = Y_tensor[:,1:10] - X_tensor[:,1:10]
        controls = X_tensor[:, 19:23]
        return torch.cat([state_delta, controls], dim=1), X_tensor, Y_tensor
    
    def get_true_mu_vals(self):
        return self.df.iloc[:,-2:]

def train(dataloader, model, loss_fn, optimizer):
    size = len(dataloader)
    model.train()

    for batch, (model_in, X,Y) in enumerate(dataloader):
        model_input = model_in.to(device)
        X_seq, Y_seq = X.to(device), Y.to(device)

        pred = model(model_in)
        l_r, l_l = loss_fn(pred, X_seq, Y_seq)
        total_loss = l_r + l_l

        optimizer.zero_grad()
        total_loss.backward()
        optimizer.step()

        if batch % 100 == 0:
            current = (batch + 1) * len(X)
            print(f"loss: {total_loss.item()}  [{current}/{size}]")
        if batch > 500:
            break

def test(dataloader, model, loss_fn):
    num_batches = len(dataloader)
    model.eval()
    test_loss = 0.0
    all_mu_r, all_mu_l = [], []

    with torch.no_grad():
        for batch, (model_in, X, Y) in enumerate(dataloader):
            model_in = model_in.to(device)
            x,y = X.to(device), Y.to(device)

            mu_r,mu_l = model(model_in)
            l_r, l_l = loss_fn((mu_r, mu_l), x, y)
            test_loss += (l_r + l_l).item()
            all_mu_r.append(mu_r.cpu())
            all_mu_l.append(mu_l.cpu())

            if batch % 100 == 0: print(f"Testing... # {batch}")
            if batch > 500: break
        test_loss /= num_batches
        print(f"Test loss: {test_loss}\n")

    mu_r_vals = torch.cat(all_mu_r, dim=0)[:, -1, 0].numpy()
    mu_l_vals = torch.cat(all_mu_l, dim=0)[:, -1, 0].numpy()
    plt.plot(mu_r_vals, label='mu_r')
    plt.plot(mu_l_vals, label='mu_l')
    act = np.asarray(dataset.get_true_mu_vals())[:-1]
    plt.plot(act[9:, 0], label='mu_r actual')
    plt.plot(act[9:, 1], label='mu_l actual')
    plt.legend()
    plt.show()

def load_terrain(path: Path):
    with path.open("rb") as handle:
        magic, rows, cols, x_min, x_max, y_min, y_max = struct.unpack("<8sII4d", handle.read(48))
        z_grid = np.fromfile(handle, dtype=np.float64, count=rows * cols).reshape(rows, cols)

    x_axis = np.linspace(x_min, x_max, cols)
    y_axis = np.linspace(y_min, y_max, rows)
    interp_func = RegularGridInterpolator((y_axis, x_axis), z_grid, bounds_error=False, fill_value=0)
    
    return interp_func


env = load_terrain(Path("terrain.bin"))
rover_dims = {
    'r' : 0.17 / 2,
    'B' : 0.2,
    'L' : 0.25
}
dt = 0.01


def sim_func(state, control_input, env, dt, rover_dims, mu_r, mu_l):
    r = rover_dims['r']
    B = rover_dims['B']
    L = rover_dims['L']

    x_pos = state[:, 0]
    y_pos = state[:, 1]
    pitch = state[:, 7]
    yaw = state[:, 8]

    w_r = control_input[:, 0]
    w_l = control_input[:, 1]
    dw_r = control_input[:, 2]
    dw_l = control_input[:, 3]

    v = (r / 2.0) * (mu_r * w_r + mu_l * w_l)

    vx = v * torch.cos(yaw) * torch.cos(pitch)
    vy = v * torch.sin(yaw) * torch.cos(pitch)
    vz = v * torch.sin(pitch)

    dv = (r / 2.0) * (mu_r * dw_r + mu_l * dw_l) / dt

    body_wheels = torch.tensor([
        [L / 2.0, L / 2.0, -L / 2.0, -L / 2.0],
        [-B / 2.0, B / 2.0, -B / 2.0, B / 2.0]
    ], dtype=torch.float32,device=state.device)

    eps_p = 1e-6

    def att(xq, yq, yq_yaw):
        Rq = torch.stack(
            [torch.cos(yq_yaw), -torch.sin(yq_yaw), torch.sin(yq_yaw),  torch.cos(yq_yaw)], dim=1)
        Rq = Rq.reshape(-1,2,2)


        wq = Rq @ body_wheels + torch.stack([xq, yq], dim=1).unsqueeze(-1)
        wq_transpose = wq.transpose(1,2)
        points = wq_transpose.flip(-1).reshape(-1,2).detach().cpu().numpy()
        zq_np = env(points)
        zq = torch.tensor(zq_np, dtype=torch.float32, device=state.device).reshape(-1, 4).unsqueeze(-1)

        ones_col = torch.ones(wq.shape[0], 4, 1, dtype=torch.float32, device=state.device)
        Aq = torch.concat([wq_transpose, ones_col], dim=2)
        
        lstsq_result = torch.linalg.lstsq(Aq, zq)
        abcq = lstsq_result.solution
        
        a = abcq[:,0,0]
        b = abcq[:,1,0]

        ab = torch.cos(yq_yaw) * a + torch.sin(yq_yaw) * b
        bb = -torch.sin(yq_yaw) * a + torch.cos(yq_yaw) * b
        
        pq = torch.atan(ab)
        rq = torch.atan(bb)
        return rq, pq

    dyaw = (r / B) * (mu_r * w_r - mu_l * w_l)
    
    att_x_plus   = att(x_pos + eps_p, y_pos, yaw)
    att_x_minus  = att(x_pos - eps_p, y_pos, yaw)
    att_y_plus   = att(x_pos, y_pos + eps_p, yaw)
    att_y_minus  = att(x_pos, y_pos - eps_p, yaw)
    att_yaw_plus = att(x_pos, y_pos, yaw + eps_p)
    att_yaw_minus= att(x_pos, y_pos, yaw - eps_p)

    droll = (att_x_plus[0] - att_x_minus[0]) / (2.0 * eps_p) * vx + \
            (att_y_plus[0] - att_y_minus[0]) / (2.0 * eps_p) * vy + \
            (att_yaw_plus[0] - att_yaw_minus[0]) / (2.0 * eps_p) * dyaw

    dpitch = (att_x_plus[1] - att_x_minus[1]) / (2.0 * eps_p) * vx + \
             (att_y_plus[1] - att_y_minus[1]) / (2.0 * eps_p) * vy + \
             (att_yaw_plus[1] - att_yaw_minus[1]) / (2.0 * eps_p) * dyaw

    ax = torch.cos(yaw) * torch.cos(pitch) * dv - v * torch.cos(pitch) * torch.sin(yaw) * dyaw - v * torch.cos(yaw) * torch.sin(pitch) * dpitch
    ay = torch.sin(yaw) * torch.cos(pitch) * dv + v * torch.cos(pitch) * torch.cos(yaw) * dyaw - v * torch.sin(yaw) * torch.sin(pitch) * dpitch
    az = torch.sin(pitch) * dv + v * torch.cos(pitch) * dpitch

    return torch.stack([vx, vy, vz, ax, ay, az, droll, dpitch, dyaw], dim=1)


def wrap_pi(x):
    return (x + torch.pi) % (2 * torch.pi) - torch.pi


# def loss(predicted_mu, xn, xn_1):
#     # we're going to take every xn in the batch, integrate it forward by the seqlen, and compare the result from the 
#     # associated predicted mu to the xn_1 term 

#     mu_r_pred, mu_l_pred = predicted_mu
#     batch_size = xn.shape[0]
#     seq_len = xn.shape[1]

#     state = xn[:, 0, 1:10].clone()
#     total_loss = 0
    
#     for t in range(seq_len):
#         # integrate the dynamics from state
#         mu_r = mu_r_pred[:,t,0]
#         mu_l = mu_l_pred[:,t,0]
#         controls = xn[:,t,19:23]

#         state = state + sim_func(state, controls, env, dt, rover_dims, mu_r, mu_l) * dt

#         total_loss = total_loss + torch.mean((state - xn_1[:,t,1:10]) ** 2)

#     return total_loss, total_loss
def loss(predicted_mu, xn, xn_1):
    mu_r_pred, mu_l_pred = predicted_mu
    seq_len = xn.shape[1]
    r = rover_dims['r']
    B = rover_dims['B']

    x = xn[:, 0, 1].clone()
    y = xn[:, 0, 2].clone()
    yaw = xn[:, 0, 9].clone()

    total_loss = 0

    for t in range(seq_len):
        mu_r = mu_r_pred[:, t, 0]
        mu_l = mu_l_pred[:, t, 0]
        w_r = xn[:, t, 19]
        w_l = xn[:, t, 20]

        v = (r / 2) * (mu_r * w_r + mu_l * w_l)
        dyaw = (r / B) * (mu_r * w_r - mu_l * w_l)

        x = x + v * torch.cos(yaw) * dt
        y = y + v * torch.sin(yaw) * dt
        yaw = yaw + dyaw * dt

        total_loss = total_loss + torch.mean(
            (x - xn_1[:, t, 1]) ** 2 +
            (y - xn_1[:, t, 2]) ** 2 +
            wrap_pi(yaw - xn_1[:, t, 9]) ** 2
        )

    return total_loss, total_loss
# for idx in range(dataset.final_idx):
#     # merge keys with data into dict
#     item1 = dataset.df.iloc[idx].to_list()
#     item2 = dataset.df.iloc[idx+1].to_list()

#     curr_state = torch.tensor(item1[1:10])
#     curr_control = torch.tensor(item1[19:23])

#     next_state_actual = torch.tensor(item1[1:10])
#     mu_range = np.arange(0.1, 1.0, 0.05)
#     diff = []
#     mu_pairs = []
#     from itertools import product
#     for mu_r, mu_l in product(mu_range, mu_range):
#         next_state_pred = curr_state + dt * sim_func(curr_state, curr_control, env, dt, rover_dims, mu_r, mu_l)
#         diff.append((next_state_pred - next_state_actual).detach().numpy())
#         mu_pairs.append((mu_r, mu_l))
        

#     diff = np.asarray(diff)
#     # i want to plot each col and row
#     labels = ['x', 'y', 'z', 'vx', 'vy', 'vz', 'roll', 'pitch', 'yaw']

#     fig, axes = plt.subplots(3, 3, figsize=(14, 10))
#     for i, ax in enumerate(axes.flat):
#         resid = (diff[:, i] ** 2).reshape(len(mu_range), len(mu_range))
#         im = ax.imshow(resid, extent=[mu_range[0], mu_range[-1], mu_range[-1], mu_range[0]], aspect='auto')
#         ax.set_title(labels[i])
#         ax.set_xlabel('mu_l')
#         ax.set_ylabel('mu_r')
#         fig.colorbar(im, ax=ax)

#     plt.suptitle(f"Per-channel squared residual vs (mu_r, mu_l) @ t={item1[0]}")
#     plt.tight_layout()
#     plt.show()

input_dim = 13
model = SlipEstimator(input_dim).to(device)
optimizer = optim.Adam(model.parameters(), lr=1e-3)
dataset = SimDataset("full_state_output.csv")
dataloader = torch.utils.data.DataLoader(dataset,batch_size=8, shuffle=True)

loss_fn = loss
epochs = 20
for t in range(epochs):
    print(f"Epoch {t+1}\n-------------------------------")
    train(dataloader, model, loss_fn, optimizer)
    eval_loader = torch.utils.data.DataLoader(dataset, batch_size=8, shuffle=False)
    test(eval_loader, model, loss_fn)

torch.save(model.state_dict(), "slip_model.pth")
torch.save(model.state_dict(), "slip_model.pth")
'''
Neural Networks are an advanced pattern matching algorithm, that uses gradient
descent to adjust weights to learn an arbitrary mapping. 

The rule though is that the mapping, th einput and output data sets, must
have some analytical, or semi-analytical law, or at the very least a regressive
law that approximates the trend. 

If there is no trend, in that output B and input A are turly independent from each other,
then the neural network will just learn a random number generator, that will yield spurious 
results. 

In the case of slip estimation, we know for certain that the law that is applied is V/wr. 

Yet, in using this equation to approximate a prior estimation in wheel slip, we inevitably 
run into incorrect approximations due to heavy noise. We cannot necessarily also use the previous
time step's estimation of velocity and the current time step's estimation of wheel speed either. 

Consider that in division with p,q << 1 in R and s = p/q, and s' = p'/q'. We intend to approximate s' 
without knowing q' but assuming that q is approximately q', when in reality q' = q + epsilon for epsilon > 0.

then the relative error is |p'/q' - p'/q| * |q'|/|p'| 

since q' is in reality q + epsilon the relative error is |epsilon * p' / (q' * q)| * |q' / p'| = |epsilon| / |q|.
When the error term epsilon is not necessarily the same order of q, our relative error explodes. To ameliorate this,
we may put an additional bound on our control decimation and run the controller at a far quicker refresh rate. Yet still
the true slip ratio is obscured by noise in the velocity determination and is bounded by hardware restrictions. The same
logic now applies when the estimation of p and p' are noisy and deviate from ground truth. 

We desire a system that can determine the true slip ratio using estimations of the slip ratio from the noisy measurement method and
the state, as well as the control. 

We require a loss function that implictly embeds the dynamics used to compute the slip ratio but also uses the noisy estimation of the 
slip ratio as a weak ruler, akin to a Kalman Filter. The issue with the Kalman Filter approach is that there is no method of propagating 
the slip ratio values as a derivative without increasing the state to include angular accelerations which may need finite difference methods
which are incredibly noisy for estimated measurements. 

There are two possibilities of mappings: mu <- state, or mu' <- state dynamics. In lieu of mu' which may require state dynamics we do not have access
to,  we formulate our problem around mu <- state. 

Let [mu1, mu2] = f(x,u,t)_prior subject to the dynamical equations x' = f(x,u,mu,t). We desire a loss function that mixes the mercurical slip estimation 
mu' with the system output of mu, and use that to approximate a blending between the estimated mu and the found mu. 

this would be all fine if it wasn't for the fact that V / rw is the wrong mapping; a mu function that is strictly positive definite can never 
be represented. Consider for example a mu mapping such that the wheel spin rate being 0 yields a finite value. V/rw will explode in error. 

We thus require a two step process that can verify a "latent" estimation of mu that matches the change in dynamics rather than use these poor estimations of 
wheel slip. 

Suppose we feed in some X with u,du into the NN, we require the neural network to spit out Xn with mu being a latent state. 

Nevermind this requires the network to learn an arbitrary terrain. FUCK. 

WAIT. we have the loss function be computed by how that mu value interacts with the current state and is propagated forward. So internally we take the 
mu guess, we propagate a step into the future, and then compare what the predicted dynamics are with the actual dynamics, and then use the delta in the state
to back prop. 

I also cant use the same network to generate both mu_r and mu_l since they might be different relations but the network uses the same gradient to find both. inevitably we 
just find that they're both basically the same. 

'''