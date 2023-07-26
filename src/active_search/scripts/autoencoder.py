from pathlib import Path
import os
import rospkg
import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import open3d as o3d
from tqdm import trange

# Step 1: Data Preprocessing
def load_voxel_grids(file_paths):
    voxel_grids = []
    grid_size = (40, 40, 40)
    for file_path in file_paths:
        grid = np.zeros(grid_size)
        pcd = o3d.io.read_point_cloud(file_path)
        points = np.asarray(pcd.points).astype(int)
        grid[points[:, 0], points[:, 1], points[:, 2]] = 1
        # voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=0.0075)

        # Expand the voxel grid to have a single channel
        grid = grid[np.newaxis, :, :, :]
        voxel_grids.append(grid)
    return voxel_grids

def normalize_voxel_grids(voxel_grids):
    max_value = np.max(voxel_grids)
    min_value = np.min(voxel_grids)
    normalized_grids = (voxel_grids - min_value) / (max_value - min_value)
    return normalized_grids

# Step 2: Autoencoder Architecture (define the neural network)
class Autoencoder(nn.Module):
    def __init__(self, input_shape, encoding_dim):
        super(Autoencoder, self).__init__()
        # self.encoder = nn.Sequential(
        #     nn.Linear(input_shape, encoding_dim),
        #     nn.ReLU()
        # )
        # self.decoder = nn.Sequential(
        #     nn.Linear(encoding_dim, input_shape),
        #     nn.Sigmoid()
        # )
                # Encoder layers
        self.encoder = nn.Sequential(
            nn.Conv3d(1, 32, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.MaxPool3d(kernel_size=2, stride=2),
            nn.Conv3d(32, 64, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.MaxPool3d(kernel_size=2, stride=2),
            nn.Conv3d(64, 128, kernel_size=3, stride=1, padding=1),
            nn.ReLU()
        )
        # Decoder layers
        self.decoder = nn.Sequential(
            nn.Conv3d(128, 64, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.Upsample(scale_factor=2),
            nn.Conv3d(64, 32, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.Upsample(scale_factor=2),
            nn.Conv3d(32, 1, kernel_size=3, stride=1, padding=1),
            nn.Sigmoid()
        )

    def forward(self, x):
        x = self.encoder(x)
        x = self.decoder(x)
        return x

    def forward(self, x):
        encoded = self.encoder(x)
        decoded = self.decoder(encoded)
        return decoded

# Step 3: Training
# Training function
def train_autoencoder(train_data, val_data, input_shape, encoding_dim, num_epochs, batch_size):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print("Training on:", device)
    autoencoder = Autoencoder(input_shape, encoding_dim).to(device)
    criterion = nn.MSELoss()
    optimizer = optim.Adam(autoencoder.parameters(), lr=0.001)

    train_loader = torch.utils.data.DataLoader(train_data, batch_size=batch_size, shuffle=True)
    val_loader = torch.utils.data.DataLoader(val_data, batch_size=batch_size, shuffle=False)


    t = trange(num_epochs)
    for epoch in t:
        running_loss = 0.0
        for batch in train_loader:  # Loop through batches in DataLoader
            inputs = batch.to(device)
            optimizer.zero_grad()
            outputs = autoencoder(inputs)
            loss = criterion(outputs, inputs)
            loss.backward()
            optimizer.step()
            running_loss += loss.item() * inputs.size(0)
        epoch_loss = running_loss / len(train_loader.dataset)
        # print(f"Epoch {epoch + 1}/{num_epochs}, Loss: {epoch_loss:.4f}")

        # Validation
        val_loss = 0.0
        with torch.no_grad():
            for batch in val_loader:  # Loop through batches in DataLoader
                inputs = batch.to(device)
                outputs = autoencoder(inputs)
                loss = criterion(outputs, inputs)
                val_loss += loss.item() * inputs.size(0)
            val_loss /= len(val_loader.dataset)

        # print(f'Epoch {epoch + 1}/{num_epochs}, Train Loss: {epoch_loss:.4f}, Val Loss: {val_loss:.4f}')
        t.set_description(f'Epoch {epoch + 1}/{num_epochs}, Train Loss: {epoch_loss:.4f}, Val Loss: {val_loss:.4f}')

    return autoencoder

# Step 4: Evaluation
# Evaluation can be done similarly as in the previous pseudo-code.
# Calculate reconstruction error or other relevant metrics on the test dataset.

# Step 5: Encoding and Decoding
# Use the trained autoencoder to encode and decode new voxel grids
def encode_voxel_grids(autoencoder, new_voxel_grids):
    # new_voxel_grids = torch.tensor(new_voxel_grids.reshape(-1, 40*40*40), dtype=torch.float32)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    new_voxel_grids = new_voxel_grids.clone().detach().to(device)
    encoded_voxel = autoencoder.encoder(new_voxel_grids)
    return encoded_voxel

def decode_voxel_grids(autoencoder, encoded_voxel):
    decoded_voxel = autoencoder.decoder(encoded_voxel)
    return decoded_voxel


import os

# Function to get file paths for all .pcd files in the specified directory
def get_pcd_file_paths(directory_path):
    file_paths = []
    for filename in os.listdir(directory_path):
        if filename.endswith(".pcd"):
            file_path = os.path.join(directory_path, filename)
            file_paths.append(file_path)
    return file_paths

def main():

    rospack = rospkg.RosPack()
    pkg_root = Path(rospack.get_path("active_search"))
    data_folder_path = str(pkg_root)+"/training/"
    file_paths = get_pcd_file_paths(data_folder_path)  # List of file paths to your .pcd files
    voxel_grids = load_voxel_grids(file_paths)

    # Convert to PyTorch tensors and flatten the voxel grids
    # voxel_tensors = torch.tensor(np.asarray(voxel_grids).reshape(-1, 40*40*40), dtype=torch.float32)
    voxel_tensors = torch.tensor(np.asarray(voxel_grids), dtype=torch.float32)

    num_data = int(voxel_tensors.shape[0]*0.9)
    data = voxel_tensors[:num_data]
    holdout_data = voxel_tensors[num_data:]

    # Split into training and validation sets
    num_train_samples = int(data.shape[0]*0.8)
    train_data = data[:num_train_samples]
    val_data = data[num_train_samples:]

    # Define autoencoder parameters
    input_shape = 40*40*40
    encoding_dim = 200  # Dimension of the latent representation
    num_epochs = 100
    batch_size = 32

    # Train the autoencoder
    trained_autoencoder = train_autoencoder(train_data, val_data, input_shape, encoding_dim, num_epochs, batch_size)

    # Step 4: Evaluation (similar as before)

    # Step 5: Encoding and Decoding  
    encoded_voxel = encode_voxel_grids(trained_autoencoder, holdout_data)
    decoded_voxel = decode_voxel_grids(trained_autoencoder, encoded_voxel)

if __name__ == "__main__":
    main()