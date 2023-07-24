import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import open3d as o3d

# Step 1: Data Preprocessing
def load_voxel_grids(file_paths):
    voxel_grids = []
    for file_path in file_paths:
        pcd = o3d.io.read_point_cloud(file_path)
        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=0.0075)
        voxel_grids.append(voxel_grid)
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
        self.encoder = nn.Sequential(
            nn.Linear(input_shape, encoding_dim),
            nn.ReLU()
        )
        self.decoder = nn.Sequential(
            nn.Linear(encoding_dim, input_shape),
            nn.Sigmoid()
        )

    def forward(self, x):
        encoded = self.encoder(x)
        decoded = self.decoder(encoded)
        return decoded

# Step 3: Training
def train_autoencoder(train_data, val_data, input_shape, encoding_dim, num_epochs, batch_size):
    autoencoder = Autoencoder(input_shape, encoding_dim)
    criterion = nn.MSELoss()
    optimizer = optim.Adam(autoencoder.parameters(), lr=0.001)

    train_loader = torch.utils.data.DataLoader(train_data, batch_size=batch_size, shuffle=True)
    val_loader = torch.utils.data.DataLoader(val_data, batch_size=batch_size, shuffle=False)

    for epoch in range(num_epochs):
        train_loss = 0.0
        for data in train_loader:
            inputs = data.view(-1, input_shape)
            optimizer.zero_grad()
            outputs = autoencoder(inputs)
            loss = criterion(outputs, inputs)
            loss.backward()
            optimizer.step()
            train_loss += loss.item() * inputs.size(0)

        train_loss /= len(train_loader.dataset)

        # Validation
        val_loss = 0.0
        with torch.no_grad():
            for data in val_loader:
                inputs = data.view(-1, input_shape)
                outputs = autoencoder(inputs)
                loss = criterion(outputs, inputs)
                val_loss += loss.item() * inputs.size(0)

            val_loss /= len(val_loader.dataset)

        print(f'Epoch {epoch+1}/{num_epochs}, Train Loss: {train_loss:.4f}, Val Loss: {val_loss:.4f}')

    return autoencoder

# Step 4: Evaluation
# Evaluation can be done similarly as in the previous pseudo-code.
# Calculate reconstruction error or other relevant metrics on the test dataset.

# Step 5: Encoding and Decoding
# Use the trained autoencoder to encode and decode new voxel grids
def encode_voxel_grids(autoencoder, new_voxel_grids):
    new_voxel_grids = torch.tensor(new_voxel_grids.reshape(-1, 40*40*40), dtype=torch.float32)
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

# Main script
file_paths = get_pcd_file_paths("/home/tom/dev_ws/thesis_ws/src/active_search/training")  # List of file paths to your .pcd files
voxel_grids = load_voxel_grids(file_paths)
# normalized_grids = normalize_voxel_grids(voxel_grids)

# Convert to PyTorch tensors and flatten the voxel grids
normalized_tensors = torch.tensor(np.asarray(voxel_grids).reshape(-1, 40*40*40), dtype=torch.float32)

# Split into training and validation sets
num_train_samples = 10
train_data = normalized_tensors[:num_train_samples]
val_data = normalized_tensors[num_train_samples:]

# Define autoencoder parameters
input_shape = 40*40*40
encoding_dim = 200  # Dimension of the latent representation
num_epochs = 100
batch_size = 32

# Train the autoencoder
trained_autoencoder = train_autoencoder(train_data, val_data, input_shape, encoding_dim, num_epochs, batch_size)

# Step 4: Evaluation (similar as before)

# Step 5: Encoding and Decoding
new_voxel_grids = ...  # Load new voxel grids for encoding/decoding
encoded_voxel = encode_voxel_grids(trained_autoencoder, new_voxel_grids)
decoded_voxel = decode_voxel_grids(trained_autoencoder, encoded_voxel)