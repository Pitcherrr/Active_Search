from pathlib import Path
import os
import rospkg
import open3d as o3d
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader

# Define the Autoencoder class
class Autoencoder3D(nn.Module):
    def __init__(self):
        super(Autoencoder3D, self).__init__()
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

# Define a custom dataset for loading point clouds from .pcd files
class PointCloudDataset(Dataset):
    def __init__(self, root_dir):
        self.root_dir = root_dir
        self.file_list = os.listdir(root_dir)

    def __len__(self):
        return len(self.file_list)

    def __getitem__(self, idx):
        file_name = self.file_list[idx]
        file_path = os.path.join(self.root_dir, file_name)
        point_cloud = self.load_point_cloud(file_path)
        return point_cloud

    def load_point_cloud(self, file_path):
        pcd = o3d.io.read_point_cloud(file_path)
        voxel_data = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, 0.0075)
        points = torch.tensor(voxel_data, dtype=torch.float32).unsqueeze(0)
        return points

# Training function
def train_autoencoder(autoencoder, dataloader, epochs=50, batch_size=32):
    criterion = nn.MSELoss()
    optimizer = optim.Adam(autoencoder.parameters(), lr=0.001)

    for epoch in range(epochs):
        running_loss = 0.0
        for i, data in enumerate(dataloader, 0):
            inputs = data
            optimizer.zero_grad()
            outputs = autoencoder(inputs)
            loss = criterion(outputs, inputs)
            loss.backward()
            optimizer.step()
            running_loss += loss.item()
        print(f"Epoch {epoch + 1}/{epochs}, Loss: {running_loss}")

if __name__ == "__main__":
    # Replace 'data_folder_path' with the folder containing your 1000 .pcd files
    rospack = rospkg.RosPack()
    pkg_root = Path(rospack.get_path("active_search"))
    data_folder_path = str(pkg_root)+"/training/"

    # Create the dataset and dataloader
    dataset = PointCloudDataset(data_folder_path)
    dataloader = DataLoader(dataset, batch_size=32, shuffle=True)

    # Create an instance of the Autoencoder
    autoencoder = Autoencoder3D()

    # Train the autoencoder
    train_autoencoder(autoencoder, dataloader)




# # Create an instance of the Autoencoder
# autoencoder = Autoencoder3D()

# # Print model summary
# print(autoencoder)


# import torch.optim as optim

# # Load your 3D point cloud data into 'data'
# # data.shape should be (number_of_samples, 1, 40, 40, 40)

# # Normalize the data to [0, 1]
# # data = data.astype('float32') / 255.0

# def load_pcd_file(file_path):
#     pcd = o3d.io.read_point_cloud(file_path)
#     return pcd

# rospack = rospkg.RosPack()
# pkg_root = Path(rospack.get_path("active_search"))
# file_dir = str(pkg_root)+"/training/p"+str(1)+".pcd"
# print(file_dir)

# data = load_pcd_file(file_dir)

# # Convert numpy array to torch tensor
# x_train = torch.tensor(data.points)

# # Define loss function and optimizer
# criterion = nn.MSELoss()
# optimizer = optim.Adam(autoencoder.parameters(), lr=0.001)

# # Train the autoencoder
# epochs = 50
# batch_size = 32
# for epoch in range(epochs):
#     running_loss = 0.0
#     for i in range(0, x_train.size(0), batch_size):
#         inputs = x_train[i:i + batch_size]
#         optimizer.zero_grad()
#         outputs = autoencoder(inputs)
#         loss = criterion(outputs, inputs)
#         loss.backward()
#         optimizer.step()
#         running_loss += loss.item()
#     print(f"Epoch {epoch + 1}/{epochs}, Loss: {running_loss}")

# # Test the autoencoder by reconstructing a sample
# sample = x_train[0:1]
# reconstructed_sample = autoencoder(sample)

# # Convert torch tensor to numpy array
# reconstructed_sample = reconstructed_sample.detach().numpy()

# # Print the shape of the reconstructed sample
# print(reconstructed_sample.shape)