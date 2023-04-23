import torch
import numpy as np 
import pybullet as p
import open3d as o3d

from active_grasp.bbox import AABBox
from .dynamic_perception import SceneTSDFVolume
from robot_helpers.spatial import Transform

def get_target_bb(sim, uid):
    target_bb =  o3d.geometry.AxisAlignedBoundingBox()
    # target_bb =  o3d.geometry.OrientedBoundingBox()      
    min_bound, max_bound = p.getAABB(uid)
    
    origin = Transform.from_translation(sim.scene.origin)

    min_bound_t = (Transform.from_translation(min_bound)*origin.inv()).translation
    max_bound_t = (Transform.from_translation(max_bound)*origin.inv()).translation

    target_bb.min_bound = min_bound_t + np.array([0,0,0.1])
    target_bb.max_bound = max_bound_t + np.array([0,0,0.1])

    target_bb.scale(0.8, target_bb.get_center())

    return target_bb


def get_tsdf(sim, reset_tsdf= False):
        
    if reset_tsdf:
        tsdf = SceneTSDFVolume(sim.scene.length, 40)
    
    cam_data = sim.camera.get_image()
    image = cam_data[0]
    depth_img = cam_data[1]

    tsdf.integrate(depth_img, sim.camera.intrinsic, (sim.camera.pose.inv()* Transform.from_translation(sim.scene.origin)).as_matrix()) 

    return tsdf


def get_poi_torch(tsdf, target_bb):
    resolution = tsdf.resolution
    voxel_size = tsdf.voxel_size
    print(resolution)
    print(voxel_size)

    volume = tsdf.o3dvol.extract_volume_tsdf()
    vol_array = np.asarray(volume)

    vol_mat = vol_array[:,0].reshape(resolution, resolution, resolution)
    bb_voxel = np.array(np.floor(target_bb.get_extent()/voxel_size), int)
    print(bb_voxel)
    # bb_voxel = [10,10,10]

    vol_mat = torch.from_numpy(vol_mat).to(torch.device("cuda"))

    occ_mat = torch.zeros_like(vol_mat, device="cuda")
    tsdf_check = occ_mat
    max_tsdf_slices = occ_mat

    tsdf_slices = vol_mat.unfold(0, int(bb_voxel[0]), 1).unfold(1, int(bb_voxel[1]), 1).unfold(2, int(bb_voxel[2]), 1)
    # max_tsdf_slices[0:resolution-bb_voxel[0]+1,0:resolution-bb_voxel[1]+1,0:resolution-bb_voxel[2]+1]  = tsdf_slices.amax(dim=(3, 4, 5))
    max_tsdf_slices = tsdf_slices.amax(dim=(3, 4, 5))
    # print(max_tsdf_slices.shape)

    tsdf_check[0:resolution-bb_voxel[0]+1,0:resolution-bb_voxel[1]+1,0:resolution-bb_voxel[2]+1] = max_tsdf_slices <= 0
    occ_mat[0:resolution, 0:resolution, 0:resolution] = tsdf_check.squeeze().to(dtype=torch.uint8)

    occ_mat_result = occ_mat.cpu().numpy()

    coordinate_mat = np.argwhere(occ_mat_result > 0)

    poi_arr = np.zeros_like(coordinate_mat)

    poi_arr = coordinate_mat*voxel_size+[0.009+round(bb_voxel[0]/2)*voxel_size,0.009+round(bb_voxel[1]/2)*voxel_size,round(bb_voxel[2]/2)*voxel_size]

    # aabb_arr = np.zeros(poi_arr.shape[1])
    aabb_arr = []

    #poi mat is a list of the min bound for a bounding box
    for i in range(poi_arr.shape[1]-1):
        aabb_arr.append(AABBox(poi_arr[i], poi_arr[i]+target_bb.get_extent()))
    return aabb_arr