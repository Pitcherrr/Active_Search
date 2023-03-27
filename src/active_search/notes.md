## This is a file where I'll write some things I come accross through development

# TSDF's
Was using auniform TSDF volume (open3d.pipelines.integration.UniformTSDFVolume) however this regects depth values in the negative x,y,z space. This was giving issues where I would only see points from the positive axes, Basically showing a quater of the image. ScalableTSDFVolume does not do this. 