from pathlib import Path
import sys
import rospkg
from object2urdf import ObjectUrdfBuilder
from pathlib import Path

# Build entire libraries of URDFs
rospack = rospkg.RosPack()
pkg_root = Path(rospack.get_path("active_search"))
object_folder = pkg_root / "assets/occluding_objs/hat"
builder = ObjectUrdfBuilder(object_folder)
builder.build_library(force_overwrite=True, decompose_concave=True, force_decompose=False, center = 'geometric')
