from .search_policy import register
from active_grasp.baselines import *
from .nbv_search import NextBestView

register("initial-view", InitialView)
register("top-view", TopView)
register("top-trajectory", TopTrajectory)
register("fixed-trajectory", FixedTrajectory)
register("nbv", NextBestView)