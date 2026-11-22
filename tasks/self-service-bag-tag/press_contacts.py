"""GPU-native PhysX contact samples between pairs of tag panels (paper to paper only)."""
import numpy as np

from tag_dims import SEGMENTS
_VIEW = None


def reset():
    global _VIEW
    _VIEW = None


def array(value):
    if hasattr(value, 'detach'):
        return value.detach().cpu().numpy()
    return np.asarray(value.numpy() if hasattr(value, 'numpy') else value)


def read():
    """{(a, b): [(point, unit normal, impulse vector)], a < b}. Caller divides impulses by the physics dt."""
    global _VIEW
    if _VIEW is None:
        from isaaclab.sim.utils.stage import get_current_stage
        from isaaclab_physx.physics.physx_manager import PhysxManager
        from pxr import UsdPhysics
        paths = {}
        for p in get_current_stage().Traverse():
            if p.HasAPI(UsdPhysics.RigidBodyAPI):
                for i in range(SEGMENTS):
                    if f'/tag_panel_{i:02d}/body' in str(p.GetPath()):
                        paths[i] = str(p.GetPath())
        if set(paths) != set(range(SEGMENTS)):
            raise RuntimeError('Missing tag panel contact bodies')
        sim_view = PhysxManager.get_physics_sim_view()
        # One view per panel a, filtered to the higher-numbered panels b > a.
        _VIEW = [(a, list(range(a + 1, SEGMENTS)), sim_view.create_rigid_contact_view(
                     [paths[a]], filter_patterns=[[paths[b] for b in range(a + 1, SEGMENTS)]],
                     max_contact_data_count=512))
                 for a in range(SEGMENTS - 1)]
    out = {}
    for a, others, view in _VIEW:
        # dt=1 returns impulses; caller divides by the actual physics dt.
        force, point, normal, separation, count, start = map(array, view.get_contact_data(dt=1.))
        count = count.reshape(-1)
        start = start.reshape(-1)
        if count.size != len(others):
            raise RuntimeError(f'Unexpected contact layout for panel {a}: {count.size} vs {len(others)}')
        for k, b in enumerate(others):
            items = []
            for i in range(int(start[k]), int(start[k] + count[k])):
                if not np.isfinite(force[i]).all():
                    raise RuntimeError('Nonfinite tag contact')
                items.append((point[i], normal[i], normal[i] * float(np.asarray(force[i]).reshape(-1)[0])))
            if items:
                out[(a, b)] = items
    return out
