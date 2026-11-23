"""Human demo simulation assumptions, separate from measured board calibration."""
import json
from pathlib import Path
import numpy as np

WORKSPACE = Path(__file__).resolve().parents[1]/'config/human_demo_workspace.json'


def load_scene(workspace=WORKSPACE):
    import mujoco
    from ffw_sharpa_sim import model as source
    config = json.loads(Path(workspace).read_text())
    height = config['simulation']['table_surface_height_m']
    # Configure only this process's model construction, restoring module defaults.
    originals = {name: getattr(source, name) for name in
                 ('TABLE_HEIGHT', 'TABLE_POS', 'CUBE_INIT', 'TARGET_POS', 'PAN_POS', 'SPATULA_POS')}
    try:
        delta = height-source.TABLE_HEIGHT
        source.TABLE_HEIGHT = height
        source.TABLE_POS = np.array(config['simulation']['table_centre_robot_m'])
        for name in ('CUBE_INIT', 'TARGET_POS', 'PAN_POS', 'SPATULA_POS'):
            setattr(source, name, originals[name]+np.array([0., 0., delta]))
        spec = source.build_spec()
    finally:
        for name, value in originals.items():
            setattr(source, name, value)
    # The original cube-task target overlaps the printed paper.
    spec.delete(spec.geom('target'))
    spec.delete(spec.site('target_site'))
    board = spec.worldbody.add_body(name='human_demo_marker_board',
        pos=config['simulation']['board_centre_robot_m'],
        quat=(np.sqrt(.5), 0, 0, -np.sqrt(.5)))
    board.add_geom(name='human_demo_paper', type=mujoco.mjtGeom.mjGEOM_BOX,
        size=(.105, .1485, .00005), pos=(0,0,-.0001), rgba=(1,1,1,1),
        contype=0, conaffinity=0)
    cell = config['marker_side_m']/6
    for marker_id, centre in config['marker_centres_board_m'].items():
        marker = board.add_body(name='aruco_id_'+marker_id, pos=centre)
        for row, cells in enumerate(config['marker_cells'][marker_id]):
            for col, white in enumerate(cells):
                if not white:
                    marker.add_geom(name=f'aruco_{marker_id}_{row}_{col}',
                        type=mujoco.mjtGeom.mjGEOM_BOX,
                        pos=((col-2.5)*cell, (2.5-row)*cell, 0),
                        size=(cell/2, cell/2, .00002), rgba=(0,0,0,1),
                        contype=0, conaffinity=0)
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key('home').id)
    mujoco.mj_forward(model, data)
    return model, data
