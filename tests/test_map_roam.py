import numpy as np

from scout_sim.map_roam import FreeMap
from scout_sim.mapping_mission import Mission


def make_map(cells):
    return FreeMap(cells.ravel(), cells.shape[1], cells.shape[0], .05, (-1.5, -1.5, 0.))


def test_roam_stays_in_known_free_space_and_cannot_cross_wall():
    cells = np.zeros((60, 100), dtype=np.int8)
    cells[:, 50] = 100
    cells[48:, :] = -1
    grid = make_map(cells)
    path = grid.route((0., 0., 0.), None)
    assert path and all(x < .56 and y < .46 for x, y in path)
    assert not grid.clear((0., 0.), (1.7, 0.))
    assert not grid.clear((0., 0.), (0., .85))
    assert not grid.available(grid.cell((.65, 0.)))


def test_closed_map_follow_loss_resumes_wander_and_new_obstacle_stops():
    mission = Mission()
    mission.closed = True
    pose = (0., 0., 0.)
    cells = np.zeros((70, 105), dtype=np.int8)
    mission.grid = make_map(cells)
    state, speed, _ = mission.step(0., pose, None, True)
    assert state == 'wandering' and 0 < speed <= .12
    assert mission.roam.goal and mission.roam.goal[0] > 1.
    target = {'selected_id': 1, 'control_state': 'following', 'command': [.1, 0.]}
    assert mission.step(.1, pose, target, True) == ('target_acquired', 0., 0.)
    assert mission.step(.15, pose, target, True) == ('following', .1, 0.)
    assert mission.step(.2, pose, dict(target, control_state='target_missing'), True) == ('target_lost', 0., 0.)
    assert mission.step(.25, pose, None, True) == ('stopping_before_wander', 0., 0.)
    assert mission.step(.8, pose, None, True)[0] == 'wandering'
    cells[30, 34] = 100
    mission.grid = make_map(cells)
    assert mission.step(.85, pose, None, True) == ('wander_blocked', 0., 0.)
    assert mission.step(.9, pose, target, True) == ('target_acquired', 0., 0.)
    assert mission.step(.95, pose, target, True) == ('follow_blocked_by_map', 0., 0.)
