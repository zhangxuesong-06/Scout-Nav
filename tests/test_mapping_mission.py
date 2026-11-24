from scout_sim.mapping_mission import COURSE, Mission, fresh_vision, simulation_domain


def test_follow_takeover_stops_on_loss_then_resumes_the_unfinished_waypoint():
    mission = Mission()
    mission.step(0., (0., 0., 0.), None, True)
    pose = (2.4, 0., 1.57)
    assert mission.step(1., pose, None, True)[0] == 'waypoint_reached'
    tracking = dict(selected_id=2, control_state='following', command=[.25, -.50])
    assert mission.step(2., pose, tracking, True) == ('target_acquired', 0., 0.)
    assert mission.step(2.05, pose, tracking, True) == ('following', .14, -.35)
    assert mission.step(2.1, pose, dict(tracking, control_state='distance_hold'), True) == (
        'distance_hold', 0., 0.)
    assert mission.step(3., pose, dict(tracking, control_state='target_missing'), True) == (
        'target_lost', 0., 0.)
    assert mission.step(3.2, pose, None, True) == ('stopping_before_mapping', 0., 0.)
    state, speed, _ = mission.step(3.8, pose, None, True)
    assert state == 'mapping' and speed > 0 and mission.waypoint == 1


def test_lap_alone_does_not_report_a_loop_closure():
    mission = Mission()
    t = 0
    mission.step(t, (0., 0., 0.), None, True)
    mission.record_closure((0., 0., 0.), 2, 1)
    assert mission.closure is None  # At spawn, before travelling the route.
    for lap in (1, 2):
        for xy in COURSE:
            t += 1
            assert mission.step(t, (*xy, 0.), None, True)[0] == 'waypoint_reached'
        assert mission.laps == lap
        assert mission.step(t+.1, (.32, 0., 0.), None, True)[0] == (
            'mapping' if lap == 1 else 'loop_not_confirmed')


def test_only_a_near_start_slam_match_can_confirm_the_route():
    mission = Mission()
    mission.step(0., (0., 0., 0.), None, True)
    for t, xy in enumerate(COURSE[:-1], 1):
        mission.step(t, (*xy, 0.), None, True)
    mission.record_closure((2.4, 1.8, 0.), 99, 10)
    mission.record_closure((0., 0., 1.5), 100, 9)
    assert mission.closure is None
    mission.record_closure((.3, 0., .3), 101, 5)
    assert mission.step(5., (.32, 0., 0.), None, True)[0] == 'waypoint_reached'
    assert mission.step(5.05, (.32, 0., 0.), None, True) == ('loop_closed', 0., 0.)
    assert mission.closure == (101, 5)
    assert mission.step(5.1, (.32, 0., 0.), None, True) == ('wander_blocked', 0., 0.)


def test_old_or_future_images_cannot_keep_the_simulation_moving():
    assert fresh_vision(5., 20., 4.9, 19.9)
    assert not fresh_vision(5., 20., 4.3, 19.9)  # Clock paused; no newer message.
    assert not fresh_vision(5., 20., 4.9, 19.)  # Newly received old frame.
    assert not fresh_vision(5., 20., 4.9, 21.)  # Old simulation's frame after reset.
    mission = Mission()
    vision = dict(selected_id=1, control_state='following', command=[.1, 0.])
    pose = (0., 0., 0.)
    mission.step(4., pose, vision, True)
    assert mission.step(4.1, pose, vision, True)[1] > 0
    assert mission.step(5., pose, vision, fresh_vision(5., 20., 4.1, 19.)) == (
        'waiting_for_inputs', 0., 0.)


def test_mapping_requires_a_reserved_simulation_domain(monkeypatch):
    monkeypatch.delenv('ROS_DOMAIN_ID', raising=False)
    assert not simulation_domain()
    monkeypatch.setenv('ROS_DOMAIN_ID', '0')
    assert not simulation_domain()
    monkeypatch.setenv('ROS_DOMAIN_ID', '42')
    assert simulation_domain()
