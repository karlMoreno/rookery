//! Routes between named poses (moved from rookery in row 4.1).

use mast::route::{Pose, RouteError, end_of, route, travel_z};
use mast::{Axis, SceneMeta, cell_pose, home};

const ROOKERY_JSON: &str = include_str!("fixtures/rookery.json");

fn meta() -> SceneMeta {
    SceneMeta::from_json(ROOKERY_JSON).expect("fixture parses")
}

#[test]
fn a_route_reaches_the_engage_pose_and_only_descends_in_front_of_the_handle() {
    let m = meta();
    let cmds = route(home(), Pose::Cell(3, 3), &m).unwrap();
    let goal = cell_pose(3, 3, &m).unwrap();
    assert_eq!(end_of(home(), &cmds).0[..3], goal.0[..3]);
    let descend = cmds
        .iter()
        .position(|c| c.axis == Axis::MastZ && c.goal == goal.0[1])
        .unwrap();
    let before = &cmds[descend - 1];
    assert_eq!(before.axis, Axis::ArmX);
    assert!(
        before.goal < goal.0[2],
        "descends only with the finger in front of the handle"
    );
}

#[test]
fn leaving_a_cell_backs_out_before_rising() {
    let m = meta();
    let at = cell_pose(1, 2, &m).unwrap();
    let cmds = route(at, Pose::Home, &m).unwrap();
    assert_eq!(cmds[0].axis, Axis::ArmX);
    assert!(cmds[0].goal < at.0[2]);
    assert_eq!(cmds[1].axis, Axis::MastZ);
    assert_eq!(end_of(at, &cmds), home());
}

#[test]
fn travel_clears_every_handle() {
    let m = meta();
    assert!(travel_z(&m) > m.derived.exit_pose[2] + m.box_size[2] + m.handle_height);
}

#[test]
fn a_cell_outside_the_grid_is_an_error() {
    assert!(matches!(
        route(home(), Pose::Cell(9, 0), &meta()),
        Err(RouteError::NoSuchCell { r: 9, .. })
    ));
}

#[test]
fn home_to_home_is_empty() {
    assert!(route(home(), Pose::Home, &meta()).unwrap().is_empty());
}

#[test]
fn a_transfer_is_a_job_not_a_route() {
    assert!(matches!(
        route(home(), Pose::Transfer(1, 1), &meta()),
        Err(RouteError::NotARoute(Pose::Transfer(1, 1)))
    ));
}
