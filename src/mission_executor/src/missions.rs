use nalgebra::{
    Point3, UnitQuaternion, Vector2, Vector3, Vector6
};

use crate::{
    MapObject, Mission, MissionExecutor, ObjectCls,
};
use std::{sync::{atomic::Ordering, Arc}, thread::sleep, time::Duration};

// this could hold some state if necessary
// like the some sort of queue if a sequence of reactions is necessary
pub(crate) struct PrecualifyMission {}

const FAR_ENOUGH: f64 = 2.0;
const OVERSHOOT: f64 = 2.0;

impl PrecualifyMission {
    pub(crate) fn new() -> Self {
        Self {}
    }

    fn go_around(&self, td: &MissionExecutor, idx: usize) {
        let object = MapObject::from(&td.map.load().objects[idx]);
        let object_pos = object.bbox.center.pos;
        let object_rot = object.bbox.center.rot;

        // corners of a square centered in 0, with area 1
        let square_corners: [Vector2<f64>; 4] = [
            Vector2::new(0.5, 0.5),
            Vector2::new(0.5, -0.5),
            Vector2::new(-0.5, -0.5),
            Vector2::new(-0.5, 0.5),
        ];

        // absolute distance from any corner of the object
        const DISTANCE_TO_CORNER: f64 = 2.0;
        // a big number so that any corner is always closer
        const HUGE_NUMBER: f64 = 10000000.0;

        let mut corner_pluss = [Vector3::new(0.0, 0.0, 0.0); 4];
        let mut starting_corner = Vector2::new(HUGE_NUMBER, HUGE_NUMBER);
        let mut starting_i = usize::MAX;
        let initial_sub_pos = td.pose.load().pos;
        for (i, square_corner) in square_corners.iter().enumerate() {
            let sub_pose = td.pose.load();
            let rot_unit = UnitQuaternion::from_quaternion(object_rot);
            let actual_corner = square_corner.component_mul(&object.bbox.size.xy());
            let rotated_corner = (rot_unit * actual_corner.push(0.0)).xy();
            let pos2d = Vector2::new(object_pos.x, object_pos.y);
            let sub2d = Vector2::new(sub_pose.pos.x, sub_pose.pos.y);
            let corner_plus =
                pos2d + rotated_corner + rotated_corner.normalize() * DISTANCE_TO_CORNER;
            corner_pluss[i] = Vector3::new(corner_plus.x, corner_plus.y, sub_pose.pos.z);

            if (corner_plus - sub2d).norm() < starting_corner.norm() {
                starting_i = i;
                starting_corner = corner_plus;
            }
        }

        for i in 0..corner_pluss.len() {
            td.move_to(corner_pluss[(starting_i + i) % corner_pluss.len()]);
        }
        td.move_to(initial_sub_pos);
    }

    fn go_through(&self, td: &MissionExecutor, idx: usize) {
        let sub_pose = td.pose.load();
        let map = td.map.load();
        let object = MapObject::from(&map.objects[idx]);

        let object_pos = object.bbox.center.pos;
        let object_pos_2d = object_pos.xy();
        let sub_pos_2d = sub_pose.pos.xy();

        let direction_2d = (object_pos_2d - sub_pos_2d).normalize();

        let before_2d = object_pos_2d - direction_2d * FAR_ENOUGH;
        let before = Vector3::new(before_2d.x, before_2d.y, object_pos.z);

        r2r::log_info!("before object_pos", "{object_pos_2d:?}");
        r2r::log_info!("before sub_pos", "{sub_pos_2d:?}");
        r2r::log_info!("before", "{before:?}");

        let overshoot_2d = object_pos_2d + direction_2d * OVERSHOOT;
        let overshoot = Vector3::new(overshoot_2d.x, overshoot_2d.y, object_pos.z);

        let point_list: Vec<Point3<f64>> = vec![before.into(), overshoot.into()];

        td.move_to_points(point_list);
    }
}

impl Mission for PrecualifyMission {
    fn react_to_object(&mut self, td: &MissionExecutor, idx: usize) {
        let object = MapObject::from(&td.map.load().objects[idx]);
        match object.cls {
            ObjectCls::Rectangle | ObjectCls::Cube => self.go_around(td, idx),
            ObjectCls::Gate => self.go_through(td, idx),
            ObjectCls::Shark => (),
            ObjectCls::Other => (),
            ObjectCls::SwordFish => (),
        }
    }
}

macro_rules! add_flags {
    ($thing:expr, $($flag:expr),+ $(,)?) => {{
        let mask = 0i32 $(| (1 << ($flag as i32)))+;
        $thing |= mask;
    }};
}

macro_rules! has_flags {
    ($thing:expr, $($flag:expr),+ $(,)?) => {{
        let mask = 0i32 $(| (1 << ($flag as i32)))+;
        ($thing as i32 & mask) == mask
    }};
}

enum DropIntoBoxMissionStep {
    FindGateSharkAndSwordFish,
    GoAboveBox,
    DropTheThing,
}

pub(crate) struct DropIntoBoxMission {
    step: DropIntoBoxMissionStep,
    seen: i32, // asuming only 32 classes
    gate_idx: usize,
    shark_idx: usize,
    sword_fish_idx: usize,
    box_idx: usize,
}

impl Mission for DropIntoBoxMission {
    fn react_to_object(&mut self, td: &MissionExecutor, idx: usize) {
        let object = MapObject::from(&td.map.load().objects[idx]);
        add_flags!(self.seen, object.cls);
        match self.step {
            DropIntoBoxMissionStep::FindGateSharkAndSwordFish => {
                match object.cls {
                    ObjectCls::Gate => self.gate_idx = idx,
                    ObjectCls::Shark => self.shark_idx = idx,
                    ObjectCls::SwordFish => self.sword_fish_idx = idx,
                    _ => (),
                }
                if has_flags!(self.seen, ObjectCls::Gate, ObjectCls::Shark, ObjectCls::SwordFish) {
                    self.move_to_gate_prefered_side(&td);
                    self.step = DropIntoBoxMissionStep::GoAboveBox;
                }
            },
            DropIntoBoxMissionStep::GoAboveBox => {
                match object.cls {
                    ObjectCls::Cube => self.box_idx = idx,
                    _ => (),
                }
                if has_flags!(self.seen, ObjectCls::Cube) {
                    self.move_above_cube(&td);
                    self.seen = 0; // forget about previous
                    self.step = DropIntoBoxMissionStep::DropTheThing;
                }
            },
            DropIntoBoxMissionStep::DropTheThing => {
                match object.cls {
                    ObjectCls::Shark => self.shark_idx = idx,
                    ObjectCls::SwordFish => self.sword_fish_idx = idx,
                    _ => (),
                }
                if has_flags!(self.seen, ObjectCls::Shark, ObjectCls::SwordFish) {
                    self.move_to_gate_prefered_side(&td);
                    td.stop.store(true, Ordering::Relaxed);
                }
            }
        }
    }
}

impl DropIntoBoxMission {
    pub(crate) fn new() -> Self {
        Self {
            step: DropIntoBoxMissionStep::FindGateSharkAndSwordFish,
            seen: 0,
            gate_idx: 0,
            shark_idx: 0,
            sword_fish_idx: 0,
            box_idx: 0,
        }
    }

    const PREFERED_GATE_OPTION: ObjectCls = ObjectCls::SwordFish;
    fn move_to_gate_prefered_side(&mut self, td: &MissionExecutor) {
        let objects = &td.map.load().objects;
        let shark_object = MapObject::from(&objects[self.shark_idx]);
        let sword_fish_object = MapObject::from(&objects[self.sword_fish_idx]);
        let gate_object = MapObject::from(&objects[self.gate_idx]);
        todo!("actually cross gate in prefered side")
    }

    fn move_above_cube(&mut self, td: &MissionExecutor) {
        let objects = &td.map.load().objects;
        let box_object = MapObject::from(&objects[self.box_idx]);
        todo!("actually move above cube")
    }

    fn move_above_prefered_and_drop_thing(&mut self, td: &MissionExecutor) {
        let objects = &td.map.load().objects;
        let box_object = MapObject::from(&objects[self.box_idx]);
        let shark_object = MapObject::from(&objects[self.shark_idx]);
        let sword_fish_object = MapObject::from(&objects[self.sword_fish_idx]);
        todo!("actually drop thing in prefered side")
    }
}

pub(crate) struct CardinalDirections {}

impl Mission for CardinalDirections {
    fn react_to_object(&mut self, td: &MissionExecutor, idx: usize) {
        let sub = **td.pose.load();
        let sub_vec = Vector6::new(
            sub.pos[0], sub.pos[1], sub.pos[2], 
            sub.rot[0], sub.rot[1], sub.rot[2]
        );
        let cardinals = [
            Vector2::new( 10.0,  00.0),
            Vector2::new( 00.0,  10.0),
            Vector2::new(-10.0,  00.0),
            Vector2::new( 00.0, -10.0),
        ];

        loop {
            for cardinal in cardinals {
                let mut goal = sub_vec;
                goal.x += cardinal.x;
                goal.y += cardinal.y;
                _ = td.goal.swap(Arc::new(goal));
                r2r::log_info!("cardinal", "--------------------------------------");
                r2r::log_info!("cardinal", "{cardinal:?}");
                r2r::log_info!("cardinal", "--------------------------------------");
                sleep(Duration::from_secs(10));
            }
        }
    }
}

impl CardinalDirections {
    pub(crate) fn new() -> Self {
        Self {}
    }
}
