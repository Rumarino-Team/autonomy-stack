#![allow(unused)]
#![deny(unused_must_use)]
//! This process should just realize a `Mission` passed as a ROS arg (somehow) and terminate.
//! A `Mission` is a scenario where the submarine must react to diferent `ObjectCls` with their
//! respective sequences of actions.

mod missions;
mod teleop;
mod navigation;
mod inotify;

use std::ops::Bound;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::time::Duration;
use arc_swap::ArcSwap;
use parry3d_f64::shape::Segment;
use tokio::sync::{Mutex, Notify};
use futures::StreamExt;
use nalgebra::{DVector, Isometry3, MatrixXx6, Point3, Quaternion, UnitQuaternion, Vector3, Vector6};

#[derive(Clone, Copy, Debug)]
pub struct Pose {
    pos: Vector3<f64>,
    rot: Quaternion<f64>,
}

impl Pose {
    fn new(pos: Vector3<f64>, rot: Quaternion<f64>) -> Pose {
        Pose {
            pos,
            rot
        }
    }
}
impl From<&r2r::geometry_msgs::msg::Pose> for Pose {
    fn from(value: &r2r::geometry_msgs::msg::Pose) -> Self {
        let p = &value.position;
        let o = &value.orientation;

        Self {
            pos: Vector3::new(p.x, p.y, p.z),
            rot: Quaternion::new(o.w, o.x, o.y, o.z),
        }
    }
}

impl From<Pose> for Vector6<f64> {
    fn from(value: Pose) -> Self {
        let p = value.pos;
        let unit = UnitQuaternion::from_quaternion(value.rot);
        let (roll, pitch, yaw) = unit.euler_angles();
        Vector6::new(p.x, p.y, p.z, roll, pitch, yaw)
    }
}

struct MissionExecutor {
    pub node: Arc<Mutex<r2r::Node>>,
    pub map: ArcSwap<MapMsg>,
    pub map_objects_reacted: AtomicUsize,
    pub new_objects: Notify,
    pub pose: ArcSwap<Pose>,
    pub goal: ArcSwap<Vector6<f64>>,
    pub stop: AtomicBool,
    pub avg_current: Arc<Mutex<f64>>,
}

const CLOSE_ENOUGH: f64 = 1.0;
const THRUSTER_USAGE: f64 = 7000.0; //in milliamps
const BATTERY_CAPACITY: f64 = 5000.0; //in mAh

fn wrap_angle(angle: f64) -> f64 {
    (angle + std::f64::consts::PI).rem_euclid(2.0 * std::f64::consts::PI) - std::f64::consts::PI
}

fn pose_stamp_ns(stamp: &r2r::builtin_interfaces::msg::Time) -> i64 {
    i64::from(stamp.sec) * 1_000_000_000 + i64::from(stamp.nanosec)
}

impl MissionExecutor {
    pub fn new(node: r2r::Node) -> Self {
        // hardcoded so it doesn't freak out while it waits for first odometry
        let origin = Pose {
            pos: Vector3::new(3.0, 1.0, 1.0),
            rot: Quaternion::identity(),
        };
        let goal = Vector6::from(origin);
        Self {
            node: Arc::new(Mutex::new(node)),
            map: ArcSwap::new(Arc::new(MapMsg::default())),
            map_objects_reacted: AtomicUsize::new(0),
            new_objects: Notify::new(),
            pose: ArcSwap::new(Arc::new(origin)),
            goal: ArcSwap::new(Arc::new(goal)),
            stop: AtomicBool::new(false),
            avg_current: Arc::new(Mutex::new(0.0)),
        }
    }

    // maybe this should be done relative to an object
    /// blocks until `dest` is reached within `CLOSE_ENOUGH` distance
    pub async fn move_to(&self, dest: Vector3<f64>) {
        let mut old_goal = **self.goal.load();
        old_goal.x = dest.x;
        old_goal.y = dest.y;
        old_goal.z = dest.z;
        self.goal.store(Arc::new(old_goal));
        while !self.stop.load(Ordering::Relaxed) {
            let pose = self.pose.load();
            let goal = self.goal.load();
            let dist = pose.pos.metric_distance(&dest);
            r2r::log_info!("dist", "{dist}, pose: {pose:?}, goal: {goal:?}");
            if dist < CLOSE_ENOUGH {
                break;
            } else {
                tokio::time::sleep(Duration::from_millis(100)).await;
            }
        }
    }

    pub async fn move_to_points(&self, mut dest: Vec<Point3<f64>>) {
        let sub_pose = self.pose.load();
        let map = self.map.load();
        let object_list = &map.objects;
        let map_bounds = BoundingBox3D::from(&map.map_bounds);

        let mut i = 0;
        while i != object_list.len() {
            let object = MapObject::from(&object_list[i]);
            if object.can_collide {
                i += 1;
                continue;
            }

            let mut new_points: Vec<Point3<f64>> = Vec::new();

            let mut before_point: Point3<f64> = sub_pose.pos.into();

            for point in &mut dest {
                let segment = Segment::new(before_point, (*point).into());
                let pos1 = Isometry3::from_parts(
                    object.bbox.center.pos.into(),
                    UnitQuaternion::from_quaternion(object.bbox.center.rot),
                );
                let extra_point =
                    navigation::get_new_point(&object.bbox, &pos1, segment, &map_bounds);
                if let Some(extra_point) = extra_point {
                    new_points.push(extra_point);
                }
                new_points.push(*point);
                before_point = *point;
            }
            if new_points == dest {
                i += 1;
            } else {
                i = 0;
                dest = new_points;
            }
        }

        r2r::log_info!("point_list", "{dest:#?}");

        for point in dest {
            self.move_to(point.coords).await;
            std::thread::sleep(Duration::from_millis(500));
        }
    }
}

#[repr(i32)]
#[derive(PartialEq, Eq, Clone, Copy)]
pub enum ObjectCls {
    Cube = 0,
    Rectangle = 1,
    Gate = 2,
    Shark = 3,
    Other = 4,
    SwordFish = 5,
}

pub struct BoundingBox3D {
    center: Pose,
    size: Vector3<f64>,
}

pub struct MapObject {
    cls: ObjectCls,
    bbox: BoundingBox3D,
    can_collide: bool,
}

impl BoundingBox3D {
    pub fn new(center: Pose, size: Vector3<f64>) -> BoundingBox3D {
        BoundingBox3D {
            center,
            size
        }
    }
}

impl From<&r2r::vision_msgs::msg::BoundingBox3D> for BoundingBox3D {
    fn from(value: &r2r::vision_msgs::msg::BoundingBox3D) -> Self {
        Self {
            center: (&value.center).into(),
            size: Vector3::new(value.size.x, value.size.y, value.size.z),
        }
    }
}

impl From<&r2r::interfaces::msg::MapObject> for MapObject {
    fn from(value: &r2r::interfaces::msg::MapObject) -> Self {
        let cls = unsafe { std::mem::transmute::<i32, ObjectCls>(value.cls) };
        let can_collide = cls == ObjectCls::Gate;
        Self {
            cls,
            bbox: (&value.bbox).into(),
            can_collide,
        }
    }
}

#[async_trait::async_trait]
trait Mission: Send + Sync {
    async fn react_to_object(&mut self, td: &MissionExecutor, idx: usize);
}

#[derive(Debug, Clone)]
struct PidConfig {
    kp: Vector6<f64>,
    ki: Vector6<f64>,
    kd: Vector6<f64>,
}

#[derive(Debug, Clone)]
struct LiveConfig {
    pid: std::collections::HashMap<String, PidConfig>,
    tam: MatrixXx6<f64>,
}

fn read_vec6(table: &toml_edit::Table, key: &str) -> Vector6<f64> {
    if let Some(arr) = table.get(key).and_then(|v| v.as_array()) {
        let mut v = Vector6::zeros();
        for (i, val) in arr.iter().enumerate().take(6) {
            v[i] = val.as_float().unwrap_or(0.0);
        }
        v
    } else {
        Vector6::zeros()
    }
}

fn load_live_config(path: &str, auv_name: &str) -> Option<LiveConfig> {
    let content = std::fs::read_to_string(path).ok()?;

    let doc = content.parse::<toml_edit::DocumentMut>().ok()?;

    let auv = doc.get(auv_name)?.as_table()?;

    let tam_arr = auv.get("tam")?.as_array()?;

    let rows = tam_arr.len();
    let mut data = Vec::with_capacity(rows * 6);

    for row in tam_arr.iter() {
        let Some(row_arr) = row.as_array() else {
            return None;
        };

        for val in row_arr.iter().take(6) {
            data.push(val.as_float().unwrap_or(0.0));
        }
    }

    let tam = MatrixXx6::from_row_slice(&data);

    let mut pid_map = std::collections::HashMap::new();

    if let Some(pids) = auv.get("pid").and_then(|t| t.as_table()) {
        for (name, table) in pids.iter() {
            if let Some(table) = table.as_table() {
                pid_map.insert(
                    name.to_string(),
                    PidConfig {
                        kp: read_vec6(table, "kp"),
                        ki: read_vec6(table, "ki"),
                        kd: read_vec6(table, "kd"),
                    },
                );
            }
        }
    }

    Some(LiveConfig { pid: pid_map, tam })
}

type MapMsg = r2r::interfaces::msg::Map;
type OdometryMsg = r2r::nav_msgs::msg::Odometry;
type Float64MultiArray = r2r::std_msgs::msg::Float64MultiArray;

#[tokio::main]
async fn main() {
    let ctx = r2r::Context::create().expect("Failed to create r2r context!");
    let mut node = r2r::Node::create(ctx, "mission_executor", "namespace").expect("Failed to get Node!");
    let params = node.params.lock().unwrap();

    let mission: Box<dyn Mission> = match params.get("mission_name") {
        Some(r2r::Parameter { value, .. }) => match value {
            r2r::ParameterValue::String(str) => match str.as_str() {
                "prequalify" => Box::new(missions::PrecualifyMission::new()),
                "drop_into_box" => Box::new(missions::DropIntoBoxMission::new()),
                "cardinal_directions" => Box::new(missions::CardinalDirections::new()),
                "teleop" => Box::new(teleop::TeleopMission::new()),
                _ => panic!("mission_name param must be a mission that exists"),
            },
            _ => panic!("mission_name param must be passed a string"),
        },
        None => panic!("mission_name param must be passed to mission_executor"),
    };

    let live_config_path = match params.get("live_config_path") {
        Some(r2r::Parameter { value, .. }) => match value {
            r2r::ParameterValue::String(str) => str.to_owned(),
            _ => panic!("live_config_path param must be passed a string"),
        },
        None => panic!("live_config_path param must be passed to mission_executor"),
    };
    r2r::log_info!("live_config_path", "{live_config_path:?}");

    let bridge_name = match params.get("bridge_name") {
        Some(r2r::Parameter { value, .. }) => match value {
            r2r::ParameterValue::String(str) => str.to_owned(),
            _ => panic!("bridge_name param must be passed a string"),
        },
        None => panic!("bridge_name param must be passed to mission_executor"),
    };

    let auv_name = match params.get("auv_name") {
        Some(r2r::Parameter { value, .. }) => match value {
            r2r::ParameterValue::String(str) => str.to_owned(),
            _ => panic!("auv_name param must be passed a string"),
        },
        None => panic!("auv_name param must be passed to mission_executor"),
    };

    drop(params);

    let map_qos = r2r::QosProfile::default().keep_last(1).transient_local();
    let mut map_sub = node
        .subscribe::<MapMsg>("/vision/map", map_qos)
        .expect("Failed to subscribe to map");
    let mut odometry_sub = node
        .subscribe::<OdometryMsg>("/bridge/odometry", r2r::QosProfile::default())
        .expect("Failed to subscribe to odometry");
    let thrusters_pub = node
        .create_publisher::<Float64MultiArray>("/bridge/thrusters", r2r::QosProfile::default())
        .expect("Failed to setup thruster publisher");

    let td = Arc::new(MissionExecutor::new(node));

    let consume_map_sub = |td: Arc<MissionExecutor>| async move {
        while let Some(msg) = map_sub.next().await {
            td.map.store(Arc::new(msg));
            td.new_objects.notify_one();
        }
    };

    let cfg = Arc::new(ArcSwap::from_pointee(load_live_config(&live_config_path, &auv_name).unwrap()));

    let mut inotify_stream = inotify::InotifyStream::new();
    let live_config_watch_id = inotify_stream.watch(&live_config_path);

    let cfg_clone = cfg.clone();
    let consume_inotify_stream = || async move {
        while let Some(id) = inotify_stream.next().await {
            if id == live_config_watch_id {
                if let Some(cfg) = load_live_config(&live_config_path, &auv_name) {
                    cfg_clone.store(Arc::new(cfg));
                    r2r::log_info!("live_config", "updated");
                }
            }
        }
    };

    const THRUSTOR_SATURATE: f64 = 5.0;

    let go_to_goal = |td: Arc<MissionExecutor>| async move {
        let mut sum_err = Vector6::zeros();
        let mut prev_pose_err = Vector6::zeros();
        let mut previous_timestamp_ns: Option<i64> = None;
        let mut count = 1.0; //Technically can be an integer but since we are multiplying by float...
        while let Some(msg) = odometry_sub.next().await {
            if td.stop.load(Ordering::Relaxed) {
                break;
            }

            let pose = Pose::from(&msg.pose.pose);
            td.pose.store(Arc::new(pose));

            let current_cfg = cfg.load();
            let PidConfig { kp, ki, kd } = current_cfg.pid[&bridge_name];
            let tam_x_y_z_roll_pitch_yaw = &current_cfg.tam;

            let timestamp_ns = pose_stamp_ns(&msg.header.stamp);
            let dt = previous_timestamp_ns.and_then(|previous| {
                let elapsed_ns = timestamp_ns - previous;
                (elapsed_ns > 0).then_some(elapsed_ns as f64 * 1e-9)
            });
            previous_timestamp_ns = Some(timestamp_ns);

            let goal = **td.goal.load();
            let current_pose = Vector6::<f64>::from(pose);

            // r2r::log_info!("goal", "{goal:?}");
            // r2r::log_info!("pose", "{current_pose:?}");

            let mut pose_err = goal - current_pose;

            let rot = UnitQuaternion::from_quaternion(pose.rot);
            let forward = rot * Vector3::y(); // if vehicle’s forward is +Y in body frame
            let (_, _, current_yaw) = rot.euler_angles();

            let dir = Vector3::new(pose_err[0], pose_err[1], 0.0);
            let yaw_error = if dir.norm() > CLOSE_ENOUGH {
                let dir = dir.normalize();
                
                // rotation from current forward → target direction
                let yaw_quat = UnitQuaternion::rotation_between(&forward, &dir)
                    .unwrap_or(UnitQuaternion::identity());

                let (_, _, yaw_error) = yaw_quat.euler_angles();
                yaw_error
            } else {
                wrap_angle(goal[5] - current_yaw)
            };

            pose_err[5] = yaw_error;

            let vel_err = if let Some(dt) = dt {
                sum_err += pose_err * dt;
                (pose_err - prev_pose_err) / dt
            } else {
                Vector6::zeros()
            };

            let wrench = kp.component_mul(&pose_err)
                + ki.component_mul(&sum_err)
                + kd.component_mul(&vel_err);

            let unit = UnitQuaternion::from_quaternion(pose.rot);
            let mut rotated = unit.conjugate() * wrench.xyz();

            // only move in xy if aprox looking at goal
            rotated.x = rotated.x.max(0.0);
            rotated.y = rotated.y.max(0.0);
            if yaw_error.abs() > std::f64::consts::PI / 8.0 {
                rotated.x = 0.0;
                rotated.y = 0.0;
            }
            let input_x_y_z_roll_pitch_yaw
                = Vector6::new(rotated.x, rotated.y, wrench.z, -wrench[3], wrench[4], wrench[5]);

            // r2r::log_info!("wrench", "{wrench:?}");
            // r2r::log_info!("rotated", "{rotated:?}");

            let mut thurstor_values: DVector<f64> = tam_x_y_z_roll_pitch_yaw * input_x_y_z_roll_pitch_yaw;

            // r2r::log_info!("thurstor_values", "{thurstor_values:?}");

            for val in &mut thurstor_values {
                *val = val.clamp(-THRUSTOR_SATURATE, THRUSTOR_SATURATE);
                 *val /= 5.0;
            }
            let mut thrusters_msg = Float64MultiArray::default();
            thrusters_msg.data.extend(thurstor_values.iter());
            thrusters_pub
                .publish(&thrusters_msg)
                .expect("Failed to publish");

            let mut sum_curr = 0.0;
            for val in &thurstor_values {
                sum_curr += val.powf(2.0) * THRUSTER_USAGE;
            }
            let mut avg_curr = td.avg_current.lock().await;
            *avg_curr = (*avg_curr * (count - 1.0) + sum_curr) / count;
            count += 1.0;
            r2r::log_info!(
                "thruster_report",
                "Average thruster usage in runtime: {:.2}",
                *avg_curr
            );
            r2r::log_info!(
                "thruster_report",
                "Current sum of thrusters: {:.2}",
                sum_curr
            );
            r2r::log_info!(
                "thruster_report",
                "Estimated battery life remaining: {:.2}",
                BATTERY_CAPACITY / *avg_curr
            );
            drop(avg_curr);

            prev_pose_err = pose_err;
        }
    };

    let scout = |td: Arc<MissionExecutor>| async move {
        while !td.stop.load(Ordering::Relaxed) {
            // TODO: do scouting, this code has to have .await's so that abort works
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    };
    let mut scout_handle = tokio::spawn(scout(Arc::clone(&td)));

    let consume_new_objects = |td: Arc<MissionExecutor>| async move {
        let mut mission = mission;
        while !td.stop.load(Ordering::Relaxed) {
            td.new_objects.notified().await;
            scout_handle.abort();
            let mut reacted = td.map_objects_reacted.load(Ordering::Relaxed);
            loop {
                let objects_len = td.map.load().objects.len();
                if reacted == objects_len {
                    break;
                }
                while reacted < objects_len {
                    r2r::log_info!("reacting...", "{reacted}");
                    mission.react_to_object(&td, reacted).await;
                    r2r::log_info!("reacted", "{reacted}");
                    reacted += 1;
                }
            }
            td.map_objects_reacted.store(reacted, Ordering::Relaxed);
            scout_handle = tokio::spawn(scout(Arc::clone(&td)));
        }
    };

    tokio::spawn(consume_inotify_stream());
    tokio::spawn(consume_map_sub(Arc::clone(&td)));
    tokio::spawn(consume_new_objects(Arc::clone(&td)));
    tokio::spawn(go_to_goal(Arc::clone(&td)));

    r2r::log_info!("", "start spinning!");
    while !td.stop.load(Ordering::Relaxed) {
        let mut node_lock = td.node.lock().await;
        node_lock.spin_once(Duration::from_millis(100));
        drop(node_lock);
    }

    let avg_current = td.avg_current.lock().await;

    r2r::log_info!(
        "thruster_report",
        "Average thruster usage in runtime: {:.2}",
        *avg_current
    );
}
