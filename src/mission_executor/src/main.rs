#![allow(unused)]
#![deny(unused_must_use)]
//! This process should just realize a `Mission` passed as a ROS arg (somehow) and terminate.
//! A `Mission` is a scenario where the submarine must react to diferent `ObjectCls` with their
//! respective sequences of actions.

mod missions;
mod navigation;

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::time::{Duration, Instant};

use arc_swap::ArcSwap;
use futures::StreamExt;
use nalgebra::{
    ArrayStorage, DVector, Isometry3, Matrix1x3, Matrix1x6, Matrix4x3, MatrixXx3, MatrixXx6, Point3, Quaternion, SimdPartialOrd, UnitQuaternion, Vector, Vector3, Vector6, U8
};
type Vector8<T> = Vector<T, U8, ArrayStorage<T, 8, 1>>;
use parry3d_f64::shape::Segment;
use r2r::{Node, ParameterValue, QosProfile};
use tokio::sync::Notify;
use tokio::signal::unix::{signal, SignalKind};
use tokio::time::timeout;
use futures::lock::Mutex;
use notify::Watcher;

#[derive(Clone, Copy, Debug)]
struct Pose {
    pos: Vector3<f64>,
    rot: Quaternion<f64>,
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

struct MissionExecutor {
    pub map: ArcSwap<MapMsg>,
    pub map_objects_reacted: AtomicUsize,
    pub new_objects: Notify,
    pub pose: ArcSwap<Pose>,
    pub goal: ArcSwap<Vector6<f64>>,
    pub stop: AtomicBool,
    pub avg_current: Arc<Mutex<f64>>,
}

const CLOSE_ENOUGH: f64 = 1.0;

impl MissionExecutor {
    pub fn new() -> Self {
        // hardcoded so it doesn't freak out while it waits for first odometry
        let origin = Pose {
            pos: Vector3::new(-3.0, 1.0, 1.0),
            rot: Quaternion::identity(),
        };
        let (roll, pitch, yaw) = UnitQuaternion::from_quaternion(origin.rot).euler_angles();
        let mut goal = Vector6::zeros();
        goal.fixed_rows_mut::<3>(0).copy_from(&origin.pos);
        goal.fixed_rows_mut::<3>(3)
            .copy_from(&Vector3::new(roll, pitch, yaw));
        Self {
            map: ArcSwap::new(Arc::new(MapMsg::default())),
            map_objects_reacted: AtomicUsize::new(0),
            new_objects: Notify::new(),
            pose: ArcSwap::new(Arc::new(origin)),
            goal: ArcSwap::new(Arc::new(goal)),
            stop: AtomicBool::new(false),
            avg_current: Arc::new(Mutex::new(0.0)),
            // mission,
        }
    }

    // maybe this should be done relative to an object
    /// blocks until `dest` is reached within `CLOSE_ENOUGH` distance
    pub fn move_to(&self, dest: Vector3<f64>) {
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
                std::thread::sleep(Duration::from_millis(100));
            }
        }
    }

    pub fn move_to_points(&self, mut dest: Vec<Point3<f64>>) {
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
            self.move_to(point.coords);
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

trait Mission: Send + Sync {
    fn react_to_object(&mut self, td: &MissionExecutor, idx: usize);
}

#[derive(Debug, Clone)]
struct ControllerConfig {
    kp: Vector6<f64>,
    ki: Vector6<f64>,
    kd: Vector6<f64>,
}

#[derive(Debug, Clone)]
struct LiveConfig {
    controllers: std::collections::HashMap<String, ControllerConfig>,
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

fn load_config(path: &str) -> LiveConfig {
    let content = std::fs::read_to_string(path).unwrap_or_default();
    let doc = content
        .parse::<toml_edit::DocumentMut>()
        .unwrap_or_else(|_| toml_edit::DocumentMut::new());


    let mut controllers_map = std::collections::HashMap::new();

    if let Some(controllers) = doc.get("controllers").and_then(|t| t.as_table()) {
        for (name, table) in controllers.iter() {
            if let Some(table) = table.as_table() {

                controllers_map.insert(name.to_string(), ControllerConfig {
                    kp: read_vec6(table, "kp"),
                    ki: read_vec6(table, "ki"),
                    kd: read_vec6(table, "kd"),
                });
            }
        }
    }

    LiveConfig { controllers: controllers_map }
}

fn save_config(path: &str, cfg: &LiveConfig) {
    let mut doc = toml_edit::DocumentMut::new();
    let mut controllers = toml_edit::Table::new();

    for (name, thing) in &cfg.controllers {
        let mut table = toml_edit::Table::new();

        let mut arr = toml_edit::Array::new();
        for v in thing.kp.as_slice() {
            arr.push(*v as f64);
        }
        table["kp"] = toml_edit::Item::Value(arr.into());

        let mut arr = toml_edit::Array::new();
        for v in thing.ki.as_slice() {
            arr.push(*v as f64);
        }
        table["ki"] = toml_edit::Item::Value(arr.into());

        let mut arr = toml_edit::Array::new();
        for v in thing.kd.as_slice() {
            arr.push(*v as f64);
        }
        table["kd"] = toml_edit::Item::Value(arr.into());

        controllers[name] = toml_edit::Item::Table(table);
    }

    doc["controllers"] = toml_edit::Item::Table(controllers);
    std::fs::write(path, doc.to_string()).unwrap();
}

type MapMsg = r2r::interfaces::msg::Map;
type MapObjectMsg = r2r::interfaces::msg::MapObject;
type PointMsg = r2r::geometry_msgs::msg::Point;
type QuaternionMsg = r2r::geometry_msgs::msg::Quaternion;
type Vector3Msg = r2r::geometry_msgs::msg::Vector3;
type OdometryMsg = r2r::nav_msgs::msg::Odometry;
type Float64MultiArray = r2r::std_msgs::msg::Float64MultiArray;

#[tokio::main]
async fn main() {
    let ctx = r2r::Context::create().expect("Failed to create r2r context!");
    let mut node = Node::create(ctx, "mission_executor", "namespace").expect("Failed to get Node!");
    let params = node.params.lock().unwrap();

    let mission: Box<dyn Mission> = match params.get("mission_name") {
        Some(r2r::Parameter { value, .. }) => match value {
            ParameterValue::String(str) => match str.as_str() {
                "prequalify" => Box::new(missions::PrecualifyMission::new()),
                "drop_into_box" => Box::new(missions::DropIntoBoxMission::new()),
                _ => panic!("mission_name param must be a mission that exists"),
            },
            _ => panic!("mission_name param must be passed a string"),
        },
        None => panic!("mission_name param must be passed to mission_executor"),
    };

    let controller_name = match params.get("controller_name") {
        Some(r2r::Parameter { value, .. }) => match value {
            // get the tam and stored values for PID runtime "constants"
            ParameterValue::String(str) => match str.as_str() {
                "stonefish_hydrus" => str,
                "stonefish_proteus" => str,
                "real_proteus" => str,
                _ => panic!("controller_name param must be a controller that exists"),
            },
            _ => panic!("controller_name param must be passed a string"),
        },
        None => panic!("controller_name param must be passed to mission_executor"),
    }.to_owned();

    let tam_x_y_z_roll_pitch_yaw: MatrixXx6<f64> = match controller_name.as_str() {
        "stonefish_hydrus" =>
            MatrixXx6::from_rows(&[
                Matrix1x6::new(-1.0,  1.0,  0.0,  0.0,  0.0,  1.0),
                Matrix1x6::new(-1.0, -1.0,  0.0,  0.0,  0.0, -1.0),
                Matrix1x6::new( 1.0,  1.0,  0.0,  0.0,  0.0, -1.0),
                Matrix1x6::new( 1.0, -1.0,  0.0,  0.0,  0.0,  1.0),
                Matrix1x6::new( 0.0,  0.0, -1.0,  1.0,  1.0,  0.0),
                Matrix1x6::new( 0.0,  0.0, -1.0, -1.0,  1.0,  0.0),
                Matrix1x6::new( 0.0,  0.0, -1.0,  1.0, -1.0,  0.0),
                Matrix1x6::new( 0.0,  0.0, -1.0, -1.0, -1.0,  0.0),
            ]),
        "stonefish_proteus" =>
            MatrixXx6::from_rows(&[
                Matrix1x6::new(-1.0,  0.0,  0.0,  0.0,  0.0,  1.0),
                Matrix1x6::new(-1.0,  0.0,  0.0,  0.0,  0.0, -1.0),
                Matrix1x6::new( 0.0,  0.0, -1.0,  1.0,  1.0,  0.0),
                Matrix1x6::new( 0.0,  0.0, -1.0, -1.0,  1.0,  0.0),
                Matrix1x6::new( 0.0,  0.0, -1.0,  1.0, -1.0,  0.0),
                Matrix1x6::new( 0.0,  0.0, -1.0, -1.0, -1.0,  0.0),
            ]),
        _ => unimplemented!("no tam for {controller_name}"),
    };

    drop(params);

    let td = Arc::new(MissionExecutor::new());

    let map_qos = QosProfile::default().keep_last(1).transient_local();
    let mut map_sub = node
        .subscribe::<MapMsg>("/hydrus/map", map_qos)
        .expect("Failed to subscribe to map");
    let mut odometry_sub = node
        .subscribe::<OdometryMsg>("/hydrus/odometry", QosProfile::default())
        .expect("Failed to subscribe to odometry");
    let thrusters_pub = node
        .create_publisher::<Float64MultiArray>("/hydrus/thrusters", QosProfile::default())
        .expect("Failed to setup thruster publisher");

    let consume_map_sub = |td: Arc<MissionExecutor>| async move {
        while let Some(msg) = map_sub.next().await {
            td.map.store(Arc::new(msg));
            td.new_objects.notify_one();
        }
    };

    let consume_odometry_sub = |td: Arc<MissionExecutor>| async move {
        while let Some(msg) = odometry_sub.next().await {
            td.pose.store(Arc::new(Pose::from(&msg.pose.pose)));
        }
    };

    let path = "./src/mission_executor/live_config.toml";
    let path_buf = std::fs::canonicalize(path).unwrap();
    r2r::log_info!("config path", "{:?}", path_buf);

    // Shared atomic ArcSwap
    let cfg = Arc::new(ArcSwap::from_pointee(load_config(path)));

    // Notify channel
    let (tx, mut rx) = tokio::sync::mpsc::channel::<()>(8);

    let watch_path = path_buf.clone();
    std::thread::spawn(move || {
        let (notify_tx, notify_rx) = std::sync::mpsc::channel();

        let mut watcher = notify::RecommendedWatcher::new(
            notify_tx,
            notify::Config::default()
                .with_poll_interval(Duration::from_secs(1)),
        ).expect("failed to create watcher");

        // watch the parent directory instead of the file
        let parent = watch_path.parent().unwrap();

        watcher
            .watch(parent, notify::RecursiveMode::NonRecursive)
            .expect("watch failed");

        println!("watching directory: {:?}", parent);

        
        for event in notify_rx {
            if let Ok(ev) = event {
                match ev.kind {
                    notify::EventKind::Modify(_) | notify::EventKind::Create(_) => {
                        if ev.paths.iter().any(|p| p.ends_with(&watch_path)) {
                            let _ = tx.blocking_send(());
                        }
                    }
                    _ => {} // ignore other events
                }
            }
        }
    });

    let cfg_clone = cfg.clone();
    let path_reload = path_buf.clone();
    tokio::spawn(async move {
        while let Some(_) = rx.recv().await {
            let new_cfg = load_config(path_reload.to_str().unwrap());
            r2r::log_info!("", "config change detected {:?}", new_cfg);

            println!("NEW CONFIG: {:?}", new_cfg);

            cfg_clone.store(Arc::new(new_cfg));
        }
    });

    const THRUSTOR_SATURATE: f64 = 5.0;

    let go_to_goal = |td: Arc<MissionExecutor>| async move {
        let mut sum_err = Vector6::zeros();
        let mut prev_pose_err = Vector6::zeros();
        let mut prev_now = Instant::now();
        let mut count = 1.0; //Technically can be an integer but since we are multiplying by float...
        let log_interval = Duration::from_millis(500);
        let mut last_log = Instant::now();
        while !td.stop.load(Ordering::Relaxed) {
            let current_cfg = cfg.load();
            let ControllerConfig { kp, ki, kd } = current_cfg.controllers[&controller_name];

            let now = Instant::now();
            let dt = now.duration_since(prev_now).as_secs_f64();

            let pose = **td.pose.load();
            let goal = **td.goal.load();
            let p = pose.pos;
            let unit = UnitQuaternion::from_quaternion(pose.rot);
            let (roll, pitch, yaw) = unit.euler_angles();
            let current_pose = Vector6::new(p.x, p.y, p.z, roll, pitch, yaw);

            // r2r::log_info!("goal", "{goal:?}");
            // r2r::log_info!("pose", "{current_pose:?}");

            let pose_err = goal - current_pose;
            let vel_err = (pose_err - prev_pose_err) / dt;
            sum_err += pose_err * dt;

            let wrench = kp.component_mul(&pose_err)
                + ki.component_mul(&sum_err)
                + kd.component_mul(&vel_err);

            let rotated = unit.conjugate() * wrench.xyz();
            let xy_error = wrench.xy().norm();
            // only apply yaw when close
            let yaw_scale = if xy_error < 0.5 { 1.0 } else { 0.0 };

            let input_x_y_z_roll_pitch_yaw
                = Vector6::new(rotated.x, rotated.y, wrench.z, -wrench[3], wrench[4], wrench[5] * yaw_scale);

            // r2r::log_info!("wrench", "{wrench:?}");
            // r2r::log_info!("rotated", "{rotated:?}");

            let mut thurstor_values: DVector<f64> = &tam_x_y_z_roll_pitch_yaw * input_x_y_z_roll_pitch_yaw;

            // r2r::log_info!("thurstor_values", "{thurstor_values:?}");

            let minn = DVector::repeat(thurstor_values.len(), -THRUSTOR_SATURATE);
            let maxx = DVector::repeat(thurstor_values.len(), THRUSTOR_SATURATE);
            thurstor_values = thurstor_values.simd_clamp(minn, maxx) / THRUSTOR_SATURATE;

            let mut thrusters_msg = Float64MultiArray::default();
            thrusters_msg.data.extend(thurstor_values.iter());
            thrusters_pub
                .publish(&thrusters_msg)
                .expect("Failed to publish");

            let mut sum_curr = 0.0;
            for val in &thurstor_values {
                sum_curr += val.abs();
            }
            let mut avg_curr = td.avg_current.lock().await;
            *avg_curr = (*avg_curr * (count - 1.0) + sum_curr) / count;
            count += 1.0;
            if now.duration_since(last_log) >= log_interval {
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
                last_log = now;
            }
            drop(avg_curr);

            prev_pose_err = pose_err;
            prev_now = now;

            tokio::time::sleep(Duration::from_millis(100)).await;
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
                    mission.react_to_object(&td, reacted);
                    r2r::log_info!("reacted", "{reacted}");
                    reacted += 1;
                }
            }
            td.map_objects_reacted.store(reacted, Ordering::Relaxed);
            scout_handle = tokio::spawn(scout(Arc::clone(&td)));
        }
    };

    tokio::spawn(consume_map_sub(Arc::clone(&td)));
    tokio::spawn(consume_odometry_sub(Arc::clone(&td)));
    tokio::spawn(consume_new_objects(Arc::clone(&td)));
    tokio::spawn(go_to_goal(Arc::clone(&td)));

    r2r::log_info!("", "start spinning!");
    while !td.stop.load(Ordering::Relaxed) {
        node.spin_once(Duration::from_millis(100));
    }

    let avg_current = td.avg_current.lock().await;

    r2r::log_info!(
        "thruster_report",
        "Average thruster usage in runtime: {:.2}",
        *avg_current
    );
}
