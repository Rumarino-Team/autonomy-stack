use futures::StreamExt;
use nix::libc;
use std::io::Read;
use std::mem;
use std::os::fd::AsRawFd;
use std::sync::{Arc, atomic::Ordering};
use tokio::io::AsyncReadExt;

use crate::{Mission, MissionExecutor};

type JoyMsg = r2r::sensor_msgs::msg::Joy;

pub(crate) struct TeleopMission {
    prev_buttons: Vec<i32>,
}

#[async_trait::async_trait]
impl Mission for TeleopMission {
    async fn react_to_object(&mut self, td: &MissionExecutor, _idx: usize) {
        self.print_instructions();
        self.input_listener_blocking(td).await;
    }
}

impl TeleopMission {
    pub(crate) fn new() -> Self {
        Self {
            prev_buttons: vec![],
        }
    }

    const STEP_X: f64 = 0.1;
    const STEP_Y: f64 = 0.1;
    const STEP_Z: f64 = 0.1;
    const STEP_ROLL: f64 = 0.1;
    const STEP_PITCH: f64 = 0.1;
    const STEP_YAW: f64 = 0.1;

    fn print_instructions(&self) {
        println!("\n=== Hydrus Setpoint Teleop (Mission Executor Controller) ===");
        println!("Controls:");
        println!("  w : Decrease y goal by {:.2} m", Self::STEP_Y);
        println!("  a : Decrease x goal by {:.2} m", Self::STEP_X);
        println!("  s : Increase y goal by {:.2} m", Self::STEP_Y);
        println!("  d : Increase x goal by {:.2} m", Self::STEP_X);
        println!("  q : Increase z goal by {:.2} m", Self::STEP_Z);
        println!("  e : Decrease z goal by {:.2} m", Self::STEP_Z);
        println!("  u : Decrease roll goal by {:.2} rad", Self::STEP_ROLL);
        println!("  o : Increase roll goal by {:.2} rad", Self::STEP_ROLL);
        println!("  i : Increase pitch goal by {:.2} rad", Self::STEP_PITCH);
        println!("  k : Decrease pitch goal by {:.2} rad", Self::STEP_PITCH);
        println!("  j : Decrease yaw goal by {:.2} rad", Self::STEP_YAW);
        println!("  l : Increase yaw goal by {:.2} rad", Self::STEP_YAW);
        println!("  x : Hold current pose");
        println!("  z : Quit");
        println!("============================================================\n");
    }

    fn process_key(&self, td: &MissionExecutor, key: char) {
        let key = key.to_ascii_lowercase();

        if key == 'z' {
            println!("Quitting...");
            td.stop.store(true, Ordering::Relaxed);
            return;
        }

        let mut goal = **td.goal.load();
        match key {
            'w' => goal[1] -= Self::STEP_Y,
            'a' => goal[0] -= Self::STEP_X,
            's' => goal[1] += Self::STEP_Y,
            'd' => goal[0] += Self::STEP_X,
            'q' => goal[2] += Self::STEP_Z,
            'e' => goal[2] -= Self::STEP_Z,
            'u' => goal[3] -= Self::STEP_ROLL,
            'o' => goal[3] += Self::STEP_ROLL,
            'i' => goal[4] += Self::STEP_PITCH,
            'k' => goal[4] -= Self::STEP_PITCH,
            'j' => goal[5] -= Self::STEP_YAW,
            'l' => goal[5] += Self::STEP_YAW,
            'x' => {
                let pos = (**td.pose.load()).pos;
                let rot = (**td.pose.load()).rot;
                let (_, _, yaw) = nalgebra::UnitQuaternion::from_quaternion(rot).euler_angles();
                goal.x = pos.x;
                goal.y = pos.y;
                goal.z = pos.z;
                goal[5] = yaw;
            }
            _ => {}
        }
        td.goal.store(Arc::new(goal));
    }

    async fn input_listener_blocking(&mut self, td: &MissionExecutor) {
        let mut joy_sub = td
            .node
            .lock()
            .await
            .subscribe::<JoyMsg>("/joy", r2r::QosProfile::default())
            .expect("Failed to subscribe to joy");

        let mut stdin = tokio::io::stdin();
        let mut buffer = [0u8; 1];

        let fd = stdin.as_raw_fd();
        unsafe {
            if libc::isatty(fd) != 1 {
                eprintln!("stdin is not a TTY, keyboard disabled");
                return;
            }
        }

        let mut term = mem::MaybeUninit::uninit();
        let mut term = unsafe {
            assert!(libc::tcgetattr(fd, term.as_mut_ptr()) == 0);
            term.assume_init()
        };

        let old_term = term.clone();

        unsafe {
            libc::cfmakeraw(&mut term);
            assert!(libc::tcsetattr(fd, libc::TCSANOW, &term) == 0);
        }

        while !td.stop.load(Ordering::Relaxed) {
            tokio::select! {
                Ok(n) = stdin.read(&mut buffer) => {
                    if n == 1 {
                        let key = buffer[0] as char;
                        self.process_key(td, key);
                    }
                }
                Some(msg) = joy_sub.next() => {
                    let axes = &msg.axes;
                    let buttons = &msg.buttons;

                    let mut goal = **td.goal.load();

                    // === AXES ===
                    let x_input = axis_value(axes, 1, 0.15, true);
                    let y_input = axis_value(axes, 0, 0.15, false);
                    let yaw_input = axis_value(axes, 2, 0.15, false);

                    goal[0] -= y_input * Self::STEP_X * 0.1;
                    goal[1] += x_input * Self::STEP_Y * 0.5;
                    goal[5] += yaw_input * Self::STEP_YAW * 0.5;

                    // === BUTTONS ===
                    let z_input =
                        button_value(buttons, 4) - button_value(buttons, 5);

                    goal[2] += (z_input as f64) * Self::STEP_Z;

                    // HOLD (like 'x' key on playstation or 'A' on xbox)
                    if button_rising_edge(buttons, &self.prev_buttons, 0) {
                        let pos = (**td.pose.load()).pos;
                        let rot = (**td.pose.load()).rot;
                        let (_, _, yaw) =
                            nalgebra::UnitQuaternion::from_quaternion(rot).euler_angles();

                        goal[0] = pos.x;
                        goal[1] = pos.y;
                        goal[2] = pos.z;
                        goal[5] = yaw;
                    }

                    // QUIT
                    if button_rising_edge(buttons, &self.prev_buttons, 1) {
                        td.stop.store(true, Ordering::Relaxed);
                    }

                    td.goal.store(Arc::new(goal));

                    // update prev (reuse allocation)
                    if self.prev_buttons.len() != buttons.len() {
                        self.prev_buttons = buttons.clone();
                    } else {
                        self.prev_buttons.copy_from_slice(buttons);
                    }
                }
            };
        }

        unsafe {
            _ = libc::tcsetattr(fd, libc::TCSADRAIN, &old_term);
        }
    }
}

fn axis_value(axes: &[f32], idx: i32, deadzone: f32, invert: bool) -> f64 {
    if idx < 0 || (idx as usize) >= axes.len() {
        return 0.0;
    }

    let mut v = axes[idx as usize] as f64;
    if invert {
        v = -v;
    }

    if v.abs() < deadzone as f64 { 0.0 } else { v }
}

fn button_value(buttons: &[i32], idx: i32) -> i32 {
    if idx < 0 || (idx as usize) >= buttons.len() {
        0
    } else {
        buttons[idx as usize]
    }
}

fn button_rising_edge(buttons: &[i32], prev: &[i32], idx: i32) -> bool {
    button_value(buttons, idx) == 1 && button_value(prev, idx) == 0
}
