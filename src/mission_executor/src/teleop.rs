use nix::libc;
use std::io::Read;
use std::mem;
use std::os::fd::AsRawFd;
use std::sync::{Arc, atomic::Ordering};

use crate::{Mission, MissionExecutor};

pub(crate) struct TeleopMission {}

impl Mission for TeleopMission {
    fn react_to_object(&mut self, td: &MissionExecutor, _idx: usize) {
        self.print_instructions();
        self.input_listener_blocking(td);
    }
}

impl TeleopMission {
    pub(crate) fn new() -> Self {
        Self {}
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

    fn input_listener_blocking(&self, td: &MissionExecutor) {
        let stdin = std::io::stdin();
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

        let mut buffer = [0u8; 1];
        while !td.stop.load(Ordering::Relaxed) {
            // TODO: joystick, r2r subscriber is async so not sure how
            //       get around that

            // keyboard
            if let Ok(n) = stdin.lock().read(&mut buffer) {
                if n == 1 {
                    let key = buffer[0] as char;
                    self.process_key(td, key);
                }
            }
        }

        unsafe {
            _ = libc::tcsetattr(fd, libc::TCSADRAIN, &old_term);
        }
    }
}
