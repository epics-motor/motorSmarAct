use std::cmp::Ordering;
use std::time::SystemTime;

// Constants
/** MCS2 Axis status flags **/
pub const CH_STATE_ACTIVELY_MOVING: u32 = 0x0001;
pub const CH_STATE_CLOSED_LOOP_ACTIVE: u32 = 0x0002;
pub const CH_STATE_CALIBRATING: u32 = 0x0004;
pub const CH_STATE_REFERENCING: u32 = 0x0008;
pub const CH_STATE_MOVE_DELAYED: u32 = 0x0010;
pub const CH_STATE_SENSOR_PRESENT: u32 = 0x0020;
pub const CH_STATE_IS_CALIBRATED: u32 = 0x0040;
pub const CH_STATE_IS_REFERENCED: u32 = 0x0080;
pub const CH_STATE_END_STOP_REACHED: u32 = 0x0100;
pub const CH_STATE_RANGE_LIMIT_REACHED: u32 = 0x0200;
pub const CH_STATE_FOLLOWING_LIMIT_REACHED: u32 = 0x0400;
pub const CH_STATE_MOVEMENT_FAILED: u32 = 0x0800;
pub const CH_STATE_STREAMING: u32 = 0x1000;
pub const CH_STATE_POSITIONER_OVERLOAD: u32 = 0x2000;
pub const CH_STATE_OVERTEMP: u32 = 0x4000;
pub const CH_STATE_REFERENCE_MARK: u32 = 0x8000;
pub const CH_STATE_IS_PHASED: u32 = 0x00010000;
pub const CH_STATE_POSITIONER_FAULT: u32 = 0x00020000;
pub const CH_STATE_AMPLIFIER_ENABLED: u32 = 0x00040000;
pub const CH_STATE_IN_POSITION: u32 = 0x00080000;
pub const CH_STATE_BRAKE_ENABLED: u32 = 0x00100000;

#[derive(Debug)]
pub struct Mcs2axis {
    axis_no: usize,
    state_actively_moving: bool,
    state_closed_loop_active: bool,
    state_calibrating_counter: u16,
    time_calibrating_started: SystemTime,
    state_referencing_counter: u16,
    time_referencing_started: SystemTime,
    state_move_delayed: bool,
    state_sensor_present: bool,
    state_is_calibrated: bool,
    state_is_referenced: bool,
    state_end_stop_reached: bool,
    state_range_limit_reached: bool,
    state_following_limit_reached: bool,
    state_movement_failed: bool,
    state_streaming: bool,
    state_positioner_overload: bool,
    state_overtemp: bool,
    state_reference_mark: bool,
    state_is_phased: bool,
    state_positioner_fault: bool,
    state_amplifier_enabled: bool,
    state_in_position: bool,
    state_brake_enabled: bool,
    acc: f64,
    hold_time: i32,
    sens_del: u32,  // sensor delay
    sens_mode: i32, // sensor mode: 0=disabled, 1=enabled, 2=powersave
    mclf: i32,      // Max Closed Loop Frequency
    mmod: i32,      // move modus. TODO: Is this and int32 ?
    internal_mmod: i32,
    ref_opt: i32,
    step_freq: i32,
    pos_act: i64,  // (actual) position
    pos_targ: i64, // target position (commanded from host)
    time_pos_targ_started: SystemTime,
    in_target_window: i64, // Need a better name ?
    external_vel: f64,     // velocity as commanded from host
    internal_vel: f64,     // velocity as used for open/closed loop
    step_size_openloop_f: i64,
    step_size_openloop_r: i64,
}

impl Mcs2axis {
    pub fn new(axisno: usize) -> Mcs2axis {
        Mcs2axis {
            axis_no: axisno,
            state_actively_moving: false,
            state_closed_loop_active: false,
            state_calibrating_counter: 0,
            time_calibrating_started: SystemTime::now(),
            state_referencing_counter: 0,
            time_referencing_started: SystemTime::now(),
            state_move_delayed: false,
            state_sensor_present: true,
            state_is_calibrated: false,
            state_is_referenced: false,
            state_end_stop_reached: false,
            state_range_limit_reached: false,
            state_following_limit_reached: false,
            state_movement_failed: false,
            state_streaming: false,
            state_positioner_overload: false,
            state_overtemp: false,
            state_reference_mark: false,
            state_is_phased: false,
            state_positioner_fault: false,
            state_amplifier_enabled: false,
            state_in_position: false,
            state_brake_enabled: false,
            acc: 0.0,
            hold_time: 0,
            sens_del: 100,
            sens_mode: 1,
            mclf: 5000,
            mmod: 0,
            internal_mmod: -1,
            ref_opt: 0,
            step_freq: 0,
            pos_act: 0,
            pos_targ: 0,
            time_pos_targ_started: SystemTime::now(),
            in_target_window: 1_000_000, // 1 µm
            external_vel: 0.0,
            internal_vel: 0.0,
            step_size_openloop_f: 2500000, // simulated step size open loop forward
            step_size_openloop_r: 4000000, // simulated step size open loop reverse
        }
    }
    pub fn do_cal(&mut self) -> bool {
        println!("mcs2axis::do_cal[{}]", &self.axis_no);
        self.state_is_calibrated = false;
        self.state_calibrating_counter = 100;
        self.time_calibrating_started = SystemTime::now();
        true
    }
    pub fn do_move(&mut self, move_value: i64) -> bool {
        if self.mmod == 0 {
            // move absolute, closed loop
            println!(
                "mcs2axis::do_move[{}] old={} absolute move_value={}",
                &self.axis_no, self.pos_targ, move_value
            );
            self.pos_targ = move_value;
            self.internal_vel = self.external_vel;
        } else if self.mmod == 1 {
            // move relative, closed loop
            println!(
                "mcs2axis::do_move[{}] old={} relative move_value={}",
                &self.axis_no, self.pos_targ, move_value
            );
            self.pos_targ += move_value;
            self.internal_vel = self.external_vel;
        } else if self.mmod == 4 {
            // step move, open loop
            println!(
                "mcs2axis::do_move[{}] old={} steps move_value={}",
                &self.axis_no, self.pos_targ, move_value
            );
            match move_value.cmp(&0) {
                Ordering::Greater => {
                    self.internal_vel = (self.step_size_openloop_f * self.step_freq as i64) as f64;
                    self.pos_targ += move_value * self.step_size_openloop_f;
                }
                Ordering::Less => {
                    self.internal_vel = (self.step_size_openloop_r * self.step_freq as i64) as f64;
                    self.pos_targ += move_value * self.step_size_openloop_r;
                }
                Ordering::Equal => {}
            }
            println!(
                "mcs2axis::do_move[{}] step mode move_value={} pos_targ={} step_freq={} vel={}",
                &self.axis_no, move_value, self.pos_targ, self.step_freq, self.internal_vel
            );
        } else {
            println!(
                "mcs2axis::do_move[{}] illegal mmod={} move_value={}",
                &self.axis_no, self.mmod, move_value
            );
            return true;
        }
        self.time_pos_targ_started = SystemTime::now();
        self.internal_mmod = self.mmod;
        true
    }
    pub fn do_ref(&mut self) -> bool {
        println!("mcs2axis::do_ref[{}]", &self.axis_no);
        self.state_is_referenced = false;
        self.state_referencing_counter = 100;
        self.time_referencing_started = SystemTime::now();
        true
    }
    pub fn do_stop(&mut self) -> bool {
        println!("mcs2axis::do_ref[{}]", &self.axis_no);
        self.internal_vel = 0.0;
        self.state_calibrating_counter = 0;
        self.state_referencing_counter = 0;
        true
    }
    pub fn get_mclf(&self) -> i32 {
        self.mclf
    }
    pub fn get_pos_act(&self) -> i64 {
        self.pos_act
    }
    pub fn get_pos_targ(&self) -> i64 {
        self.pos_targ
    }
    pub fn get_sens_del(&self) -> u32 {
        self.sens_del
    }
    pub fn get_sens_mode(&self) -> i32 {
        self.sens_mode
    }

    fn status_do_calibibrate(&mut self) -> u32 {
        let mut ret = 0;
        if self.state_calibrating_counter > 0 {
            match self.time_calibrating_started.elapsed() {
                Ok(elapsed) => {
                    let time_msec: u128 = elapsed.as_millis();
                    println!(
                        "mcs2axis::state_calibrating_counter elapsed time_msec={:?}",
                        time_msec
                    );
                    if time_msec > 3000 {
                        // break the loop below
                        self.state_calibrating_counter = 1;
                    }
                }
                Err(e) => {
                    println!("mcs2axis::state_calibrating_counter elapsed e={:?}", e);
                }
            }
            self.state_calibrating_counter -= 1;
            if self.state_calibrating_counter > 0 {
                // Still calibrating
                ret += CH_STATE_CALIBRATING;
                ret += CH_STATE_ACTIVELY_MOVING;
            } else {
                // calibrating just succeeded
                self.state_is_calibrated = true;
            }
        }
        ret
    }
    fn status_do_reference(&mut self) -> u32 {
        let mut ret = 0;
        if self.state_referencing_counter > 0 {
            match self.time_referencing_started.elapsed() {
                Ok(elapsed) => {
                    let time_msec: u128 = elapsed.as_millis();
                    println!(
                        "mcs2axis::state_referencing_counter elapsed time_msec={:?}",
                        time_msec
                    );
                    if time_msec > 3000 {
                        // break the loop below
                        self.state_referencing_counter = 1;
                    }
                }
                Err(e) => {
                    println!("mcs2axis::state_referencing_counter elapsed e={:?}", e);
                }
            }
            self.state_referencing_counter -= 1;
            if self.state_referencing_counter > 0 {
                // still referencing
                ret += CH_STATE_REFERENCING;
                ret += CH_STATE_ACTIVELY_MOVING;
            } else {
                // referencing just succeeded
                self.state_is_referenced = true;
            }
        }
        ret
    }

    fn status_do_move(&mut self) -> u32 {
        if self.internal_mmod < 0 || self.internal_vel <= 0.0 {
            return 0;
        }
        let mut ret = 0;
        match self.time_pos_targ_started.elapsed() {
            Ok(elapsed) => {
                let time_usec = elapsed.as_micros();
                // vel is picometer/second. From pico to micro is 1000000
                let travel_distance = (self.internal_vel as u128 * time_usec) / 1_000_000;
                println!(
                    "mcs2axis::status_do_move elapsed time_usec={:?} vel={:?} pos={:?} pos_targ={:?} travel_distance={:?}",
                    time_usec, self.internal_vel, self.pos_act, self.pos_targ, travel_distance
                );
                if self.pos_targ > (self.pos_act - self.in_target_window) {
                    // need to move forward
                    self.pos_act += travel_distance as i64;
                    if self.pos_act > (self.pos_targ + self.in_target_window) {
                        // We are there
                        self.pos_act = self.pos_targ + self.in_target_window;
                        self.internal_mmod = -1;
                    } else {
                        self.time_pos_targ_started = SystemTime::now();
                        ret += CH_STATE_ACTIVELY_MOVING;
                    }
                } else if self.pos_targ < (self.pos_act + self.in_target_window) {
                    // need to move backard
                    self.pos_act -= travel_distance as i64;
                    if self.pos_act < (self.pos_targ - self.in_target_window) {
                        // We are there
                        self.pos_act = self.pos_targ - self.in_target_window;
                        self.internal_mmod = -1;
                    } else {
                        self.time_pos_targ_started = SystemTime::now();
                        ret += CH_STATE_ACTIVELY_MOVING;
                    }
                } else {
                    // inside the tolerance window
                    self.internal_mmod = -1;
                }
            }
            Err(e) => {
                println!("mcs2axis::status_do_move elapsed e={:?}", e);
            }
        }
        ret
    }

    pub fn get_status(&mut self) -> u32 {
        let mut ret = self.status_do_calibibrate();
        if ret == 0 {
            ret = self.status_do_reference();
        }
        if ret == 0 {
            ret = self.status_do_move();
        }
        if self.state_actively_moving {
            ret += CH_STATE_ACTIVELY_MOVING;
        }
        if self.state_closed_loop_active {
            ret += CH_STATE_CLOSED_LOOP_ACTIVE;
        }
        if self.state_move_delayed {
            ret += CH_STATE_MOVE_DELAYED
        }
        if self.state_sensor_present {
            ret += CH_STATE_SENSOR_PRESENT
        }
        if self.state_is_calibrated {
            ret += CH_STATE_IS_CALIBRATED
        }
        if self.state_is_referenced {
            ret += CH_STATE_IS_REFERENCED
        }
        if self.state_end_stop_reached {
            ret += CH_STATE_END_STOP_REACHED
        }
        if self.state_range_limit_reached {
            ret += CH_STATE_RANGE_LIMIT_REACHED
        }
        if self.state_following_limit_reached {
            ret += CH_STATE_FOLLOWING_LIMIT_REACHED
        }
        if self.state_movement_failed {
            ret += CH_STATE_MOVEMENT_FAILED
        }
        if self.state_streaming {
            ret += CH_STATE_STREAMING
        }
        if self.state_positioner_overload {
            ret += CH_STATE_POSITIONER_OVERLOAD
        }
        if self.state_overtemp {
            ret += CH_STATE_OVERTEMP
        }
        if self.state_reference_mark {
            ret += CH_STATE_REFERENCE_MARK
        }
        if self.state_is_phased {
            ret += CH_STATE_IS_PHASED
        }
        if self.state_positioner_fault {
            ret += CH_STATE_POSITIONER_FAULT
        }
        if self.state_amplifier_enabled {
            ret += CH_STATE_AMPLIFIER_ENABLED
        }
        if self.state_in_position {
            ret += CH_STATE_IN_POSITION
        }
        if self.state_brake_enabled {
            ret += CH_STATE_BRAKE_ENABLED
        }
        ret
    }
    /* TODO
    pub fn get_hold_time(&self) -> i32 {
        self.hold_time
    }
     */
    pub fn set_acc(&mut self, new_acc: f64) -> bool {
        println!(
            "mcs2axis::set_acc[{}] old={} new={}",
            &self.axis_no, self.acc, new_acc
        );
        if new_acc >= 0.0 {
            self.acc = new_acc;
            return true;
        }
        false
    }
    pub fn set_hold_time(&mut self, new_hold_time: i32) -> bool {
        println!(
            "mcs2axis::set_hold_time[{}] old={} new={}",
            &self.axis_no, self.hold_time, new_hold_time
        );
        if new_hold_time >= -1 {
            self.hold_time = new_hold_time;
            return true;
        }
        false
    }
    pub fn set_mclf(&mut self, new_mclf: i32) -> bool {
        println!(
            "mcs2axis::set_mclf[{}] old={} new={}",
            &self.axis_no, self.mclf, new_mclf
        );
        if new_mclf >= 1 {
            self.mclf = new_mclf;
            return true;
        }
        false
    }
    pub fn set_mmod(&mut self, new_mmod: i32) -> bool {
        println!(
            "mcs2axis::set_mmod{}] old={} new={}",
            &self.axis_no, self.mmod, new_mmod
        );
        if new_mmod >= 0 {
            self.mmod = new_mmod;
            return true;
        }
        false
    }

    pub fn set_ref_opt(&mut self, new_ref_opt: i32) -> bool {
        println!(
            "mcs2axis::set_ref_opt[{}] old={} new={}",
            &self.axis_no, self.ref_opt, new_ref_opt
        );
        if new_ref_opt >= 0 {
            self.ref_opt = new_ref_opt;
            return true;
        }
        false
    }
    pub fn set_sens_del(&mut self, new_sens_del: u32) -> bool {
        println!(
            "mcs2axis::set_sens_del{}] old={} new={}",
            &self.axis_no, self.sens_del, new_sens_del
        );
        if new_sens_del <= 5000 {
            self.sens_del = new_sens_del;
            return true;
        }
        false
    }
    pub fn set_sens_mode(&mut self, new_sens_mode: i32) -> bool {
        println!(
            "mcs2axis::set_sens_mode{}] old={} new={}",
            &self.axis_no, self.sens_mode, new_sens_mode
        );
        //if new_sens_mode >= 0 && new_sens_mode <= 2 {
        if (0..=2).contains(&new_sens_mode) {
            self.sens_mode = new_sens_mode;
            return true;
        }
        false
    }
    pub fn set_step_freq(&mut self, new_step_freq: i32) -> bool {
        println!(
            "mcs2axis::set_step_freq[{}] old={} new={}",
            &self.axis_no, self.step_freq, new_step_freq
        );
        if new_step_freq >= 0 {
            self.step_freq = new_step_freq;
            return true;
        }
        false
    }
    pub fn set_vel(&mut self, new_vel: f64) -> bool {
        println!(
            "mcs2axis::set_vel[{}] old={} new={}",
            &self.axis_no, self.external_vel, new_vel
        );
        if new_vel >= 0.0 {
            self.external_vel = new_vel;
            return true;
        }
        false
    }
}
