use hxdmp::hexdump;
use regex::Regex;
use std::io::{self, Read, Write};
use std::net::{TcpListener, TcpStream};

mod mcs2axis;
use mcs2axis::Mcs2axis;

pub struct Netcommand {
    dump: bool,
    ip_port: u16,
    mcs2axes: Vec<Mcs2axis>,
    re_channel: Regex,
    re_cmd_axis_nr: Regex,
    re_cmd_signed: Regex,
    re_move_pos: Regex,
}

const NETWORK_BUFFER_SIZE: usize = 256; // TODO: check with real HW

impl Netcommand {
    pub fn new(dump: bool) -> Netcommand {
        let tmp_mcs2axes = vec![
            Mcs2axis::new(1),
            Mcs2axis::new(2),
            Mcs2axis::new(3),
            Mcs2axis::new(4),
            Mcs2axis::new(5),
            Mcs2axis::new(6),
        ];

        Netcommand {
            dump,
            ip_port: 55551,
            mcs2axes: tmp_mcs2axes,
            re_channel: Regex::new("^:CHAN([0-5]):(.*)$").unwrap(), // Support 6 channels
            re_cmd_axis_nr: Regex::new("^:([A-Za-z:]*)([0-5])$").unwrap(), // :REF1 :STOP1
            re_cmd_signed: Regex::new("^([A-Za-z:]*) ([-0-9.]*)$").unwrap(), // :CHAN1:ACC 10000000000.000000 both int and floating point
            re_move_pos: Regex::new("^:([A-Za-z]*)([0-5]) *([-0-9.]*)$").unwrap(), // :MOVE1 1000000000.000000
        }
    }

    /* Handle commands like this ":CHAN1:HOLD -1"
     * This function receives as parameters:
     *  1, "HOLD", "-1"
     */

    fn handle_channel_command_param_set_value(
        &mut self,
        ch_no: usize,
        cmd_key: String,
        cmd_val: String,
        mut _buf: [u8; NETWORK_BUFFER_SIZE],
    ) -> usize {
        if cmd_key == "ACC" {
            let val = cmd_val.parse::<f64>().unwrap();
            let _ok = &mut self.mcs2axes[ch_no].set_acc(val);
            return 0;
        } else if cmd_key == "HOLD" {
            let val = cmd_val.parse::<i32>().unwrap();
            let _ok = &mut self.mcs2axes[ch_no].set_hold_time(val);
            return 0;
        } else if cmd_key == "MCLF:CURR" || cmd_key == "MCLF" {
            let val = cmd_val.parse::<i32>().unwrap();
            let _ok = &mut self.mcs2axes[ch_no].set_mclf(val);
            return 0;
        } else if cmd_key == "MMOD" {
            let val = cmd_val.parse::<i32>().unwrap();
            let _ok = &mut self.mcs2axes[ch_no].set_mmod(val);
            return 0;
        } else if cmd_key == "REF:OPT" {
            let val = cmd_val.parse::<i32>().unwrap();
            let _ok = &mut self.mcs2axes[ch_no].set_ref_opt(val);
            return 0;
        } else if cmd_key == "SENS:DEL" {
            let val = cmd_val.parse::<u32>().unwrap();
            let _ok = &mut self.mcs2axes[ch_no].set_sens_del(val);
            return 0;
        } else if cmd_key == "SENS:MODE" {
            let val = cmd_val.parse::<i32>().unwrap();
            let _ok = &mut self.mcs2axes[ch_no].set_sens_mode(val);
            return 0;
        } else if cmd_key == "STEP:FREQ" {
            let val = cmd_val.parse::<i32>().unwrap();
            let _ok = &mut self.mcs2axes[ch_no].set_step_freq(val);
            return 0;
        } else if cmd_key == "VEL" {
            let val = cmd_val.parse::<f64>().unwrap();
            let _ok = &mut self.mcs2axes[ch_no].set_vel(val);
            return 0;
        }

        /*
                    let string =
                    &self.mcs2axes[ch_no].get_status().to_string();
                    let len = string.len();
                    buf[..len].copy_from_slice(string.as_bytes());
        */
        println!(
            "RX89 not handled: ch_no={} cmd_key={:?} cmd_val={:?}",
            ch_no, &cmd_key, &cmd_val
        );
        0 // return 0
    }

    fn handle_command_cal_ref_stop(
        &mut self,
        cmd_line: &str,
        _buf: &mut [u8; NETWORK_BUFFER_SIZE],
    ) -> usize {
        let captures = self
            .re_cmd_axis_nr
            .captures(cmd_line)
            .ok_or("cmd_axis_nr_match");
        if self.dump {
            println!("RX32: re_cmd_axis={:?}", &captures);
        }
        match captures {
            Ok(captures2) => {
                let ch_no_asc = &captures2[2];
                let ch_cmd = &captures2[1];
                let ch_no = ch_no_asc.parse::<usize>().unwrap();
                if true
                /* || self.dump */
                {
                    println!("RX33: ch_no_asc={:?} ch_cmd={:?}", &ch_no_asc, &ch_cmd);
                }
                // Handle the different requests.
                //  Try to keep the alphabetically ordered (in the source code)
                if ch_cmd == "CAL" {
                    let _ok = &mut self.mcs2axes[ch_no].do_cal();
                    return 0;
                } else if ch_cmd == "REF" {
                    let _ok = &mut self.mcs2axes[ch_no].do_ref();
                    return 0;
                } else if ch_cmd == "STOP" {
                    let _ok = &mut self.mcs2axes[ch_no].do_stop();
                    return 0;
                }
                println!("RX38: ch_no_asc={:?} ch_cmd={:?}", &ch_no_asc, &ch_cmd);
            }
            Err(e) => {
                println!("RX39: captures2 e={:?}", e);
            }
        }
        0
    }
    fn handle_command_move_pos(
        &mut self,
        cmd_line: &str,
        _buf: &mut [u8; NETWORK_BUFFER_SIZE],
    ) -> usize {
        let captures = self
            .re_move_pos
            .captures(cmd_line)
            .ok_or("cmd_move_pos_match");
        if self.dump {
            println!("RX34: re_move_pos={:?}", &captures);
        }
        match captures {
            Ok(captures2) => {
                let ch_cmd = &captures2[1];
                let ch_no_asc = &captures2[2];
                let ch_no = ch_no_asc.parse::<usize>().unwrap();
                let ch_pos_asc = &captures2[3];
                let ch_pos = ch_pos_asc.parse::<f64>().unwrap() as i64;

                if true
                /* || self.dump */
                {
                    println!(
                        "RX34: ch_cmd={:?} ch_no_asc={:?} ch_pos_asc={:?}",
                        &ch_cmd, &ch_no_asc, &ch_pos
                    );
                }
                // Handle the different requests.
                //  Try to keep the alphabetically ordered (in the source code)
                if ch_cmd == "MOVE" {
                    let _ok = &mut self.mcs2axes[ch_no].do_move(ch_pos);
                    return 0;
                }
                println!("RX38: ch_no_asc={:?} ch_cmd={:?}", &ch_no_asc, &ch_cmd);
            }
            Err(e) => {
                println!("RX39: captures2 e={:?}", e);
            }
        }
        0
    }

    fn handle_command_channel(
        &mut self,
        cmd_line: &str,
        buf: &mut [u8; NETWORK_BUFFER_SIZE],
    ) -> usize {
        let captures = self.re_channel.captures(cmd_line).ok_or("re_channel_match");
        if self.dump {
            println!("RX121: re_channel_match={:?}", &captures);
        }
        match captures {
            Ok(captures2) => {
                let ch_no_asc = &captures2[1];
                let ch_cmd = &captures2[2];
                let ch_no = ch_no_asc.parse::<usize>().unwrap();
                if self.dump {
                    println!("RX13: ch_no_asc={:?} ch_cmd={:?}", &ch_no_asc, &ch_cmd);
                }
                // Handle the different requests.
                //  Try to keep the alphabetically ordered (in the source code)
                if ch_cmd == "MCLF?" || ch_cmd == "MCLF.CURR?" {
                    let string = &self.mcs2axes[ch_no].get_mclf().to_string();
                    let len = string.len();
                    buf[..len].copy_from_slice(string.as_bytes());
                    return len;
                } else if ch_cmd == "POS?" {
                    let string = &self.mcs2axes[ch_no].get_pos_sensor().to_string();
                    let len = string.len();
                    buf[..len].copy_from_slice(string.as_bytes());
                    return len;
                } else if ch_cmd == "POS:TARG?" {
                    let string = &self.mcs2axes[ch_no].get_pos_targ().to_string();
                    let len = string.len();
                    buf[..len].copy_from_slice(string.as_bytes());
                    return len;
                } else if ch_cmd == "PTYP?" {
                    let string = "1"; // SA_CTL_STICK_SLIP_PIEZO_DRIVER (0x0001)
                    let len = string.len();
                    buf[..len].copy_from_slice(string.as_bytes());
                    return len;
                } else if ch_cmd == "SENS:DEL?" {
                    let string = &self.mcs2axes[ch_no].get_sens_del().to_string();
                    let len = string.len();
                    buf[..len].copy_from_slice(string.as_bytes());
                    return len;
                } else if ch_cmd == "SENS:MODE?" {
                    let string = &self.mcs2axes[ch_no].get_sens_mode().to_string();
                    let len = string.len();
                    buf[..len].copy_from_slice(string.as_bytes());
                    return len;
                } else if ch_cmd == "STAT?" {
                    let string = &self.mcs2axes[ch_no].get_status().to_string();
                    let len = string.len();
                    buf[..len].copy_from_slice(string.as_bytes());
                    return len;
                } else {
                    let re_cmd_signed_match = self.re_cmd_signed.is_match(ch_cmd);
                    if self.dump {
                        println!("RX21: re_cmd_signed_match={}", re_cmd_signed_match);
                    }
                    if re_cmd_signed_match {
                        let captures = self
                            .re_cmd_signed
                            .captures(ch_cmd)
                            .ok_or("re_cmd_signed_match");
                        if self.dump {
                            println!("RX22: re_cmd_signed_match={:?}", &captures);
                        }
                        match captures {
                            Ok(captures2) => {
                                let cmd_key = &captures2[1];
                                let cmd_val = &captures2[2];
                                if self.dump {
                                    println!("RX23: cmd_key={:?} cmd_val={:?}", &cmd_key, &cmd_val);
                                }
                                return Self::handle_channel_command_param_set_value(
                                    self,
                                    ch_no,
                                    cmd_key.to_string(),
                                    cmd_val.to_string(),
                                    *buf,
                                );
                            }
                            Err(e) => {
                                println!("RX29: captures2 e={:?}", e);
                            }
                        }
                    }
                }
                println!("RX29: ch_cmd={:?}", ch_cmd);
            }
            Err(e) => {
                println!("RX19: captures2 e={:?}", e);
            }
        }
        0
    }
    /* function to handle one line comming into the controller.
      Input: The line.
      Return: the len of bytes filled into the buffer. May be 0
      Note: The original controller can handle 2 or more commands
      in one line, seperated by ';'
      This is not supported here
    */
    fn handle_command_all_of_them(
        &mut self,
        cmd_line: &str,
        buf: &mut [u8; NETWORK_BUFFER_SIZE],
    ) -> usize {
        if cmd_line == ":SYST:ERR:COUN?" {
            let string = "0";
            let len = string.len();
            buf[..len].copy_from_slice(string.as_bytes());
            return len;
        } else if cmd_line == ":DEV:SNUM?" {
            let string = "20241111";
            let len = string.len();
            buf[..len].copy_from_slice(string.as_bytes());
            return len;
        } else {
            /*
               I think that unwrap() is OK here. If we can not create
               the regex, it doesn't make sense to continue
               https://blog.burntsushi.net/unwrap/
            */
            let re_channel_match = self.re_channel.is_match(cmd_line);
            if self.dump {
                println!("RX11: re_channel_match={}", re_channel_match);
            }
            if re_channel_match {
                return Self::handle_command_channel(self, cmd_line, buf);
            }
            let re_cmd_axis_nr_match = self.re_cmd_axis_nr.is_match(cmd_line);
            if re_cmd_axis_nr_match {
                return Self::handle_command_cal_ref_stop(self, cmd_line, buf);
            }
            let re_re_move_pos_match = self.re_move_pos.is_match(cmd_line);
            if self.dump {
                println!("RX123 re_re_move_pos_match={}", re_re_move_pos_match);
            }
            if re_re_move_pos_match {
                return Self::handle_command_move_pos(self, cmd_line, buf);
            }
        }
        println!("RX99: cmd_line={:?}", cmd_line);
        0 // return 0
    }

    /*
          Function to handle one client.
          The simulated controler handles one client at time,
          and so do we here. Otherwise: A multi-threaded approach would be needed
    */
    fn handle_client(&mut self, mut stream: TcpStream) -> io::Result<()> {
        println!("Accepted: {:?}", &mut stream);
        let mut buf_recv = [0u8; NETWORK_BUFFER_SIZE];
        loop {
            let result = stream.read(&mut buf_recv);
            match result {
                Ok(read_len) => {
                    if read_len == 0 {
                        println!("RX0: read_len={}", read_len);
                        break; // End Of File, connection closed by remote
                    }
                    let mut has_cr_lf = 0;
                    let mut len_without_crlf = read_len;
                    if read_len >= 2 {
                        // test for CRLF or LF
                        if buf_recv[read_len - 1] == b'\n' {
                            if buf_recv[read_len - 2] == b'\r' {
                                has_cr_lf = 2; // CRLF
                            } else {
                                has_cr_lf = 1; // LF
                            }
                        }
                    }
                    len_without_crlf -= has_cr_lf;

                    let buf_str = std::str::from_utf8(&buf_recv[0..len_without_crlf]);
                    match buf_str {
                        Ok(buf_str) => {
                            if has_cr_lf > 0 {
                                if self.dump {
                                    println!(
                                        "RX1: read_len={} has_cr_lf={} buf=\"{}\"",
                                        read_len, has_cr_lf, &buf_str
                                    );
                                }
                                for line in buf_str.split("\r\n") {
                                    if self.dump {
                                        println!("RX1: line=\"{}\"", &line);
                                    }
                                    let mut buf_resp = [0u8; NETWORK_BUFFER_SIZE];
                                    let len =
                                        Self::handle_command_all_of_them(self, line, &mut buf_resp);
                                    if self.dump {
                                        let buf_resp_str = std::str::from_utf8(&buf_resp[..len])
                                            .expect("valid utf");
                                        println!("TX1: len={} buf=\"{}\"", len, buf_resp_str);
                                    }
                                    if len > 0 {
                                        // Always use CRLF
                                        buf_resp[len] = b'\r';
                                        buf_resp[len + 1] = b'\n';
                                        stream.write_all(&buf_resp[..len + 2])?;
                                    }
                                }
                                //println!("=======================");
                            } else {
                                let mut buf_hex = Vec::with_capacity(read_len * 4);
                                hexdump(&buf_recv[0..read_len], &mut buf_hex)?;
                                let buf_hex_str = std::str::from_utf8(&buf_hex).expect("valid utf");
                                if self.dump {
                                    println!("RX2: {}", buf_hex_str);
                                }
                            }
                        }
                        Err(e) => {
                            let mut buf_hex = Vec::with_capacity(read_len * 4);
                            hexdump(&buf_recv[0..read_len], &mut buf_hex)?;
                            let buf_hex_str = std::str::from_utf8(&buf_hex).expect("valid utf");
                            println!("RX3: error={} buf=\'{}\'", e, buf_hex_str);
                            break;
                        }
                    }
                }
                Err(e) => {
                    println!("RX4: error reading: {}", e);
                    break;
                }
            }
        }
        Ok(())
    }

    pub fn simulate(mut self) {
        let adr_port_str = format!("{}:{}", "0.0.0.0", self.ip_port);
        let listener = TcpListener::bind(&adr_port_str);
        if self.dump {
            println!("LISTEN: {} {:?}", &adr_port_str, &listener);
        }

        // accept connections and process them serially
        //for stream in listener.incoming() {
        for stream in listener.expect("REASON").incoming() {
            match stream {
                Ok(stream) => {
                    let res = Self::handle_client(&mut self, stream);
                    println!("handle_client: finished {:?}", res);
                }
                Err(e) => {
                    println!("LISTEN: failed {}", e);
                }
            }
        }
        //Ok(())
    }
}
