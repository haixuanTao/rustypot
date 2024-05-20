//! MX robotis register (protocol v1)
//!
//! Despite some minor differences among MX variants, it should work for
//! * XM430-W350
//!
//! See <chttps://emanual.robotis.com/docs/en/dxl/xm/xm430-w350/> for example.

use crate::device::*;

reg_read_only!(model_number, 0, u16);
reg_read_only!(firmware_version, 6, u8);
reg_read_write!(id, 7, u8);
reg_read_write!(baud_rate, 8, u8);
reg_read_write!(return_delay_time, 9, u8);
reg_read_write!(drive_mode, 10, u8);
reg_read_write!(operating_mode, 11, u8);
reg_read_write!(secondary_shadow_id, 12, u8);
reg_read_write!(protocol_type, 13, u8);
reg_read_write!(homing_offset, 20, i32);
reg_read_write!(moving_threshold, 24, u32);
reg_read_write!(temperature_limit, 31, u8);
reg_read_write!(max_voltage_limit, 32, u16);
reg_read_write!(min_voltage_limit, 34, u16);
reg_read_write!(acceleration_limit, 40, u32);
reg_read_write!(torque_limit, 48, u16);
reg_read_write!(velocity_limit, 44, u32);
reg_read_write!(max_position_limit, 48, u32);
reg_read_write!(min_position_limit, 52, u32);

reg_read_write!(shutdown, 63, u8);
reg_read_write!(torque_enable, 64, u8);
reg_read_write!(led, 65, u8);
reg_read_write!(status_return_level, 68, u8);
reg_read_write!(registered_instruction, 69, u8);
reg_read_write!(hardware_error_status, 70, u8);
reg_read_write!(velocity_i_gain, 76, u16);
reg_read_write!(velocity_p_gain, 78, u16);
reg_read_write!(position_d_gain, 80, u16);
reg_read_write!(position_i_gain, 82, u16);
reg_read_write!(position_p_gain, 84, u16);
reg_read_write!(feedforward_2nd_gain, 88, u16);
reg_read_write!(feedforward_1st_gain, 90, u16);
reg_read_write!(bus_watchdog, 98, u8);

reg_read_write!(goal_pwm, 100, u16);
reg_read_write!(goal_velocity, 104, u32);
reg_read_write!(profile_acceleration, 108, u32);
reg_read_write!(profile_velocity, 112, u32);
reg_read_write!(goal_position, 116, u32);
reg_read_only!(realtime_tick, 120, u16);
reg_read_only!(moving, 122, u8);
reg_read_only!(moving_status, 123, u8);
reg_read_only!(present_pwm, 124, u16);
reg_read_only!(present_current, 126, u16);
reg_read_only!(present_velocity, 128, u32);
reg_read_only!(present_position, 132, u32);
reg_read_only!(velocity_trajectory, 136, u32);
reg_read_only!(position_trajectory, 140, u32);
reg_read_only!(present_input_voltage, 144, u16);
reg_read_only!(present_temperature, 146, u8);

/// Sync read present_position, present_speed and present_current in one message
///
/// reg_read_only!(present_position_speed_current, 36, (i16, u16, u16))
pub fn sync_read_present_position_speed_load(
    io: &DynamixelSerialIO,
    serial_port: &mut dyn serialport::SerialPort,
    ids: &[u8],
) -> Result<Vec<(u32, u32, u16)>> {
    let val = io.sync_read(serial_port, ids, 126, 2 + 4 + 4)?;
    let val = val
        .iter()
        .map(|v| {
            (
                u32::from_le_bytes(v[6..10].try_into().unwrap()),
                u32::from_le_bytes(v[2..6].try_into().unwrap()),
                u16::from_le_bytes(v[0..2].try_into().unwrap()),
            )
        })
        .collect();

    Ok(val)
}

/// Unit conversion for XL-320 motors
pub mod conv {
    /// Dynamixel angular position to radians
    ///
    /// Works in joint and multi-turn mode
    pub fn pos_to_radians(pos: u32) -> f64 {
        (360.0_f64.to_radians() * (pos as f64) / 4096.0) - 180.0_f64.to_radians()
    }
    /// Radians to dynamixel angular position
    ///
    /// Works in joint and multi-turn mode
    pub fn radians_to_pos(rads: f64) -> u32 {
        (4096.0 * (180.0_f64.to_radians() + rads) / 360.0_f64.to_radians()) as u32
    }
    /// Dynamixel angular velocity to radians per second
    ///
    /// Works in joint and multi-turn mode
    pub fn abs_speed_to_rad_per_sec(speed: u32) -> f64 {
        let rpm = speed as f64 * 0.229;
        rpm * 0.10472
    }
}
