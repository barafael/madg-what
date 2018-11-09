use rand::prelude::*;

extern crate libc;
use libc::c_float;

// use f32
#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct Quaternion {
    pub a: c_float,
    pub b: c_float,
    pub c: c_float,
    pub d: c_float,
}

#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct Axis {
    pub x: c_float,
    pub y: c_float,
    pub z: c_float,
}

#[derive(Debug, Clone, Copy)]
pub struct Measurement {
    pub acc: Axis,
    pub gyro: Axis,
    pub mag: Axis,
}

pub trait RandomGenerate {
    fn generate() -> Self;
}

impl RandomGenerate for Axis {
    fn generate() -> Self {
        let mut rng = thread_rng();
        let x = (rng.gen::<f32>() * 10.0) as c_float;
        let y = (rng.gen::<f32>() * 10.0) as c_float;
        let z = (rng.gen::<f32>() * 10.0) as c_float;
        Axis { x, y, z }
    }
}

impl RandomGenerate for Measurement {
    fn generate() -> Self {
        let acc = Axis::generate();
        let gyro = Axis::generate();
        let mag = Axis::generate();

        Measurement { acc, gyro, mag }
    }
}

impl RandomGenerate for Quaternion {
    fn generate() -> Self {
        let mut rng = thread_rng();
        let a = (rng.gen::<f32>() * 10.0) as c_float;
        let b = (rng.gen::<f32>() * 10.0) as c_float;
        let c = (rng.gen::<f32>() * 10.0) as c_float;
        let d = (rng.gen::<f32>() * 10.0) as c_float;
        Quaternion { a, b, c, d }
    }
}
