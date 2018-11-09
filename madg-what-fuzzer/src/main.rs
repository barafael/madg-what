extern crate clap;
use clap::{App, Arg};

extern crate dlopen;
extern crate dlopen_derive;
extern crate rand;
use dlopen::symbor::Library;

use std::fs;
use std::path::Path;

use std::fmt;

extern crate libc;
use libc::c_float;

use dlopen::utils::PLATFORM_FILE_EXTENSION;

use std::collections::HashSet;

use rand::prelude::*;

#[derive(Debug, Clone, Copy)]
#[repr(C)]
struct Quaternion {
    pub a: c_float,
    pub b: c_float,
    pub c: c_float,
    pub d: c_float,
}

#[derive(Debug, Clone, Copy)]
#[repr(C)]
struct Axis {
    pub x: c_float,
    pub y: c_float,
    pub z: c_float,
}

#[derive(Debug, Clone, Copy)]
struct Measurement {
    pub acc: Axis,
    pub gyro: Axis,
    pub mag: Axis,
}

struct FilterLib {
    filename: String,
    handle: Library,
}

#[derive(Debug)]
struct TestBench {
    libs: Vec<FilterLib>,
}

#[derive(Debug)]
struct TestRun {
    measurement: Measurement,
    results: Vec<(String, Option<Quaternion>)>,
}

trait MadgwickFilter {
    fn madgwick_filter(&self, measurement: Measurement) -> Option<Quaternion>;
    fn set_beta(&self, beta: f32);
    fn set_deltat(&self, deltat: f32);
}

impl MadgwickFilter for FilterLib {
    fn madgwick_filter(&self, measurement: Measurement) -> Option<Quaternion> {
        let sym = unsafe {
            self.handle
                .symbol::<unsafe extern "C" fn(acc: Axis, gyro: Axis, mag: Axis) -> Quaternion>(
                    "madgwick_filter",
                )
        };
        match sym {
            Ok(filter) => {
                Some(unsafe { filter(measurement.acc, measurement.gyro, measurement.mag) })
            }
            Err(e) => {
                eprintln!("{}", e);
                None
            }
        }
    }

    fn set_beta(&self, beta: f32) {
        let sym = unsafe {
            self.handle
                .symbol::<unsafe extern "C" fn(c_float)>("set_beta")
        };
        match sym {
            Ok(set) => {
                unsafe { set(beta as c_float) };
            }
            Err(e) => {
                eprintln!("{}", e);
            }
        }
    }

    fn set_deltat(&self, deltat: f32) {
        let sym = unsafe {
            self.handle
                .symbol::<unsafe extern "C" fn(c_float)>("set_deltat")
        };
        match sym {
            Ok(set) => {
                unsafe { set(deltat as c_float) };
            }
            Err(e) => {
                eprintln!("{}", e);
            }
        }
    }
}

impl fmt::Debug for FilterLib {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Filename: {}", self.filename)
    }
}

impl TestBench {
    pub fn run(&self, measurement: Measurement) -> TestRun {
        let mut result = TestRun {
            measurement,
            results: Vec::new(),
        };
        for lib in self.libs.iter() {
            let quat = lib.madgwick_filter(measurement);
            result.results.push((lib.filename.clone(), quat));
        }
        result
    }
}

trait RandomGenerate {
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

fn main() {
    let matches = App::new("madg-what-fuzzer")
        .version("0.1")
        .author("Rafael Bachmann bachmann@fim.uni-passau.de")
        .about("Fuzz data fusion algorithms")
        .arg(
            Arg::with_name("file")
                .takes_value(true)
                .min_values(1)
                .short("f"),
        )
        .arg(
            Arg::with_name("dir")
                .takes_value(true)
                .min_values(1)
                .short("d"),
        )
        .get_matches();

    let mut files = HashSet::<String>::new();

    if let Some(f) = matches.values_of("file") {
        for path in f.collect::<Vec<&str>>().iter() {
            if Path::new(path).exists() {
                files.insert(path.to_string());
            }
        }
    }

    if let Some(d) = matches.values_of("dir") {
        for dir in d.collect::<Vec<&str>>().iter() {
            let entries = match fs::read_dir(dir) {
                Ok(entries) => entries,
                Err(e) => {
                    eprintln!("{}", e);
                    continue;
                }
            };
            for entry in entries {
                let dir_entry = match entry {
                    Ok(entry) => entry,
                    Err(e) => {
                        eprintln!("{}", e);
                        continue;
                    }
                };
                let file_path = dir_entry.path();
                if !(file_path.exists() && file_path.is_file()) {
                    continue;
                }
                file_path.extension().map(|ext| {
                    if ext == PLATFORM_FILE_EXTENSION {
                        if let Some(s) = file_path.to_str() {
                            files.insert(String::from(s));
                        }
                    }
                });
            }
        }
    }

    println!("{:#?}", files);

    let mut test_bench = TestBench { libs: Vec::new() };

    for filename in files {
        let lib = Library::open(&filename);
        match lib {
            Err(e) => eprintln!("{}", e),
            Ok(handle) => {
                test_bench.libs.push(FilterLib { filename, handle });
            }
        }
    }

    println!("{:#?}", test_bench);

    let measurement = Measurement::generate();

    let result = test_bench.run(measurement);

    println!("{:#?}", result);
}
