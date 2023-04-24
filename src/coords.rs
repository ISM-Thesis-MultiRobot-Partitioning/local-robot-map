/// Create 3D coordinates. Assumes *meter* as the unit of measurement.
///
/// # Examples
///
/// ```
/// use local_robot_map::Coords;
/// let coords = Coords::new(1.0, 2.0, 3.0);
/// assert_eq!(coords.x, 1.0);
/// assert_eq!(coords.y, 2.0);
/// assert_eq!(coords.z, 3.0);
/// ```
#[derive(Debug, PartialEq)]
pub struct Coords {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Coords {
    pub fn new(x: f64, y: f64, z: f64) -> Coords {
        Coords { x, y, z }
    }
}
