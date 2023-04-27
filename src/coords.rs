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
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// Compute distance between two points along the `x` axis.
    ///
    /// # Example
    ///
    /// ```
    /// use local_robot_map::Coords;
    ///
    /// let p1 = Coords::new(0.0, 0.0, 0.0);
    /// let p2 = Coords::new(1.0, 2.0, 3.0);
    /// assert_eq!(p1.distance_x(&p2), 1.0);
    /// ```
    pub fn distance_x(&self, other: &Self) -> f64 {
        (other.x - self.x).abs()
    }

    /// Compute distance between two points along the `y` axis.
    ///
    /// # Example
    /// ```
    /// use local_robot_map::Coords;
    ///
    /// let p1 = Coords::new(0.0, 0.0, 0.0);
    /// let p2 = Coords::new(1.0, 2.0, 3.0);
    /// assert_eq!(p1.distance_y(&p2), 2.0);
    /// ```
    pub fn distance_y(&self, other: &Self) -> f64 {
        (other.y - self.y).abs()
    }

    /// Compute distance between two points along the `z` axis.
    ///
    /// # Example
    /// ```
    /// use local_robot_map::Coords;
    ///
    /// let p1 = Coords::new(0.0, 0.0, 0.0);
    /// let p2 = Coords::new(1.0, 2.0, 3.0);
    /// assert_eq!(p1.distance_z(&p2), 3.0);
    /// ```
    pub fn distance_z(&self, other: &Self) -> f64 {
        (other.z - self.z).abs()
    }

    /// Compute the euclidean distance between two points in space.
    ///
    /// Note that the 3D distance is computed, meaning the `z` component is
    /// being taken into account even though most of the implementation only
    /// considers the `x` and `y` components.
    ///
    /// # Example
    ///
    /// ```
    /// use local_robot_map::Coords;
    ///
    /// let p1 = Coords::new(0.0, 0.0, 0.0);
    /// let p2 = Coords::new(1.0, 1.0, 1.0);
    /// assert_eq!(p1.distance(&p2), 3.0_f64.sqrt());
    /// ```
    ///
    /// A 2D Euclidean distance will be computed if one of the coordinate
    /// components for both points is the same (i.e. both points lie in the
    /// same plane). This is simply a matter of the math, no specifc
    /// implementation is given for this case.
    ///
    /// ```
    /// use local_robot_map::Coords;
    ///
    /// let random_z_value = 85476.23545;
    /// let p1 = Coords::new(0.0, 0.0, random_z_value);
    /// let p2 = Coords::new(1.0, 1.0, random_z_value);
    /// assert_eq!(p1.distance(&p2), 2.0_f64.sqrt());
    /// ```
    pub fn distance(&self, other: &Self) -> f64 {
        (self.distance_x(other).powi(2)
            + self.distance_y(other).powi(2)
            + self.distance_z(other).powi(2))
        .sqrt()
    }
}

/// Struct specifiying the *resolution* (i.e. how many pixels) per axis.
///
/// See also: [`Polygon::to_cell_map`] and [`CellMap::new`].
///
/// # Example
///
/// To illustrate, we shall ignore the `z` component and pretend we are working
/// in 2D space.
///
/// The following means we have one pixel per square meter.
///
/// ```
/// use local_robot_map::{AxisResolution, CellMap, Coords};
///
/// let map = CellMap::new(
///     Coords::new(0.0, 0.0, 0.0),
///     Coords::new(1.0, 1.0, 0.0),
///     AxisResolution::uniform(1.0),
/// );
/// assert_eq!(map.width(), 1);
/// assert_eq!(map.height(), 1);
/// ```
///
/// The following means we have four pixels per square meter. We have 2 pixels
/// along the `x` axis, and 2 pixels along the `y` axis, which will subdivide
/// the square into 4.
///
/// ```
/// use local_robot_map::{AxisResolution, CellMap, Coords};
///
/// let map = CellMap::new(
///     Coords::new(0.0, 0.0, 0.0),
///     Coords::new(1.0, 1.0, 0.0),
///     AxisResolution::uniform(2.0),
/// );
/// assert_eq!(map.width(), 2);
/// assert_eq!(map.height(), 2);
/// ```
///
/// The following means we have one pixel per meter on the X axis, and 10 pixels
/// per meter on the Y axis.
///
/// ```
/// use local_robot_map::{AxisResolution, CellMap, Coords};
///
/// let map = CellMap::new(
///     Coords::new(0.0, 0.0, 0.0),
///     Coords::new(1.0, 1.0, 0.0),
///     AxisResolution::new(1.0, 10.0, 0.0),
/// );
/// assert_eq!(map.width(), 1);
/// assert_eq!(map.height(), 10);
/// ```
#[derive(Debug, PartialEq)]
pub struct AxisResolution {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl AxisResolution {
    /// Create an [`AxisResolution`]
    ///
    /// # Example
    ///
    /// ```
    /// use local_robot_map::AxisResolution;
    /// let resolution = AxisResolution::new(1.0, 2.0, 3.0);
    /// assert_eq!(
    ///     resolution,
    ///     AxisResolution {
    ///         x: 1.0,
    ///         y: 2.0,
    ///         z: 3.0
    ///     }
    /// );
    /// ```
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// Create [`AxisResolution`] with the same resolution for each axis
    ///
    /// # Example
    ///
    /// ```
    /// use local_robot_map::AxisResolution;
    /// let resolution = AxisResolution::uniform(1.0);
    /// assert_eq!(
    ///     resolution,
    ///     AxisResolution {
    ///         x: 1.0,
    ///         y: 1.0,
    ///         z: 1.0
    ///     }
    /// );
    /// ```
    pub fn uniform(resolution: f64) -> Self {
        Self {
            x: resolution,
            y: resolution,
            z: resolution,
        }
    }
}
