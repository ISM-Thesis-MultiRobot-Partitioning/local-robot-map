use std::ops::{Add, Deref, Sub};

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
#[derive(Debug, PartialEq, Clone, Copy)]
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

    pub fn x(&self) -> f64 {
        self.x
    }
    pub fn y(&self) -> f64 {
        self.y
    }
    pub fn z(&self) -> f64 {
        self.z
    }
}

impl Add for Coords {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl Sub for Coords {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

/// Explicitly describe real world coordinates.
///
/// A thin wrapper around [`Coords`] which allows making a clear distinction
/// between *real world* coordinates, and *internal* ones. Real world
/// coordinates are what entities outside this crate use.
///
/// # Reasoning behind this
///
/// Internally, such coordinates are likely to require offsetting due to the
/// internal data structures. For instance, negative real world coordinates
/// would lead to negative indexing in the matrix representing the map. While a
/// custom matrix type could be created for this exact issue, we opted to offset
/// the coordinates to accomodate for such cases. There are a few other cases
/// where offsetting takes place internally, but it is outside the scope of this
/// description.
///
/// See [`RealWorldLocation::into_internal`] for more details.
#[derive(Debug, PartialEq)]
pub struct RealWorldLocation {
    /// The location in terms of real world coordinates.
    location: Coords,
}

impl RealWorldLocation {
    pub fn new(location: Coords) -> Self {
        Self { location }
    }

    /// Construct a real world location using x, y, and z coordinates.
    ///
    /// Think of it as a convenience function which takes care of creating the
    /// actual [`Coords`] type for you. See also [`Coords::new`].
    pub fn from_xyz(x: f64, y: f64, z: f64) -> Self {
        Self::new(Coords::new(x, y, z))
    }

    /// Translate from real-world coordinates to internal ones.
    ///
    /// # What is happening
    ///
    /// To visualize what exactly is happening, consider we draw a bounding box
    /// around a set of locations. We consider its bottom left corner to
    /// be the origin of the internal reference frame. Now, the coordinate of
    /// that origin in the real world reference frame indicates our offset.
    /// So we can bring the coordinates from the real world reference frame
    /// into the internal reference frame by translating them using this
    /// offset.
    ///
    /// # Example
    ///
    /// Check out the unit tests for examples.
    pub(crate) fn into_internal(self, offset: Coords) -> InternalLocation {
        InternalLocation::new(self.location - offset, offset)
    }

    pub fn location(&self) -> &Coords {
        &self.location
    }
    pub fn x(&self) -> f64 {
        self.location().x
    }
    pub fn y(&self) -> f64 {
        self.location().y
    }
    pub fn z(&self) -> f64 {
        self.location().z
    }
}

impl Deref for RealWorldLocation {
    type Target = Coords;

    fn deref(&self) -> &Self::Target {
        self.location()
    }
}

pub(crate) struct InternalLocation {
    location: Coords,
    offset: Coords,
}

impl InternalLocation {
    /// Creates a new [`InternalLocation`].
    ///
    /// # Assumption
    ///
    /// The `location` is the already offset coordinate; this function performs
    /// no calculations. See [`RealWorldLocation::into_internal`] for more
    /// details.
    pub(crate) fn new(location: Coords, offset: Coords) -> Self {
        Self { location, offset }
    }

    /// Translate from internal location back to the original real-world one.
    pub(crate) fn into_real_world(self) -> RealWorldLocation {
        RealWorldLocation::new(self.location + self.offset)
    }

    /// Recompute the internal location given a new offset.
    ///
    /// Note that the offset is given in real world coordinates and not relative
    /// to the existing offset (i.e. you provide the same offset you would
    /// provide to [`RealWorldLocation::into_internal`]). The implementation
    /// should take care of calculating the relative offset, and thus alleviate
    /// the programmer.
    pub(crate) fn change_offset(self, offset: Coords) -> Self {
        self.into_real_world().into_internal(offset)
    }

    pub(crate) fn location(&self) -> &Coords {
        &self.location
    }
    pub(crate) fn x(&self) -> f64 {
        self.location().x
    }
    pub(crate) fn y(&self) -> f64 {
        self.location().y
    }
    pub(crate) fn z(&self) -> f64 {
        self.location().z
    }
}

/// Struct specifiying the *resolution* (i.e. how many pixels) per axis.
///
/// See also: [`crate::PolygonMap::to_cell_map`] and [`crate::CellMap::new`].
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

#[cfg(test)]
mod tests {
    use super::*;

    /// Considering the explanation from [`RealWorldLocation::into_internal`],
    /// we can draw a bounding box around the following locations whose
    /// origin sits at `(-1.0, -1.0, -1.0)`; this will be our offset into
    /// internal coordinates.
    #[test]
    fn external_to_internal_coords() {
        let external_locations = vec![
            RealWorldLocation::new(Coords::new(-1.0, -1.0, -1.0)),
            RealWorldLocation::new(Coords::new(0.0, 0.0, 0.0)),
            RealWorldLocation::new(Coords::new(1.0, 1.0, 1.0)),
        ];

        let internal_locations: Vec<InternalLocation> = external_locations
            .into_iter()
            .map(|loc| loc.into_internal(Coords::new(-1.0, -1.0, -1.0)))
            .collect();

        assert_eq!(
            internal_locations
                .iter()
                .map(|iloc| iloc.location())
                .collect::<Vec<&Coords>>(),
            vec![
                &Coords::new(0.0, 0.0, 0.0),
                &Coords::new(1.0, 1.0, 1.0),
                &Coords::new(2.0, 2.0, 2.0),
            ]
        )
    }

    /// Same example as before, but this time we go from the internal back to
    /// external. Note that the internal coordinates will store the offset
    /// that they were given, so we need to construct them via the external
    /// coordinates.
    #[test]
    fn internal_to_external_coords() {
        let offset = Coords::new(-1.0, -1.0, -1.0);
        let internal_locations: Vec<InternalLocation> = vec![
            InternalLocation::new(Coords::new(0.0, 0.0, 0.0), offset),
            InternalLocation::new(Coords::new(1.0, 1.0, 1.0), offset),
            InternalLocation::new(Coords::new(2.0, 2.0, 2.0), offset),
        ];

        let external_locations: Vec<RealWorldLocation> = internal_locations
            .into_iter()
            .map(|loc| loc.into_real_world())
            .collect();

        assert_eq!(
            external_locations
                .iter()
                .map(|loc| loc.location())
                .collect::<Vec<&Coords>>(),
            vec![
                &Coords::new(-1.0, -1.0, -1.0),
                &Coords::new(0.0, 0.0, 0.0),
                &Coords::new(1.0, 1.0, 1.0),
            ]
        )
    }

    /// It is possible that internal locations need to be offset to accomodate
    /// for new locations. This should be a transparent process where one simply
    /// gives it the new offset in the real world reference frame (the same
    /// one that would be used to convert from [`RealWorldLocation`] to
    /// [`InternalLocation`]).
    #[test]
    fn internal_new_offset() {
        let internal_locations: Vec<InternalLocation> = vec![
            RealWorldLocation::new(Coords::new(-1.0, -1.0, -1.0)),
            RealWorldLocation::new(Coords::new(0.0, 0.0, 0.0)),
            RealWorldLocation::new(Coords::new(1.0, 1.0, 1.0)),
        ]
        .into_iter()
        .map(|loc| loc.into_internal(Coords::new(-1.0, -1.0, -1.0)))
        .collect();

        let offset_internal_locations: Vec<InternalLocation> =
            internal_locations
                .into_iter()
                .map(|iloc| iloc.change_offset(Coords::new(-2.0, -2.0, -2.0)))
                .collect();

        assert_eq!(
            offset_internal_locations
                .iter()
                .map(|iloc| iloc.location())
                .collect::<Vec<&Coords>>(),
            vec![
                &Coords::new(1.0, 1.0, 1.0),
                &Coords::new(2.0, 2.0, 2.0),
                &Coords::new(3.0, 3.0, 3.0),
            ]
        )
    }

    #[test]
    fn internal_new_offset_matches_external() {
        let internal_locations: Vec<InternalLocation> = vec![
            RealWorldLocation::new(Coords::new(-1.0, -1.0, -1.0)),
            RealWorldLocation::new(Coords::new(0.0, 0.0, 0.0)),
            RealWorldLocation::new(Coords::new(1.0, 1.0, 1.0)),
        ]
        .into_iter()
        .map(|loc| loc.into_internal(Coords::new(-1.0, -1.0, -1.0)))
        .collect();

        let offset_internal_locations: Vec<InternalLocation> =
            internal_locations
                .into_iter()
                .map(|iloc| iloc.change_offset(Coords::new(-2.0, -2.0, -2.0)))
                .collect();

        let external_locations: Vec<RealWorldLocation> =
            offset_internal_locations
                .into_iter()
                .map(|loc| loc.into_real_world())
                .collect();

        assert_eq!(
            external_locations
                .iter()
                .map(|loc| loc.location())
                .collect::<Vec<&Coords>>(),
            vec![
                &Coords::new(-1.0, -1.0, -1.0),
                &Coords::new(0.0, 0.0, 0.0),
                &Coords::new(1.0, 1.0, 1.0),
            ]
        )
    }
}
