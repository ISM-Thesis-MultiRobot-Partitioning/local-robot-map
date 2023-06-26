//! Library crate to assist in managing local robot maps in the domain of
//! multi-robot coverage tasks.
//!
//! Robots generally need information about the environment they operate in, and
//! maps are a simple way to store this information. The case being made here is
//! more specifically on multi-robot exploration where one tries to obtain full
//! area coverage. However, care is being put into ensuring that the
//! implemenation can generalize to adjacent disciplines outside of terrain
//! mapping (i.e. perform full map coverage, determine resource concentrations,
//! etc.)
//!
//! The aim is to use this library in a decentralized setting, hence the *local*
//! part in the name. Each robot should hold its own version of the map, upon
//! which it can make decisions. Synchronizing of the maps across robots is
//! outside the scope of this library; this one merely provides a basis on which
//! to get started.

mod cell_map;
mod coords;
mod local_map;
mod polygon_map;

pub use cell_map::Cell;
pub use cell_map::CellMap;
pub use coords::AxisResolution;
pub use coords::Coords;

pub use coords::RealWorldLocation;
use ndarray::Array2;
pub use polygon_map::{PolygonMap, PolygonMapError};

pub use local_map::LocalMap;

pub type LocationType = MapState;
pub type MapStateMatrix = Array2<LocationType>;
/// The function signature which the partitioning algorithm should have.
///
/// `T` is the type of the map to be partitioned. The function is intended to
/// consume the map and then return a "new" one.
///
/// `F` is a type which captures partitioning factors. They can be used to
/// influence how the partitions are made, for example a robot's speed could be
/// such a factor and used for weighting other metrics.
///
/// Note that `F` is given as an [`Option`], allowing to not pass any additional
/// factors beyond what is already encoded in the map `T`.
pub(crate) type Algorithm<T, F> = fn(T, Option<F>) -> T;

/// Visualize a map.
pub trait Visualize {
    /// Type of the image.
    type ImageType;
    /// Convert the map to an image.
    ///
    /// The [`image::ImageBuffer`] type is an interesting target. Its
    /// [`image::ImageBuffer::from_fn`] function allows converting from any
    /// arbitrary data towards and [`image::ImageBuffer`].
    ///
    /// # Usage
    ///
    /// - This can be used to save the map to a file (using
    ///   [`image::ImageBuffer::save`]).
    /// - Provides an easier to work with interface for inspecting the map
    ///   (matrices tend to have quirks which can make them less intuitive).
    ///
    /// # Implementation tip
    ///
    /// Check out the [`LocationType::to_luma`] and [`LocationType::to_rgb`]
    /// functions. They provide a central method for converting the
    /// [`LocationType`] variants to colors that can be used by the
    /// [`image::ImageBuffer`] being output in this function.
    fn as_image(&self) -> Self::ImageType;
    /// Visualize the map using a GUI window.
    ///
    /// # Panics
    ///
    /// As of now, **no implementation is given**. Attempting to call
    /// the function will panic due to a `todo!` macro. Writing the image to a
    /// file appears to be the more straigthforward way to do display it.
    ///
    /// Future note: this function should make use of [`Visualize::as_image`].
    /// This allows providing a default implementation.
    fn visualize(&self) {
        todo!("How to display images using a GUI window?");
    }
}

/// Partitiong the map.
///
/// This trait requires implementing a partitioning algorithm.
/// Upon calling the `partition()` function, the map will be consumed and a new
/// map with updated information will be returned.
///
/// # Intended usage
///
/// This is essentially at the heart of what we aimed this implementation
/// to provide, so no default implementation is given.
///
/// The overarching idea was to allow multiple partitioning schemes to be
/// implemented, which can be done by creating multiple crates/modules which
/// each implement the partitioning in any way they see fit.
pub trait Partition<F> {
    /// Consumes the map and returns the partitioned version thereof.
    fn partition(mut self, factors: Option<F>) -> Result<Self, PartitionError>
    where
        Self: Sized,
    {
        let partition_algorithm = self
            .get_partition_algorithm()?
            .take()
            .expect("Partitioning algorithm was provided");
        let mut map: Self = partition_algorithm(self, factors);
        map.set_partition_algorithm(partition_algorithm);
        Ok(map)
    }
    fn set_partition_algorithm(&mut self, algorithm: Algorithm<Self, F>);
    fn get_partition_algorithm(
        &mut self,
    ) -> Result<&mut Option<Algorithm<Self, F>>, PartitionError>;
}

#[derive(Debug, PartialEq)]
pub enum PartitionError {
    /// No algorithm was provided for partitioning.
    NoPartitioningAlgorithm,
    /// No (suitable) map was provided for partitioning.
    /// See also [`PolygonMapError::NotEnoughVertices`]
    NoMap,
}

/// Retrieve a subarea of the map based on a condition.
pub trait Mask {
    /// Retrieve a subarea of the map by filtering the locations based on a
    /// condition.
    fn get_map_region(
        &self,
        filter: impl Fn(LocationType) -> bool,
    ) -> Vec<Cell>;
}

/// Retrieve a subarea of the map based on a [`MapState`]
///
/// This trait is automatically implemented when [`Mask`] is implemented, and
/// the type `T` can be compared to a [`MapState`].
pub trait MaskMapState {
    fn get_map_state(&self, state: MapState) -> Vec<Cell>;
}

impl<T: Mask> MaskMapState for T {
    fn get_map_state(&self, state: MapState) -> Vec<Cell> {
        self.get_map_region(|e| e == state)
    }
}

/// Describe states of locations in the map.
///
/// For example, in the case of a [`CellMap`] it allows indicating what the
/// state of each cell is. The [`Mask`] trait allows filtering of the map
/// according to these states.
#[derive(PartialEq, Copy, Clone, Debug)]
pub enum MapState {
    /// Indicates the location is outside the map region (mostly relevant for
    /// non-square maps such as those which can be produced by [`PolygonMap`])
    OutOfMap,
    /// Indicates the location is occupied by another robot
    OtherRobot,
    /// Indicates the location is occupied by the current robot
    MyRobot,
    /// Indicates the location has already been explored by some robot
    Explored,
    /// Indicates the location has not been explored yet
    Unexplored,
    /// Indicates the location is a frontier which marks boundary between
    /// [`MapState::Explored`] and [`MapState::Unexplored`]
    Frontier,
    /// Indicates the location is assigned to the current robot
    Assigned,
}

impl MapState {
    /// Returns a corresponding [`image::Luma`] of the [`MapState`].
    ///
    /// This is useful when trying to visualize the map. Can be
    /// used when implementing [`Visualize::as_image`].
    pub fn to_luma(&self) -> image::Luma<u8> {
        self.into()
    }

    /// Returns a corresponding [`image::Rgb`] of the [`MapState`].
    ///
    /// This is useful when trying to visualize the map. Can be
    /// used when implementing [`Visualize::as_image`].
    pub fn to_rgb(&self) -> image::Rgb<u8> {
        self.into()
    }
}

impl From<&MapState> for &str {
    fn from(value: &MapState) -> Self {
        match value {
            MapState::OutOfMap => "OutOfMap",
            MapState::OtherRobot => "OtherRobot",
            MapState::MyRobot => "MyRobot",
            MapState::Explored => "Explored",
            MapState::Unexplored => "Unexplored",
            MapState::Frontier => "Frontier",
            MapState::Assigned => "Assigned",
        }
    }
}

impl From<&MapState> for image::Luma<u8> {
    fn from(value: &MapState) -> Self {
        use image::Luma;
        match value {
            MapState::OutOfMap => Luma([0]),
            MapState::OtherRobot => Luma([40]),
            MapState::MyRobot => Luma([50]),
            MapState::Explored => Luma([180]),
            MapState::Unexplored => Luma([120]),
            MapState::Frontier => Luma([220]),
            MapState::Assigned => Luma([255]),
        }
    }
}

impl From<&MapState> for image::Rgb<u8> {
    fn from(value: &MapState) -> Self {
        use image::Rgb;
        match value {
            MapState::OutOfMap => Rgb([0, 0, 0]),
            MapState::OtherRobot => Rgb([50, 255, 50]),
            MapState::MyRobot => Rgb([255, 50, 50]),
            MapState::Explored => Rgb([200, 200, 200]),
            MapState::Unexplored => Rgb([100, 100, 100]),
            MapState::Frontier => Rgb([255, 100, 255]),
            MapState::Assigned => Rgb([255, 255, 0]),
        }
    }
}

/// Transparently translate between real-world coordinates and internal matrix
/// coordinates.
///
/// It is tricky to make sure we are working in the right coordinate frame.
/// For the outside world and users, it would ideally be possible to work with
/// real world coordinates. However, the implementation often requires
/// converting these coordiantes to an internal reference frame. Hence the need
/// arises to convert between real-world and internal coordinates.
///
/// This trait provides functions that make common actions easier to work with
/// to the outside world and users. It does so by ensuring that only real-world
/// coordinates are being input and output from these trait functions. The
/// functions then take care of transparently converting the coordinates
/// accordingly.
pub trait Location {
    /// Retrieve the value at the given location.
    ///
    /// If the location can be successfully accessed, an `Ok(value)` will be
    /// returned.
    ///
    /// # Errors
    ///
    /// This function will return an error if there was an issue accessing the
    /// location. See [`LocationError`] for details.
    fn get_location(
        &self,
        coord: &RealWorldLocation,
    ) -> Result<LocationType, LocationError>;
    /// Updates the given location in the map with a new value.
    ///
    /// If a value was already present at the given location, it should be
    /// overwritten by the new value. The function returns `Ok(())` if the value
    /// was successfully replaced.
    ///
    /// # Errors
    ///
    /// Same as [`Location::get_location`].
    fn set_location(
        &mut self,
        coord: &RealWorldLocation,
        value: LocationType,
    ) -> Result<(), LocationError>;
}

#[derive(Debug, PartialEq)]
pub enum LocationError {
    /// The requested location is outside the map area and cannot be accessed.
    OutOfMap,
}
