use crate::{
    coords::InternalLocation, AxisResolution, Coords, Location, LocationError,
    LocationType, MapStateMatrix, Mask, RealWorldLocation, Visualize,
};
use num::cast::ToPrimitive;

use image::{ImageBuffer, RgbImage};

/// Describe a map using a 2D grid of cells.
///
/// Note that only the `x` and `y` components of [`Coords`] are used, and the
/// `z` component will be ignored.
///
/// Another important thing to note is that real-world coordinates are to be
/// provided and output from the [`CellMap`].
///
/// # Example
///
/// ```
/// use local_robot_map::{AxisResolution, CellMap, Coords, RealWorldLocation};
///
/// let point1 = RealWorldLocation::from_xyz(-1.0, -2.0, 0.0);
/// let point2 = RealWorldLocation::from_xyz(0.5, 1.0, 0.0);
/// let resolution = AxisResolution::uniform(2.0);
///
/// let map = CellMap::new(point1, point2, resolution);
///
/// assert_eq!(
///     map.resolution(),
///     &AxisResolution {
///         x: 2.0,
///         y: 2.0,
///         z: 2.0
///     }
/// );
/// assert_eq!(map.offset(), &Coords::new(-1.0, -2.0, 0.0));
/// assert_eq!(map.width(), 3);
/// assert_eq!(map.height(), 6);
/// ```
///
/// Here is what happens if the cells don't perfectly fit. The width in
/// this case should be `1.5`, but we cannot have *half a cell*. As you can
/// see, the value will simply be truncated and removes anything after the
/// floating point. The resulting imprecision of the map can be remedied by
/// setting a higher `resolution` like in the previous example.
///
/// ```
/// use local_robot_map::{AxisResolution, CellMap, RealWorldLocation};
///
/// let point1 = RealWorldLocation::from_xyz(-1.0, -2.0, 0.0);
/// let point2 = RealWorldLocation::from_xyz(0.5, 1.0, 0.0);
/// let resolution = AxisResolution::uniform(1.0);
///
/// let map = CellMap::new(point1, point2, resolution);
///
/// assert_eq!(map.width(), 1);
/// assert_eq!(map.height(), 3);
/// ```
///
/// ```
/// use local_robot_map::{AxisResolution, CellMap, RealWorldLocation};
///
/// let point1 = RealWorldLocation::from_xyz(-1.0, -2.0, 0.0);
/// let point2 = RealWorldLocation::from_xyz(0.5, 1.0, 0.0);
/// let resolution = AxisResolution::uniform(1.0);
///
/// let map = CellMap::new(point1, point2, resolution);
///
/// assert_eq!(map.width(), 1);
/// assert_eq!(map.height(), 3);
/// ```
#[derive(Debug, PartialEq)]
pub struct CellMap {
    /// A matrix representing the cells along with their states.
    cells: MapStateMatrix,
    /// Cell resolution, assumed in *pixels per meter*.
    resolution: AxisResolution,
    /// Matrices usually cannot have negative indices, which prevents the
    /// representation of negative real-world coordinates. To this end, an
    /// offset is calculated to bring the map's bounding box's bottom left
    /// corner to `Coords { x: 0.0, y: 0.0, z: 0.0 }`. Even positive
    /// coordinates will be shifted as a matter of consistency.
    offset: Coords,
}

impl CellMap {
    /// Create a new [`CellMap`]. It takes 2 [`Coords`] indicating the square
    /// bounding box area. The resolution affects how many pixels/cells per
    /// meter will be generated.
    pub fn new(
        point1: RealWorldLocation,
        point2: RealWorldLocation,
        resolution: AxisResolution,
    ) -> Self {
        let columns = point1.distance_x(&point2) * resolution.x;
        let rows = point1.distance_y(&point2) * resolution.y;

        let offset = Coords {
            x: point1.x.min(point2.x),
            y: point1.y.min(point2.y),
            z: point1.z.min(point2.z),
        };

        Self {
            cells: MapStateMatrix::from_elem(
                (
                    rows.to_usize().expect("No conversion issues"),
                    columns.to_usize().expect("No conversion issues"),
                ),
                LocationType::Unexplored,
            ),
            resolution,
            offset,
        }
    }

    /// Manually create a [`CellMap`] based off an existing matrix.
    ///
    /// Note that the values passed on to this function will be taken *as-is*.
    /// This means that there are no checks to ensure the `resolution` and
    /// `offset` were correctly specified.
    pub fn from_raster(
        cells: MapStateMatrix,
        resolution: AxisResolution,
        offset: Coords,
    ) -> Self {
        Self {
            cells,
            resolution,
            offset,
        }
    }

    /// Convert a floating point location into its corresponding
    /// [`MapStateMatrix`] cell index.
    ///
    /// If conversion was succcessful, it returns the `[row, col]` index to be
    /// used on the `[MapStateMatrix]`; see [`ndarray`
    /// slicing](ndarray::ArrayBase#indexing-and-dimension).
    ///
    /// # Errors
    ///
    /// This function will return an error if .
    ///
    /// # More details on what it does
    ///
    /// The internal [`CellMap`] cannot represent every real-world location. The
    /// *resolution* specifies how fine grained the map shall be in order to
    /// represent the locations more precisely. Higher resolution will provide
    /// more precsision, whereas lower resolution will make the distinction of
    /// locations on a smaller scale impossible.
    ///
    /// That said, one can think of the *resolution* as a way to subdivide a
    /// region into smaller subregions. In the case of a 2D map, we would
    /// end up with smaller rectangular subdivisions. Converting the real-world
    /// floating point location then consists in identifying which such
    /// rectangle the location in question corresponds to.
    pub fn location_to_map_index(
        &self,
        location: &RealWorldLocation,
    ) -> Result<[usize; 2], LocationError> {
        let coord: InternalLocation = match location
            .clone()
            .into_internal(self.offset, self.resolution)
        {
            Ok(c) => c,
            Err((location_error, _)) => return Err(location_error),
        };

        let col: usize =
            coord.x().floor().to_usize().expect(
                "An overflow likely occured when converting f64 to usize",
            );
        let row: usize =
            coord.y().floor().to_usize().expect(
                "An overflow likely occured when converting f64 to usize",
            );

        if col >= self.width() || row >= self.height() {
            return Err(LocationError::OutOfMap);
        };

        Ok([row, col])
    }

    pub fn resolution(&self) -> &AxisResolution {
        &self.resolution
    }
    pub fn offset(&self) -> &Coords {
        &self.offset
    }
    pub fn cells(&self) -> &MapStateMatrix {
        &self.cells
    }
    pub fn ncols(&self) -> usize {
        self.cells().ncols()
    }
    pub fn nrows(&self) -> usize {
        self.cells().nrows()
    }
    pub fn width(&self) -> usize {
        self.ncols()
    }
    pub fn height(&self) -> usize {
        self.nrows()
    }
}

impl Visualize for CellMap {
    type ImageType = RgbImage;

    fn as_image(&self) -> Self::ImageType {
        ImageBuffer::from_fn(
            self.width().to_u32().expect("No conversion issues"),
            self.height().to_u32().expect("No conversion issues"),
            |x, y| -> image::Rgb<_> {
                let row = y.to_usize().expect("No conversion issues");
                let col = x.to_usize().expect("No conversion issues");
                let cell: LocationType = self.cells[[row, col]];
                cell.to_rgb()
            },
        )
    }
}

impl Mask for CellMap {
    fn get_map_region(
        &self,
        filter: impl Fn(LocationType) -> bool,
    ) -> Vec<Cell> {
        self.cells
            .indexed_iter()
            .filter(|((_, _), e)| filter(**e))
            .map(|((row, col), e)| {
                Cell::new(
                    InternalLocation::new(
                        Coords::new(
                            col.to_f64().expect("usize to f64 should work"),
                            row.to_f64().expect("usize to f64 should work"),
                            0.0,
                        ),
                        *self.offset(),
                        *self.resolution(),
                    )
                    .expect("indexed_iter() will not return negative indexes"),
                    e,
                )
            })
            .collect()
    }
}

impl Location for CellMap {
    fn get_location(
        &self,
        coord: &RealWorldLocation,
    ) -> Result<LocationType, crate::LocationError> {
        let index = self.location_to_map_index(coord)?;
        Ok(self.cells[index])
    }

    fn set_location(
        &mut self,
        coord: &RealWorldLocation,
        value: LocationType,
    ) -> Result<(), crate::LocationError> {
        let index = self.location_to_map_index(coord)?;
        self.cells[index] = value;
        Ok(())
    }
}

#[derive(Debug, PartialEq)]
pub struct Cell<'a> {
    location: RealWorldLocation,
    value: &'a LocationType,
}

impl<'a> Cell<'a> {
    pub(crate) fn new(
        location: InternalLocation,
        value: &'a LocationType,
    ) -> Self {
        Self {
            location: location.into_real_world(),
            value,
        }
    }

    pub fn location(&self) -> &RealWorldLocation {
        &self.location
    }
    pub fn x(&self) -> &f64 {
        &self.location.x
    }
    pub fn y(&self) -> &f64 {
        &self.location.y
    }
    pub fn value(&self) -> &'a LocationType {
        self.value
    }
}

#[cfg(test)]
pub mod tests {
    use std::collections::HashMap;

    use crate::MaskMapState;

    use super::*;

    pub fn make_map() -> (CellMap, Coords) {
        let ms = HashMap::from([
            ("OOM", LocationType::OutOfMap),
            ("OTR", LocationType::OtherRobot),
            ("MYR", LocationType::MyRobot),
            ("EXP", LocationType::Explored),
            ("UNE", LocationType::Unexplored),
            ("FNT", LocationType::Frontier),
            ("ASS", LocationType::Assigned),
        ]);

        let offset = Coords::new(0.0, 0.0, 0.0);
        let cell = CellMap::from_raster(
            MapStateMatrix::from_shape_vec(
                (5, 3),
                vec![
                    *ms.get("OOM").unwrap(),
                    *ms.get("OTR").unwrap(),
                    *ms.get("MYR").unwrap(), //
                    *ms.get("FNT").unwrap(),
                    *ms.get("UNE").unwrap(),
                    *ms.get("EXP").unwrap(), //
                    *ms.get("ASS").unwrap(),
                    *ms.get("OOM").unwrap(),
                    *ms.get("OTR").unwrap(), //
                    *ms.get("MYR").unwrap(),
                    *ms.get("UNE").unwrap(),
                    *ms.get("ASS").unwrap(), //
                    *ms.get("UNE").unwrap(),
                    *ms.get("EXP").unwrap(),
                    *ms.get("FNT").unwrap(), //
                ],
            )
            .unwrap(),
            AxisResolution::uniform(1.0),
            offset,
        );

        (cell, offset)
    }

    #[test]
    fn create_cell_map_one_by_one() {
        let map = CellMap::new(
            RealWorldLocation::from_xyz(0.0, 0.0, 0.0),
            RealWorldLocation::from_xyz(1.0, 1.0, 0.0),
            AxisResolution::uniform(1.0),
        );
        assert_eq!(
            map.resolution(),
            &AxisResolution {
                x: 1.0,
                y: 1.0,
                z: 1.0
            }
        );
        assert_eq!(map.width(), 1);
        assert_eq!(map.height(), 1);
        assert_eq!(
            map.offset(),
            &Coords {
                x: 0.0,
                y: 0.0,
                z: 0.0
            }
        );
    }

    #[test]
    fn create_cell_map_one_by_one_negative() {
        let map = CellMap::new(
            RealWorldLocation::from_xyz(0.0, 0.0, 0.0),
            RealWorldLocation::from_xyz(-1.0, -1.0, 0.0),
            AxisResolution::uniform(1.0),
        );
        assert_eq!(
            map.resolution(),
            &AxisResolution {
                x: 1.0,
                y: 1.0,
                z: 1.0
            }
        );
        assert_eq!(map.width(), 1);
        assert_eq!(map.height(), 1);
        assert_eq!(
            map.offset(),
            &Coords {
                x: -1.0,
                y: -1.0,
                z: 0.0
            }
        );
    }

    #[test]
    fn create_cell_map_offset() {
        let (x, y) = (14.26, 95.21);
        let map = CellMap::new(
            RealWorldLocation::from_xyz(x, y, 0.0),
            RealWorldLocation::from_xyz(x + 1.0, y + 1.0, 0.0),
            AxisResolution::uniform(1.0),
        );
        assert_eq!(
            map.resolution(),
            &AxisResolution {
                x: 1.0,
                y: 1.0,
                z: 1.0
            }
        );
        assert_eq!(map.width(), 1);
        assert_eq!(map.height(), 1);
        assert_eq!(map.offset(), &Coords { x, y, z: 0.0 });
    }

    #[test]
    fn create_cell_map_offset_negative() {
        let (x, y) = (-126.83, -7165.1137);
        let map = CellMap::new(
            RealWorldLocation::from_xyz(x, y, 0.0),
            RealWorldLocation::from_xyz(x + 1.0, y + 1.0, 0.0),
            AxisResolution::uniform(1.0),
        );
        assert_eq!(
            map.resolution(),
            &AxisResolution {
                x: 1.0,
                y: 1.0,
                z: 1.0
            }
        );
        assert_eq!(map.width(), 1);
        assert_eq!(map.height(), 1);
        assert_eq!(map.offset(), &Coords { x, y, z: 0.0 });
    }

    #[test]
    fn create_cell_map_resolution() {
        let map = CellMap::new(
            RealWorldLocation::from_xyz(0.0, 0.0, 0.0),
            RealWorldLocation::from_xyz(1.0, 1.0, 0.0),
            AxisResolution::uniform(7.0),
        );
        assert_eq!(
            map.resolution(),
            &AxisResolution {
                x: 7.0,
                y: 7.0,
                z: 7.0
            }
        );
        assert_eq!(map.width(), 7);
        assert_eq!(map.height(), 7);
        assert_eq!(
            map.offset(),
            &Coords {
                x: 0.0,
                y: 0.0,
                z: 0.0
            }
        );
    }

    #[test]
    fn create_cell_map_resolution_negative() {
        let map = CellMap::new(
            RealWorldLocation::from_xyz(0.0, 0.0, 0.0),
            RealWorldLocation::from_xyz(-1.0, -1.0, 0.0),
            AxisResolution::uniform(7.0),
        );
        assert_eq!(
            map.resolution(),
            &AxisResolution {
                x: 7.0,
                y: 7.0,
                z: 7.0
            }
        );
        assert_eq!(map.width(), 7);
        assert_eq!(map.height(), 7);
        assert_eq!(
            map.offset(),
            &Coords {
                x: -1.0,
                y: -1.0,
                z: 0.0
            }
        );
    }

    #[test]
    fn create_cell_map_dimension() {
        let map = CellMap::new(
            RealWorldLocation::from_xyz(1.0, 3.0, 0.0),
            RealWorldLocation::from_xyz(10.0, 4.0, 0.0),
            AxisResolution::uniform(1.0),
        );
        assert_eq!(
            map.resolution(),
            &AxisResolution {
                x: 1.0,
                y: 1.0,
                z: 1.0
            }
        );
        assert_eq!(map.width(), 9);
        assert_eq!(map.height(), 1);
        assert_eq!(
            map.offset(),
            &Coords {
                x: 1.0,
                y: 3.0,
                z: 0.0
            }
        );
    }

    #[test]
    fn create_cell_map_dimension_negative() {
        let map = CellMap::new(
            RealWorldLocation::from_xyz(-10.0, -4.0, 0.0),
            RealWorldLocation::from_xyz(1.0, 3.0, 0.0),
            AxisResolution::uniform(1.0),
        );
        assert_eq!(
            map.resolution(),
            &AxisResolution {
                x: 1.0,
                y: 1.0,
                z: 1.0
            }
        );
        assert_eq!(map.width(), 11);
        assert_eq!(map.height(), 7);
        assert_eq!(
            map.offset(),
            &Coords {
                x: -10.0,
                y: -4.0,
                z: 0.0
            }
        );
    }

    #[test]
    fn submap_get_map_region() {
        let (map, offset) = make_map();

        let cells = map.get_map_region(|e| e == LocationType::OutOfMap);

        assert_eq!(cells.len(), 2);
        assert_eq!(
            cells,
            vec![
                Cell::new(
                    InternalLocation::new(
                        Coords::new(0.0, 0.0, 0.0),
                        offset,
                        *map.resolution()
                    )
                    .unwrap(),
                    &LocationType::OutOfMap
                ),
                Cell::new(
                    InternalLocation::new(
                        Coords::new(1.0, 2.0, 0.0),
                        offset,
                        *map.resolution()
                    )
                    .unwrap(),
                    &LocationType::OutOfMap
                ),
            ]
        );
    }

    #[test]
    fn submap_get_map_region_high_resolution() {
        const OOM: LocationType = LocationType::OutOfMap;
        const OTR: LocationType = LocationType::OtherRobot;
        let offset = Coords::new(-1.0, -1.0, 0.0);
        let map = CellMap::from_raster(
            MapStateMatrix::from_shape_vec(
                (10, 10),
                vec![
                    OTR, OOM, OOM, OOM, OOM, OOM, OOM, OOM, OOM, OOM, //
                    OOM, OOM, OOM, OOM, OOM, OOM, OOM, OOM, OOM, OOM, //
                    OOM, OOM, OOM, OOM, OOM, OOM, OOM, OOM, OOM, OOM, //
                    OOM, OOM, OOM, OOM, OOM, OOM, OOM, OOM, OTR, OOM, //
                    OOM, OOM, OOM, OOM, OOM, OOM, OOM, OOM, OOM, OOM, //
                    OOM, OOM, OOM, OOM, OOM, OTR, OOM, OOM, OOM, OOM, //
                    OOM, OOM, OOM, OOM, OOM, OOM, OOM, OOM, OOM, OOM, //
                    OOM, OOM, OOM, OOM, OOM, OOM, OOM, OOM, OOM, OOM, //
                    OOM, OOM, OOM, OOM, OOM, OOM, OOM, OOM, OOM, OOM, //
                    OOM, OOM, OOM, OOM, OOM, OOM, OOM, OOM, OOM, OOM, //
                ],
            )
            .unwrap(),
            AxisResolution::uniform(5.0),
            offset,
        );

        let cells = map.get_map_region(|e| e == OTR);

        assert_eq!(cells.len(), 3);
        assert_eq!(
            cells,
            vec![
                Cell::new(
                    InternalLocation::new(
                        Coords::new(0.0, 0.0, 0.0),
                        offset,
                        *map.resolution()
                    )
                    .unwrap(),
                    &OTR
                ),
                Cell::new(
                    InternalLocation::new(
                        Coords::new(8.0, 3.0, 0.0),
                        offset,
                        *map.resolution()
                    )
                    .unwrap(),
                    &OTR
                ),
                Cell::new(
                    InternalLocation::new(
                        Coords::new(5.0, 5.0, 0.0),
                        offset,
                        *map.resolution()
                    )
                    .unwrap(),
                    &OTR
                ),
            ]
        );
    }

    #[test]
    fn submap_get_out_of_map() {
        let (map, offset) = make_map();

        let cells = map.get_map_state(LocationType::OutOfMap);

        assert_eq!(cells.len(), 2);
        assert_eq!(
            cells,
            vec![
                Cell::new(
                    InternalLocation::new(
                        Coords::new(0.0, 0.0, 0.0),
                        offset,
                        *map.resolution()
                    )
                    .unwrap(),
                    &LocationType::OutOfMap
                ),
                Cell::new(
                    InternalLocation::new(
                        Coords::new(1.0, 2.0, 0.0),
                        offset,
                        *map.resolution()
                    )
                    .unwrap(),
                    &LocationType::OutOfMap
                ),
            ]
        );
    }

    #[test]
    fn submap_get_explored() {
        let (map, offset) = make_map();

        let cells = map.get_map_state(LocationType::Explored);

        assert_eq!(cells.len(), 2);
        assert_eq!(
            cells,
            vec![
                Cell::new(
                    InternalLocation::new(
                        Coords::new(2.0, 1.0, 0.0),
                        offset,
                        *map.resolution()
                    )
                    .unwrap(),
                    &LocationType::Explored
                ),
                Cell::new(
                    InternalLocation::new(
                        Coords::new(1.0, 4.0, 0.0),
                        offset,
                        *map.resolution()
                    )
                    .unwrap(),
                    &LocationType::Explored
                ),
            ]
        );
    }

    #[test]
    fn submap_get_unexplored() {
        let (map, offset) = make_map();

        let cells = map.get_map_state(LocationType::Unexplored);

        assert_eq!(cells.len(), 3);
        assert_eq!(
            cells,
            vec![
                Cell::new(
                    InternalLocation::new(
                        Coords::new(1.0, 1.0, 0.0),
                        offset,
                        *map.resolution()
                    )
                    .unwrap(),
                    &LocationType::Unexplored
                ),
                Cell::new(
                    InternalLocation::new(
                        Coords::new(1.0, 3.0, 0.0),
                        offset,
                        *map.resolution()
                    )
                    .unwrap(),
                    &LocationType::Unexplored
                ),
                Cell::new(
                    InternalLocation::new(
                        Coords::new(0.0, 4.0, 0.0),
                        offset,
                        *map.resolution()
                    )
                    .unwrap(),
                    &LocationType::Unexplored
                ),
            ]
        );
    }

    #[test]
    fn submap_get_frontier() {
        let (map, offset) = make_map();

        let cells = map.get_map_state(LocationType::Frontier);

        assert_eq!(cells.len(), 2);
        assert_eq!(
            cells,
            vec![
                Cell::new(
                    InternalLocation::new(
                        Coords::new(0.0, 1.0, 0.0),
                        offset,
                        *map.resolution()
                    )
                    .unwrap(),
                    &LocationType::Frontier
                ),
                Cell::new(
                    InternalLocation::new(
                        Coords::new(2.0, 4.0, 0.0),
                        offset,
                        *map.resolution()
                    )
                    .unwrap(),
                    &LocationType::Frontier
                ),
            ]
        );
    }

    #[test]
    fn submap_get_assigned() {
        let (map, offset) = make_map();

        let cells = map.get_map_state(LocationType::Assigned);

        assert_eq!(cells.len(), 2);
        assert_eq!(
            cells,
            vec![
                Cell::new(
                    InternalLocation::new(
                        Coords::new(0.0, 2.0, 0.0),
                        offset,
                        *map.resolution()
                    )
                    .unwrap(),
                    &LocationType::Assigned
                ),
                Cell::new(
                    InternalLocation::new(
                        Coords::new(2.0, 3.0, 0.0),
                        offset,
                        *map.resolution()
                    )
                    .unwrap(),
                    &LocationType::Assigned
                ),
            ]
        );
    }

    #[test]
    fn save_map_to_png() {
        let (map, _) = make_map();
        map.as_image().save("test_save_map.png").unwrap();
    }

    #[test]
    fn location_index_origin() {
        let (map, _) = make_map();
        let index = map
            .location_to_map_index(&RealWorldLocation::from_xyz(0.0, 0.0, 0.0))
            .unwrap();
        assert_eq!(index, [0, 0]);
    }

    #[test]
    fn location_index_inside() {
        let (map, _) = make_map();
        let index = map
            .location_to_map_index(&RealWorldLocation::from_xyz(2.4, 3.8, 0.0))
            .unwrap();
        assert_eq!(index, [3, 2]);
    }

    #[test]
    fn location_index_inside_high_resolution() {
        let map = CellMap::new(
            RealWorldLocation::from_xyz(-1.0, -1.0, -1.0),
            RealWorldLocation::from_xyz(1.0, 1.0, 1.0),
            AxisResolution::uniform(3.0),
        );
        let index = map
            .location_to_map_index(&RealWorldLocation::from_xyz(0.1, -0.3, 0.0))
            .unwrap();
        assert_eq!(index, [2, 3]);
    }

    #[test]
    fn location_index_inside_uneven_high_resolution() {
        let map = CellMap::new(
            RealWorldLocation::from_xyz(-1.0, -1.0, -1.0),
            RealWorldLocation::from_xyz(1.0, 1.0, 1.0),
            AxisResolution::new(7.0, 3.0, 1.0),
        );
        let index = map
            .location_to_map_index(&RealWorldLocation::from_xyz(0.1, -0.3, 0.0))
            .unwrap();
        assert_eq!(index, [2, 7]);
    }

    #[test]
    fn location_index_far_corner() {
        let (map, _) = make_map();
        let index = map
            .location_to_map_index(&RealWorldLocation::from_xyz(
                map.width() as f64 - 0.3,
                map.height() as f64 - 0.7,
                0.0,
            ))
            .unwrap();
        assert_eq!(index, [map.nrows() - 1, map.ncols() - 1]);
    }

    #[test]
    fn location_index_too_far_right() {
        let (map, _) = make_map();
        let index = map.location_to_map_index(&RealWorldLocation::from_xyz(
            map.width() as f64 + 1.0,
            0.0,
            0.0,
        ));
        assert_eq!(index, Err(LocationError::OutOfMap));
    }

    #[test]
    fn location_index_too_far_left() {
        let (map, _) = make_map();
        let index = map.location_to_map_index(&RealWorldLocation::from_xyz(
            -1.0, 0.0, 0.0,
        ));
        assert_eq!(index, Err(LocationError::OutOfMap));
    }

    #[test]
    fn location_index_too_far_up() {
        let (map, _) = make_map();
        let index = map.location_to_map_index(&RealWorldLocation::from_xyz(
            0.0,
            map.height() as f64 + 1.0,
            0.0,
        ));
        assert_eq!(index, Err(LocationError::OutOfMap));
    }

    #[test]
    fn location_index_too_far_down() {
        let (map, _) = make_map();
        let index = map.location_to_map_index(&RealWorldLocation::from_xyz(
            0.0, -1.0, 0.0,
        ));
        assert_eq!(index, Err(LocationError::OutOfMap));
    }
}
