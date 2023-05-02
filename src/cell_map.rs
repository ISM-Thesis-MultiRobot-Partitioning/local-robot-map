use crate::{
    AxisResolution, Coords, MapState, MapStateMatrix, Mask, Visualize,
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
/// use local_robot_map::{AxisResolution, CellMap, Coords, MapState};
///
/// let point1 = Coords::new(-1.0, -2.0, 0.0);
/// let point2 = Coords::new(0.5, 1.0, 0.0);
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
/// use local_robot_map::{AxisResolution, CellMap, Coords, MapState};
///
/// let point1 = Coords::new(-1.0, -2.0, 0.0);
/// let point2 = Coords::new(0.5, 1.0, 0.0);
/// let resolution = AxisResolution::uniform(1.0);
///
/// let map = CellMap::new(point1, point2, resolution);
///
/// assert_eq!(map.width(), 1);
/// assert_eq!(map.height(), 3);
/// ```
///
/// ```
/// use local_robot_map::{AxisResolution, CellMap, Coords, MapState};
///
/// let point1 = Coords::new(-1.0, -2.0, 0.0);
/// let point2 = Coords::new(0.5, 1.0, 0.0);
/// let resolution = AxisResolution::uniform(1.0);
///
/// let map = CellMap::new(point1, point2, resolution);
///
/// assert_eq!(map.width(), 1);
/// assert_eq!(map.height(), 3);
/// ```
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
        point1: Coords,
        point2: Coords,
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
                MapState::Unexplored,
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
                let cell: MapState = self.cells[[row, col]];
                cell.to_rgb()
            },
        )
    }
}

impl Mask for CellMap {
    type CellType = MapState;

    fn get_map_region(
        &self,
        filter: impl Fn(Self::CellType) -> bool,
    ) -> Vec<Cell> {
        self.cells
            .indexed_iter()
            .filter(|((_, _), e)| filter(**e))
            .map(|((row, col), e)| Cell::new(col, row, e))
            .collect()
    }
}

#[derive(Debug, PartialEq)]
pub struct Cell<'a> {
    x: usize,
    y: usize,
    value: &'a MapState,
}

impl<'a> Cell<'a> {
    pub fn new(x: usize, y: usize, value: &'a MapState) -> Self {
        Self { x, y, value }
    }

    pub fn x(&self) -> usize {
        self.x
    }
    pub fn y(&self) -> usize {
        self.y
    }
    pub fn value(&self) -> &'a MapState {
        self.value
    }
}

#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use crate::MaskMapState;

    use super::*;

    #[test]
    fn create_cell_map_one_by_one() {
        let map = CellMap::new(
            Coords::new(0.0, 0.0, 0.0),
            Coords::new(1.0, 1.0, 0.0),
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
            Coords::new(0.0, 0.0, 0.0),
            Coords::new(-1.0, -1.0, 0.0),
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
            Coords::new(x, y, 0.0),
            Coords::new(x + 1.0, y + 1.0, 0.0),
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
            Coords::new(x, y, 0.0),
            Coords::new(x + 1.0, y + 1.0, 0.0),
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
            Coords::new(0.0, 0.0, 0.0),
            Coords::new(1.0, 1.0, 0.0),
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
            Coords::new(0.0, 0.0, 0.0),
            Coords::new(-1.0, -1.0, 0.0),
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
            Coords::new(1.0, 3.0, 0.0),
            Coords::new(10.0, 4.0, 0.0),
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
            Coords::new(-10.0, -4.0, 0.0),
            Coords::new(1.0, 3.0, 0.0),
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

    fn make_map() -> CellMap {
        let ms = HashMap::from([
            ("OOM", MapState::OutOfMap),
            ("OTR", MapState::OtherRobot),
            ("MYR", MapState::MyRobot),
            ("EXP", MapState::Explored),
            ("UNE", MapState::Unexplored),
            ("FNT", MapState::Frontier),
            ("ASS", MapState::Assigned),
        ]);


        CellMap::from_raster(
            MapStateMatrix::from_shape_vec(
                (5, 3),
                vec![
                    *ms.get("OOM").unwrap(), *ms.get("OTR").unwrap(), *ms.get("MYR").unwrap(), //
                    *ms.get("FNT").unwrap(), *ms.get("UNE").unwrap(), *ms.get("EXP").unwrap(), //
                    *ms.get("ASS").unwrap(), *ms.get("OOM").unwrap(), *ms.get("OTR").unwrap(), //
                    *ms.get("MYR").unwrap(), *ms.get("UNE").unwrap(), *ms.get("ASS").unwrap(), //
                    *ms.get("UNE").unwrap(), *ms.get("EXP").unwrap(), *ms.get("FNT").unwrap(), //
                ],
            )
            .unwrap(),
            AxisResolution::uniform(1.0),
            Coords::new(0.0, 0.0, 0.0),
        )
    }

    #[test]
    fn submap_get_map_region() {
        let map = make_map();

        let cells = map.get_map_region(|e| e == MapState::OutOfMap);

        assert_eq!(cells.len(), 2);
        assert_eq!(
            cells,
            vec![
                Cell::new(0, 0, &MapState::OutOfMap),
                Cell::new(1, 2, &MapState::OutOfMap),
            ]
        );
    }

    #[test]
    fn submap_get_out_of_map() {
        let map = make_map();

        let cells = map.get_map_state(MapState::OutOfMap);

        assert_eq!(cells.len(), 2);
        assert_eq!(
            cells,
            vec![
                Cell::new(0, 0, &MapState::OutOfMap),
                Cell::new(1, 2, &MapState::OutOfMap),
            ]
        );
    }

    #[test]
    fn submap_get_explored() {
        let map = make_map();

        let cells = map.get_map_state(MapState::Explored);

        assert_eq!(cells.len(), 2);
        assert_eq!(
            cells,
            vec![
                Cell::new(2, 1, &MapState::Explored),
                Cell::new(1, 4, &MapState::Explored),
            ]
        );
    }

    #[test]
    fn submap_get_unexplored() {
        let map = make_map();

        let cells = map.get_map_state(MapState::Unexplored);

        assert_eq!(cells.len(), 3);
        assert_eq!(
            cells,
            vec![
                Cell::new(1, 1, &MapState::Unexplored),
                Cell::new(1, 3, &MapState::Unexplored),
                Cell::new(0, 4, &MapState::Unexplored),
            ]
        );
    }

    #[test]
    fn submap_get_frontier() {
        let map = make_map();

        let cells = map.get_map_state(MapState::Frontier);

        assert_eq!(cells.len(), 2);
        assert_eq!(
            cells,
            vec![
                Cell::new(0, 1, &MapState::Frontier),
                Cell::new(2, 4, &MapState::Frontier),
            ]
        );
    }

    #[test]
    fn submap_get_assigned() {
        let map = make_map();

        let cells = map.get_map_state(MapState::Assigned);

        assert_eq!(cells.len(), 2);
        assert_eq!(
            cells,
            vec![
                Cell::new(0, 2, &MapState::Assigned),
                Cell::new(2, 3, &MapState::Assigned),
            ]
        );
    }
}
