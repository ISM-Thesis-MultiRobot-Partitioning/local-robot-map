use crate::{Coords, MapState, MapStateMatrix};

/// Describe a map using a 2D grid of cells.
///
/// Note that only the `x` and `y` components of [`Coords`] are used, and the
/// `z` component will be ignored.
///
/// Another important thing to note is that real-world coordinates are to be
/// provided and output from the [`CellMap`].
pub struct CellMap {
    /// A matrix representing the cells along with their states.
    cells: MapStateMatrix,
    /// Cell resolution, assumed in *pixels per meter*.
    resolution: f64,
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
    ///
    /// # Example
    /// ```
    /// use local_robot_map::{CellMap, Coords, MapState};
    /// use matrix::{matrix, Size};
    ///
    /// let point1 = Coords::new(-1.0, -2.0, 0.0);
    /// let point2 = Coords::new(0.5, 1.0, 0.0);
    /// let resolution = 2.0;
    ///
    /// let map = CellMap::new(point1, point2, resolution);
    ///
    /// assert_eq!(map.resolution(), &2.0);
    /// assert_eq!(map.offset(), &Coords::new(-1.0, -2.0, 0.0));
    /// assert_eq!(map.ncols(), 6);
    /// assert_eq!(map.nrows(), 3);
    /// ```
    pub fn new(point1: Coords, point2: Coords, resolution: f64) -> Self {
        let columns = (point2.x - point1.x).abs() * resolution;
        let rows = (point2.y - point1.y).abs() * resolution;

        let offset = Coords {
            x: point1.x.min(point2.x),
            y: point1.y.min(point2.y),
            z: point1.z.min(point2.z),
        };

        Self {
            cells: MapStateMatrix::from_elem(
                (rows as usize, columns as usize),
                MapState::Unexplored,
            ),
            resolution,
            offset,
        }
    }

    pub fn from_raster(
        cells: MapStateMatrix,
        resolution: f64,
        offset: Coords,
    ) -> Self {
        Self {
            cells,
            resolution,
            offset,
        }
    }

    pub fn resolution(&self) -> &f64 {
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
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn create_cell_map_one_by_one() {
        let map = CellMap::new(
            Coords::new(0.0, 0.0, 0.0),
            Coords::new(1.0, 1.0, 0.0),
            1.0,
        );
        assert_eq!(map.resolution(), &1.0);
        assert_eq!(map.ncols(), 1);
        assert_eq!(map.nrows(), 1);
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
            1.0,
        );
        assert_eq!(map.resolution(), &1.0);
        assert_eq!(map.ncols(), 1);
        assert_eq!(map.nrows(), 1);
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
            1.0,
        );
        assert_eq!(map.resolution(), &1.0);
        assert_eq!(map.ncols(), 1);
        assert_eq!(map.nrows(), 1);
        assert_eq!(map.offset(), &Coords { x, y, z: 0.0 });
    }

    #[test]
    fn create_cell_map_offset_negative() {
        let (x, y) = (-126.83, -7165.1137);
        let map = CellMap::new(
            Coords::new(x, y, 0.0),
            Coords::new(x + 1.0, y + 1.0, 0.0),
            1.0,
        );
        assert_eq!(map.resolution(), &1.0);
        assert_eq!(map.ncols(), 1);
        assert_eq!(map.nrows(), 1);
        assert_eq!(map.offset(), &Coords { x, y, z: 0.0 });
    }

    #[test]
    fn create_cell_map_resolution() {
        let map = CellMap::new(
            Coords::new(0.0, 0.0, 0.0),
            Coords::new(1.0, 1.0, 0.0),
            7.0,
        );
        assert_eq!(map.resolution(), &7.0);
        assert_eq!(map.ncols(), 7);
        assert_eq!(map.nrows(), 7);
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
            7.0,
        );
        assert_eq!(map.resolution(), &7.0);
        assert_eq!(map.ncols(), 7);
        assert_eq!(map.nrows(), 7);
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
            1.0,
        );
        assert_eq!(map.resolution(), &1.0);
        assert_eq!(map.ncols(), 9);
        assert_eq!(map.nrows(), 1);
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
            1.0,
        );
        assert_eq!(map.resolution(), &1.0);
        assert_eq!(map.ncols(), 11);
        assert_eq!(map.nrows(), 7);
        assert_eq!(
            map.offset(),
            &Coords {
                x: -10.0,
                y: -4.0,
                z: 0.0
            }
        );
    }
}
