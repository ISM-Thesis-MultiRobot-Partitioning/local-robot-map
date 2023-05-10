use geo::{BoundingRect, MapCoords};
use geo_rasterize::BinaryBuilder;
use num::ToPrimitive;

use crate::cell_map::CellMap;
use crate::coords::{AxisResolution, Coords};
use crate::{MapState, MapStateMatrix, RealWorldLocation};

/// Describe a map using a polygon.
///
/// The polygon is described using a set of coordinates making up its vertices.
///
/// # Examples
/// ```
/// use local_robot_map::PolygonMap;
/// use local_robot_map::RealWorldLocation;
///
/// let p1 = RealWorldLocation::from_xyz(0.0, 0.0, 0.0);
/// let p2 = RealWorldLocation::from_xyz(1.0, 1.0, 0.0);
/// let p3 = RealWorldLocation::from_xyz(2.0, 0.0, 0.0);
/// let polygon = PolygonMap::new(vec![p1, p2, p3]);
///
/// assert_eq!(
///     polygon.vertices(),
///     &vec![
///         RealWorldLocation::from_xyz(0.0, 0.0, 0.0),
///         RealWorldLocation::from_xyz(1.0, 1.0, 0.0),
///         RealWorldLocation::from_xyz(2.0, 0.0, 0.0),
///     ]
/// );
/// ```
pub struct PolygonMap {
    vertices: Vec<RealWorldLocation>,
}

impl PolygonMap {
    pub fn new(vertices: Vec<RealWorldLocation>) -> Self {
        Self { vertices }
    }

    /// Convert this map to a [`CellMap`].
    ///
    /// The [`CellMap`] is more straightforward to work with, hence this
    /// function allows to easily make the conversion. The [`PolygonMap`] is
    /// mostly interesting for specifying a map region.
    ///
    /// The `resolution` is used to impact the size/dimension of the
    /// [`CellMap`]. See also [`AxisResolution`].
    pub fn to_cell_map(self, resolution: AxisResolution) -> CellMap {
        let (cells, offset) = self.rasterize_polygon(&resolution);
        CellMap::from_raster(cells, resolution, offset)
    }

    /// Internal helper function to convert the polygon to a corresponding
    /// matrix [`MapStateMatrix`] for use with [`CellMap`]. The function should
    /// be used by [`PolygonMap::to_cell_map`].
    ///
    /// # Panics
    ///
    /// The [`geo`] crate allows to obtain the polygon's *bounding box* as well
    /// *rasterizing* it. Both of these procedures can cause panics, which
    /// is not expected to happen. They boil down to **invalid polygon shapes**
    /// and **NaN or infinite values** in the vertex coordinates.
    ///
    /// - Extract the Bounding Box: will panic if no bounding box can be made. A
    ///   properly formed polygon should always have a properly defined bounding
    ///   box. It should be checked elsewhere that the polygons have a valid
    ///   shape.
    /// - The *BinaryBuilder* used to rasterize the polygon can panic as well if
    ///   there are NaN of infinite values.
    /// - The rasterization itself can panic as well if there are NaN of
    ///   infinite values.
    fn rasterize_polygon(
        &self,
        resolution: &AxisResolution,
    ) -> (MapStateMatrix, Coords) {
        let polygon = geo::Polygon::new(
            geo::LineString::from(
                self.vertices
                    .iter()
                    .map(|e| (e.x(), e.y()))
                    .collect::<Vec<_>>(),
            ),
            vec![],
        );

        let bbox = match polygon.bounding_rect() {
            Some(b) => b,
            None => panic!("No bounding box for polygon"),
        };
        let offset = Coords::new(bbox.min().x, bbox.min().y, 0.0);
        // convert to pixels
        let width = bbox.width() * resolution.x;
        let height = bbox.height() * resolution.y;
        let polygon = polygon.map_coords(|geo::Coord { x, y }| {
            let internal_location =
                RealWorldLocation::new(Coords::new(x, y, 0.0))
                    .into_internal(offset)
                    .expect("Coordinates should be valid");
            geo::Coord {
                x: internal_location.x() * resolution.x,
                y: internal_location.y() * resolution.y,
            }
        });

        let mut rasterizer = BinaryBuilder::new()
            .width(width.to_usize().expect("No conversion issues"))
            .height(height.to_usize().expect("No conversion issues"))
            .build()
            .expect("There should be no NaN or infinite values among the polygon vertices");

        rasterizer
            .rasterize(&polygon)
            .expect("There should be no NaN of infinite values");

        let cells = rasterizer.finish().map(|e| match e {
            true => MapState::Unexplored,
            false => MapState::OutOfMap,
        });

        (cells, offset)
    }

    pub fn vertices(&self) -> &Vec<RealWorldLocation> {
        &self.vertices
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::MapState;

    const OOM: MapState = MapState::OutOfMap;
    const UNE: MapState = MapState::Unexplored;

    /// Note how the rasterized polygon seems tilted to the right and not
    /// perfectly centered/symmetric. I assume this is an artifact from the
    /// [`geo_rasterize`] crate, but I could not find any relevant information
    /// thereon. It should not pose too big of an issue though.
    #[test]
    fn polygon_map_to_cell_map_positive() {
        let p1 = RealWorldLocation::from_xyz(0.0, 0.0, 0.0);
        let p2 = RealWorldLocation::from_xyz(4.0, 4.0, 0.0);
        let p3 = RealWorldLocation::from_xyz(8.0, 0.0, 0.0);

        let resolution = AxisResolution::uniform(1.0);
        let cellmap: CellMap =
            PolygonMap::new(vec![p1, p2, p3]).to_cell_map(resolution);

        assert_eq!(cellmap.width(), 8);
        assert_eq!(cellmap.height(), 4);

        assert_eq!(
            cellmap.cells(),
            MapStateMatrix::from_shape_vec(
                (cellmap.nrows(), cellmap.ncols()),
                vec![
                    UNE, UNE, UNE, UNE, UNE, UNE, UNE, UNE, //
                    OOM, UNE, UNE, UNE, UNE, UNE, UNE, UNE, //
                    OOM, OOM, UNE, UNE, UNE, UNE, UNE, OOM, //
                    OOM, OOM, OOM, UNE, UNE, UNE, OOM, OOM, //
                ]
            )
            .unwrap()
        )
    }

    #[test]
    fn polygon_map_to_cell_map_negative() {
        let p1 = RealWorldLocation::from_xyz(0.0, -2.0, 0.0);
        let p2 = RealWorldLocation::from_xyz(-2.0, 0.0, 0.0);
        let p3 = RealWorldLocation::from_xyz(-4.0, -2.0, 0.0);

        let resolution = AxisResolution::uniform(2.0);
        let cellmap: CellMap =
            PolygonMap::new(vec![p1, p2, p3]).to_cell_map(resolution);

        assert_eq!(cellmap.width(), 8);
        assert_eq!(cellmap.height(), 4);

        assert_eq!(
            cellmap.cells(),
            MapStateMatrix::from_shape_vec(
                (cellmap.nrows(), cellmap.ncols()),
                vec![
                    UNE, UNE, UNE, UNE, UNE, UNE, UNE, UNE, //
                    OOM, UNE, UNE, UNE, UNE, UNE, UNE, UNE, //
                    OOM, OOM, UNE, UNE, UNE, UNE, UNE, OOM, //
                    OOM, OOM, OOM, UNE, UNE, UNE, OOM, OOM, //
                ]
            )
            .unwrap()
        )
    }

    #[test]
    fn polygon_map_to_cell_map_partly_negative() {
        let p1 = RealWorldLocation::from_xyz(-2.0, 0.0, 0.0);
        let p2 = RealWorldLocation::from_xyz(0.0, 2.0, 0.0);
        let p3 = RealWorldLocation::from_xyz(2.0, 0.0, 0.0);

        let resolution = AxisResolution::uniform(2.0);
        let cellmap: CellMap =
            PolygonMap::new(vec![p1, p2, p3]).to_cell_map(resolution);

        assert_eq!(cellmap.width(), 8);
        assert_eq!(cellmap.height(), 4);

        assert_eq!(
            cellmap.cells(),
            MapStateMatrix::from_shape_vec(
                (cellmap.nrows(), cellmap.ncols()),
                vec![
                    UNE, UNE, UNE, UNE, UNE, UNE, UNE, UNE, //
                    OOM, UNE, UNE, UNE, UNE, UNE, UNE, UNE, //
                    OOM, OOM, UNE, UNE, UNE, UNE, UNE, OOM, //
                    OOM, OOM, OOM, UNE, UNE, UNE, OOM, OOM, //
                ]
            )
            .unwrap()
        )
    }
}
