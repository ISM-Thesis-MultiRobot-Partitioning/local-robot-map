use geo::{BoundingRect, MapCoords};
use geo_rasterize::BinaryBuilder;
use num::ToPrimitive;

use crate::cell_map::CellMap;
use crate::coords::{AxisResolution, Coords, InternalLocation};
use crate::{Location, LocationType, RealWorldLocation};

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
/// let polygon = PolygonMap::new(vec![p1, p2, p3]).unwrap();
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
    /// Vertices of the polygon describing the region to be explored.
    vertices: Vec<RealWorldLocation>,
    /// List of vertices describing polygons of the already explored regions.
    explored: Option<Vec<Vec<RealWorldLocation>>>,
}

impl PolygonMap {
    /// Create a new and empty polygon map.
    ///
    /// It is important to note here, that the here map is **empty** and
    /// is intended to contain nothing other than the specified polygon
    /// indicating the region to be explored.
    ///
    /// # Errors
    ///
    /// This function will return an error if the polygon has not enough
    /// vertices (strictly less than 3 vertices) and thus describes an invalid
    /// polygon.
    pub fn new(
        vertices: Vec<RealWorldLocation>,
    ) -> Result<Self, PolygonMapError> {
        Ok(Self {
            vertices: Self::verify_polygon(vertices)?,
            explored: None,
        })
    }

    /// Same as [`PolygonMap::new`], but also sets *already explored* regions.
    ///
    /// # Errors
    ///
    /// Same errors as [`PolygonMap::new`].
    pub fn new_explored(
        vertices: Vec<RealWorldLocation>,
        explored: Option<Vec<Vec<RealWorldLocation>>>,
    ) -> Result<Self, PolygonMapError> {
        // TODO: find a better way to make this check in-place when creating the
        // struct? The [`Self::verify_polygon`] function was made such that it
        // returns the polygon itself for this particular situation, in case
        // there was no error. The issue with using `.map()` is that a
        // `return` inside the closure will not return from the parent
        // function. Also the following expression (equally with a `match`) will
        // partially move the value, hence a clone is necessary.
        if let Some(e) = explored.clone() {
            for polygon in e {
                Self::verify_polygon(polygon)?;
            }
        }

        Ok(Self {
            vertices: Self::verify_polygon(vertices)?,
            explored,
        })
    }

    /// Internal function to verify validity of a polygon.
    ///
    /// # Errors
    ///
    /// This function will return an error if the polygon has too few vertices
    /// (less than 3) to describe a valid shape.
    fn verify_polygon(
        vertices: Vec<RealWorldLocation>,
    ) -> Result<Vec<RealWorldLocation>, PolygonMapError> {
        if vertices.len() < 3 {
            Err(PolygonMapError::NotEnoughVertices)
        } else {
            Ok(vertices)
        }
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
        let (cells, offset) =
            self.rasterize_polygon(&self.vertices, &resolution);
        let cells = cells.map(|e| match e {
            true => LocationType::Unexplored,
            false => LocationType::OutOfMap,
        });
        let mut cellmap = CellMap::from_raster(cells, resolution, offset);

        // Set already-explored cells in `cellmap`
        if let Some(explored) = &self.explored {
            for polygon in explored {
                let (cells_explored, offset_explored) =
                    self.rasterize_polygon(polygon, &resolution);
                let explored_locations: Vec<RealWorldLocation> = cells_explored
                    .indexed_iter()
                    .filter(|((_, _), e)| **e)
                    .map(|((row, col), _)| {
                        InternalLocation::new(
                            Coords::new(
                                col.to_f64().expect(
                                    "No overflow converting usize to f64",
                                ),
                                row.to_f64().expect(
                                    "No overflow converting usize to f64",
                                ),
                                0.0,
                            ),
                            offset_explored,
                            resolution,
                        )
                        .expect(
                            "indexed_iter() will not return negative indexes",
                        )
                        .into_real_world()
                    })
                    .filter(|location| cellmap.get_location(location).is_ok())
                    .collect();

                for loc in &explored_locations {
                    cellmap
                        .set_location(loc, LocationType::Explored)
                        .expect("Invalid locations were filtered out");
                }
            }
        }

        cellmap
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
        vertices: &[RealWorldLocation],
        resolution: &AxisResolution,
    ) -> (ndarray::Array2<bool>, Coords) {
        let polygon = geo::Polygon::new(
            geo::LineString::from(
                vertices.iter().map(|e| (e.x(), e.y())).collect::<Vec<_>>(),
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
                    .into_internal(offset, *resolution)
                    .expect("Coordinates should be valid");
            geo::Coord {
                x: internal_location.x(),
                y: internal_location.y(),
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

        (rasterizer.finish(), offset)
    }

    pub fn vertices(&self) -> &Vec<RealWorldLocation> {
        &self.vertices
    }
}

#[derive(Debug, PartialEq)]
pub enum PolygonMapError {
    /// At least 3 vertices are needed to form a proper polygon on which
    /// anything meaningful can be done.
    NotEnoughVertices,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::LocationType;

    const OOM: LocationType = LocationType::OutOfMap;
    const UNE: LocationType = LocationType::Unexplored;

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
        let cellmap: CellMap = PolygonMap::new(vec![p1, p2, p3])
            .unwrap()
            .to_cell_map(resolution);

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
        let cellmap: CellMap = PolygonMap::new(vec![p1, p2, p3])
            .unwrap()
            .to_cell_map(resolution);

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
        let cellmap: CellMap = PolygonMap::new(vec![p1, p2, p3])
            .unwrap()
            .to_cell_map(resolution);

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
