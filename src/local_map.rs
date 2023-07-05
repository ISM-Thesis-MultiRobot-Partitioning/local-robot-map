use crate::{
    Location, LocationError, MapState, MaskMapState, Partition,
    RealWorldLocation, Visualize,
};

/// Wrapper type to store robot's location **and** related parameters.
///
/// The parameters are intended to store additional information about a robot.
/// They are given as a generic type `P` in order to give full control and
/// flexibility to users of the crate.
///
/// One use case for the parameters could be to add identifiers to the robots,
/// or to include factors that shall influence the partitioning.
#[derive(Debug)]
pub struct Robot<P> {
    location: RealWorldLocation,
    parameters: P,
}

impl<P> Robot<P> {
    pub fn new(location: RealWorldLocation, parameters: P) -> Self {
        Self {
            location,
            parameters,
        }
    }
    pub fn location(&self) -> &RealWorldLocation {
        &self.location
    }
    pub fn parameters(&self) -> &P {
        &self.parameters
    }
}

/// Type for map stored locally on a robot.
///
/// # Generic Types
///
/// - `T` describes the type of the map that is used for partitioning. It is
///   restricted under certain traits which allows for a more uniform and
///   standardized implementation.
/// - `F` is a type containing additional partitioning factors. They are passed
///   as arguments to the `partition_algorithm` function.
///
/// Note that if you are not interested in additional partitioning factors, you
/// can set `F` to be the empty type `()`. And then simply perform the
/// partitioning by passing [`None`] as the partitioning factors.
pub struct LocalMap<T, P>
where
    T: Location + MaskMapState + Visualize + std::fmt::Debug,
{
    map: T,
    my_robot: Robot<P>,
    other_robots: Vec<Robot<P>>,
}

impl<T, P> LocalMap<T, P>
where
    T: Location + MaskMapState + Visualize + std::fmt::Debug,
{
    /// Create a [`LocalMap`] which does not allow out-of-map robots.
    ///
    /// If a robot happens to be placed outside the map area, it will be
    /// considered [`LocationError::OutOfMap`]. See also
    /// [`LocalMap::new_expand`] which can deal with out-of-map robots.
    ///
    /// # Errors
    ///
    /// If a robot is placed such that a [`LocationError`] occurs, the function
    /// will return both the error in question as well as the provided
    /// coordinate of the offending robot.
    pub fn new_noexpand(
        mut map: T,
        my_robot: Robot<P>,
        other_robots: Vec<Robot<P>>,
    ) -> Result<Self, (LocationError, RealWorldLocation)> {
        if let Err(location_error) =
            map.set_location(my_robot.location(), MapState::MyRobot)
        {
            return Err((location_error, my_robot.location));
        };

        for pos in &other_robots {
            if let Err(location_error) =
                map.set_location(pos.location(), MapState::OtherRobot)
            {
                return Err((location_error, pos.location().clone()));
            }
        }

        Ok(Self {
            map,
            my_robot,
            other_robots,
        })
    }

    /// Create a [`LocalMap`] which allows out-of-map robots.
    ///
    /// It works the same as [`LocalMap::new_noexpand`], except that it will
    /// allow robots to be placed such that they would result in a
    /// [`LocationError::OutOfMap`]
    ///
    /// # Errors
    ///
    /// Same as [`LocalMap::new_noexpand`] except that
    /// [`LocationError::OutOfMap`] will not be returned.
    pub fn new_noexpand_nooutofmap(
        mut map: T,
        my_robot: Robot<P>,
        other_robots: Vec<Robot<P>>,
    ) -> Result<Self, (LocationError, RealWorldLocation)> {
        match map.set_location(my_robot.location(), MapState::MyRobot) {
            Ok(_) => {}
            Err(e) => match e {
                LocationError::OutOfMap => {}
                #[allow(unreachable_patterns)]
                _ => return Err((e, my_robot.location().clone())),
            },
        }

        for pos in &other_robots {
            match map.set_location(pos.location(), MapState::OtherRobot) {
                Ok(_) => {}
                Err(e) => match e {
                    LocationError::OutOfMap => {}
                    #[allow(unreachable_patterns)]
                    _ => return Err((e, my_robot.location().clone())),
                },
            }
        }

        Ok(Self {
            map,
            my_robot,
            other_robots,
        })
    }

    pub fn new_expand(
        mut map: T,
        my_position: RealWorldLocation,
        other_positions: Vec<RealWorldLocation>,
    ) -> Self {
        #![allow(unused_variables, unused_mut)]
        // See also [`crate::coords::InternalLocation::change_offset`].
        todo!()
    }

    pub fn map(&self) -> &T {
        &self.map
    }
    pub fn map_mut(&mut self) -> &mut T {
        &mut self.map
    }
    pub fn my_position(&self) -> &RealWorldLocation {
        &self.my_robot.location
    }
    pub fn other_positions(&self) -> Vec<RealWorldLocation> {
        self.other_robots
            .iter()
            .map(|r| r.location().clone())
            .collect()
    }
    pub fn my_robot(&self) -> &Robot<P> {
        &self.my_robot
    }
    pub fn other_robots(&self) -> &Vec<Robot<P>> {
        &self.other_robots
    }
}

impl<T, P> Partition for LocalMap<T, P> where
    T: Location + MaskMapState + Visualize + std::fmt::Debug
{
}

impl<T, P> Visualize for LocalMap<T, P>
where
    T: Location + MaskMapState + Visualize + std::fmt::Debug,
{
    type ImageType = <T as Visualize>::ImageType;

    fn as_image(&self) -> Self::ImageType {
        self.map.as_image()
    }
}

impl<T, P> std::fmt::Debug for LocalMap<T, P>
where
    T: Location + MaskMapState + Visualize + std::fmt::Debug,
    P: std::fmt::Debug,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "LocalMap: map = {:?}, my_robot = {:?}, other_robots = {:?}",
            self.map, self.my_robot, self.other_robots,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        cell_map::tests::make_map, CellMap, LocationType, RealWorldLocation,
    };

    fn make_random_local_map(
        my_position: RealWorldLocation,
        other_positions: Vec<RealWorldLocation>,
    ) -> LocalMap<CellMap, ()> {
        let (map, _) = make_map();

        LocalMap::new_noexpand(
            map,
            Robot::new(my_position, ()),
            other_positions
                .into_iter()
                .map(|loc| Robot::new(loc, ()))
                .collect(),
        )
        .unwrap()
    }

    fn make_local_map(
        my_position: RealWorldLocation,
        other_positions: Vec<RealWorldLocation>,
    ) -> LocalMap<CellMap, ()> {
        LocalMap::new_noexpand(
            CellMap::new(
                RealWorldLocation::from_xyz(0.0, 0.0, 0.0),
                RealWorldLocation::from_xyz(10.0, 10.0, 10.0),
                crate::AxisResolution::uniform(1.0),
            ),
            Robot::new(my_position, ()),
            other_positions
                .into_iter()
                .map(|loc| Robot::new(loc, ()))
                .collect(),
        )
        .unwrap()
    }

    fn get_mapstate_pos_from_map(
        map: &CellMap,
        state: LocationType,
    ) -> Vec<RealWorldLocation> {
        map.get_map_state(state)
            .iter()
            .map(|cell| cell.location().clone())
            .collect()
    }

    #[test]
    fn new_noexpand_robots_in_map() {
        const OFFSET: f64 = 5.0;
        let lmap: LocalMap<CellMap, ()> = {
            let my_position = RealWorldLocation::from_xyz(
                1.0 - OFFSET,
                1.0 - OFFSET,
                1.0 - OFFSET,
            );
            let other_positions = vec![
                RealWorldLocation::from_xyz(
                    2.0 - OFFSET,
                    2.0 - OFFSET,
                    2.0 - OFFSET,
                ),
                RealWorldLocation::from_xyz(
                    3.0 - OFFSET,
                    3.0 - OFFSET,
                    3.0 - OFFSET,
                ),
                RealWorldLocation::from_xyz(
                    4.0 - OFFSET,
                    4.0 - OFFSET,
                    4.0 - OFFSET,
                ),
            ];
            LocalMap::new_noexpand(
                CellMap::new(
                    RealWorldLocation::from_xyz(
                        0.0 - OFFSET,
                        0.0 - OFFSET,
                        0.0 - OFFSET,
                    ),
                    RealWorldLocation::from_xyz(
                        10.0 - OFFSET,
                        10.0 - OFFSET,
                        10.0 - OFFSET,
                    ),
                    crate::AxisResolution::uniform(1.0),
                ),
                Robot::new(my_position, ()),
                other_positions
                    .into_iter()
                    .map(|loc| Robot::new(loc, ()))
                    .collect(),
            )
        }
        .expect("No location error");

        assert_eq!((lmap.map().width(), lmap.map().height()), (10, 10))
    }

    #[test]
    fn new_noexpand_myrobot_out_of_map() {
        const OFFSET: f64 = 5.0;
        let lmap = {
            let my_position = RealWorldLocation::from_xyz(
                11.0 - OFFSET,
                11.0 - OFFSET,
                11.0 - OFFSET,
            );
            let other_positions = vec![
                RealWorldLocation::from_xyz(
                    2.0 - OFFSET,
                    2.0 - OFFSET,
                    2.0 - OFFSET,
                ),
                RealWorldLocation::from_xyz(
                    3.0 - OFFSET,
                    3.0 - OFFSET,
                    3.0 - OFFSET,
                ),
                RealWorldLocation::from_xyz(
                    4.0 - OFFSET,
                    4.0 - OFFSET,
                    4.0 - OFFSET,
                ),
            ];
            LocalMap::new_noexpand(
                CellMap::new(
                    RealWorldLocation::from_xyz(
                        0.0 - OFFSET,
                        0.0 - OFFSET,
                        0.0 - OFFSET,
                    ),
                    RealWorldLocation::from_xyz(
                        10.0 - OFFSET,
                        10.0 - OFFSET,
                        10.0 - OFFSET,
                    ),
                    crate::AxisResolution::uniform(1.0),
                ),
                Robot::new(my_position, ()),
                other_positions
                    .into_iter()
                    .map(|loc| Robot::new(loc, ()))
                    .collect(),
            )
        };

        assert_eq!(
            lmap.unwrap_err(),
            (
                LocationError::OutOfMap,
                RealWorldLocation::from_xyz(
                    11.0 - OFFSET,
                    11.0 - OFFSET,
                    11.0 - OFFSET
                )
            )
        )
    }

    #[test]
    fn new_noexpand_other_robot_out_of_map() {
        const OFFSET: f64 = 5.0;
        let lmap = {
            let my_position = RealWorldLocation::from_xyz(
                1.0 - OFFSET,
                1.0 - OFFSET,
                1.0 - OFFSET,
            );
            let other_positions = vec![
                RealWorldLocation::from_xyz(
                    2.0 - OFFSET,
                    2.0 - OFFSET,
                    2.0 - OFFSET,
                ),
                RealWorldLocation::from_xyz(
                    -1.0 - OFFSET,
                    -1.0 - OFFSET,
                    -1.0 - OFFSET,
                ),
                RealWorldLocation::from_xyz(
                    4.0 - OFFSET,
                    4.0 - OFFSET,
                    4.0 - OFFSET,
                ),
            ];
            LocalMap::new_noexpand(
                CellMap::new(
                    RealWorldLocation::from_xyz(
                        0.0 - OFFSET,
                        0.0 - OFFSET,
                        0.0 - OFFSET,
                    ),
                    RealWorldLocation::from_xyz(
                        10.0 - OFFSET,
                        10.0 - OFFSET,
                        10.0 - OFFSET,
                    ),
                    crate::AxisResolution::uniform(1.0),
                ),
                Robot::new(my_position, ()),
                other_positions
                    .into_iter()
                    .map(|loc| Robot::new(loc, ()))
                    .collect(),
            )
        };

        assert_eq!(
            lmap.unwrap_err(),
            (
                LocationError::OutOfMap,
                RealWorldLocation::from_xyz(
                    -1.0 - OFFSET,
                    -1.0 - OFFSET,
                    -1.0 - OFFSET
                )
            )
        )
    }

    #[test]
    fn new_noexpand_multiple_other_robot_out_of_map() {
        const OFFSET: f64 = 5.0;
        let lmap = {
            let my_position = RealWorldLocation::from_xyz(
                1.0 - OFFSET,
                1.0 - OFFSET,
                1.0 - OFFSET,
            );
            let other_positions = vec![
                RealWorldLocation::from_xyz(
                    12.0 - OFFSET,
                    12.0 - OFFSET,
                    12.0 - OFFSET,
                ),
                RealWorldLocation::from_xyz(
                    3.0 - OFFSET,
                    3.0 - OFFSET,
                    3.0 - OFFSET,
                ),
                RealWorldLocation::from_xyz(
                    -4.0 - OFFSET,
                    -4.0 - OFFSET,
                    -4.0 - OFFSET,
                ),
            ];
            LocalMap::new_noexpand(
                CellMap::new(
                    RealWorldLocation::from_xyz(
                        0.0 - OFFSET,
                        0.0 - OFFSET,
                        0.0 - OFFSET,
                    ),
                    RealWorldLocation::from_xyz(
                        10.0 - OFFSET,
                        10.0 - OFFSET,
                        10.0 - OFFSET,
                    ),
                    crate::AxisResolution::uniform(1.0),
                ),
                Robot::new(my_position, ()),
                other_positions
                    .into_iter()
                    .map(|loc| Robot::new(loc, ()))
                    .collect(),
            )
        };

        assert_eq!(
            lmap.unwrap_err(),
            (
                LocationError::OutOfMap,
                RealWorldLocation::from_xyz(
                    12.0 - OFFSET,
                    12.0 - OFFSET,
                    12.0 - OFFSET
                )
            )
        )
    }

    #[test]
    fn new_expand_robots_in_map() {
        const OFFSET: f64 = 5.0;
        let map: LocalMap<CellMap, ()> = {
            let my_position = RealWorldLocation::from_xyz(
                1.0 - OFFSET,
                1.0 - OFFSET,
                1.0 - OFFSET,
            );
            let other_positions = vec![
                RealWorldLocation::from_xyz(
                    2.0 - OFFSET,
                    2.0 - OFFSET,
                    2.0 - OFFSET,
                ),
                RealWorldLocation::from_xyz(
                    3.0 - OFFSET,
                    3.0 - OFFSET,
                    3.0 - OFFSET,
                ),
                RealWorldLocation::from_xyz(
                    4.0 - OFFSET,
                    4.0 - OFFSET,
                    4.0 - OFFSET,
                ),
            ];
            LocalMap::new_expand(
                CellMap::new(
                    RealWorldLocation::from_xyz(
                        0.0 - OFFSET,
                        0.0 - OFFSET,
                        0.0 - OFFSET,
                    ),
                    RealWorldLocation::from_xyz(
                        10.0 - OFFSET,
                        10.0 - OFFSET,
                        10.0 - OFFSET,
                    ),
                    crate::AxisResolution::uniform(1.0),
                ),
                my_position,
                other_positions,
            )
        };

        assert_eq!((map.map().width(), map.map().height()), (10, 10))
    }

    #[test]
    fn new_expand_robot_right() {
        const OFFSET: f64 = 5.0;
        let map: LocalMap<CellMap, ()> = {
            let my_position = RealWorldLocation::from_xyz(
                16.84 - OFFSET,
                1.0 - OFFSET,
                1.0 - OFFSET,
            );
            let other_positions = vec![
                RealWorldLocation::from_xyz(
                    2.0 - OFFSET,
                    2.0 - OFFSET,
                    2.0 - OFFSET,
                ),
                RealWorldLocation::from_xyz(
                    3.0 - OFFSET,
                    3.0 - OFFSET,
                    3.0 - OFFSET,
                ),
                RealWorldLocation::from_xyz(
                    4.0 - OFFSET,
                    4.0 - OFFSET,
                    4.0 - OFFSET,
                ),
            ];
            LocalMap::new_expand(
                CellMap::new(
                    RealWorldLocation::from_xyz(
                        0.0 - OFFSET,
                        0.0 - OFFSET,
                        0.0 - OFFSET,
                    ),
                    RealWorldLocation::from_xyz(
                        10.0 - OFFSET,
                        10.0 - OFFSET,
                        10.0 - OFFSET,
                    ),
                    crate::AxisResolution::uniform(1.0),
                ),
                my_position,
                other_positions,
            )
        };

        assert_eq!((map.map().width(), map.map().height()), (16, 10))
    }

    #[test]
    fn new_expand_robot_right_up() {
        const OFFSET: f64 = 5.0;
        let map: LocalMap<CellMap, ()> = {
            let my_position = RealWorldLocation::from_xyz(
                1.0 - OFFSET,
                1.0 - OFFSET,
                1.0 - OFFSET,
            );
            let other_positions = vec![
                RealWorldLocation::from_xyz(
                    2.0 - OFFSET,
                    2.0 - OFFSET,
                    2.0 - OFFSET,
                ),
                RealWorldLocation::from_xyz(
                    13.47 - OFFSET,
                    17.08 - OFFSET,
                    3.0 - OFFSET,
                ),
                RealWorldLocation::from_xyz(
                    4.0 - OFFSET,
                    4.0 - OFFSET,
                    4.0 - OFFSET,
                ),
            ];
            LocalMap::new_expand(
                CellMap::new(
                    RealWorldLocation::from_xyz(
                        0.0 - OFFSET,
                        0.0 - OFFSET,
                        0.0 - OFFSET,
                    ),
                    RealWorldLocation::from_xyz(
                        10.0 - OFFSET,
                        10.0 - OFFSET,
                        10.0 - OFFSET,
                    ),
                    crate::AxisResolution::uniform(1.0),
                ),
                my_position,
                other_positions,
            )
        };

        assert_eq!((map.map().width(), map.map().height()), (13, 17))
    }

    #[test]
    fn new_expand_robot_up() {
        const OFFSET: f64 = 5.0;
        let map: LocalMap<CellMap, ()> = {
            let my_position = RealWorldLocation::from_xyz(
                1.0 - OFFSET,
                1.0 - OFFSET,
                1.0 - OFFSET,
            );
            let other_positions = vec![
                RealWorldLocation::from_xyz(
                    2.0 - OFFSET,
                    2.0 - OFFSET,
                    2.0 - OFFSET,
                ),
                RealWorldLocation::from_xyz(
                    3.0 - OFFSET,
                    3.0 - OFFSET,
                    3.0 - OFFSET,
                ),
                RealWorldLocation::from_xyz(
                    4.0 - OFFSET,
                    14.0 - OFFSET,
                    4.0 - OFFSET,
                ),
            ];
            LocalMap::new_expand(
                CellMap::new(
                    RealWorldLocation::from_xyz(
                        0.0 - OFFSET,
                        0.0 - OFFSET,
                        0.0 - OFFSET,
                    ),
                    RealWorldLocation::from_xyz(
                        10.0 - OFFSET,
                        10.0 - OFFSET,
                        10.0 - OFFSET,
                    ),
                    crate::AxisResolution::uniform(1.0),
                ),
                my_position,
                other_positions,
            )
        };

        assert_eq!((map.map().width(), map.map().height()), (10, 14))
    }

    #[test]
    fn new_expand_robot_left_up() {
        const OFFSET: f64 = 5.0;
        let map: LocalMap<CellMap, ()> = {
            let my_position = RealWorldLocation::from_xyz(
                1.0 - OFFSET,
                1.0 - OFFSET,
                1.0 - OFFSET,
            );
            let other_positions = vec![
                RealWorldLocation::from_xyz(
                    -1.87 - OFFSET,
                    12.0 - OFFSET,
                    2.0 - OFFSET,
                ),
                RealWorldLocation::from_xyz(
                    3.0 - OFFSET,
                    3.0 - OFFSET,
                    3.0 - OFFSET,
                ),
                RealWorldLocation::from_xyz(
                    4.0 - OFFSET,
                    4.0 - OFFSET,
                    4.0 - OFFSET,
                ),
            ];
            LocalMap::new_expand(
                CellMap::new(
                    RealWorldLocation::from_xyz(
                        0.0 - OFFSET,
                        0.0 - OFFSET,
                        0.0 - OFFSET,
                    ),
                    RealWorldLocation::from_xyz(
                        10.0 - OFFSET,
                        10.0 - OFFSET,
                        10.0 - OFFSET,
                    ),
                    crate::AxisResolution::uniform(1.0),
                ),
                my_position,
                other_positions,
            )
        };

        assert_eq!((map.map().width(), map.map().height()), (12, 12))
    }

    #[test]
    fn new_expand_robot_left() {
        const OFFSET: f64 = 5.0;
        let map: LocalMap<CellMap, ()> = {
            let my_position = RealWorldLocation::from_xyz(
                -4.0 - OFFSET,
                1.0 - OFFSET,
                1.0 - OFFSET,
            );
            let other_positions = vec![
                RealWorldLocation::from_xyz(
                    2.0 - OFFSET,
                    2.0 - OFFSET,
                    2.0 - OFFSET,
                ),
                RealWorldLocation::from_xyz(
                    3.0 - OFFSET,
                    3.0 - OFFSET,
                    3.0 - OFFSET,
                ),
                RealWorldLocation::from_xyz(
                    4.0 - OFFSET,
                    4.0 - OFFSET,
                    4.0 - OFFSET,
                ),
            ];
            LocalMap::new_expand(
                CellMap::new(
                    RealWorldLocation::from_xyz(
                        0.0 - OFFSET,
                        0.0 - OFFSET,
                        0.0 - OFFSET,
                    ),
                    RealWorldLocation::from_xyz(
                        10.0 - OFFSET,
                        10.0 - OFFSET,
                        10.0 - OFFSET,
                    ),
                    crate::AxisResolution::uniform(1.0),
                ),
                my_position,
                other_positions,
            )
        };

        assert_eq!((map.map().width(), map.map().height()), (14, 10))
    }

    #[test]
    fn new_expand_robot_left_down() {
        const OFFSET: f64 = 5.0;
        let map: LocalMap<CellMap, ()> = {
            let my_position = RealWorldLocation::from_xyz(
                1.0 - OFFSET,
                1.0 - OFFSET,
                1.0 - OFFSET,
            );
            let other_positions = vec![
                RealWorldLocation::from_xyz(
                    2.0 - OFFSET,
                    2.0 - OFFSET,
                    2.0 - OFFSET,
                ),
                RealWorldLocation::from_xyz(
                    3.0 - OFFSET,
                    3.0 - OFFSET,
                    3.0 - OFFSET,
                ),
                RealWorldLocation::from_xyz(
                    -3.92 - OFFSET,
                    -1.35 - OFFSET,
                    4.0 - OFFSET,
                ),
            ];
            LocalMap::new_expand(
                CellMap::new(
                    RealWorldLocation::from_xyz(
                        0.0 - OFFSET,
                        0.0 - OFFSET,
                        0.0 - OFFSET,
                    ),
                    RealWorldLocation::from_xyz(
                        10.0 - OFFSET,
                        10.0 - OFFSET,
                        10.0 - OFFSET,
                    ),
                    crate::AxisResolution::uniform(1.0),
                ),
                my_position,
                other_positions,
            )
        };

        assert_eq!((map.map().width(), map.map().height()), (14, 12))
    }

    #[test]
    fn new_expand_robot_down() {
        const OFFSET: f64 = 5.0;
        let map: LocalMap<CellMap, ()> = {
            let my_position = RealWorldLocation::from_xyz(
                1.0 - OFFSET,
                -3.0 - OFFSET,
                1.0 - OFFSET,
            );
            let other_positions = vec![
                RealWorldLocation::from_xyz(
                    2.0 - OFFSET,
                    2.0 - OFFSET,
                    2.0 - OFFSET,
                ),
                RealWorldLocation::from_xyz(
                    3.0 - OFFSET,
                    3.0 - OFFSET,
                    3.0 - OFFSET,
                ),
                RealWorldLocation::from_xyz(
                    4.0 - OFFSET,
                    4.0 - OFFSET,
                    4.0 - OFFSET,
                ),
            ];
            LocalMap::new_expand(
                CellMap::new(
                    RealWorldLocation::from_xyz(
                        0.0 - OFFSET,
                        0.0 - OFFSET,
                        0.0 - OFFSET,
                    ),
                    RealWorldLocation::from_xyz(
                        10.0 - OFFSET,
                        10.0 - OFFSET,
                        10.0 - OFFSET,
                    ),
                    crate::AxisResolution::uniform(1.0),
                ),
                my_position,
                other_positions,
            )
        };

        assert_eq!((map.map().width(), map.map().height()), (10, 13))
    }

    #[test]
    fn new_expand_robot_right_down() {
        const OFFSET: f64 = 5.0;
        let map: LocalMap<CellMap, ()> = {
            let my_position = RealWorldLocation::from_xyz(
                1.0 - OFFSET,
                1.0 - OFFSET,
                1.0 - OFFSET,
            );
            let other_positions = vec![
                RealWorldLocation::from_xyz(
                    2.0 - OFFSET,
                    2.0 - OFFSET,
                    2.0 - OFFSET,
                ),
                RealWorldLocation::from_xyz(
                    13.0 - OFFSET,
                    -3.0 - OFFSET,
                    3.0 - OFFSET,
                ),
                RealWorldLocation::from_xyz(
                    4.0 - OFFSET,
                    4.0 - OFFSET,
                    4.0 - OFFSET,
                ),
            ];
            LocalMap::new_expand(
                CellMap::new(
                    RealWorldLocation::from_xyz(
                        0.0 - OFFSET,
                        0.0 - OFFSET,
                        0.0 - OFFSET,
                    ),
                    RealWorldLocation::from_xyz(
                        10.0 - OFFSET,
                        10.0 - OFFSET,
                        10.0 - OFFSET,
                    ),
                    crate::AxisResolution::uniform(1.0),
                ),
                my_position,
                other_positions,
            )
        };

        assert_eq!((map.map().width(), map.map().height()), (13, 13))
    }

    #[test]
    fn get_my_position() {
        let my_position = RealWorldLocation::from_xyz(0.0, 0.0, 0.0);
        let other_positions = vec![];

        let lmap = make_local_map(my_position, other_positions);
        let my_map_pos: Vec<RealWorldLocation> =
            get_mapstate_pos_from_map(lmap.map(), LocationType::MyRobot);

        assert_eq!(
            my_map_pos.len(),
            1,
            "There should only be 1 position for my robot"
        );
        assert_eq!(lmap.my_position(), &my_map_pos[0]);
    }

    #[test]
    fn create_local_map_other_positions_no_robots() {
        let my_position = RealWorldLocation::from_xyz(0.0, 0.0, 0.0);
        let other_positions = vec![];

        let lmap = make_local_map(my_position, other_positions);
        let positions =
            get_mapstate_pos_from_map(lmap.map(), LocationType::OtherRobot);

        assert_eq!(positions.len(), 0, "There should only be no other robots");
        assert_eq!(lmap.other_positions(), positions);
    }

    #[test]
    fn create_local_map_other_positions_one_robots() {
        let my_position = RealWorldLocation::from_xyz(0.0, 0.0, 0.0);
        let other_positions = vec![RealWorldLocation::from_xyz(1.0, 1.0, 0.0)];

        let lmap = make_local_map(my_position, other_positions);
        let positions =
            get_mapstate_pos_from_map(lmap.map(), LocationType::OtherRobot);

        assert_eq!(positions.len(), 1, "There should only be 1 other robots");
        assert_eq!(lmap.other_positions(), positions);
    }

    #[test]
    fn create_local_map_other_positions_multiple_robots() {
        let my_position = RealWorldLocation::from_xyz(0.0, 0.0, 0.0);
        let other_positions = vec![
            RealWorldLocation::from_xyz(1.0, 1.0, 0.0),
            RealWorldLocation::from_xyz(2.0, 2.0, 0.0),
            RealWorldLocation::from_xyz(3.0, 3.0, 0.0),
        ];

        let lmap = make_local_map(my_position, other_positions);
        let positions =
            get_mapstate_pos_from_map(lmap.map(), LocationType::OtherRobot);

        assert_eq!(positions.len(), 3, "There should only be 3 other robots");
        assert_eq!(lmap.other_positions(), positions);
    }

    #[test]
    fn partition_map_closure() {
        let lmap = make_random_local_map(
            RealWorldLocation::from_xyz(0.0, 0.0, 0.0),
            vec![],
        );

        let _partitioned_map =
            lmap.partition(|map| map).expect("No error partitioning");
    }

    #[test]
    fn partition_map_function() {
        let lmap = make_random_local_map(
            RealWorldLocation::from_xyz(0.0, 0.0, 0.0),
            vec![],
        );

        // set dummy algorithm for the test
        fn algorithm(map: LocalMap<CellMap, ()>) -> LocalMap<CellMap, ()> {
            map
        }
        let _partitioned_map =
            lmap.partition(algorithm).expect("No error partitioning");
    }

    #[test]
    fn partition_map_algorithm_is_transferred() {
        let lmap = make_random_local_map(
            RealWorldLocation::from_xyz(0.0, 0.0, 0.0),
            vec![],
        );

        // set dummy algorithm for the test
        fn algorithm(map: LocalMap<CellMap, ()>) -> LocalMap<CellMap, ()> {
            map
        }

        let _partitioned_map =
            lmap.partition(algorithm).expect("No error partitioning");
        let map_algorithm = algorithm;
        // function pointer equality: https://stackoverflow.com/a/57834304
        assert_eq!(map_algorithm as usize, algorithm as usize);
    }

    #[test]
    fn call_map_trait_function_visualize() {
        let lmap = make_random_local_map(
            RealWorldLocation::from_xyz(0.0, 0.0, 0.0),
            vec![],
        );
        lmap.map().as_image();
    }

    #[test]
    fn call_map_trait_function_visualize_and_then_save() {
        let lmap = make_random_local_map(
            RealWorldLocation::from_xyz(0.0, 0.0, 0.0),
            vec![],
        );
        lmap.map()
            .as_image()
            .save("test_save_local_map.jpg")
            .unwrap();
    }

    #[test]
    fn call_map_trait_function_mask_mapstate() {
        let lmap = make_random_local_map(
            RealWorldLocation::from_xyz(0.0, 0.0, 0.0),
            vec![],
        );
        lmap.map().get_map_state(LocationType::Unexplored);
    }
}
