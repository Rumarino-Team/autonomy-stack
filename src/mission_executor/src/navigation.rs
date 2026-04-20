use crate::{BoundingBox3D, CLOSE_ENOUGH};
use nalgebra::{Isometry3, Point3, UnitQuaternion, UnitVector3, Vector3};
use parry3d_f64::query::{PointQuery, contact};
use parry3d_f64::shape::{Cuboid, Segment};

pub fn get_new_point(
    bbox: &BoundingBox3D,
    cuboid_pos: &Isometry3<f64>,
    segment: Segment,
    map_bounds: &BoundingBox3D,
) -> Option<Point3<f64>> {
    let half_extents = bbox.size * 0.5;

    let cuboid = Cuboid::new(half_extents);
    let mut target_point = segment.b;

    let pos2 = Isometry3::identity();

    //TODO: Instead of a segment, it should be two cuboids
    let contact = contact(cuboid_pos, &cuboid, &pos2, &segment, 0.0).unwrap()?;

    if target_point.coords.metric_distance(&contact.point1.coords) < CLOSE_ENOUGH
        || target_point.coords.metric_distance(&contact.point2.coords) < CLOSE_ENOUGH
    {
        r2r::log_warn!(
            "mission_executor",
            "Point {:?} is too close to object. Moving target_point to avoid collision...",
            target_point
        );
        let offset = Point3::new(bbox.size.x, bbox.size.y, bbox.size.z);
        target_point -= offset.coords;
        todo!()
    }

    let seg_dir = segment.direction().unwrap();

    let dist_to_cuboid = segment.a.coords.metric_distance(&contact.point1.coords);

    let safe_base_point = if dist_to_cuboid < 3.0 {
        contact.point1.coords - seg_dir.into_inner() * dist_to_cuboid
    } else {
        contact.point1.coords
    };

    let points = get_points(
        &cuboid,
        cuboid_pos,
        &safe_base_point.into(),
        &seg_dir,
        map_bounds,
    );

    let mut best_point = None;
    let mut smallest_dist = f64::MAX;

    for point in points {
        let dist = target_point.coords.metric_distance(&point.coords);

        if dist < smallest_dist {
            best_point = Some(point);
            smallest_dist = dist;
        }
    }
    best_point
}

fn get_points(
    cuboid: &Cuboid,
    cuboid_pos: &Isometry3<f64>,
    midpoint: &Point3<f64>,
    segment_dir: &UnitVector3<f64>,
    map_bounds: &BoundingBox3D,
) -> Vec<Point3<f64>> {
    let face_data: [(Vector3<f64>, f64); 6] = [
        (Vector3::x(), cuboid.half_extents.x),
        (-Vector3::x(), cuboid.half_extents.x),
        (Vector3::y(), cuboid.half_extents.y),
        (-Vector3::y(), cuboid.half_extents.y),
        (Vector3::z(), cuboid.half_extents.z),
        (-Vector3::z(), cuboid.half_extents.z),
    ];

    let local_midpoint = cuboid_pos.inverse_transform_vector(&midpoint.coords);
    let colliding_axis = get_dominant_axis(&segment_dir);

    let mut points = Vec::with_capacity(4);

    for (i, (face, extent)) in face_data.iter().enumerate() {
        let axis = i / 2;

        if axis == colliding_axis {
            continue;
        }

        let axis_value = match axis {
            0 => local_midpoint.x,
            1 => local_midpoint.y,
            2 => local_midpoint.z,
            _ => panic!(),
        };

        let dist = (extent - axis_value).abs();

        let world_face = cuboid_pos * face;

        //TODO: This should not be a magic number. Also I'm not sure if this is totally safe either
        let translation = world_face * (dist + 2.0);
        let point = midpoint + translation;

        let half_extents = map_bounds.size * 0.5;
        let bound_cuboid = Cuboid::new(half_extents);
        let pos = Isometry3::from_parts(map_bounds.center.pos.into(), UnitQuaternion::identity());
        if !bound_cuboid.contains_point(&pos, &point) {
            continue;
        }

        points.push(point);
    }

    points
}

fn get_dominant_axis(normal: &UnitVector3<f64>) -> usize {
    let normal_abs = normal.abs();

    if normal_abs.x >= normal_abs.y && normal_abs.x >= normal_abs.z {
        0
    } else if normal_abs.y >= normal_abs.z {
        1
    } else {
        2
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Matrix3x1, Quaternion};

    use super::*;
    use crate::Pose;
    fn setup(sub_pos: Matrix3x1<f64>, target_pos: Matrix3x1<f64>, cuboid_pos: Matrix3x1<f64>, cuboid_size: Matrix3x1<f64>, 
        cuboid_rot: Quaternion<f64>) -> (BoundingBox3D, Isometry3<f64>, Segment, BoundingBox3D) {
        let center = Pose::new(cuboid_pos, cuboid_rot);
        let bbox = BoundingBox3D::new(center, cuboid_size);

        let sub_point = Point3::from(sub_pos);
        let target_point = Point3::from(target_pos); 
        let segment = Segment::new(
            sub_point,
            target_point,
        );

        let unit_quaternion = UnitQuaternion::from_quaternion(Quaternion::identity());
        let cuboid_pos = Isometry3::from_parts(
            cuboid_pos.into(), unit_quaternion
        );

        let map_pos = Matrix3x1::zeros();
        let map_pos = Pose::new(map_pos, Quaternion::identity());
        let map_size = Matrix3x1::new(1000.0, 1000.0, 1000.0);

        let map_bounds = BoundingBox3D::new(map_pos, map_size);

        (bbox, cuboid_pos, segment, map_bounds)
    }

    fn collides(sub_point: Point3<f64>, bbox: BoundingBox3D, cuboid_pos: Isometry3<f64>, new_point: Point3<f64>) -> bool {
        let test_segment = Segment::new(
            sub_point,
            new_point
        );
        let half_extents = bbox.size * 0.5;
        let cuboid = Cuboid::new(half_extents);
        let res =
            contact(&cuboid_pos, &cuboid, &Isometry3::identity(), &test_segment, 0.0)
            .expect("Should be supported");
        res.is_some()
    }

    #[test]
    fn no_collisions() {
        let sub_pos = Matrix3x1::zeros();
        let target_pos = Matrix3x1::new(5.0, 5.0, 5.0);
        let cuboid_pos = Matrix3x1::new(-1.0, -1.0, -1.0);
        let cuboid_size = Matrix3x1::new(1.0, 1.0, 1.0);
        let rot = Quaternion::identity();

        let (bbox, cuboid_pos, segment, map_bounds) = 
            setup(sub_pos, target_pos, cuboid_pos, cuboid_size, rot);

        let new_point = get_new_point(&bbox, &cuboid_pos, segment, &map_bounds);
        assert!(new_point.is_none())
    }

    #[test]
    fn simple_collision() {
        let sub_pos = Matrix3x1::zeros();
        let target_pos = Matrix3x1::new(6.0, 6.0, 6.0);
        let cuboid_pos = Matrix3x1::new(3.0, 3.0, 3.0);
        let cuboid_size = Matrix3x1::new(1.0, 1.0, 1.0);
        let rot = Quaternion::identity();

        let (bbox, cuboid_pos, segment, map_bounds) = 
            setup(sub_pos, target_pos, cuboid_pos, cuboid_size, rot);

        let new_point = get_new_point(&bbox, &cuboid_pos, segment, &map_bounds)
            .expect("Collision didn't happen");
        let point = Point3::new(0.0, 2.5, 0.0);
        assert_eq!(point, new_point);

        assert!(!collides(segment.a, bbox, cuboid_pos, new_point))
    }

    fn barely_colliding() {
        let sub_pos = Matrix3x1::zeros();
        let target_pos = Matrix3x1::new(5.0, 5.0, 5.0);
        let cuboid_pos = Matrix3x1::new(1.0, 1.0, 1.0);
        let cuboid_size = Matrix3x1::new(1.0, 1.0, 1.0);
        let rot = Quaternion::identity();

        let (bbox, cuboid_pos, segment, map_bounds) = 
            setup(sub_pos, target_pos, cuboid_pos, cuboid_size, rot);

        let new_point = get_new_point(&bbox, &cuboid_pos, segment, &map_bounds)
            .expect("Collision didn't happen");
        let point = Point3::new(0.0, 2.5, 0.0);
        assert_eq!(point, new_point);

        assert!(!collides(segment.a, bbox, cuboid_pos, new_point))
    }
}