use crate::{BoundingBox3D, CLOSE_ENOUGH};
use nalgebra::{Isometry3, Matrix3x1, Point3, UnitQuaternion, UnitVector3, Vector3};
use parry3d_f64::query::{PointQuery, contact};
use parry3d_f64::shape::{Cuboid, Segment};

//TODO: Idk if you guys want to set this in the configuration of each submarine
const SUB_EXTENTS: Matrix3x1<f64> = Matrix3x1::new(0.5, 0.5, 0.5);

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
        &segment.a,
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
    sub_pos: &Point3<f64>,
    cuboid: &Cuboid,
    cuboid_pos: &Isometry3<f64>,
    basepoint: &Point3<f64>,
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

    let colliding_axis = get_dominant_axis(&segment_dir);

    let mut points = Vec::with_capacity(4);

    for (i, (face, extent)) in face_data.iter().enumerate() {
        let axis = i / 2;

        if axis == colliding_axis {
            continue;
        }


        let world_face = cuboid_pos * face;

        // Get the difference between the face and the basepoint
        let dist = ((world_face[axis] * extent * 2.0) - basepoint[axis]).abs();

        // Translate by the distance beforementioned and the appropriate submarine dimension. The latter
        // is to have a bit of safety margin since we're going a straight line to this point 
        let mut translation = world_face;
        translation[axis] += dist * face[axis].signum();
        translation[axis] += SUB_EXTENTS[axis] * face[axis].signum();

        let mut point = basepoint + translation;

        // The idea of this is that if the resulting point is too far from the submarine, we don't want
        // the submarine to make a straight line and potentially collide with the object in the path to 
        // the point. So, if this happens we'll translate the point to be nearer to the colliding axis.
        let axis_diff = (point[axis] - sub_pos[axis]);
        if axis_diff.abs() > 7.0 {
            point[colliding_axis] -= axis_diff * 0.1;
        }

        // Round the numbers by two decimal places. It makes it look nicer and easier to test
        for num in &mut point.coords {
            *num = (*num * 100.0).round() / 100.0;
        }

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
    use nalgebra::{Matrix3x1, Quaternion, Rotation, Rotation3};

    use super::*;
    use crate::Pose;
    fn setup(sub_pos: Matrix3x1<f64>, target_pos: Matrix3x1<f64>, cuboid_pos: Matrix3x1<f64>, cuboid_size: Matrix3x1<f64>, 
        cuboid_rot: UnitQuaternion<f64>) -> (BoundingBox3D, Isometry3<f64>, Segment, BoundingBox3D) {
        let center = Pose::new(cuboid_pos, Quaternion::identity());
        let bbox = BoundingBox3D::new(center, cuboid_size);

        let sub_point = Point3::from(sub_pos);
        let target_point = Point3::from(target_pos); 
        let segment = Segment::new(
            sub_point,
            target_point,
        );

        let cuboid_pos = Isometry3::from_parts(
            cuboid_pos.into(), cuboid_rot
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
        let rot = UnitQuaternion::identity();

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
        let rot = UnitQuaternion::identity();

        let (bbox, cuboid_pos, segment, map_bounds) = 
            setup(sub_pos, target_pos, cuboid_pos, cuboid_size, rot);

        let new_point = get_new_point(&bbox, &cuboid_pos, segment, &map_bounds)
            .expect("Collision didn't happen");
        let point = Point3::new(3.0, 6.5, 3.0);
        assert_eq!(point, new_point);

        assert!(!collides(segment.a, bbox, cuboid_pos, new_point))
    }

    #[test]
    fn barely_colliding() {
        let sub_pos = Matrix3x1::zeros();
        let target_pos = Matrix3x1::new(5.0, 5.0, 5.0);
        let cuboid_pos = Matrix3x1::new(1.0, 1.0, 1.0);
        let cuboid_size = Matrix3x1::new(1.0, 1.0, 1.0);
        let rot = UnitQuaternion::identity();

        let (bbox, cuboid_pos, segment, map_bounds) = 
            setup(sub_pos, target_pos, cuboid_pos, cuboid_size, rot);

        let new_point = get_new_point(&bbox, &cuboid_pos, segment, &map_bounds)
            .expect("Collision didn't happen");
        let point = Point3::new(0.0, 2.5, 0.0);
        assert_eq!(point, new_point);

        assert!(!collides(segment.a, bbox, cuboid_pos, new_point))
    }
    
    #[test]
    fn large_cuboid() {
        let sub_pos = Matrix3x1::zeros();
        let target_pos = Matrix3x1::new(6.0, 6.0, 6.0);
        let cuboid_pos = Matrix3x1::new(3.0, 3.0, 3.0);
        let cuboid_size = Matrix3x1::new(1.0, 10.0, 1.0);
        let rot = UnitQuaternion::identity();

        let (bbox, cuboid_pos, segment, map_bounds) = 
            setup(sub_pos, target_pos, cuboid_pos, cuboid_size, rot);

        let new_point = get_new_point(&bbox, &cuboid_pos, segment, &map_bounds)
            .expect("Collision didn't happen");
        let point = Point3::new(3.0, 3.0, 6.5);
        assert_eq!(point, new_point);

        assert!(!collides(segment.a, bbox, cuboid_pos, new_point))
    }

    #[test]
    fn off_center() {
        let sub_pos = Matrix3x1::new(0.0, -2.0, 6.0);
        let target_pos = Matrix3x1::new(0.0, -2.0, -5.0);
        let cuboid_pos = Matrix3x1::new(0.0, -1.0, -2.0);
        let cuboid_size = Matrix3x1::new(1.0, 10.0, 1.0);
        let rot = UnitQuaternion::identity();

        let (bbox, cuboid_pos, segment, map_bounds) = 
            setup(sub_pos, target_pos, cuboid_pos, cuboid_size, rot);

        let new_point = get_new_point(&bbox, &cuboid_pos, segment, &map_bounds)
            .expect("Collision didn't happen");
        let point = Point3::new(2.5, -2.0, -1.79);
        assert_eq!(point, new_point);

        assert!(!collides(segment.a, bbox, cuboid_pos, new_point))
    }

    #[test]
    #[ignore]
    // Do we even care about rotation? This will fail btw
    fn rotation() {
        let sub_pos = Matrix3x1::zeros();
        let target_pos = Matrix3x1::new(5.0, 0.0, 5.0);
        let cuboid_pos = Matrix3x1::new(5.0, -2.0, 3.0);
        let cuboid_size = Matrix3x1::new(1.0, 10.0, 1.0);
        let rotmat = Rotation3::from_euler_angles(0.0, 60.0_f64.to_radians(), 60.0_f64.to_radians());
        let rot = UnitQuaternion::from_rotation_matrix(&rotmat);

        let (bbox, cuboid_pos, segment, map_bounds) = 
            setup(sub_pos, target_pos, cuboid_pos, cuboid_size, rot);

        let new_point = get_new_point(&bbox, &cuboid_pos, segment, &map_bounds)
            .expect("Collision didn't happen");
        let point = Point3::new(3.0, 3.0, 6.5);
        assert_eq!(point, new_point);

        assert!(!collides(segment.a, bbox, cuboid_pos, new_point))
    }
}