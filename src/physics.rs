use crate::{
    math::{EPSILON, EpsilonEquals, Plane, Vec3},
    world::{Cuboid, N, World},
};
const MAX_MANIFOLD_VERTICES: usize = 8;

#[derive(PartialEq, Debug)]
pub enum CollisionType {
    Face,
    Edge,
}
pub struct CollisionInfo {
    pub instance_index: usize,
    pub other_index: usize,
    pub mtv: Vec3,
    pub collision_type: CollisionType,
    pub manifold: [Option<(Vec3, f32)>; MAX_MANIFOLD_VERTICES],
}
fn sat(instance: &Cuboid, other: &Cuboid) -> Option<(Vec3, CollisionType)> {
    let mut edge_axes = [Vec3::default(); 9];
    for i in 0..3 {
        for j in 0..3 {
            let mut cross = instance.face_axes[i].cross(&other.face_axes[j]);
            if cross.mag() > EPSILON {
                cross = cross.normalize();
            }
            edge_axes[i * 3 + j] = cross;
        }
    }
    let mut mtv: Vec3 = Vec3::default();
    let mut min_overlap = f32::INFINITY;
    for axis in instance
        .face_axes
        .iter()
        .chain(other.face_axes.iter())
        .chain(edge_axes.iter())
    {
        if axis.mag() <= EPSILON {
            continue;
        }
        let projected_corners1 = instance.corners.map(|x| x.dot(axis));
        let projected_corners2 = other.corners.map(|x| x.dot(axis));
        let mut min1 = f32::INFINITY;
        let mut min2 = f32::INFINITY;
        let mut max1 = f32::NEG_INFINITY;
        let mut max2 = f32::NEG_INFINITY;
        for x in projected_corners1 {
            min1 = min1.min(x);
            max1 = max1.max(x);
        }
        for x in projected_corners2 {
            min2 = min2.min(x);
            max2 = max2.max(x);
        }
        if max1 < min2 || max2 < min1 {
            return None;
        }
        let overlap = max1.min(max2) - min1.max(min2);
        if overlap < min_overlap {
            min_overlap = overlap;
            mtv = *axis;
        }
    }

    let mut scaled_mtv = mtv * min_overlap;
    if (instance.position - other.position).dot(&scaled_mtv) < 0.0 {
        scaled_mtv = -scaled_mtv; //other->self
    }

    Some((scaled_mtv, CollisionType::Face)) //TODO: support edge-edge etc if needed
}

fn calc_contact_manifold(
    instance: &Cuboid,
    other: &Cuboid,
    collision_normal: Vec3,
) -> [Option<(Vec3, f32)>; MAX_MANIFOLD_VERTICES] {
    //sutherland-hodgman

    //remember collision_normal is other->self
    let incident_face = most_aligned_with(&other.face_axes, &collision_normal);
    let incident_face_vertices =
        order_face_vertices(&incident_face, get_face_vertices(&collision_normal, other));
    let mut cur_clipped: [Option<Vec3>; MAX_MANIFOLD_VERTICES] = [
        Some(incident_face_vertices[0]),
        Some(incident_face_vertices[1]),
        Some(incident_face_vertices[2]),
        Some(incident_face_vertices[3]),
        None,
        None,
        None,
        None,
    ];
    let mut cur_clipped_len = 4;
    let all_instance_normals = [
        instance.face_axes[0],
        instance.face_axes[1],
        instance.face_axes[2],
        -instance.face_axes[0],
        -instance.face_axes[1],
        -instance.face_axes[2],
    ];
    for reference_plane in all_instance_normals.map(|x| Plane {
        point: get_face_vertices(&x, instance)[0],
        normal: x,
    }) {
        if reference_plane
            .normal
            .dot(&incident_face)
            .epsilon_equals(-1.0)
        {
            continue; //back face
        }
        let mut next_clipped: [Option<Vec3>; MAX_MANIFOLD_VERTICES] = [None; 8];
        let mut next_clipped_len = 0;
        for (i, p1) in cur_clipped.iter().enumerate() {
            let Some(p1) = *p1 else {
                break;
            };

            let Some(p2) = cur_clipped[(i + 1) % cur_clipped_len] else {
                break;
            };
            // <=0.0 bc plane normals point outward
            let p1_inside = reference_plane.distance_to_point(&p1) <= 0.0;
            let p2_inside = reference_plane.distance_to_point(&p2) <= 0.0;
            match (p1_inside, p2_inside) {
                (true, true) => {
                    if next_clipped_len < MAX_MANIFOLD_VERTICES {
                        next_clipped[next_clipped_len] = Some(p2);
                        next_clipped_len += 1;
                    }
                }

                (true, false) => {
                    if next_clipped_len < MAX_MANIFOLD_VERTICES {
                        next_clipped[next_clipped_len] =
                            Some(reference_plane.intersect_with_line_segment(&p1, &p2));
                        next_clipped_len += 1;
                    }
                }
                (false, true) => {
                    if next_clipped_len < MAX_MANIFOLD_VERTICES - 1 {
                        next_clipped[next_clipped_len] =
                            Some(reference_plane.intersect_with_line_segment(&p1, &p2));
                        next_clipped_len += 1;
                        next_clipped[next_clipped_len] = Some(p2);
                        next_clipped_len += 1;
                    }
                }
                (false, false) => (),
            }
        }
        cur_clipped = next_clipped;
        cur_clipped_len = next_clipped_len;
    }
    let reference_face = most_aligned_with(&all_instance_normals, &-collision_normal);
    let reference_plane = Plane {
        normal: reference_face,
        //OPTIMIZE: could reuse get_face_vertices() call from before (not sure if worth it)
        point: get_face_vertices(&reference_face, instance)[0],
    };
    let mut manifold = [None; MAX_MANIFOLD_VERTICES];
    for i in 0..cur_clipped_len {
        let Some(point) = cur_clipped[i] else {
            break;
        };
        let depth = reference_plane.distance_to_point(&point);
        if depth <= 0.0 {
            manifold[i] = Some((point, depth.abs()));
        }
    }
    manifold
}
fn order_face_vertices(normal: &Vec3, mut vertices: [Vec3; 4]) -> [Vec3; 4] {
    let normal = normal.normalize();
    let mut centre = Vec3::default();
    for v in vertices {
        centre += v;
    }
    centre /= 4.0;
    vertices.sort_by(|a, b| {
        let perpendicular = (*a - centre).cross(&(*b - centre));
        let res = perpendicular.dot(&normal); //normal is "up" from bird pov
        if res > 0.0 {
            std::cmp::Ordering::Less
        } else {
            //change order of a,b->b,a
            std::cmp::Ordering::Greater
        }
    });
    vertices //ccw order from bird pov
}
fn get_face_vertices(normal: &Vec3, instance: &Cuboid) -> [Vec3; 4] {
    let normal = normal.normalize();
    let mut cube_vertices = instance.corners.clone();
    let centre = instance.centre;
    cube_vertices.sort_by(|a, b| {
        let distance1 = (*a - centre).dot(&normal);
        let distance2 = (*b - centre).dot(&normal);
        distance2 //bc vertex-centre should point in same general dir as normal (outward)
            .partial_cmp(&distance1)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    [
        cube_vertices[0],
        cube_vertices[1],
        cube_vertices[2],
        cube_vertices[3],
    ]
}
fn most_aligned_with(vectors: &[Vec3], target: &Vec3) -> Vec3 {
    let target = target.normalize();
    let mut best = (Vec3::default(), f32::NEG_INFINITY);
    for vector in vectors {
        let dot = vector.normalize().dot(&target);
        if dot > best.1 {
            best.0 = *vector;
            best.1 = dot;
        }
    }
    best.0
}
pub fn apply_impulse(info: &CollisionInfo, instances: &mut [Cuboid; N + 1], dt: f32) {
    let (instance_index, other_index) = (
        info.instance_index.min(info.other_index),
        info.instance_index.max(info.other_index),
    );

    let (slice1, slice2) = instances.split_at_mut(other_index);
    let instance = &mut slice1[instance_index];
    let other = &mut slice2[0];
    const RESTITUTION_COEFF: f32 = 0.5;
    const SOLVER_ITERATIONS: i32 = 8;
    const BAUMGARTE_BIAS: f32 = 0.2;
    let collision_normal = info.mtv.normalize();
    let inv_m1 = instance.get_inverse_mass();
    let inv_m2 = other.get_inverse_mass();
    let e = RESTITUTION_COEFF;
    let inv_moi1 = instance.get_inverse_moment_of_inertia();
    let inv_moi2 = other.get_inverse_moment_of_inertia();

    for _ in 0..SOLVER_ITERATIONS {
        for point in info.manifold {
            let Some((point, depth)) = point else {
                break;
            };
            let v1 = instance.velocity;
            let v2 = other.velocity;
            let w1 = instance.angular_velocity;
            let w2 = other.angular_velocity;
            let r1 = point - instance.centre;
            let r2 = point - other.centre;
            let v_rel = (v1 + w1.cross(&r1)) - (v2 + w2.cross(&r2));
            let v_n = v_rel.dot(&collision_normal);
            //baumgarte stabilization
            let bias = -(BAUMGARTE_BIAS / dt) * depth.max(0.0);

            let rot_inertia1 =
                (&inv_moi1 * &(r1.cross(&collision_normal)).cross(&r1)).dot(&collision_normal);
            let rot_inertia2 =
                (&inv_moi2 * &(r2.cross(&collision_normal)).cross(&r2)).dot(&collision_normal);
            let effective_mass = inv_m1 + inv_m2 + rot_inertia1 + rot_inertia2;
            let impulse_mag = (-(1.0 + e) * v_n + bias) / effective_mass;
            let impulse = impulse_mag * collision_normal;
            instance.velocity += impulse * inv_m1;
            instance.angular_velocity += &inv_moi1 * &(r1.cross(&impulse));

            other.velocity -= impulse * inv_m2;
            other.angular_velocity -= &inv_moi2 * &(r2.cross(&impulse));
        }
    }
}
pub fn detect_collision(instance: &Cuboid, other: &Cuboid) -> Option<CollisionInfo> {
    if instance.aabb.intersects(&other.aabb) {
        let (mtv, collision_type) = sat(instance, other)?;
        let manifold = calc_contact_manifold(instance, other, mtv.normalize());
        let collision_info = CollisionInfo {
            instance_index: instance.index,
            other_index: other.index,
            mtv,
            collision_type,
            manifold,
        };

        Some(collision_info)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {

    use super::*;
    #[test]
    fn test_sat_no_collision() {
        let mut c1 = Cuboid::default();
        let mut c2 = Cuboid {
            position: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 10.0,
            },
            ..Default::default()
        };
        c1.update_derived();
        c2.update_derived();
        assert!(sat(&c1, &c2).is_none());
    }
    #[test]
    fn test_sat_collision_type() {
        //between Face and Edge
        let mut c1 = Cuboid::default();
        let mut c2 = Cuboid {
            position: Vec3 {
                x: 0.0,
                y: 0.5,
                z: 0.0,
            },
            ..Default::default()
        };
        c1.update_derived();
        c2.update_derived();
        let res = sat(&c1, &c2).expect("sat was None");
        assert_eq!(res.1, CollisionType::Face);
        assert!(res.0.x.abs() < 1e-5);
        assert!((res.0.y + 0.5).abs() < 1e-5);
        assert!(res.0.z.abs() < 1e-5);
    }
    #[test]
    fn test_contact_manifold_flush_face() {
        let mut c1 = Cuboid::default();
        let mut c2 = Cuboid {
            position: Vec3 {
                x: 0.0,
                y: -0.5,
                z: 0.0,
            },
            scale: Vec3 {
                x: 100.0,
                y: 1.0,
                z: 100.0,
            },

            ..Default::default()
        };
        c1.update_derived();
        c2.update_derived();
        let manifold: Vec<(Vec3, f32)> = calc_contact_manifold(
            &c1,
            &c2,
            Vec3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
        )
        .into_iter()
        .flatten()
        .collect();
        assert_eq!(manifold.len(), 4);
        assert_eq!(
            manifold.iter().map(|x| x.1).collect::<Vec<_>>(),
            vec![0.5, 0.5, 0.5, 0.5]
        );
    }
}
