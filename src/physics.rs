use crate::{
    math::{EPSILON, EpsilonEquals, Plane, Vec3},
    world::Cuboid,
};
const MAX_MANIFOLD_VERTICES: usize = 8;

#[derive(PartialEq, Debug)]
pub enum CollisionType {
    Face,
    EdgeEdge(Vec3, Vec3),
}
#[derive(Debug)]
pub struct CollisionInfo {
    pub instance_index: usize,
    pub other_index: usize,
    pub mtv: Vec3,
    pub collision_type: CollisionType,
    pub manifold: [Option<ContactPoint>; MAX_MANIFOLD_VERTICES],
}

#[derive(Clone, Copy, Debug)]
pub struct ContactPoint {
    pub point: Vec3,
    pub depth: f32,
}
fn sat(instance: &Cuboid, other: &Cuboid) -> Option<(Vec3, CollisionType)> {
    let mut edge_axes = [Vec3::default(); 9];
    for i in 0..3 {
        for j in 0..3 {
            let cross = instance.face_axes[i].cross(&other.face_axes[j]);
            if let Some(cross) = cross.normalize() {
                edge_axes[i * 3 + j] = cross;
            }
        }
    }
    let mut mtv: Vec3 = Vec3::default();
    let mut min_overlap = f32::INFINITY;
    let mut mtv_index = 0;
    for (i, axis) in instance
        .face_axes
        .iter()
        .chain(other.face_axes.iter())
        .chain(edge_axes.iter())
        .enumerate()
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
            mtv_index = i;
        }
    }

    let mut scaled_mtv = mtv * min_overlap;
    if (instance.position - other.position).dot(&scaled_mtv) < 0.0 {
        scaled_mtv = -scaled_mtv; //other->self
    }

    Some((
        scaled_mtv,
        if mtv_index < 6 {
            CollisionType::Face
        } else {
            CollisionType::EdgeEdge(
                edge_axes[(mtv_index - 6) / 3],
                edge_axes[(mtv_index - 6) % 3],
            )
        },
    ))
}

fn calc_contact_manifold(
    instance: &Cuboid,
    other: &Cuboid,
    collision_normal: Vec3,
) -> [Option<ContactPoint>; MAX_MANIFOLD_VERTICES] {
    //sutherland-hodgman
    //remember collision_normal is other->self
    let incident_face = most_aligned_with(&other.get_all_face_axes(), &collision_normal);
    let reference_face = most_aligned_with(&instance.get_all_face_axes(), &-collision_normal);
    let incident_face_vertices =
        order_face_vertices(&incident_face, get_face_vertices(&incident_face, other));
    let reference_face_vertices = order_face_vertices(
        &reference_face,
        get_face_vertices(&reference_face, instance),
    );
    let reference_face_sides = [
        (reference_face_vertices[0], reference_face_vertices[1]),
        (reference_face_vertices[1], reference_face_vertices[2]),
        (reference_face_vertices[2], reference_face_vertices[3]),
        (reference_face_vertices[3], reference_face_vertices[0]),
    ];

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
    for (ref_p1, ref_p2) in reference_face_sides {
        let reference_plane = Plane {
            point: ref_p1,
            normal: (ref_p2 - ref_p1)
                .cross(&reference_face)
                .normalize()
                .unwrap(),
        };

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
            let p1_inside = reference_plane.distance_to_point(&p1) <= EPSILON;
            let p2_inside = reference_plane.distance_to_point(&p2) <= EPSILON;
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
                            reference_plane.intersect_with_line_segment(&p1, &p2);
                        next_clipped_len += 1;
                    }
                }
                (false, true) => {
                    if next_clipped_len < MAX_MANIFOLD_VERTICES - 1 {
                        next_clipped[next_clipped_len] =
                            reference_plane.intersect_with_line_segment(&p1, &p2);
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

    let reference_plane = Plane {
        normal: reference_face,
        //OPTIMIZE: could reuse get_face_vertices() call from before (not sure if worth it)
        point: get_face_vertices(&reference_face, instance)[0],
    };
    let mut manifold = [None; MAX_MANIFOLD_VERTICES];
    let mut manifold_len = 0;
    for point in cur_clipped.iter().take(cur_clipped_len) {
        let Some(point) = point else {
            break;
        };
        let depth = reference_plane.distance_to_point(point);
        if depth < EPSILON {
            manifold[manifold_len] = Some(ContactPoint {
                point: *point,
                depth: depth.abs(),
            });
            manifold_len += 1;
        }
    }
    manifold
}
fn order_face_vertices(normal: &Vec3, vertices: [Vec3; 4]) -> [Vec3; 4] {
    //2d
    let normal = normal.normalize().unwrap();
    let mut centre = Vec3::default();
    for v in vertices {
        centre += v;
    }
    centre /= 4.0;
    let mut ans = vertices;
    ans.sort_by(|a, b| {
        let perpendicular = (*a - centre).cross(&(*b - centre));
        let res = perpendicular.dot(&normal); //normal is "up" from bird pov
        if res > 0.0 {
            std::cmp::Ordering::Less
        } else {
            //change order of a,b->b,a
            std::cmp::Ordering::Greater
        }
    });
    ans //ccw order from bird pov
}
fn get_face_vertices(normal: &Vec3, instance: &Cuboid) -> [Vec3; 4] {
    let normal = normal.normalize().unwrap();
    let mut cube_vertices = instance.corners;
    let centre = instance.position;
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
    let target = target.normalize().unwrap();
    let mut best = (Vec3::default(), f32::NEG_INFINITY);
    for vector in vectors {
        let dot = vector.normalize().unwrap().dot(&target);
        if dot > best.1 {
            best.0 = *vector;
            best.1 = dot;
        }
    }
    best.0
}
pub fn resolve_collisions(collisions: &[CollisionInfo], instances: &mut [Cuboid], dt: f32) {
    const SOLVER_ITERATIONS: i32 = 8;
    for _ in 0..SOLVER_ITERATIONS {
        for info in collisions {
            let (instance_index, other_index) = (
                info.instance_index.min(info.other_index),
                info.instance_index.max(info.other_index),
            );

            let (slice1, slice2) = instances.split_at_mut(other_index);
            let instance = &mut slice1[instance_index];
            let other = &mut slice2[0];

            const RESTITUTION_COEFF: f32 = 0.5;
            const STATIC_FRICTION_COEFF: f32 = 0.6;
            const BAUMGARTE_BIAS: f32 = 0.3;
            const PENETRATION_TOLERANCE: f32 = 0.001;
            let collision_normal = info.mtv.normalize().unwrap();
            let inv_m1 = instance.get_inverse_mass();
            let inv_m2 = other.get_inverse_mass();
            let inv_moi1 = instance.get_inverse_moment_of_inertia();
            let inv_moi2 = other.get_inverse_moment_of_inertia();

            for point in &info.manifold {
                let Some(ContactPoint { point, depth }) = point else {
                    continue;
                };
                let point = *point;
                let depth = (*depth - PENETRATION_TOLERANCE).max(0.0);
                let r1 = point - instance.position;
                let r2 = point - other.position;
                enum ImpulseType {
                    Normal,
                    Tangent(f32),
                }
                let apply_impulse = |instance: &mut Cuboid,
                                     other: &mut Cuboid,
                                     impulse_type: ImpulseType|
                 -> Option<f32> {
                    let v1 = instance.velocity;
                    let v2 = other.velocity;
                    let w1 = instance.angular_velocity;
                    let w2 = other.angular_velocity;
                    let v_rel = (v1 + w1.cross(&r1)) - (v2 + w2.cross(&r2));
                    let v_n = v_rel.dot(&collision_normal);
                    let impulse_dir = match impulse_type {
                        ImpulseType::Normal => collision_normal,
                        ImpulseType::Tangent(_) => {
                            let v_rel_tangent = v_rel - v_n * collision_normal;
                            v_rel_tangent.normalize()?
                        }
                    };
                    let v_error = match impulse_type {
                        ImpulseType::Normal => v_n,
                        ImpulseType::Tangent(_) => v_rel.dot(&impulse_dir),
                    };
                    if matches!(impulse_type, ImpulseType::Normal) && v_error > 0.0 {
                        return None;
                    }

                    let unit_delta_w1 = &inv_moi1 * &(r1.cross(&impulse_dir));
                    let unit_delta_v1 = unit_delta_w1.cross(&r1);
                    let unit_delta_w2 = &inv_moi2 * &(r2.cross(&impulse_dir));
                    let unit_delta_v2 = unit_delta_w2.cross(&r2);

                    let effective_inv_mass = inv_m1
                        + inv_m2
                        + unit_delta_v1.dot(&impulse_dir)
                        + unit_delta_v2.dot(&impulse_dir);
                    if effective_inv_mass.epsilon_equals(0.0) {
                        return None;
                    }

                    let target_velo = match impulse_type {
                        ImpulseType::Normal => {
                            let restitution_velo = -v_error * RESTITUTION_COEFF;
                            let baumgarte = BAUMGARTE_BIAS / dt * depth;
                            restitution_velo + baumgarte
                        }
                        ImpulseType::Tangent(_) => 0.0,
                    };
                    let impulse_mag = (target_velo - v_error) / effective_inv_mass;
                    let impulse_mag = match impulse_type {
                        ImpulseType::Normal => impulse_mag.max(0.0), //only push, never pull

                        ImpulseType::Tangent(normal_impulse_mag) => {
                            let max_friction = STATIC_FRICTION_COEFF * normal_impulse_mag;
                            impulse_mag.clamp(-max_friction, max_friction)
                        }
                    };
                    let impulse = impulse_mag * impulse_dir;
                    instance.velocity += impulse * inv_m1;
                    instance.angular_velocity += &inv_moi1 * &(r1.cross(&impulse));
                    other.velocity -= impulse * inv_m2;
                    other.angular_velocity -= &inv_moi2 * &(r2.cross(&impulse));
                    match impulse_type {
                        ImpulseType::Normal => Some(impulse_mag),
                        ImpulseType::Tangent(_) => None,
                    }
                };

                let Some(normal_impulse_mag) = apply_impulse(instance, other, ImpulseType::Normal)
                else {
                    continue;
                };
                apply_impulse(instance, other, ImpulseType::Tangent(normal_impulse_mag));
            }
        }
    }
}

pub fn detect_collision(instance: &Cuboid, other: &Cuboid) -> Option<CollisionInfo> {
    if instance.aabb.intersects(&other.aabb) {
        let (mtv, collision_type) = sat(instance, other)?;
        //NOTE:i should handle edgeedge separately, but there's not much of a diff since edge-edge
        //is rare
        // match collision_type {
        //     CollisionType::Face => (), //default sutherland-hodgman logic
        //     CollisionType::EdgeEdge(instance_edge, other_edge) => {
        //         //find closest point between these 2 3d vectors
        //     }
        // }
        let manifold = calc_contact_manifold(instance, other, mtv.normalize()?);
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

    use crate::math::Quaternion;

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
        let manifold: Vec<ContactPoint> = calc_contact_manifold(
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
            manifold.iter().map(|x| x.depth).collect::<Vec<_>>(),
            vec![0.5, 0.5, 0.5, 0.5]
        );
        let mut points = manifold.iter().map(|x| x.point).collect::<Vec<_>>();
        points.sort_by(|a, b| {
            if a.x != b.x {
                a.x.partial_cmp(&b.x).unwrap()
            } else if a.y != b.y {
                a.y.partial_cmp(&b.y).unwrap()
            } else {
                a.z.partial_cmp(&b.z).unwrap()
            }
        });
        assert_eq!(
            points,
            vec![
                Vec3 {
                    x: -0.5,
                    y: 0.0,
                    z: -0.5
                },
                Vec3 {
                    x: -0.5,
                    y: 0.0,
                    z: 0.5
                },
                Vec3 {
                    x: 0.5,
                    y: 0.0,
                    z: -0.5
                },
                Vec3 {
                    x: 0.5,
                    y: 0.0,
                    z: 0.5
                }
            ]
        );
        assert!(manifold.iter().all(|x| (x.depth - 0.5).abs() < EPSILON));
    }
    #[test]
    fn test_contact_manifold_vertex_face() {
        let mut c1 = Cuboid::default();
        let mut c2 = Cuboid {
            position: Vec3 {
                x: 0.0,
                y: 1.3,
                z: 0.0,
            },
            rotation: Quaternion::from_angle(
                &Vec3 {
                    x: std::f32::consts::FRAC_1_SQRT_2,
                    y: 0.0,
                    z: -std::f32::consts::FRAC_1_SQRT_2,
                },
                2.1862,
            ),
            ..Default::default()
        };
        c1.update_derived();
        c2.update_derived();
        let manifold: Vec<ContactPoint> = calc_contact_manifold(
            &c1,
            &c2,
            Vec3 {
                x: 0.0,
                y: -1.0,
                z: 0.0,
            },
        )
        .into_iter()
        .flatten()
        .collect();
        dbg!(&manifold);
        assert_eq!(manifold.len(), 1);
    }
}
