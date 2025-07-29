use crate::{
    math::{EPSILON, EpsilonEquals, Plane, Vec3},
    world::Cuboid,
};
const MAX_MANIFOLD_VERTICES: usize = 8; //8 contact points if 2 faces are flush

pub fn sat(instance: &Cuboid, other: &Cuboid) -> Option<Vec3> {
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

    Some(scaled_mtv)
}

fn calc_contact_manifold(
    instance: &Cuboid,
    other: &Cuboid,
    collision_normal: Vec3,
) -> [Option<(Vec3, f32)>; MAX_MANIFOLD_VERTICES] {
    //sutherland-hodgman

    //remember collision_normal is other->self
    let incident_face = most_aligned_with(&other.face_axes, &collision_normal);
    let incident_face_vertices = order_face_vertices(
        &incident_face,
        get_face_vertices(&collision_normal, other.corners),
    );
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
    for reference_plane in instance
        .face_axes
        .iter()
        .copied()
        .chain(instance.face_axes.iter().map(|&x| -x))
        .map(|x| Plane {
            point: get_face_vertices(&x, instance.corners)[0],
            normal: x,
        })
    {
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
            let p1_inside = reference_plane.distance_to_point(p1) >= 0.0;
            let p2_inside = reference_plane.distance_to_point(p2) >= 0.0;
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
                            Some(reference_plane.intersect_with_line_segment(p1, p2));
                        next_clipped_len += 1;
                    }
                }
                (false, true) => {
                    if next_clipped_len < MAX_MANIFOLD_VERTICES - 1 {
                        next_clipped[next_clipped_len] =
                            Some(reference_plane.intersect_with_line_segment(p1, p2));
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
    let reference_face = most_aligned_with(&instance.face_axes, &-collision_normal);
    let reference_plane = Plane {
        normal: reference_face,
        //OPTIMIZE: could reuse get_face_vertices() call from before (not sure if worth it)
        point: get_face_vertices(&reference_face, instance.corners)[0],
    };
    let mut manifold = [None; MAX_MANIFOLD_VERTICES];
    for i in 0..cur_clipped_len {
        let Some(point) = cur_clipped[i] else {
            break;
        };
        let depth = reference_plane.distance_to_point(point);
        if depth > 0.0 {
            manifold[i] = Some((point, depth));
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
fn get_face_vertices(normal: &Vec3, mut cube_vertices: [Vec3; 8]) -> [Vec3; 4] {
    let normal = normal.normalize();
    let mut centre = Vec3::default();
    for v in cube_vertices {
        centre += v;
    }
    centre /= 8.0;
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
fn apply_impulse(
    instance: &mut Cuboid,
    other: &mut Cuboid,
    collision_normal: Vec3,
    contact_point: Vec3,
) {
    //TODO: support angular impulse
    const RESTITUTION_COEFF: f32 = 1.0; //TODO: 0.5
    let m1 = instance.get_mass();
    let m2 = other.get_mass();
    let e = RESTITUTION_COEFF;
    let v1_ni = instance.velocity.dot(&collision_normal);
    let v2_ni = other.velocity.dot(&collision_normal);

    let v1_nf = collision_normal
        * if instance.frozen {
            0.0
        } else if other.frozen {
            -v1_ni * e
        } else {
            (m1 * v1_ni + m2 * v2_ni - m2 * e * (v1_ni - v2_ni)) / (m1 + m2)
        };
    let v2_nf = collision_normal
        * if other.frozen {
            0.0
        } else if instance.frozen {
            -v2_ni * e
        } else {
            (m1 * v1_ni + m2 * v2_ni + m1 * e * (v1_ni - v2_ni)) / (m1 + m2)
        };
    let v1_ti = instance.velocity - v1_ni * collision_normal;
    let v2_ti = other.velocity - v2_ni * collision_normal; //TODO: implement friction
    let final_v1 = v1_nf + v1_ti;
    let final_v2 = v2_nf + v2_ti;
    instance.velocity = final_v1;
    other.velocity = final_v2;
}
pub fn resolve(instance: &mut Cuboid, other: &mut Cuboid, mtv: Vec3) {
    // apply_impulse(
    //     instance,
    //     other,
    //     mtv.normalize(),
    //     calc_contact_point(instance, other, mtv),
    // );
}
