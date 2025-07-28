use crate::{
    math::{EPSILON, Vec3},
    world::Cuboid,
};

pub fn sat(instance: &Cuboid, other: &Cuboid) -> Option<Vec3> {
    let global_axes = [
        Vec3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        },
        Vec3 {
            x: 0.0,
            y: 1.0,
            z: 0.0,
        },
        Vec3 {
            x: 0.0,
            y: 0.0,
            z: 1.0,
        },
    ];
    let face_axes: [Vec3; 6] = std::array::from_fn(|i| {
        if i % 2 == 0 {
            global_axes[i / 2].rotate(instance.rotation).normalize()
        } else {
            global_axes[i / 2].rotate(other.rotation).normalize()
        }
    });
    let mut edge_axes = [Vec3::default(); 9];
    for i in 0..3 {
        for j in 0..3 {
            let mut cross = face_axes[i].cross(&face_axes[j + 3]);
            if cross.mag() > EPSILON {
                cross = cross.normalize();
            }
            edge_axes[i * 3 + j] = cross;
        }
    }

    let mut mtv: Vec3 = Vec3::default();
    let mut min_overlap = f32::INFINITY;
    for axis in face_axes.iter().chain(edge_axes.iter()) {
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

fn apply_mtv(instance: &mut Cuboid, other: &mut Cuboid, mtv: Vec3) {
    if !instance.frozen && !other.frozen {
        let im1 = 1.0 / instance.get_mass();
        let im2 = 1.0 / other.get_mass();
        let sum = im1 + im2;

        instance.position += mtv * (im1 / sum); //move less if heavier
        other.position -= mtv * (im2 / sum);
    } else {
        #[allow(clippy::collapsible_else_if)]
        if instance.frozen {
            other.position -= mtv;
        } else {
            instance.position += mtv;
        }
    }
}
fn calc_contact_point(instance: &mut Cuboid, other: &mut Cuboid, mtv: Vec3) {
    //sutherland-hodgman
}
pub fn resolve(instance: &mut Cuboid, other: &mut Cuboid, mtv: Vec3) {
    const RESTITUTION_COEFF: f32 = 1.0; //TODO: 0.5
    apply_mtv(instance, other, mtv);
    {
        //velocities
        let collision_normal = mtv.normalize();
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
}
