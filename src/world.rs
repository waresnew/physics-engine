use crate::math::{EPSILON, Quaternion, Vec3};

pub struct World {
    pub instances: [Cuboid; N * N],
    floor: Cuboid,
}

// SI units
const N: usize = 2;
const RESTITUTION_COEFF: f32 = 1.0;
const DENSITY: f32 = 850.0;
const INSTANCE_SPACING: f32 = 2.0;
const GRAV_ACCEL: Vec3 = Vec3 {
    x: 0.0,
    y: -9.81,
    z: 0.0,
};

impl World {
    #[allow(clippy::new_without_default)]
    pub fn new() -> Self {
        let mut instances: [Cuboid; N * N] = [Cuboid::default(); N * N];
        for i in 0..N {
            for j in 0..N {
                let scale = Vec3 {
                    x: 1.0,
                    y: 1.0,
                    z: 1.0,
                };
                let position = Vec3 {
                    x: i as f32 * INSTANCE_SPACING * scale.x,
                    y: 10.0,
                    z: j as f32 * INSTANCE_SPACING * scale.z,
                };
                instances[i * N + j] = Cuboid {
                    position,
                    scale,
                    ..Default::default()
                };
            }
        }
        let mut floor = Cuboid {
            scale: Vec3 {
                x: 200.0,
                y: 1.0,
                z: 200.0,
            },
            frozen: true,
            ..Default::default()
        };
        floor.update_derived();

        Self { instances, floor }
    }

    pub fn update(&mut self, dt: f32) {
        for i in 0..self.instances.len() {
            if self.instances[i].frozen {
                continue;
            }
            let instance = &mut self.instances[i];
            instance.velocity += GRAV_ACCEL * dt;
            instance.position += instance.velocity * dt;
            instance.update_derived();

            //OPTIMIZE:O(n^2)
            for j in (i + 1)..self.instances.len() {
                let (a, b) = self.instances.split_at_mut(j);
                resolve(&mut a[i], &mut b[0]);
            }
            resolve(&mut self.instances[i], &mut self.floor);

            // instance.rotation = (instance.rotation
            //     * Quaternion::from_angle(
            //         &Vec3 {
            //             x: 1.0,
            //             y: 1.0,
            //             z: 1.0,
            //         },
            //         0.02,
            //     ))
            // .normalize();
        }
        fn resolve(instance: &mut Cuboid, other: &mut Cuboid) {
            if instance.aabb.intersects(&other.aabb) {
                if let Some(mtv) = instance.sat(other) {
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
                    instance.collide(other, &mtv.normalize());
                }
            }
        }
    }
}

#[derive(Copy, Clone)]
pub struct Cuboid {
    position: Vec3,       //centre
    rotation: Quaternion, //TODO: normalize quaternion after updating it so that i can assume it already is when reading it
    velocity: Vec3,
    angular_velocity: Vec3,
    scale: Vec3,
    corners: [Vec3; 8],
    aabb: AABB,
    frozen: bool,
}
impl Cuboid {
    pub fn update_derived(&mut self) {
        self.calc_corners();
        self.calc_aabb();
    }
    pub fn collide(&mut self, other: &mut Cuboid, normal: &Vec3) {}
    pub fn get_mass(&self) -> f32 {
        self.scale.mag() * DENSITY
    }
    fn calc_corners(&mut self) {
        let delta: Vec3 = self.scale / 2.0;
        let mut index = 0;
        let mut ans = [Vec3::default(); 8];
        for i in [-1.0, 1.0] {
            for j in [-1.0, 1.0] {
                for k in [-1.0, 1.0] {
                    ans[index] = self.position
                        + (Vec3 {
                            x: i * delta.x,
                            y: j * delta.y,
                            z: k * delta.z,
                        }
                        .rotate(self.rotation));
                    index += 1;
                }
            }
        }
        self.corners = ans;
    }
    pub fn sat(&self, other: &Cuboid) -> Option<Vec3> {
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
                global_axes[i / 2].rotate(self.rotation).normalize()
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
            let projected_corners1 = self.corners.map(|x| x.dot(axis));
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
        if (self.position - other.position).dot(&scaled_mtv) < 0.0 {
            scaled_mtv = -scaled_mtv; //other->self
        }

        Some(scaled_mtv)
    }
    pub fn calc_aabb(&mut self) {
        let mut min_x = f32::INFINITY;
        let mut min_y = f32::INFINITY;
        let mut min_z = f32::INFINITY;
        let mut max_x = f32::NEG_INFINITY;
        let mut max_y = f32::NEG_INFINITY;
        let mut max_z = f32::NEG_INFINITY;
        for corner in self.corners {
            min_x = min_x.min(corner.x);
            max_x = max_x.max(corner.x);
            min_y = min_y.min(corner.y);
            max_y = max_y.max(corner.y);
            min_z = min_z.min(corner.z);
            max_z = max_z.max(corner.z);
        }
        self.aabb = AABB {
            min: Vec3 {
                x: min_x,
                y: min_y,
                z: min_z,
            },
            max: Vec3 {
                x: max_x,
                y: max_y,
                z: max_z,
            },
        }
    }
    #[rustfmt::skip]
    pub fn to_raw(&self)->CuboidRaw {
        let rotation_matrix=self.rotation.to_mat3();
        CuboidRaw {
            model:[
                    rotation_matrix.array[0]*self.scale.x, rotation_matrix.array[3]*self.scale.x, rotation_matrix.array[6]*self.scale.x, 0.0,
                    rotation_matrix.array[1]*self.scale.y, rotation_matrix.array[4]*self.scale.y, rotation_matrix.array[7]*self.scale.y, 0.0,
                    rotation_matrix.array[2]*self.scale.z, rotation_matrix.array[5]*self.scale.z, rotation_matrix.array[8]*self.scale.z, 0.0,
                    self.position.x,        self.position.y,        self.position.z,                               1.0,
            ]
        }
    }
}
impl Default for Cuboid {
    fn default() -> Self {
        Self {
            scale: Vec3 {
                x: 1.0,
                y: 1.0,
                z: 1.0,
            },
            position: Vec3::default(),
            rotation: Quaternion::default(),
            velocity: Vec3::default(),
            angular_velocity: Vec3::default(),
            corners: [Vec3::default(); 8],
            aabb: AABB::default(),
            frozen: false,
        }
    }
}
#[derive(Debug, Default, Copy, Clone)]
pub struct AABB {
    min: Vec3,
    max: Vec3,
}

impl AABB {
    pub fn new(min: Vec3, max: Vec3) -> Self {
        assert!(
            min.x <= max.x && min.y <= max.y && min.z <= max.z,
            "invalid AABB: min:{min:?} max:{max:?}",
        );
        Self { min, max }
    }

    pub fn get_dimensions(&self) -> Vec3 {
        Vec3 {
            x: self.max.x - self.min.x,
            y: self.max.y - self.min.y,
            z: self.max.z - self.min.z,
        }
    }
    pub fn intersects(&self, other: &AABB) -> bool {
        !((self.min.x > other.max.x || self.min.y > other.max.y || self.min.z > other.max.z)
            || (self.max.x < other.min.x || self.max.y < other.min.y || self.max.z < other.min.z))
    }
}

#[repr(C)]
#[derive(Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct CuboidRaw {
    model: [f32; 16],
}

impl CuboidRaw {
    //4 vec4s = 1 mat4
    const ATTRIBUTES: [wgpu::VertexAttribute; 4] =
        wgpu::vertex_attr_array![3=>Float32x4,4=>Float32x4,5=>Float32x4,6=>Float32x4];
    pub fn desc() -> wgpu::VertexBufferLayout<'static> {
        use std::mem;
        wgpu::VertexBufferLayout {
            array_stride: mem::size_of::<Self>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Instance,
            attributes: &Self::ATTRIBUTES,
        }
    }
}
