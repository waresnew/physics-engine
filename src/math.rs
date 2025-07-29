use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};
pub const EPSILON: f32 = 1e-5;
pub trait EpsilonEquals {
    fn epsilon_equals(self, other: Self) -> bool;
}
impl EpsilonEquals for f32 {
    fn epsilon_equals(self, other: Self) -> bool {
        (self - other).abs() <= EPSILON
    }
}
#[derive(PartialEq, Copy, Clone, Debug, Default)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

macro_rules! impl_vec3_op {
    ($trait:ident, $method:ident,$op:tt, $trait_assign:ident,$method_assign:ident, $op_assign:tt) =>{
        impl $trait for Vec3 {
                type Output = Self;
            fn $method(self,other:Self)->Self {
                Self {
                    x: self.x $op other.x,
                    y:self.y $op other.y,
                    z:self.z $op other.z
                }
            }
        }

        impl $trait<f32> for Vec3 {
                type Output = Self;
            fn $method(self,other:f32)->Self{
                Self{
                    x:self.x $op other,
                    y:self.y $op other,
                    z:self.z $op other
                }
            }
        }

        impl $trait<Vec3> for f32 {
            type Output = Vec3;
            fn $method(self,other:Vec3)->Vec3 {
                Vec3{
                    x:self $op other.x,
                    y:self $op other.y,
                    z:self $op other.z
                }
            }
        }

        impl $trait_assign for Vec3 {
            fn $method_assign(&mut self, other:Self) {
                self.x $op_assign other.x;
                self.y $op_assign other.y;
                self.z $op_assign other.z;
            }
        }

        impl $trait_assign<f32> for Vec3 {
            fn $method_assign(&mut self, other:f32) {
                self.x $op_assign other;
                self.y $op_assign other;
                self.z $op_assign other;
            }
        }
        impl $trait for &Vec3 {
            type Output = Vec3;
            fn $method(self, other: &Vec3) -> Vec3 {
                Vec3 {
                    x: self.x $op other.x,
                    y: self.y $op other.y,
                    z: self.z $op other.z,
                }
            }
        }

        impl $trait<f32> for &Vec3 {
            type Output = Vec3;
            fn $method(self, other: f32) -> Vec3 {
                Vec3 {
                    x: self.x $op other,
                    y: self.y $op other,
                    z: self.z $op other,
                }
            }
        }

        impl $trait<&Vec3> for f32 {
            type Output = Vec3;
            fn $method(self, other: &Vec3) -> Vec3 {
                Vec3 {
                    x: self $op other.x,
                    y: self $op other.y,
                    z: self $op other.z,
                }
            }
        }

        impl $trait_assign<&Vec3> for Vec3 {
            fn $method_assign(&mut self, other: &Vec3) {
                self.x $op_assign other.x;
                self.y $op_assign other.y;
                self.z $op_assign other.z;
            }
        }
    }
}

impl_vec3_op!(Add,add,+,AddAssign,add_assign,+=);
impl_vec3_op!(Sub, sub, -, SubAssign, sub_assign, -=);
impl_vec3_op!(Mul, mul, *, MulAssign, mul_assign, *=);
impl_vec3_op!(Div, div, /, DivAssign, div_assign, /=);

impl Neg for Vec3 {
    type Output = Self;
    fn neg(self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }
}

impl Vec3 {
    pub fn mag(&self) -> f32 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }
    pub fn normalize(self) -> Self {
        self / self.mag()
    }
    pub fn dot(&self, &other: &Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }
    pub fn cross(&self, &other: &Self) -> Self {
        Self {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }

    pub fn rotate(&self, quaternion: Quaternion) -> Self {
        (quaternion * Quaternion::from_vec3(self) * quaternion.conj()).to_vec3()
    }
}
//1d array so i can enforce column major when sending to shader
// note that wgpu expects contiguous cells to represent a column, so each "row" i specify is actually a column from top-bottom
pub struct Mat4 {
    pub array: [f32; 16],
}

impl Mat4 {
    #[rustfmt::skip]
    pub fn zero() -> Self {
        Self {
            array: [
                0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0,
            ],
        }
    }
}

impl Default for Mat4 {
    #[rustfmt::skip]
    fn default()->Self {
        Self {
            array: [
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0,
            ],
        }
    }
}

impl Mul for Mat4 {
    type Output = Self;
    fn mul(self, other: Self) -> Self {
        let mut ans = Mat4::zero();
        for col in 0..4 {
            for row in 0..4 {
                for k in 0..4 {
                    ans.array[col * 4 + row] += self.array[k * 4 + row] * other.array[col * 4 + k];
                }
            }
        }
        ans
    }
}
pub struct Mat3 {
    pub array: [f32; 9],
}

impl Mat3 {
    #[rustfmt::skip]
    pub fn zero() -> Self {
        Self {
            array: [
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
            ],
        }
    }
}

impl Default for Mat3 {
    #[rustfmt::skip]
    fn default() -> Self {
        Self {
            array: [
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0
            ],
        }
    }
}

impl Mul for Mat3 {
    type Output = Self;
    fn mul(self, other: Self) -> Self {
        let mut ans = Self::zero();
        for col in 0..3 {
            for row in 0..3 {
                for k in 0..3 {
                    ans.array[col * 3 + row] += self.array[k * 3 + row] * other.array[col * 3 + k];
                }
            }
        }
        ans
    }
}
impl Mul<&Vec3> for &Mat3 {
    type Output = Vec3;

    fn mul(self, v: &Vec3) -> Vec3 {
        Vec3 {
            x: self.array[0] * v.x + self.array[3] * v.y + self.array[6] * v.z,
            y: self.array[1] * v.x + self.array[4] * v.y + self.array[7] * v.z,
            z: self.array[2] * v.x + self.array[5] * v.y + self.array[8] * v.z,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Quaternion {
    pub real: f32,
    pub x: f32, //i
    pub y: f32, //j
    pub z: f32, //k
}

impl Quaternion {
    pub fn zero() -> Self {
        Self {
            real: 0.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    pub fn from_angle(axis: &Vec3, angle: f32) -> Self {
        let axis = axis.normalize();
        let (sin, cos) = (angle / 2.0).sin_cos();
        Self {
            real: cos,
            x: sin * axis.x,
            y: sin * axis.y,
            z: sin * axis.z,
        }
    }

    pub fn from_vec3(vec3: &Vec3) -> Self {
        Self {
            real: 0.0,
            x: vec3.x,
            y: vec3.y,
            z: vec3.z,
        }
    }

    pub fn to_vec3(&self) -> Vec3 {
        Vec3 {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }

    //equal to inverse for unit quaternions
    pub fn conj(&self) -> Self {
        Self {
            real: self.real,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    pub fn mag(&self) -> f32 {
        (self.real * self.real + self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }
    pub fn normalize(&self) -> Self {
        let mag = self.mag();
        Self {
            real: self.real / mag,
            x: self.x / mag,
            y: self.y / mag,
            z: self.z / mag,
        }
    }
    #[rustfmt::skip]
    pub fn to_mat3(&self) -> Mat3 {
        //expand q*v*q^-1
        //remember each "row" here is a column on the gpu
        let (r,x,y,z)=(self.real,self.x,self.y,self.z);
        Mat3 {
            array: [
                r*r+x*x-y*y-z*z, 2.0*r*z+2.0*x*y, -2.0*r*y+2.0*x*z,
                -2.0*r*z+2.0*x*y, r*r+y*y-x*x-z*z, 2.0*r*x+2.0*y*z,
                2.0*r*y+2.0*x*z, -2.0*r*x+2.0*y*z, r*r+z*z-x*x-y*y
            ]
        }
    }
}
impl Default for Quaternion {
    fn default() -> Self {
        Self {
            real: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }
}
impl Mul for Quaternion {
    type Output = Self;
    fn mul(self, other: Self) -> Self {
        let (r1, r2) = (self.real, other.real);
        let (x1, x2) = (self.x, other.x);
        let (y1, y2) = (self.y, other.y);
        let (z1, z2) = (self.z, other.z);
        //ij=-ji=k (right hand rule)
        //jk=-kj=i
        //ki=-ik=j
        Self {
            real: r1 * r2 - x1 * x2 - y1 * y2 - z1 * z2,
            x: r1 * x2 + r2 * x1 + y1 * z2 - z1 * y2,
            y: r1 * y2 - x1 * z2 + y1 * r2 + z1 * x2,
            z: r1 * z2 + x1 * y2 - y1 * x2 + z1 * r2,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Plane {
    pub point: Vec3,
    pub normal: Vec3,
}
impl Plane {
    pub fn distance_to_point(&self, point: &Vec3) -> f32 {
        ((point - &self.point).dot(&self.normal)) / self.normal.mag()
    }
    pub fn intersect_with_line_segment(&self, point1: &Vec3, point2: &Vec3) -> Vec3 {
        let dir = point2 - point1;
        let numerator = self.normal.dot(&(&self.point - point1));
        let denominator = self.normal.dot(&dir);
        if denominator.abs() < EPSILON {
            panic!(
                "line is parallel to plane, {:?}, {:?}, {:?}",
                self, point1, point2
            );
        } else {
            let t = numerator / denominator;
            if t >= 0.0 && t <= 1.0 {
                point1 + &(t * dir)
            } else {
                panic!(
                    "line segment does not intersect with plane, {:?}, {:?}, {:?}",
                    self, point1, point2
                );
            }
        }
    }
}
