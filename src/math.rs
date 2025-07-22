use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};
#[derive(Copy, Clone, Debug)]
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
}
pub struct Mat4 {
    pub array: [[f32; 4]; 4],
}

impl Mat4 {
    pub fn new() -> Self {
        Self {
            array: [
                [0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0],
            ],
        }
    }
}

impl Default for Mat4 {
    fn default() -> Self {
        Self::new()
    }
}

impl Mul for Mat4 {
    type Output = Self;
    fn mul(self, other: Self) -> Self {
        let mut ans = Mat4::new();
        for i in 0..4 {
            for j in 0..4 {
                for k in 0..4 {
                    ans.array[i][j] += self.array[i][k] * other.array[k][j];
                }
            }
        }
        ans
    }
}
