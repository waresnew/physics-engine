use crate::math::Vec3;
pub struct Camera {
    pub position: Vec3,
    pub yaw: f32,
    pub pitch: f32,
}

impl Camera {
    pub fn new(position: Vec3, yaw: f32, pitch: f32) {
        Self {
            position,
            yaw,
            pitch,
        }
    }

    pub fn calc_matrix(&self) {}
}
