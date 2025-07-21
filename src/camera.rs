use std::time::Duration;

use crate::math::{Mat4, Vec3};
use winit::keyboard::KeyCode;
pub struct Camera {
    pub position: Vec3,
    pub yaw: f32,
    pub pitch: f32,
    pub aspect_ratio: f32,
}

impl Camera {
    //moves world so that camera is the origin
    #[rustfmt::skip]
    pub fn calc_view_matrix(&self) -> Mat4 {
        let forward = -Vec3 {
            x: self.yaw.cos() * self.pitch.cos(), //2d rotation matrix x-component * pitch projection onto XZ
            //plane
            y: self.pitch.sin(),
            z: self.yaw.sin() * self.pitch.cos(),
        };
        let global_up = Vec3 {
            x: 0.0,
            y: 1.0,
            z: 0.0,
        };
        let right = forward.cross(&global_up).normalize();
        let local_up = forward.cross(&right).normalize();
        Mat4 {
            array: [
                [right.x, right.y, right.z, -right.dot(&self.position)],
                [local_up.x, local_up.y, local_up.z, -local_up.dot(&self.position)],
                [forward.x, forward.y, forward.z, -forward.dot(&self.position)],
                [0.0, 0.0, 0.0, 1.0],
            ],
        }
    }

    //squash world for perspective
    pub fn calc_projection_matrix(&self) -> Mat4 {
        const Z_NEAR: f32 = 0.1;
        const Z_FAR: f32 = 100.0; //min and max distance from camera
        const FOV_Y: f32 = std::f32::consts::PI / 2.0;
        let f = 1.0 / ((FOV_Y / 2.0).tan());
        Mat4 {
            array: [
                [f / self.aspect_ratio, 0.0, 0.0, 0.0],     //x scaling
                [0.0, f, 0.0, 0.0],                         //y scaling
                [0.0, 0.0, Z_FAR / (Z_NEAR - Z_FAR), -1.0], //z scaling
                [0.0, 0.0, (Z_NEAR * Z_FAR) / (Z_NEAR - Z_FAR), 0.0],
            ],
        }
    }
}

pub struct CameraController {
    left: f32,
    right: f32,
    forward: f32,
    backward: f32,
    up: f32,
    down: f32,
    rotate_horizontal: f32,
    rotate_vertical: f32,
}

impl CameraController {
    pub fn new() -> Self {
        Self {
            left: 0.0,
            right: 0.0,
            forward: 0.0,
            backward: 0.0,
            up: 0.0,
            down: 0.0,
            rotate_horizontal: 0.0,
            rotate_vertical: 0.0,
        }
    }
    pub fn on_key(&mut self, key: KeyCode, pressed: bool) {
        let amount = if pressed { 1.0 } else { 0.0 };
        match key {
            KeyCode::KeyW | KeyCode::ArrowUp => self.forward = amount,
            KeyCode::KeyS | KeyCode::ArrowDown => self.backward = amount,
            KeyCode::KeyA | KeyCode::ArrowLeft => self.left = amount,
            KeyCode::KeyD | KeyCode::ArrowRight => self.right = amount,
            KeyCode::Space => self.up = amount,
            KeyCode::ShiftLeft => self.down = amount,
            _ => {}
        }
    }

    pub fn on_mouse(&mut self, dx: f64, dy: f64) {
        self.rotate_horizontal = dx as f32;
        self.rotate_vertical = dy as f32;
    }

    pub fn update(&mut self, camera: &mut Camera, dt: Duration) {
        const SPEED: f32 = 4.0;
        const SENS: f32 = 0.4;
        const SAFE_FRAC_PI_2: f32 = std::f32::consts::FRAC_PI_2 - 0.0001;
        let dt = dt.as_secs_f32();
        let forward = Vec3 {
            x: camera.yaw.cos(),
            y: 0.0,
            z: camera.yaw.sin(),
        };
        let right = Vec3 {
            x: -camera.yaw.sin(),
            y: 0.0,
            z: camera.yaw.cos(),
        };
        camera.position += forward * (self.forward - self.backward) * SPEED * dt;
        camera.position += right * (self.right - self.left) * SPEED * dt;
        camera.position.y += (self.up - self.down) * SPEED * dt;
        camera.yaw += self.rotate_horizontal * SENS * dt;
        camera.pitch += self.rotate_vertical * SENS * dt;
        camera.pitch = camera.pitch.clamp(-SAFE_FRAC_PI_2, SAFE_FRAC_PI_2);
    }
}

impl Default for CameraController {
    fn default() -> Self {
        Self::new()
    }
}
