pub mod camera;
pub mod math;
pub mod physics;
pub mod window;
pub mod world;

#[repr(C)]
#[derive(Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Vertex {
    position: [f32; 3],
    colour: [f32; 3],
    normal: [f32; 3],
}

impl Vertex {
    const ATTRIBUTES: [wgpu::VertexAttribute; 3] =
        wgpu::vertex_attr_array![0=>Float32x3,1=>Float32x3,2=>Float32x3];
    pub fn desc() -> wgpu::VertexBufferLayout<'static> {
        use std::mem;
        wgpu::VertexBufferLayout {
            array_stride: mem::size_of::<Self>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRIBUTES,
        }
    }
}
pub const CUBE_VERTICES: &[Vertex] = &[
    // z axis is outward, x is right, y is up
    // front face
    Vertex {
        position: [0.5, -0.5, 0.5],
        colour: [0.8627, 0.0784, 0.2353],
        normal: [0.0, 0.0, 1.0],
    },
    Vertex {
        position: [0.5, 0.5, 0.5],
        colour: [0.8627, 0.0784, 0.2353],
        normal: [0.0, 0.0, 1.0],
    },
    Vertex {
        position: [-0.5, 0.5, 0.5],
        colour: [0.8627, 0.0784, 0.2353],
        normal: [0.0, 0.0, 1.0],
    },
    Vertex {
        position: [-0.5, -0.5, 0.5],
        colour: [0.8627, 0.0784, 0.2353],
        normal: [0.0, 0.0, 1.0],
    },
    // back face
    Vertex {
        position: [-0.5, -0.5, -0.5],
        colour: [0.8627, 0.0784, 0.2353],
        normal: [0.0, 0.0, -1.0],
    },
    Vertex {
        position: [-0.5, 0.5, -0.5],
        colour: [0.8627, 0.0784, 0.2353],
        normal: [0.0, 0.0, -1.0],
    },
    Vertex {
        position: [0.5, 0.5, -0.5],
        colour: [0.8627, 0.0784, 0.2353],
        normal: [0.0, 0.0, -1.0],
    },
    Vertex {
        position: [0.5, -0.5, -0.5],
        colour: [0.8627, 0.0784, 0.2353],
        normal: [0.0, 0.0, -1.0],
    },
    //left face
    Vertex {
        position: [-0.5, -0.5, 0.5],
        colour: [0.8627, 0.0784, 0.2353],
        normal: [-1.0, 0.0, 0.0],
    },
    Vertex {
        position: [-0.5, 0.5, 0.5],
        colour: [0.8627, 0.0784, 0.2353],
        normal: [-1.0, 0.0, 0.0],
    },
    Vertex {
        position: [-0.5, 0.5, -0.5],
        colour: [0.8627, 0.0784, 0.2353],
        normal: [-1.0, 0.0, 0.0],
    },
    Vertex {
        position: [-0.5, -0.5, -0.5],
        colour: [0.8627, 0.0784, 0.2353],
        normal: [-1.0, 0.0, 0.0],
    },
    //right face
    Vertex {
        position: [0.5, -0.5, -0.5],
        colour: [0.8627, 0.0784, 0.2353],
        normal: [1.0, 0.0, 0.0],
    },
    Vertex {
        position: [0.5, 0.5, -0.5],
        colour: [0.8627, 0.0784, 0.2353],
        normal: [1.0, 0.0, 0.0],
    },
    Vertex {
        position: [0.5, 0.5, 0.5],
        colour: [0.8627, 0.0784, 0.2353],
        normal: [1.0, 0.0, 0.0],
    },
    Vertex {
        position: [0.5, -0.5, 0.5],
        colour: [0.8627, 0.0784, 0.2353],
        normal: [1.0, 0.0, 0.0],
    },
    //top face
    Vertex {
        position: [0.5, 0.5, 0.5],
        colour: [0.8627, 0.0784, 0.2353],
        normal: [0.0, 1.0, 0.0],
    },
    Vertex {
        position: [0.5, 0.5, -0.5],
        colour: [0.8627, 0.0784, 0.2353],
        normal: [0.0, 1.0, 0.0],
    },
    Vertex {
        position: [-0.5, 0.5, -0.5],
        colour: [0.8627, 0.0784, 0.2353],
        normal: [0.0, 1.0, 0.0],
    },
    Vertex {
        position: [-0.5, 0.5, 0.5],
        colour: [0.8627, 0.0784, 0.2353],
        normal: [0.0, 1.0, 0.0],
    },
    //bottom face
    Vertex {
        position: [-0.5, -0.5, 0.5],
        colour: [0.8627, 0.0784, 0.2353],
        normal: [0.0, -1.0, 0.0],
    },
    Vertex {
        position: [-0.5, -0.5, -0.5],
        colour: [0.8627, 0.0784, 0.2353],
        normal: [0.0, -1.0, 0.0],
    },
    Vertex {
        position: [0.5, -0.5, -0.5],
        colour: [0.8627, 0.0784, 0.2353],
        normal: [0.0, -1.0, 0.0],
    },
    Vertex {
        position: [0.5, -0.5, 0.5],
        colour: [0.8627, 0.0784, 0.2353],
        normal: [0.0, -1.0, 0.0],
    },
];

#[rustfmt::skip]
pub const CUBE_INDICES: &[u16] = &[
    // each triangle's vertices are defined CCW order if i were to look at it head on
    // front face
    0,1,2,
    2,3,0,
    //back face
    4,5,6,
    6,7,4,
    //left face
    8,9,10,
    10,11,8,
    //right face
    12,13,14,
    14,15,12,
    //top face
    16,17,18,
    18,19,16,
    //bottom face
    20,21,22,
    22,23,20
];

pub const FLOOR_VERTICES: &[Vertex] = &[
    //triangle 1
    Vertex {
        position: [100.0, 0.0, 100.0],
        colour: [0.38, 0.38, 0.38],
        normal: [0.0, 1.0, 0.0],
    },
    Vertex {
        position: [100.0, 0.0, -100.0],
        colour: [0.38, 0.38, 0.38],
        normal: [0.0, 1.0, 0.0],
    },
    Vertex {
        position: [-100.0, 0.0, -100.0],
        colour: [0.38, 0.38, 0.38],
        normal: [0.0, 1.0, 0.0],
    },
    //triangle 2
    Vertex {
        position: [-100.0, 0.0, -100.0],
        colour: [0.38, 0.38, 0.38],
        normal: [0.0, 1.0, 0.0],
    },
    Vertex {
        position: [-100.0, 0.0, 100.0],
        colour: [0.38, 0.38, 0.38],
        normal: [0.0, 1.0, 0.0],
    },
    Vertex {
        position: [100.0, 0.0, 100.0],
        colour: [0.38, 0.38, 0.38],
        normal: [0.0, 1.0, 0.0],
    },
];
