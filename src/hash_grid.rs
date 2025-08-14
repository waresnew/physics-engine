use crate::{math::Vec3, world::Cuboid};

const BUCKET_CAPACITY: usize = 8;
#[derive(Debug)]
pub struct HashGrid {
    //aka spatial hashing
    pub spacing: Vec3,
    pub buckets: Vec<Vec<usize>>,
    n: usize,
}

impl HashGrid {
    pub fn new(instances: &[Cuboid], n: usize) -> Self {
        let mut buckets = Vec::with_capacity(n);
        for _ in 0..n {
            buckets.push(Vec::with_capacity(BUCKET_CAPACITY));
        }
        let mut spacing = Vec3::default();
        for i in 0..n {
            spacing += instances[i].scale;
        }
        spacing /= n as f32;
        spacing *= 4.0;
        Self {
            spacing,
            buckets,
            n,
        }
    }
    pub fn init(&mut self, instances: &[Cuboid]) {
        let mut largest: usize = 0;
        for i in 0..self.n {
            let instance = &instances[i];
            Self::for_each_cell(instance, self.spacing, self.n, |index| {
                self.buckets[index].push(instance.index);
            });
            largest = largest.max(self.buckets[i].len());
        }
        if largest > self.n / 10 {
            eprintln!("buckets are getting big, too many hash collisions?");
        }
    }
    pub fn for_each_cell<F>(instance: &Cuboid, spacing: Vec3, n: usize, mut f: F)
    where
        F: FnMut(usize),
    {
        let aabb = instance.aabb;
        let Vec3 {
            x: min_x_float,
            y: min_y_float,
            z: min_z_float,
        } = aabb.min / spacing;
        let Vec3 {
            x: max_x_float,
            y: max_y_float,
            z: max_z_float,
        } = aabb.max / spacing;
        let min_x = min_x_float.floor() as i32;
        let min_y = min_y_float.floor() as i32;
        let min_z = min_z_float.floor() as i32;
        let max_x = max_x_float.ceil() as i32;
        let max_y = max_y_float.ceil() as i32;
        let max_z = max_z_float.ceil() as i32;
        for x in min_x..max_x {
            for y in min_y..max_y {
                for z in min_z..max_z {
                    let index = Self::hash((x, y, z), n);
                    f(index);
                }
            }
        }
    }
    fn hash(cell: (i32, i32, i32), n: usize) -> usize {
        (((cell.0 * 73856093) ^ (cell.1 * 19349663) ^ (cell.2 * 83492791)) //https://matthias-research.github.io/pages/publications/tetraederCollision.pdf
            as usize)
            % n
    }
    pub fn clear(&mut self) {
        for bucket in &mut self.buckets {
            bucket.clear();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_cell_iterator() {
        let n = 2;
        let mut instances = vec![
            Cuboid {
                index: 0,
                position: Vec3 {
                    //in 1 cell
                    x: 1.0,
                    y: 1.0,
                    z: 1.0,
                },
                ..Default::default()
            },
            Cuboid {
                position: Vec3 {
                    x: 0.75,
                    y: 1.0,
                    z: 1.0,
                },
                index: 1,
                ..Default::default()
            },
        ];
        for x in &mut instances {
            x.update_derived();
        }
        let mut grid = HashGrid::new(&instances, n);
        grid.init(&instances);
        let mut len = 0;
        HashGrid::for_each_cell(&instances[0], grid.spacing, grid.n, |cell_index| {
            let bucket = &grid.buckets[cell_index];
            dbg!(bucket);
            assert_eq!(bucket.len(), 2);
            len += 1;
        });
        assert_eq!(len, 1);
    }
}
