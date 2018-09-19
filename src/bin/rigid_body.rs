use ncollide2d::shape::{ConvexPolygon, ShapeHandle};

//use nalgebra as na;
use nalgebra::base::{Scalar, Vector2};
use nphysics2d::math::{Isometry, Point, Vector};
use nphysics2d::object::Material;
use nphysics2d::volumetric::Volumetric;
use nphysics2d::world::World;

use std::{thread, time};

fn main() {
    let mut world = World::new();
    world.set_gravity(Vector::y() * 9.81);

    let polygon = ShapeHandle::new(
        ConvexPolygon::try_new(vec![
            Point::new(1.0, 2.0),
            Point::new(2.0, 2.0),
            Point::new(2.0, 3.0),
            Point::new(1.0, 3.0),
        ]).unwrap(),
    );
    let local_inertia = polygon.inertia(0.1);
    let local_center_of_mass = polygon.center_of_mass();
    let rigid_body_handle = world.add_rigid_body(
        Isometry::new(Vector::new(2.0, 10.0), 3.0),
        local_inertia,
        local_center_of_mass,
    );

    // Create a collider attached to a previously-added rigid-body with handle `rigid_body_handle`.
    let material = Material::default(); // Custom material.
    let collider_handle = world.add_collider(
        0.04,
        polygon,
        rigid_body_handle,
        Isometry::identity(),
        material,
    );

    loop {
        world.step();
        let collider = world
            .collider(collider_handle)
            .expect("Collider handle was invalid");
        println!("{}", collider.position());
        let shape: &ConvexPolygon<_> = collider
            .shape()
            .as_shape()
            .expect("Failed to cast shapehandle to a ConvexPolygon");
        for vertex in shape.points() {
            let position = collider.position();
            println!("{}", position * vertex);
            // Own implementation:
            let (center_x, center_y) = elements(&position.translation.vector);
            let rotation: f64 = position.rotation.angle();
            let (orig_x, orig_y) = (vertex.x + center_x, vertex.y + center_y);
            let rotated_x = rotation.cos() * (orig_x - center_x)
                - rotation.sin() * (orig_y - center_y)
                + center_x;
            let rotated_y = rotation.sin() * (orig_x - center_x)
                + rotation.cos() * (orig_y - center_y)
                + center_y;
            println!("[{}, {}]", rotated_x, rotated_y);
        }
        thread::sleep(time::Duration::from_millis(1000 / 60));
    }
}

fn elements<N>(vector: &Vector2<N>) -> (N, N)
where
    N: Scalar,
{
    let mut iter = vector.iter();

    (*iter.next().unwrap(), *iter.next().unwrap())
}
