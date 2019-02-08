use ncollide2d::shape::{ConvexPolygon, ShapeHandle};

//use nalgebra as na;
use nalgebra::base::{Scalar, Vector2};
use nphysics2d::algebra::ForceType;
use nphysics2d::force_generator::ForceGenerator;
use nphysics2d::material::{BasicMaterial, MaterialHandle};
use nphysics2d::math::{Force, Isometry, Point, Vector, Velocity};
use nphysics2d::object::{BodyHandle, BodySet, ColliderDesc, RigidBodyDesc};
use nphysics2d::solver::IntegrationParameters;
use nphysics2d::volumetric::Volumetric;
use nphysics2d::world::World;
use std::{thread, time};

fn main() {
    let mut world = World::new();
    //world.set_gravity(Vector::y() * 9.81);

    let polygon = ShapeHandle::new(
        ConvexPolygon::try_new(vec![
            Point::new(1.0, 2.0),
            Point::new(2.0, 2.0),
            Point::new(2.0, 3.0),
            Point::new(1.0, 3.0),
        ])
        .unwrap(),
    );
    let local_inertia = polygon.inertia(0.1);
    let local_center_of_mass = polygon.center_of_mass();
    let rigid_body_handle = RigidBodyDesc::new()
        .position(Isometry::new(Vector::new(2.0, 10.0), 3.0))
        .local_inertia(local_inertia)
        .local_center_of_mass(local_center_of_mass)
        .build(&mut world)
        .part_handle();

    // Create a collider attached to a previously-added rigid-body with handle `rigid_body_handle`.
    let material = MaterialHandle::new(BasicMaterial::default()); // Custom material.

    let collider_handle = ColliderDesc::new(polygon.clone())
        .margin(0.04)
        .position(Isometry::identity())
        .material(material.clone())
        .build_with_parent(rigid_body_handle, &mut world)
        .unwrap()
        .handle();

    let rigid_body_handle_two = RigidBodyDesc::new()
        .position(Isometry::new(Vector::new(20.0, 25.0), 3.0))
        .local_inertia(local_inertia)
        .local_center_of_mass(local_center_of_mass)
        .build(&mut world)
        .part_handle();

    ColliderDesc::new(polygon)
        .margin(0.04)
        .position(Isometry::identity())
        .material(material)
        .build_with_parent(rigid_body_handle_two, &mut world)
        .unwrap()
        .handle();

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
            //println!("{}", position * vertex);
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
            //println!("[{}, {}]", rotated_x, rotated_y);
        }

        for proximity in world.proximity_events() {
            //println!("{:?}", proximity);
        }

        let body_handle = world.collider_body_handle(collider_handle).unwrap();
        let body = world.body_mut(body_handle).unwrap();
        let force = Force::from_slice(&[5.00, 2.00, 0.5]);

        body.apply_force(0, &force, ForceType::Force, true);

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
