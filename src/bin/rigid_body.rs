use ncollide2d::shape::{ConvexPolygon, ShapeHandle};

//use nalgebra as na;
use nalgebra::base::{Scalar, Vector2};
use nphysics2d::force_generator::ForceGenerator;
use nphysics2d::math::{Force, Isometry, Point, Vector, Velocity};
use nphysics2d::object::{BodyHandle, BodySet, Material};
use nphysics2d::solver::IntegrationParameters;
use nphysics2d::volumetric::Volumetric;
use nphysics2d::world::World;
use std::{thread, time};

#[derive(Default)]
pub struct RadialForce {
    parts: Vec<BodyHandle>, // Body parts affected by the force generator.
    is_done: bool,
}

impl RadialForce {
    /// Add a body part to be affected by this force generator.
    pub fn add_body_part(&mut self, body: BodyHandle) {
        self.parts.push(body)
    }
}

impl ForceGenerator<f64> for RadialForce {
    fn apply(&mut self, _: &IntegrationParameters<f64>, bodies: &mut BodySet<f64>) -> bool {
        if self.is_done {
            return false;
        }
        self.is_done = true;
        for &handle in &self.parts {
            // Generate the force only if the body has not been removed from the world.
            if bodies.contains(handle) {
                let mut body = bodies.body_part_mut(handle);
                let force = Force::from_slice(&[5.00, 2.00, 0.5]);
                body.apply_force(&force);
            }
        }

        // If `false` is returned, the physis world will remove
        // this force generator after this call.
        true
    }
}

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
    let rigid_body_handle = world.add_rigid_body(
        Isometry::new(Vector::new(2.0, 10.0), 3.0),
        local_inertia,
        local_center_of_mass,
    );

    // Create a collider attached to a previously-added rigid-body with handle `rigid_body_handle`.
    let material = Material::default(); // Custom material.
    let collider_handle = world.add_collider(
        0.04,
        polygon.clone(),
        rigid_body_handle,
        Isometry::identity(),
        material.clone(),
    );

    let rigid_body_handle_two = world.add_rigid_body(
        Isometry::new(Vector::new(20.0, 25.0), 3.0),
        local_inertia,
        local_center_of_mass,
    );

    let collider_handle_two = world.add_collider(
        0.04,
        polygon,
        rigid_body_handle_two,
        Isometry::identity(),
        material,
    );

    let sensor_shape = ShapeHandle::new(
        ConvexPolygon::try_new(vec![
            Point::new(-50.0, -50.0),
            Point::new(50.0, -50.0),
            Point::new(50.0, 50.0),
            Point::new(-50.0, 50.0),
        ])
        .unwrap(),
    );

    let sensor_position = Isometry::new(Vector::new(0.0, 0.0), 0.0);
    let sensor_handle = world.add_sensor(
        sensor_shape,      // The geometric shape of the sensor.
        rigid_body_handle, // The handle of the body part this sensor is attached to.
        sensor_position,   // The relative position of this sensor wrt. its parent.
    );

    let mut force_generator = RadialForce::default();
    force_generator.add_body_part(rigid_body_handle);
    world.add_force_generator(force_generator);

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
