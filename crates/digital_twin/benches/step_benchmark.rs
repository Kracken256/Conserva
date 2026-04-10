use criterion::{Criterion, black_box, criterion_group, criterion_main};
use digital_twin::prelude::*;
use digital_twin_glue::prelude::*;
use nalgebra::Vector3;
use uom::si::f64::{Length, Time};
use uom::si::length::meter;
use uom::si::time::second;

fn bench_step(c: &mut Criterion) {
    let config = get_rocket_design();
    let state = get_rocket_initial_state(&config);
    let mesh_generator = RocketMesh::default();

    let target_vec = Vector3::new(1000.0, 1000.0, 1000.0);
    let waypoint = Some(target_vec.map(|c| Length::new::<meter>(c)));

    let rocket = TheRocket::new(config.clone(), waypoint);
    let mut twin = DigitalTwin::new(config, state, Box::new(mesh_generator), Box::new(rocket));

    // Warm up the twin
    twin.step(0.01);

    let mut group = c.benchmark_group("digital_twin");
    group.bench_function("step_0_01", |b| {
        b.iter(|| {
            twin.step(black_box(0.01));
            // Just mutate the time back manually to avoid state explosion during bench
            twin.state.time = Time::new::<second>(0.0);
        })
    });
    group.finish();
}

criterion_group!(benches, bench_step);
criterion_main!(benches);
