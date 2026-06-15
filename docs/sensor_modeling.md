# Sensor Modeling

All models begin with ideal simulated measurements and add effects in a defined
order. The first implementation includes Gaussian noise, constant or
random-walk bias, encoder quantization, range clipping, and seeded randomness.

Latency fields are present in configuration but are not yet implemented as
queues. The ROS and Isaac adapters will own timestamps and delayed publication.

## Initial assumptions

- LiDAR noise is independent per Cartesian coordinate.
- IMU acceleration and angular velocity biases evolve as random walks.
- Encoders are quantized to their configured resolution, with optional noise.
- Distance sensors return clipped scalar ranges and preserve invalid values.
- Homing switches and module state sensors are discrete and need no analog
  noise model in the first scaffold.

These are engineering placeholders, not calibrated sensor specifications.
