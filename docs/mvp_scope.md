# MVP Scope

## Included

- Simplified facade, modular grid, and mobile robot geometry
- Facade-grid transform calibration interfaces
- Differential-drive odometry placeholder
- Grid stability score placeholder
- Passive sensor measurement and noise models
- Reproducible scenarios and noise using seeded random generators
- ROS 2 node and launch scaffolding

## Excluded

- Drilling, fastening, cutting, and material-removal physics
- Force/torque and contact sensing
- Safety PLCs, emergency circuits, and certified control functions
- Embedded computer and real-time firmware design
- Motor drives, batteries, cabling, and power electronics
- Industrial deployment, installation, and regulatory details

The scaffold must make excluded areas possible to add later without implying
that they are currently modeled or validated.
