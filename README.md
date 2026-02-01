# High Speed Railcar Suspension Dynamics Project (In Progress)

## Overview:
With the emergence of high-speed rail projects on the US West Coast, this project develops a reduced-order framework for analyzing railcar suspension dynamics and ride comfort. Railcars tend to experience more vibrational disturbances from their track and environment the faster they move, hence the importance of defining smarter suspension systems. 

## Project Goals
* Simulate High Speed Railcar Body-Bogie suspension dynamics using reduced-order mass-spring-damper systems in Python.
* Apply semi-active control law (skyhook) to maximize frequency weighted ride comfort metrics
* Produce CAD models of the suspension, bogie frames, and simplified railcar body in Fusion 360 to validate dimensions and mass assumptions

## Performance Metrics
* Frequency-weighted RMS carbody vertical acceleration (ISO 2631-1)
* Unweighted RMS acceleration (for comparison)
* Suspension travel limits

## Tools & Methods
* Python (NumPy, SciPy, MatPlotLib)
* State-space dynamic modeling
* Semi-active suspension control (skyhook)
* CAD modeling (Fusion 360)

## Current Status
✅ Literature Review and Scope Proposal<br>
✅ Working Dynamics Model in Python (Primary and Secondary Suspension)<br>
✅ Rail Track Step and Noise Disturbance Model + Performance Test<br>
✅ Semi-Active Control Integrated on Secondary Suspension System<br>
🕔 Create Visual Railcar Dynamics Simulation
🕔 CAD Model of the Simplified Railcar Body, Bogie, and suspension

## Scope and Limitations
* To simplify calculations, the model will begin in two dimensions, resulting infive degrees of freedom. If time allows, lateral and yaw disturbance will be added to both the bogie and body. The following     five degrees will be modeled:
    - Bogie: Bounce, Roll
    - Body: Bounce, Pitch, Roll

* Wheel to rail contact will not be explicitly modeled
* The track will be treated simply as an input for disturbance, not as a separate dynamical system.
* Railcar body assumed to be rigid. Structural flexibility of materials will be ignored.

## Details
See Project Scope Report under "docs" for a more detailed report, or [click here](docs/project_scope.pdf)

## License
Licensed under the MIT License. See LICENSE for details.
