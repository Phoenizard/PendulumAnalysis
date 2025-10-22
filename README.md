# Pendulum Ride Physical Simulation

## Model

### Release Model

The pendulum is released from a given hight with $\theta_0$. We obtain ODE:

$$
\frac{d\theta}{dt} = [\frac{2g}{L}(\cos{\theta} - \cos{\theta_0})]^{\frac{1}{2}} \\

\theta(0) = \theta_0
$$

The code can be found in component/simple-release

## Appendix

### Symbol

| Symbol | Meaning | Unit | Type |
|--------|---------|------|------|
| θ | Angular position | rad or ° | State variable |
| ω | Angular velocity | rad/s | State variable |
| L | Pendulum arm length | m | Parameter |
| M | System mass | kg | Parameter |
| g | Gravitational acceleration | m/s² | Constant |
| v | Linear velocity magnitude | m/s | Derived |
| a_x | Local x-axis acceleration | m/s² | Output |
| a_z | Local z-axis acceleration | m/s² | Output |
| G_x | x-axis g-force | g | Output |
| G_z | z-axis g-force | g | Output |




### Version History

| Version | Date | Changes | Author |
|---------|------|---------|--------|
| 1.0 | 2025-10-22 | Initial specification | Phoenizard |