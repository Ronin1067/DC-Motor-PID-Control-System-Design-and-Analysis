# DC Motor PID Control System Design and Analysis

[![Python Version](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![Control](https://img.shields.io/badge/library-python--control-orange.svg)](https://python-control.readthedocs.io/)

A comprehensive implementation and analysis of a PID controller for DC motor speed regulation using Python. This project includes mathematical modeling, controller design, stability analysis, and publication-quality visualization.

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [System Performance](#system-performance)
- [Usage](#usage)
- [Mathematical Model](#mathematical-model)
- [Results](#results)
- [Contributing](#contributing)
- [License](#license)
- [Authors](#authors)

## 🎯 Overview

This project presents a complete design and analysis framework for a Proportional-Integral-Derivative (PID) controller applied to armature-controlled DC motor speed regulation. The implementation includes:

- First-principles mathematical modeling of DC motor dynamics
- Multiple PID tuning methodologies (Ziegler-Nichols, Cohen-Coon, Manual Optimization)
- Comprehensive time-domain and frequency-domain analysis
- Publication-quality visualization with 9 detailed analysis plots
- Complete Python implementation using the Control Systems Library

### Key Objectives

1. **Mathematical Modeling**: Derive transfer function from Kirchhoff's voltage law and Newton's second law
2. **Controller Design**: Implement and compare different PID tuning techniques
3. **Performance Analysis**: Evaluate time-domain response and frequency-domain stability
4. **Visualization**: Generate professional-grade plots for academic publication

## ✨ Features

- 🎛️ **Complete PID Implementation**: Full PID controller with proportional, integral, and derivative actions
- 📊 **Publication-Quality Plots**: 9 comprehensive analysis figures (PNG + PDF formats)
- 🔍 **Multiple Tuning Methods**: Ziegler-Nichols, Cohen-Coon, and manual optimization
- 📈 **Comprehensive Analysis**: Step response, Bode plots, Nyquist diagrams, root locus
- 🛡️ **Robust Design**: Infinite gain margin and 94.64° phase margin
- 🎨 **Professional Visualization**: Color-coded plots with IEEE-standard formatting
- 📝 **Complete Documentation**: Full LaTeX report with mathematical derivations
- 🔧 **Disturbance Rejection**: Tests and visualizations for load disturbance handling

## 🚀 System Performance

### Achieved Metrics

| Performance Metric | Open-Loop | Closed-Loop (PID) | Improvement |
|-------------------|-----------|-------------------|-------------|
| **Rise Time** | 1.136 s | 0.132 s | **88.4%** ⬇️ |
| **Settling Time** | 2.067 s | 0.258 s | **87.5%** ⬇️ |
| **Overshoot** | 0.0% | 1.03% | Minimal ✅ |
| **Steady-State Error** | 90.01% | -0.03% | **~100%** ⬇️ |

### Stability Margins

- **Gain Margin**: ∞ dB (Exceptional stability)
- **Phase Margin**: 94.64° (Highly damped)
- **Gain Crossover Frequency**: 19.04 rad/s

### Closed-Loop Poles (Overdamped System)

- p₁ = -23.29 (Fast pole)
- p₂ = -5.69 (Intermediate pole)
- p₃ = -3.02 (Dominant pole)

All poles in left-half plane → **Absolutely stable**

## 💻 Usage

### Basic Simulation

```bash
python code.py
```

This will:
1. Define DC motor parameters
2. Create motor transfer function
3. Design PID controller
4. Perform time-domain and frequency-domain analysis
5. Generate 9 publication-quality figures in `pid_figures/` directory

### Output

The simulation produces:

```
pid_figures/
├── fig1_step_response_comparison.png (300 DPI)
├── fig1_step_response_comparison.pdf (vector)
├── fig2_closed_loop_detailed.png
├── fig2_closed_loop_detailed.pdf
├── ... (all 9 figures in both formats)
└── fig9_comprehensive_analysis.pdf
```

### Console Output

```
======================================================================
DC MOTOR PID CONTROL SIMULATION
======================================================================

Motor Transfer Function:
             0.01
  ---------------------------
  0.005 s^2 + 0.06 s + 0.1001

...

Performance Improvements:
  Rise Time: 88.4% reduction
  Settling Time: 87.5% reduction
  SS Error: 100.0% reduction
```

## 📐 Mathematical Model

### DC Motor Transfer Function

The DC motor is modeled as a second-order system:

```
G(s) = K / [L_a*J*s² + (L_a*B + R_a*J)*s + (R_a*B + K²)]
```

Where:
- **J** = 0.01 kg·m² (Moment of inertia)
- **B** = 0.1 N·m·s/rad (Viscous friction)
- **K** = 0.01 N·m/A (Motor constant)
- **R** = 1.0 Ω (Armature resistance)
- **L** = 0.5 H (Armature inductance)

### PID Controller

```
C(s) = Kp + Ki/s + Kd*s = (Kd*s² + Kp*s + Ki) / s
```

Optimized parameters:
- **Kp** = 100 (Proportional gain)
- **Ki** = 200 (Integral gain)
- **Kd** = 10 (Derivative gain)

### Closed-Loop Transfer Function

```
T(s) = C(s)*G(s) / [1 + C(s)*G(s)]
```

## 📊 Results

### Time-Domain Analysis

The PID controller achieves:
- **Fast response**: 0.132s rise time
- **Quick settling**: 0.258s settling time
- **Minimal overshoot**: 1.03%
- **Zero steady-state error**: -0.03% (essentially perfect)

### Frequency-Domain Analysis

**Bode Plot Analysis:**
- Infinite gain margin indicates exceptional stability
- 94.64° phase margin ensures well-damped response
- Gain crossover at 19.04 rad/s provides good bandwidth

**Nyquist Plot:**
- No encirclements of critical point (-1, 0)
- Large separation from instability region
- Confirms robust stability

**Root Locus:**
- All poles in left-half plane
- Overdamped response (three real poles)
- Dominant pole at s = -3.02 determines settling behavior

### Disturbance Rejection

The controller recovers from a 20% load disturbance in approximately **0.4 seconds**, demonstrating excellent disturbance rejection capabilities.

## 📚 Documentation

### Full Report

A comprehensive LaTeX report is included in the `docs/` directory, containing:

- Complete mathematical derivations
- Detailed controller design methodology
- Extensive performance analysis
- All simulation results and plots
- Practical implementation considerations
- Future research directions

**Compile the report:**

```bash
cd docs
xelatex improved_pid_report.tex
bibtex improved_pid_report
xelatex improved_pid_report.tex
xelatex improved_pid_report.tex
```

### Code Documentation

The Python code includes:
- Detailed inline comments
- Function docstrings
- Parameter explanations
- Clear variable naming

## 🎨 Visualization Examples

The project generates 9 comprehensive figures:

1. **Step Response Comparison** - Open-loop vs closed-loop
2. **Closed-Loop Detailed** - With settling bands
3. **Tracking Error** - Error evolution over time
4. **Bode Plot** - Magnitude and phase diagrams
5. **Nyquist Plot** - Stability analysis in complex plane
6. **Root Locus** - Pole movement with gain variation
7. **Disturbance Rejection** - Response to load changes
8. **Performance Comparison** - Bar chart of metrics
9. **Comprehensive Analysis** - 9-subplot overview

All figures are generated in:
- **PNG format** (300 DPI) for presentations/web
- **PDF format** (vector) for publications/reports

## 🔬 Applications

This PID controller design is applicable to:

- **Industrial Robotics**: Joint velocity control
- **CNC Machines**: Feed drive control
- **Electric Vehicles**: Traction motor regulation
- **Aerospace Systems**: Actuator control
- **Medical Devices**: Surgical robot control
- **Consumer Electronics**: Hard drive spindle control

## 🛠️ Advanced Usage

### Custom Parameters

Modify motor parameters in the script:

```python
# DC Motor Parameters
J = 0.01   # Moment of inertia (kg.m^2)
B = 0.1    # Viscous friction (N.m.s)
K = 0.01   # Motor constant (N.m/A)
R = 1.0    # Armature resistance (Ohm)
L = 0.5    # Armature inductance (H)
```

### Custom PID Tuning

Experiment with different PID gains:

```python
# PID Parameters
Kp = 100   # Proportional gain
Ki = 200   # Integral gain
Kd = 10    # Derivative gain
```

### Output Customization

Change figure parameters:

```python
# Set publication-quality plot parameters
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.size'] = 10
plt.rcParams['figure.dpi'] = 300  # Change resolution
```


## 🐛 Known Issues

- Nyquist plot may show warning about fixed axis limits (cosmetic only)
- Root locus API varies between python-control versions

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 👥 Authors

**Yagnesh Kumar Koduru**
- Student ID: S20230020313
- Department: ECE
- Institution: IIIT Sri City
- Email: [yagneshkumar.k23@iiits.in]

## 🙏 Acknowledgments

- **Python Control Systems Library** for excellent control system tools
- **Matplotlib** for publication-quality plotting capabilities
- **NumPy/SciPy** for numerical computing foundation

## 📖 References

1. N. S. Nise, *Control Systems Engineering*, 8th ed. Wiley, 2019.
2. K. Ogata, *Modern Control Engineering*, 5th ed. Prentice Hall, 2010.
3. G. F. Franklin et al., *Feedback Control of Dynamic Systems*, 8th ed. Pearson, 2019.
4. Python Control Systems Library Documentation: https://python-control.readthedocs.io/

## 🔗 Related Projects

- [Arduino PID Library](https://github.com/br3ttb/Arduino-PID-Library)
- [PID Simulator (MATLAB)](https://www.mathworks.com/products/simulink.html)
- [Control Systems Toolbox](https://python-control.readthedocs.io/)

## 📈 Project Status

**Status**: ✅ Complete and Stable

- [x] Mathematical modeling
- [x] PID controller implementation
- [x] Time-domain analysis
- [x] Frequency-domain analysis
- [x] Publication-quality visualization
- [x] Complete documentation


## Support

For questions or issues:
- Open an issue on GitHub
- Email: [yagneshkumar.k23@iiits.in]

---

### ⭐ If you find this project helpful, please consider giving it a star!

---

**Last Updated**: February 2026  
**Version**: 1.0.0
