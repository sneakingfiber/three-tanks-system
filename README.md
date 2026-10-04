# Cascaded Three-Tank Aeration System — Modeling & Control

MATLAB/Simulink project for the *Industrial Automation* course (University of Genoa, 2026).
It covers nonlinear modeling, PI control (continuous and digital), Euler discretization, linearization, and a comparison of nonlinear vs. linearized models of a simplified water-treatment process.

The full write-up is in [`Report_Gossetti_Khan.pdf`](Report_Gossetti_Khan.pdf); the assignment text is in [`assignment-part1.pdf`](assignment-part1.pdf).

## The system

Three tanks in cascade: liquid flows Tank 1 → Tank 2 → Tank 3. Tank 2 is aerated, Tank 3 has a discharge pump and a recycle pump that sends flow back to Tank 1.

| | Variables |
|---|---|
| **States** | `h1, h2, h3` (levels [m]), `c2, c3` (dissolved oxygen in Tank 2 and 3 [mg/L]) |
| **Inputs** | `ua` aeration, `ur` recycle flow, `ud` discharge flow |
| **Outputs** | `h2, h3, c3` |

Inter-tank flows follow nonlinear hydraulic laws, `q12 = k12·√(h1−h2)` and `q23 = k23·√(h2−h3)`, and oxygen dynamics include aeration, dilution and decay. Input constraints: `0 ≤ ua ≤ 1`, `0 ≤ ur ≤ 0.08`, `0.05 ≤ ud ≤ 0.25`.

## Repository structure

| Folder | Content |
|---|---|
| `Task1_tank_open_loop/` | Nonlinear continuous-time model, open-loop simulation (`ode45`) |
| `Task2_PID_cont/` | Closed loop with continuous-time PI regulators tracking time-varying references |
| `Task3_PID_disc/` | Digital PI with zero-order hold on a continuous plant |
| `Task4_Euler_disc/` | Forward-Euler discretization of the full plant + controller (MATLAB loop and Simulink) |
| `Task5_linearize_equilibrium/` | Equilibrium point and symbolic Jacobian linearization → state-space model |
| `Task6_7_confrontation/` | Nonlinear vs. linearized model comparison, disturbed input, effect of sampling time |

## Requirements

- MATLAB (tested with a recent release; the Symbolic Math Toolbox is needed for Task 5)
- Simulink

## How to run

1. Open MATLAB and `cd` into the task folder.
2. Run the task script:

   | Task | Script |
   |---|---|
   | 1 | `tank_model.m` |
   | 2 | `tank_model_PID_cont.m` |
   | 3 | `tank_model.m` |
   | 4 | `tank_model.m` |
   | 5 | `linearize_system.m` |
   | 6–7 | `tank_confrontation_init.m` (set `sample_time` to study discretization), then open `models_confrontation.slx` |

3. For the Simulink versions, open the `.slx` file (after running the script that loads parameters into the workspace) and press **Run**.

Each task folder is self-contained (it ships its own copy of `tank_ode.m`).

## Notes

- **Task 4 (Euler discretization):** first implemented as a custom MATLAB time-stepping loop (with AI assistance to structure the loop), then replicated in Simulink with Forward Euler integrator blocks to obtain the same discrete-time behavior.
- **Task 5 (Linearization):** derived by hand first to understand the structure of the model, then implemented with symbolic Jacobians evaluated at the equilibrium point, following the approach seen in the lectures.

## Authors

Federico Gossetti, Khan — University of Genoa.
