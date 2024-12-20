# Bar and Hinge Miura-Ori Model (1 Cell)

This repository contains MATLAB scripts to simulate a **Bar and Hinge model** and its **State-Space Representation** for the **Miura-Ori Origami** structure, focusing on a single cell. The scripts compute forces, kinematics, dynamics, and state-space behavior to analyze the folding process.

---

## Files and Descriptions

### Main Scripts
- **BarandHinge_Miura.m**  
  The **main script** for simulating the Bar and Hinge Miura-Ori model. It integrates subfunctions to compute dynamics and folding behavior.

- **StateSpaceModel.m**  
  The **main script** for the linearized state-space representation of the Miura-Ori Bar and Hinge model:  
  \[
  \dot{x} = Ax + Bu + C
  \]

---

### Functions
- **ang2coordinate.m**  
  Converts angular parameters into Cartesian coordinates for the Miura-Ori vertices.

- **bah_EOM_fun.m**  
  Defines the equations of motion (EOM) for the Bar and Hinge model.

- **barlength.m**  
  Computes the lengths of bars (edges) in the Miura-Ori structure.

- **C_build.m**  
  Builds the affine term \( C \) for the linearized state-space model.

- **compute_force_matrix.m**  
  Calculates the force matrix based on node positions and applied forces.

- **crease_actuation_matrix.m**  
  Defines the input matrix \( B \) for actuation (folding) along crease lines, supporting state-space representation.

- **generate_miuraori_structure.m**  
  Generates the **nodal connectivity** and structural matrices for the Miura-Ori fold.  
  Applicable to both the **Bar and Hinge model** and the **State-Space Representation**.

- **K_global_build.m**  
  Builds the global stiffness matrix \( K \) for the structure, supporting the state-space model.

---

## How to Use

1. Open MATLAB and ensure all files are in the current working directory.
2. Choose the appropriate script to run:
   - For the Bar and Hinge model simulation:
     ```matlab
     BarandHinge_Miura
     ```
   - For the State-Space model simulation:
     ```matlab
     StateSpaceModel
     ```

---
