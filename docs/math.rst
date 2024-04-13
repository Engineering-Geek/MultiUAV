Quadcopter Motor Control - Governing Equations
==============================================

Definitions and Variables
-------------------------
Thrusts, torques, forces, and transformations are defined as follows:

- **Thrusts**:

  .. math::
      \overrightarrow{T} = \begin{bmatrix}
          T_1 \\
          T_2 \\
          T_3 \\
          T_4 \\
      \end{bmatrix}

- **Local Net Torques**:

  .. math::
      \overrightarrow{\tau_{l}} = \begin{bmatrix}
          \tau_{x, l} \\
          \tau_{y, l} \\
          \tau_{z, l} \\
      \end{bmatrix}

- **Local Net Forces**:

  .. math::
      \overrightarrow{F_{l}} = \begin{bmatrix}
          F_{x, l} \\
          F_{y, l} \\
          F_{z, l} \\
      \end{bmatrix}

- **Global Desired Accelerations**:

  .. math::
      \overrightarrow{\alpha_{g}}, \overrightarrow{a_{g}}

- **Inertia Matrix (I)** and **Mass (m)**:

  .. math::
      I, m

- **Rotation Matrix (R_{g\to l})**:

  .. math::
      R_{g\to l}

Equation Derivations
--------------------

1. **Allocation Matrix Equation**:

   .. math::
       A \cdot \begin{bmatrix}
           T_1 \\
           T_2 \\
           T_3 \\
           T_4 \\
       \end{bmatrix} = \begin{bmatrix}
           F_{x, l} \\
           F_{y, l} \\
           F_{z, l} \\
           \tau_{x, l} \\
           \tau_{y, l} \\
           \tau_{z, l} \\
       \end{bmatrix}

   Where \( A \) is:

   .. math::
       \begin{bmatrix}
           0 & 0 & 0 & 0 \\
           0 & 0 & 0 & 0 \\
           1 & 1 & 1 & 1 \\
           y_{1} & y_{2} & y_{3} & y_{4} \\
           x_{1} & x_{2} & x_{3} & x_{4} \\
           f_{1} & f_{2} & f_{3} & f_{4} \\
       \end{bmatrix}

2. **Conversion of Global to Local Accelerations**:

   .. math::
       \begin{bmatrix}
           \overrightarrow{\alpha_{l}} \\
           \overrightarrow{a_{l}} \\
       \end{bmatrix} = R_{g\to l} \cdot \begin{bmatrix}
           \overrightarrow{\alpha_{g}} \\
           \overrightarrow{a_{g}} \\
       \end{bmatrix}

3. **Force and Torque Back to Global Frame**:

   .. math::
       \begin{bmatrix}
           \overrightarrow{F_{g}} \\
           \overrightarrow{\tau_{g}} \\
       \end{bmatrix} = \begin{bmatrix}
           R_{g\to l} \cdot m \cdot \overrightarrow{a_{g}} \\
           R_{g\to l} \cdot I \cdot \overrightarrow{\alpha_{g}} \\
       \end{bmatrix}

4. **Thrust Calculation**:

   .. math::
       \overrightarrow{T} = A^{-1} \cdot \begin{bmatrix}
           R_{g\to l} \cdot m \cdot \overrightarrow{a_{g}} \\
           R_{g\to l} \cdot I \cdot \overrightarrow{\alpha_{g}} \\
       \end{bmatrix}

Note that \( A^{-1} \) refers to the pseudo-inverse of the allocation matrix due to its non-square shape.
