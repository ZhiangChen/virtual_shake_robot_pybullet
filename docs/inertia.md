# Tutorial: Configuring Inertia for the Virtual Shake Robot (VSR)

This tutorial will guide you through calculating the inertia for a rectangular box, configuring these values in a YAML file for the Virtual Shake Robot (VSR), and understanding how PyBullet reads these values in the `createMultiBody` function.

## 1. Calculating Inertia

### Inertia for a Rectangular Box

The moments of inertia for a rectangular box aligned with the coordinate axes can be calculated using the following formulas:

$$
I_{xx} = \frac{1}{12} m (h^2 + d^2)
$$

$$
I_{yy} = \frac{1}{12} m (w^2 + d^2)
$$

$$
I_{zz} = \frac{1}{12} m (w^2 + h^2)
$$

Where:
- \( m \) is the mass of the box.
- \( w \) is the width (x-dimension).
- \( h \) is the height (y-dimension).
- \( d \) is the depth (z-dimension).

### Example Calculation

Given a rectangular box with:
- Dimensions: `[10.0, 10.0, 0.1]` (width, height, depth)
- Mass: `10000.0`

The moments of inertia are calculated as follows:

#### For the x-axis (\( I_{xx} \)) and y-axis (\( I_{yy} \)):

$$
I_{xx} = I_{yy} = \frac{1}{12} \times 10000.0 \times (10.0^2 + 0.1^2) = 8334.167
$$

#### For the z-axis (\( I_{zz} \)):

$$
I_{zz} = \frac{1}{12} \times 10000.0 \times (10.0^2 + 10.0^2) = 166666.67
$$ 

Since it is a simple geometric box and symmetrically distributed around its center of mass, the other non-diagonal elements are zero.

## 2. Configuring Inertia in VSR

The calculated moments of inertia need to be configured in the YAML file used by the VSR. Here is how you can specify the configuration:

### YAML Configuration

Create or modify the YAML configuration file as follows:

```yaml
simulation_node:
  ros__parameters:
    structure:
      world_box:
        dimensions: [50.0, 14.0, 2.0]
        mass: 0.00
        inertia: [0.0, 0.0, 0.0]

      pedestal:
        dimensions: [10.0, 10.0, 0.1]
        mass: 10000.0
        inertia: [8334.167, 8334.167, 166666.67]
```

## 3. Setting the Correct Parameters in the `setJointMotorControl2`

## Calculation of Maximum Force and Maximum Velocity

### Objective
To determine the maximum force and velocity required for controlling the pedestal in the simulation based on given amplitude (A) and frequency (F) values.

### Parameters
- **Mass of the pedestal (m)**: 10000 kg
- **Amplitude (A)**: Variable
- **Frequency (F)**: Variable

### Formulae Used
1. **Acceleration Function**:
   $$
   a(t) = 4\pi^2 F^2 A \cos(2\pi F t)
   $$
   - \(a(t)\): Acceleration as a function of time.
   - \(A\): Amplitude.
   - \(F\): Frequency.
   - \(t\): Time.

2. **Newton's Second Law**:
   $$
   F = ma
   $$
   - \(F\): Force.
   - \(m\): Mass.
   - \(a\): Acceleration.

3. **Velocity Function**:
   $$
   v(t) = 2\pi AF \sin(2\pi F t)
   $$
   - \(v(t)\): Velocity as a function of time.

### Calculations

#### Maximum Acceleration and Force
The maximum acceleration occurs when 

$$ 
\\cos(2\pi F t) = 1
$$
$$
a_{max} = 4\pi^2 F^2 A
$$

For a given mass (\(m\)) of 10000 kg, the maximum force can be calculated using Newton's Second Law:

$$
F_{max} = m \times a_{max}
$$

#### Example Calculation for \((A = 1, F = 16)\):
1. Calculate maximum acceleration:
   $$
   a_{max} = 4\pi^2 \times 16^2 \times 1 = 10093.5104 \, \text{m/s}^2
   $$

2. Calculate maximum force:
   $$
   F_{max} = 10000 \times 10093.5104 = 100935104 \, \text{N}
   $$

#### Maximum Velocity
The maximum velocity occurs when \(\sin(2\pi F t) = 1\):

$$
v_{max} = 2\pi AF
$$

#### Example Calculation for \((A = 5, F = 5)\):
1. Calculate maximum velocity:
   $$
   v_{max} = 2\pi \times 5 \times 5 = 157.08 \, \text{m/s}
   $$

### Summary of Calculated Values

| Amplitude (A) | Frequency (F) | Maximum Acceleration (m/s²) | Maximum Force (N)  | Maximum Velocity (m/s) |
|---------------|----------------|------------------------------|---------------------|------------------------|
| 1             | 16             | 10093.5104                   | 100935104           | 100.53                 |
| 10            | 1              | 3959.20139                   | 39592013.9          | 62.83                  |
| 2             | 2              | 3159.36111                   | 31593611.1          | 25.13                  |
| 2             | 1              | 789.18028                    | 7891802.8           | 12.57                  |
| 5             | 5              | 19739.5064                   | 197395064           | 157.08                 |

### Decision on Maximum Force
The maximum force calculated for \((A = 1, F = 16)\) is \(100935104\) N, which is the highest among the test cases. Therefore, we decided to use this value to ensure the controller can handle the most extreme scenario.

### Implementation in Code
The calculated maximum force is used in the `setJointMotorControl2` function to ensure the joint motor can reach the desired positions and velocities:

```python
p.setJointMotorControl2(
    bodyUniqueId=robot_id,
    jointIndex=0,
    controlMode=p.POSITION_CONTROL,
    targetPosition=positions[i],
    targetVelocity=velocities[i],
    force= 5 * 10**8,  # Maximum force set based on calculations
    maxVelocity=200,  # Set maximum velocity
    physicsClientId=client_id
)
```

# Tutorial: Calculating Inertia for a Mesh File (.obj) in Autodesk Fusion 360

## Step 1: Import the Mesh File
1. **Open Fusion 360**: Launch Autodesk Fusion 360 on your computer.
2. **Create a New Project**: If you don’t already have a project, create a new one by clicking on the `New Project` button in the `Data Panel`.
3. **Import the Mesh**:
   - Go to the `Insert` menu.
   - Select `Insert Mesh`.
   - Browse to your `.obj` file and select it.
   - Click `Open` to import the mesh into your project.

## Step 2: Convert the Mesh to a Solid
1. **Prepare the Mesh**: 
   - Right-click on the mesh in the `Browser` pane.
   - Select `Mesh to BRep` (Boundary Representation).
   - Fusion 360 will prompt you to convert the mesh. Confirm and proceed.

2. **Edit the Mesh** (if necessary):
   - Use the `Repair` tool to fix any issues with the mesh. This is available under the `Mesh` tab.
   - Simplify the mesh if it’s too detailed. This can be done using the `Reduce` tool.

## Step 3: Calculate the Inertia
1. **Create a New Component**: 
   - Right-click on the `root component` in the `Browser` pane.
   - Select `New Component`.

2. **Move the Solid into the New Component**: 
   - Drag the solid body from the `Browser` pane into the newly created component.

3. **Go to the Inspect Menu**:
   - Click on the `Inspect` menu.
   - Select `Physical Properties`.

4. **Select the Solid**: 
   - Click on the solid body you want to analyze.

5. **Calculate Properties**:
   - Fusion 360 will display the mass, volume, center of mass, and moments of inertia for the selected solid.
   - You can view the moments of inertia in the results window.

6. **Export the Inertia Data** (Optional):
   - Click on `Export` to save the physical properties data to a CSV file for further analysis or record-keeping.

## Step 4: Validate and Use the Inertia Data
1. **Check Units**:
   - Ensure the units of measurement are consistent with your requirements (e.g., kg·m² for moments of inertia).
   - You can change units in the `Document Settings` at the bottom of the `Browser` pane.

2. **Use the Data**:
   - The moments of inertia provided can be used for further simulations, control algorithms, or integrating with other software like PyBullet.

## Tips and Tricks
- **Simplification**: Before converting the mesh to a solid, it might be helpful to simplify the mesh to reduce complexity and improve performance.
- **Mesh Repair**: Always check for and repair any errors in the mesh to ensure accurate physical properties.
- **Fusion 360 Help**: Utilize Fusion 360’s help resources and community forums if you encounter any issues.

This tutorial should guide you through the process of calculating inertia for a mesh file in Autodesk Fusion 360. If you need more advanced features, Autodesk’s official documentation and support can provide additional assistance.


