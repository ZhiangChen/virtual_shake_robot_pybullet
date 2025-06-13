# Configuring Mass & Inertia for the Virtual Shake Robot (VSR)

Getting the **inertia tensor** right is essential—if the tensor doesn’t match the frame of your collision mesh, PyBullet will apply torques you never asked for and the robot will wobble or spin.

---

## 1 Why inertia matters  
*PyBullet’s `createMultiBody` assumes the inertia tensor is expressed in the same frame as the collision geometry.*  
If the tensor is diagonal in one frame but the mesh is oriented differently, simulation physics go off‑track. Our goal is therefore **a diagonal inertia tensor in the same frame as the mesh**.

---

## 2 Two ways to obtain a diagonal inertia tensor

### A. Analytic box formula — for simple rectangles/cubes

1. Measure **width** `w`, **height** `h`, **depth** `d`, and **mass** `m`.  
2. Compute:

   ```math
   I_{xx}=\frac{1}{12} m (h^{2}+d^{2}) \\
   I_{yy}=\frac{1}{12} m (w^{2}+d^{2}) \\
   I_{zz}=\frac{1}{12} m (w^{2}+h^{2})
   ```

3. In YAML / URDF set

   ```yaml
   inertia: [Ixx, Iyy, Izz]    # off‑diagonals are all 0.0
   ```

   Because the box sides line up with the axes, **no quaternion is needed**.

---

### B. `inertia.py` script — for any complex mesh (`.obj`, `.stl`, …)

1. Edit the script header:

   ```python
   mesh_path   = "path/to/mesh.obj"
   USE_DENSITY = True          # or False to set a target mass
   density     = 2700.0        # kg/m³  (if USE_DENSITY = True)
   target_mass = 353.802       # kg     (if USE_DENSITY = False)
   ```

2. Run

   ```bash
   python3 inertia.py
   ```

3. The script will

   * compute the full inertia tensor,
   * rotate the mesh into its **principal‑axis frame** (so the tensor becomes diagonal),
   * export `*_pca.obj`,
   * print  
     * **mass**,  
     * diagonal inertia `[Ixx, Iyy, Izz]`,  
     * quaternion `[x, y, z, w]` giving the rotation.

4. Copy into YAML / URDF:

   ```yaml
   pedestal:
     mesh_file: "final_sp1_pca.obj"      # the oriented mesh
     mass: <printed_mass>
     inertia: [<Ixx>, <Iyy>, <Izz>]
     rock_orientation: [<x>, <y>, <z>, <w>]   # quaternion xyzw
   ```

   *Keep both the rotated mesh **and** the quaternion—this ensures mesh and inertia remain in sync.*

---

## 3 Where these numbers go (quick reference)

```yaml
simulation_node:
  ros__parameters:
    structure:
      pedestal:
        mass: 353.802
        inertia: [645.0, 817.0, 902.0]     # example
        rock_orientation: [0.0065, -0.7070, 0.0000, 0.7072]  
```

---

## 4 Sanity check before launching VSR

1. Off‑diagonal inertia terms are **≈ 0** (script prints them).  
2. Mass matches expectations.  
3. Drop the object in PyBullet — it should fall straight without tumbling.  
4. If you used the script, confirm `*_pca.obj` is the file actually loaded by the simulator.

---

**Done!** Choose the analytic route for boxes or run `inertia.py` for any mesh, paste the results, and your Virtual Shake Robot will behave as expected in simulation.