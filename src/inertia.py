#!/usr/bin/env python3
# inertia.py — oriented mesh + quaternion + inertia (works on any trimesh ≥ 3.0)

import numpy as np
import trimesh
import trimesh.transformations as tf
from pathlib import Path

# ───────────────────────── user parameters ────────────────────────────────
mesh_path   = Path("/home/akshay/ros2_ws/src/virtual_shake_robot_pybullet/models/SP1_PBRmodel/final_sp1.obj")
out_path    = mesh_path.with_name("final_sp1_pca.obj")

USE_DENSITY = True              # ⇦ pick ONE block: density or target mass
density     = 2700.0            # kg m⁻³  (granite)

target_mass = 353.802           # kg      (set USE_DENSITY = False to use this)
# ───────────────────────────────────────────────────────────────────────────


# ––– robust principal-axis helper (works on any trimesh version) –––––––––––
def _fallback_principal_axis_transform(mesh: trimesh.Trimesh):
    I = mesh.moment_inertia
    eigvals, eigvecs = np.linalg.eigh(I)
    if np.linalg.det(eigvecs) < 0:
        eigvecs[:, -1] *= -1          # right-handed
    T = np.eye(4)
    T[:3, :3] = eigvecs.T
    return T, np.diag(eigvals)

try:
    from trimesh.inertia import principal_axis_transform
except (ImportError, AttributeError):
    try:
        from trimesh.inertia import principal_inertia_transform as principal_axis_transform
    except (ImportError, AttributeError):
        principal_axis_transform = _fallback_principal_axis_transform
# –––––––––––––––––––––– end helper –––––––––––––––––––––––––––––––––––––––––

mesh = trimesh.load_mesh(mesh_path)

# ── handle mass / density ──────────────────────────────────────────────────
if USE_DENSITY:
    mesh.density = density
else:
    mesh.density = target_mass / mesh.volume

# body-frame properties
body_mass   = mesh.mass
body_I      = mesh.moment_inertia        # 3×3, about CoM in CAD frame

# principal-frame transform & diagonal inertia
T_body_to_P, I_P_diag = principal_axis_transform(mesh)   # 4×4, 3×3 diag
ixx, iyy, izz = np.diag(I_P_diag)

# write oriented mesh
mesh_P = mesh.copy()
mesh_P.apply_transform(T_body_to_P)
mesh_P.export(out_path)
print(f"✓ Oriented mesh written to {out_path}")

# Calculate and print the inertia tensor of the PCA-aligned mesh
mesh_P.density = mesh.density  # Ensure we're using the same density
principal_inertia = mesh_P.moment_inertia

print("\nPCA-aligned mesh inertia tensor (kg·m²):")
print(principal_inertia)

# Let's verify this is diagonal (or very close to diagonal)
print("\nOff-diagonal elements (should be close to zero):")
off_diags = [
    principal_inertia[0,1], principal_inertia[0,2],
    principal_inertia[1,0], principal_inertia[1,2],
    principal_inertia[2,0], principal_inertia[2,1]
]
print(off_diags)

# Verify that diagonal elements match I_P_diag
print("\nDiagonal elements comparison:")
print(f"From direct calculation: {np.diag(principal_inertia)}")
print(f"From PCA function: {np.diag(I_P_diag)}")

# Verify total mass is conserved
print(f"\nMass before PCA: {body_mass:.6f} kg")
print(f"Mass after PCA: {mesh_P.mass:.6f} kg")

# quaternion (inverse rotation)  wxyz → xyzw
q_wxyz = tf.quaternion_from_matrix(np.linalg.inv(T_body_to_P))
q_xyzw = (np.roll(q_wxyz, -1) / np.linalg.norm(q_wxyz)).tolist()
q_rpy = tf.euler_from_quaternion(q_xyzw, axes='sxyz')  # Convert quaternion to roll, pitch, yaw
print(f"rpy (rad): {q_rpy}")


# ── prints ─────────────────────────────────────────────────────────────────
np.set_printoptions(precision=6, suppress=True)

print("\nBody-frame inertia tensor (kg·m²):\n", body_I)
print("\nPrincipal-frame diagonal inertia (kg·m²):\n", I_P_diag)

print("\nCopy-paste into YAML  ➜  rock_orientation:")
print(q_xyzw)

print("\nCopy-paste into URDF  ➜  <inertia .../>  (off-diagonals zero):")
print(dict(ixx=float(ixx), ixy=0.0, ixz=0.0,
           iyy=float(iyy), iyz=0.0,
           izz=float(izz)))

print(f"\n<mass value=\"{body_mass:.6f}\"/>   ← also update this in the URDF")
