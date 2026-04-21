# Test PyChrono collision and force API availability
# Checks class names, method signatures, and collision system types for PyChrono 9.x
# Run: conda run -n chrono python test_api.py

import pychrono as chrono

print("=== PyChrono API Check ===")
print(f"Chrono version: {chrono.GetChronoVersion() if hasattr(chrono, 'GetChronoVersion') else 'N/A'}")

# 1. Collision shapes
for name in ['ChCollisionShapeBox', 'ChCollisionShapeSphere']:
    print(f"chrono.{name} exists: {hasattr(chrono, name)}")

# 2. Contact material
for name in ['ChContactMaterialNSC', 'ChContactMaterialSMC']:
    print(f"chrono.{name} exists: {hasattr(chrono, name)}")

# 3. Collision system type
print(f"chrono.ChCollisionSystem exists: {hasattr(chrono, 'ChCollisionSystem')}")
if hasattr(chrono, 'ChCollisionSystem'):
    for tname in ['Type_BULLET', 'Type_MULTICORE']:
        if hasattr(chrono.ChCollisionSystem, tname):
            print(f"  ChCollisionSystem.{tname} = {getattr(chrono.ChCollisionSystem, tname)}")

# 4. Body collision methods
body = chrono.ChBody()
for m in ['EnableCollision', 'AddCollisionShape']:
    print(f"body.{m} exists: {hasattr(body, m)}")

# 5. Test construction
print("\n=== Construction tests ===")
mat = chrono.ChContactMaterialNSC()
print("ChContactMaterialNSC() OK")

box = chrono.ChCollisionShapeBox(mat, 10.0, 10.0, 0.1)
print("ChCollisionShapeBox(mat, hx, hy, hz) OK")

sph = chrono.ChCollisionShapeSphere(mat, 0.05)
print("ChCollisionShapeSphere(mat, r) OK")

# 6. AddCollisionShape signature
body2 = chrono.ChBody()
try:
    body2.AddCollisionShape(sph, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
    print("AddCollisionShape(shape, frame) OK")
except Exception as e:
    print(f"AddCollisionShape(shape, frame) FAILED: {e}")
    body2.AddCollisionShape(sph)
    print("AddCollisionShape(shape) OK (no frame)")

# 7. Force accumulation
body3 = chrono.ChBody()
# Check both naming conventions
for method_name in ['AccumulateForce', 'Accumulate_force']:
    if hasattr(body3, method_name):
        print(f"body.{method_name} exists: True")
        fn = getattr(body3, method_name)
        # Try 2-arg: (force_vec, is_local)
        try:
            fn(chrono.ChVector3d(0, 0, -5), False)
            print(f"  {method_name}(vec, bool) OK")
        except TypeError as e:
            print(f"  {method_name}(vec, bool) FAILED: {e}")
            # Try 3-arg: (force, appl_point, is_local)
            try:
                fn(chrono.ChVector3d(0, 0, -5), chrono.ChVector3d(0, 0, 0), False)
                print(f"  {method_name}(force, point, bool) OK")
            except Exception as e2:
                print(f"  {method_name} 3-arg FAILED: {e2}")
        break
    else:
        print(f"body.{method_name} exists: False")

# 8. System collision type
sys2 = chrono.ChSystemNSC()
try:
    sys2.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    print("SetCollisionSystemType(Type_BULLET) OK")
except Exception as e:
    print(f"SetCollisionSystemType FAILED: {e}")

# 9. Full integration: ball dropping onto ground plane
print("\n=== Full drop test ===")
sys3 = chrono.ChSystemNSC()
sys3.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys3.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

gnd = chrono.ChBody()
gnd.SetFixed(True)
gnd.EnableCollision(True)
gmat = chrono.ChContactMaterialNSC()
gshape = chrono.ChCollisionShapeBox(gmat, 100.0, 100.0, 0.1)
gnd.AddCollisionShape(gshape, chrono.ChFramed(chrono.ChVector3d(0, 0, -0.05)))
sys3.AddBody(gnd)

ball = chrono.ChBody()
ball.SetPos(chrono.ChVector3d(0, 0, 1.0))
ball.SetMass(1.0)
I_s = 0.4 * 1.0 * 0.1 * 0.1
ball.SetInertiaXX(chrono.ChVector3d(I_s, I_s, I_s))
ball.EnableCollision(True)
bmat = chrono.ChContactMaterialNSC()
bshape = chrono.ChCollisionShapeSphere(bmat, 0.1)
ball.AddCollisionShape(bshape)
sys3.AddBody(ball)

for i in range(1000):
    sys3.DoStepDynamics(0.001)

z_final = ball.GetPos().z
print(f"Ball Z after 1s: {z_final:.4f}")
print(f"Ball fell and stopped: {'YES' if 0.0 < z_final < 1.0 else 'NO'}")
if z_final < -0.5:
    print("WARNING: Ball fell through ground! Collision not working.")
elif z_final > 0.9:
    print("WARNING: Ball barely moved. Gravity or collision issue.")
else:
    print("Collision ground plane works correctly.")

print("\n=== API Check Complete ===")
