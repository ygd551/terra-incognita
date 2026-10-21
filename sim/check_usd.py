"""Print world bbox and collider count of a USD scene: /isaac-sim/python.sh check_usd.py scene.usd"""
import sys
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
from pxr import Usd, UsdGeom, UsdPhysics
st = Usd.Stage.Open(sys.argv[1])
bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "guide"]).ComputeWorldBound(st.GetPseudoRoot()).ComputeAlignedRange()
n_mesh = sum(1 for p in st.Traverse() if p.IsA(UsdGeom.Mesh))
n_col = sum(1 for p in st.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI))
print(f"[CHECK] bbox min={bb.GetMin()} max={bb.GetMax()} meshes={n_mesh} colliders={n_col}", flush=True)
for p in list(st.GetPrimAtPath('/World').GetChildren())[:3]:
    b = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default","render","guide"]).ComputeWorldBound(p).ComputeAlignedRange()
    print(f"[CHECK] {p.GetName()} min={b.GetMin()} max={b.GetMax()}", flush=True)
app.close()
