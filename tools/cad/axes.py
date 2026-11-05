import sys
from inspect_step import load_instances
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.TopoDS import TopoDS
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp

doc, instances = load_instances(sys.argv[1])
for index in (2, 8, 11, 13):
    shape, info = instances[index]
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    found = set()
    while exp.More():
        surf = BRepAdaptor_Surface(TopoDS.Face(exp.Current()))
        if surf.GetType() == GeomAbs_Cylinder:
            c = surf.Cylinder()
            a, p = c.Axis().Direction(), c.Location()
            found.add((round(c.Radius(), 4), tuple(round(v, 6) for v in (p.X(), p.Y(), p.Z())), tuple(round(v, 8) for v in (a.X(), a.Y(), a.Z()))))
        if index == 13 and surf.GetType() == GeomAbs_Plane:
            props = GProp_GProps()
            BRepGProp.SurfaceProperties_s(exp.Current(), props)
            if props.Mass() > 5000:
                a = surf.Plane().Axis().Direction()
                print("plate", props.Mass(), a.X(), a.Y(), a.Z())
        exp.Next()
    print(index, sorted(found, reverse=True)[:12])
