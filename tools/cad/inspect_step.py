"""Inspect STEP assembly instances using cadquery-ocp (run in the CAD image)."""
import json
import sys
from pathlib import Path

from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_Label
try:
    from OCP.TDF import TDF_LabelSequence
except ImportError:
    from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TopLoc import TopLoc_Location
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib


def load_instances(filename):
    doc = TDocStd_Document(TCollection_ExtendedString('robot'))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if int(reader.ReadFile(str(filename))) != 1 or not reader.Transfer(doc):
        raise RuntimeError(f'Cannot read {filename}')
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)
    instances = []

    def name(label):
        attr = TDataStd_Name()
        return attr.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), attr) else 'unnamed'

    def walk(label, parent_location, path):
        location = parent_location.Multiplied(tool.GetLocation_s(label))
        referred = TDF_Label()
        definition = label
        if tool.IsReference_s(label):
            tool.GetReferredShape_s(label, referred)
            definition = referred
        path = path + [name(label)]
        children = TDF_LabelSequence()
        if tool.GetComponents_s(definition, children):
            for i in range(1, children.Length() + 1):
                walk(children.Value(i), location, path)
        else:
            shape = tool.GetShape_s(definition).Moved(location)
            box = Bnd_Box()
            BRepBndLib.AddOptimal_s(shape, box)
            instances.append((shape, {'id': len(instances), 'name': name(definition), 'path': path, 'bbox_mm': [box.CornerMin().X(), box.CornerMin().Y(), box.CornerMin().Z(), box.CornerMax().X(), box.CornerMax().Y(), box.CornerMax().Z()]}))

    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location(), [])
    return doc, instances


if __name__ == '__main__':
    doc, instances = load_instances(sys.argv[1])
    Path(sys.argv[2]).write_text(json.dumps([info for _, info in instances], ensure_ascii=False, indent=2))
    print(f'{len(instances)} leaf instances')
