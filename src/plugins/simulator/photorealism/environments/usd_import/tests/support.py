from pathlib import Path
import tempfile
from pxr import Usd, UsdGeom


def make_stage(test, *, axis='Z', meters_per_unit=1.0, start=None):
    directory = tempfile.TemporaryDirectory()
    test.addCleanup(directory.cleanup)
    stage = Usd.Stage.CreateNew(str(Path(directory.name) / 'scene.usda'))
    if axis is not None:
        UsdGeom.SetStageUpAxis(stage, axis)
    if meters_per_unit is not None:
        UsdGeom.SetStageMetersPerUnit(stage, meters_per_unit)
    if start is not None:
        stage.SetStartTimeCode(start)
        stage.SetEndTimeCode(start + 1)
    return stage


def context(stage, **options):
    from usd_import.scene import Options, read_stage
    stage.GetRootLayer().Save()
    return read_stage(Options(source=Path(stage.GetRootLayer().realPath), **options))


def author_mesh(stage, path, points, faces):
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr([len(f) for f in faces])
    mesh.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    mesh.CreateSubdivisionSchemeAttr('none')
    return mesh
