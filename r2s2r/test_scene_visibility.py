from types import SimpleNamespace

from pxr import Usd, UsdGeom, UsdPhysics

from r2s2r.core.scene_builder import create_environment


def test_nurec_collision_mesh_is_not_globally_hidden_from_rtx(tmp_path):
    asset_path = str(tmp_path / 'scene.usda')
    source = Usd.Stage.CreateNew(asset_path)
    root = UsdGeom.Xform.Define(source, '/World')
    source.SetDefaultPrim(root.GetPrim())
    mesh = UsdGeom.Mesh.Define(source, '/World/Mesh')
    mesh.GetVisibilityAttr().Set(UsdGeom.Tokens.invisible)
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim()).CreateCollisionEnabledAttr(True)
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('none')
    source.GetRootLayer().Save()

    stage = Usd.Stage.CreateInMemory()
    create_environment(SimpleNamespace(stage=stage), {
        'usdz_path': asset_path,
        'keep_invisible_collision_meshes': True,
        'add_collision': True,
    }, {'collision_approximation': 'none'})

    proxy = stage.GetPrimAtPath('/World/Environment/Mesh')
    assert UsdGeom.Imageable(proxy).ComputeVisibility() == UsdGeom.Tokens.inherited
    assert proxy.GetAttribute('primvars:hideForCamera').Get() is True
    assert UsdPhysics.CollisionAPI(proxy).GetCollisionEnabledAttr().Get() is True
    assert UsdPhysics.MeshCollisionAPI(proxy).GetApproximationAttr().Get() == 'none'


def test_visible_collidable_geometry_is_not_hidden_from_camera(tmp_path):
    path = str(tmp_path / 'visible_scene.usda')
    source = Usd.Stage.CreateNew(path)
    root = UsdGeom.Xform.Define(source, '/World')
    source.SetDefaultPrim(root.GetPrim())
    mesh = UsdGeom.Mesh.Define(source, '/World/VisibleWall')
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim()).CreateCollisionEnabledAttr(True)
    source.GetRootLayer().Save()
    stage = Usd.Stage.CreateInMemory()
    create_environment(SimpleNamespace(stage=stage), {
        'usdz_path': path, 'keep_invisible_collision_meshes': True,
    }, {'collision_approximation': 'none'})
    wall = stage.GetPrimAtPath('/World/Environment/VisibleWall')
    assert UsdGeom.Imageable(wall).ComputeVisibility() == UsdGeom.Tokens.inherited
    assert wall.GetAttribute('primvars:hideForCamera').Get() is not True
