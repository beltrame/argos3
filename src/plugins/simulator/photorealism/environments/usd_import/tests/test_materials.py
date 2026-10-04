import unittest
from pathlib import Path
import numpy as np
from PIL import Image
from pxr import UsdGeom, UsdShade, Sdf
from support import make_stage, context, author_mesh
from usd_import.scene import iter_occurrences
from usd_import.geometry import convert_geometry
from usd_import.materials import convert_material, attach_material


def fixture(test, **inputs):
    s=make_stage(test)
    mesh=author_mesh(s,'/Mesh',[[0,0,0],[1,0,0],[0,1,0]],[[0,1,2]])
    material=UsdShade.Material.Define(s,'/Material')
    shader=UsdShade.Shader.Define(s,'/Material/Surface'); shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateOutput('surface',Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
    for name,value in inputs.items():
        kind=Sdf.ValueTypeNames.Color3f if isinstance(value,tuple) else Sdf.ValueTypeNames.Float
        shader.CreateInput(name,kind).Set(value)
    return s,mesh,material,shader


def texture(stage,shader,input_name,pixels,*,channel='rgb',colorspace='raw',uv='st',wrap='repeat',scale=None,bias=None):
    path=Path(stage.GetRootLayer().realPath).parent / (input_name+'.png')
    Image.fromarray(np.asarray(pixels,dtype=np.uint8)).save(path)
    tex=UsdShade.Shader.Define(stage,'/Material/'+input_name+'Tex'); tex.CreateIdAttr('UsdUVTexture')
    tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(path.name))
    tex.CreateInput('sourceColorSpace',Sdf.ValueTypeNames.Token).Set(colorspace)
    for name in ('wrapS','wrapT'): tex.CreateInput(name,Sdf.ValueTypeNames.Token).Set(wrap)
    if scale: tex.CreateInput('scale',Sdf.ValueTypeNames.Float4).Set(scale)
    if bias: tex.CreateInput('bias',Sdf.ValueTypeNames.Float4).Set(bias)
    reader=UsdShade.Shader.Define(stage,'/Material/'+input_name+'UV'); reader.CreateIdAttr('UsdPrimvarReader_float2')
    reader.CreateInput('varname',Sdf.ValueTypeNames.Token).Set(uv)
    reader.CreateOutput('result',Sdf.ValueTypeNames.Float2)
    tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(),'result')
    typ=Sdf.ValueTypeNames.Float3 if channel=='rgb' else Sdf.ValueTypeNames.Float
    tex.CreateOutput(channel,typ)
    shader.CreateInput(input_name,typ).ConnectToSource(tex.ConnectableAPI(),channel)
    return tex,reader


class MaterialTests(unittest.TestCase):
    def test_constants_alpha_and_emission(self):
        s,m,_,_=fixture(self,diffuseColor=(0.2,0.4,0.6),metallic=0.25,roughness=0.75,opacity=0.5,emissiveColor=(2,1,0))
        data=convert_material(m.GetPrim(),context(s))
        self.assertAlmostEqual(data.pbr.metallicFactor,0.25)
        self.assertAlmostEqual(data.pbr.roughnessFactor,0.75)
        np.testing.assert_allclose(data.pbr.baseColorFactor/255,[.2,.4,.6,.5],atol=1/255)
        self.assertEqual(data.pbr.alphaMode,'BLEND')
        self.assertEqual(data.gltf_overrides['extensions']['KHR_materials_emissive_strength']['emissiveStrength'],2)
        surface=UsdShade.Shader(s.GetPrimAtPath('/Material/Surface'))
        surface.CreateInput('opacityThreshold',Sdf.ValueTypeNames.Float).Set(.3)
        data=convert_material(m.GetPrim(),context(s))
        self.assertEqual(data.pbr.alphaMode,'MASK'); self.assertAlmostEqual(data.pbr.alphaCutoff,.3)

    def test_malformed_texture_role_has_source_diagnostic(self):
        for name,channel in [('normal','r'),('roughness','rgb'),('diffuseColor','r')]:
            with self.subTest(name=name):
                s,m,_,shader=fixture(self)
                texture(s,shader,name,[[[64,128,192]]],channel=channel)
                ctx=context(s); data=convert_material(m.GetPrim(),ctx)
                self.assertTrue(any(d['code']=='shader' and d['path']=='/Material/Surface' and name in d['message'] for d in ctx.report['diagnostics']))
                with self.assertRaisesRegex(ValueError,'/Material/Surface'): convert_material(m.GetPrim(),context(s,strict=True))

    def test_constant_material_interface_input(self):
        s,m,material,shader=fixture(self)
        interface=material.CreateInput('tint',Sdf.ValueTypeNames.Color3f); interface.Set((1,0,0))
        shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).ConnectToSource(interface)
        ctx=context(s); data=convert_material(m.GetPrim(),ctx)
        np.testing.assert_allclose(data.pbr.baseColorFactor[:3],[255,0,0])
        self.assertEqual(ctx.report['diagnostics'],[])

    def test_preview_binding_beats_universal(self):
        s,m,material,_=fixture(self,roughness=.9)
        preview=UsdShade.Material.Define(s,'/Preview')
        shader=UsdShade.Shader.Define(s,'/Preview/Surface'); shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.2)
        preview.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
        UsdShade.MaterialBindingAPI(m).Bind(preview,materialPurpose='preview')
        self.assertAlmostEqual(convert_material(m.GetPrim(),context(s)).pbr.roughnessFactor,.2)

    def test_packed_metallic_roughness_and_normals(self):
        s,m,_,sh=fixture(self)
        texture(s,sh,'roughness',[[[64,0,0]]],channel='r')
        texture(s,sh,'metallic',[[[0,0,192]]],channel='b')
        texture(s,sh,'normal',[[[128,128,255]]],scale=(2,2,2,1),bias=(-1,-1,-1,0))
        ctx=context(s); data=convert_material(m.GetPrim(),ctx)
        self.assertTrue(all(image.dtype==np.uint8 for image in ctx.cache['images'].values()))
        np.testing.assert_allclose(np.asarray(data.pbr.metallicRoughnessTexture)[0,0,1:],[64,192],atol=1)
        np.testing.assert_allclose(np.asarray(data.pbr.normalTexture)[0,0],[128,128,255],atol=1)

    def test_uv_transform_wrap_and_color_space(self):
        s,m,_,sh=fixture(self)
        pv=UsdGeom.PrimvarsAPI(m).CreatePrimvar('perfuv',Sdf.ValueTypeNames.TexCoord2fArray,'vertex')
        pv.Set([(0,0),(1,0),(0,1)])
        tex,reader=texture(s,sh,'diffuseColor',[[[128,128,128]]],uv='perfuv',wrap='clamp')
        transform=UsdShade.Shader.Define(s,'/Material/Transform'); transform.CreateIdAttr('UsdTransform2d')
        transform.CreateInput('scale',Sdf.ValueTypeNames.Float2).Set((2,3))
        transform.CreateInput('translation',Sdf.ValueTypeNames.Float2).Set((.1,.2))
        transform.CreateInput('in',Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(),'result')
        tex.GetInput('st').ConnectToSource(transform.ConnectableAPI(),'result')
        ctx=context(s); data=convert_material(m.GetPrim(),ctx)
        self.assertEqual(data.uv_name,'perfuv')
        part=convert_geometry(next(iter_occurrences(ctx)),ctx)[0]
        result=attach_material(part,data,ctx)
        np.testing.assert_allclose(result.visual.uv,[[.1,.2],[2.1,.2],[.1,3.2]],atol=1e-6)
        self.assertEqual(data.gltf_overrides['samplers']['baseColorTexture']['wrapS'],33071)
        self.assertAlmostEqual(np.asarray(data.pbr.baseColorTexture)[0,0,0],188,delta=1)

    def test_resize_zero_and_missing_texture(self):
        s,m,_,sh=fixture(self)
        tex,_=texture(s,sh,'diffuseColor',np.full((1024,2048,3),255),colorspace='sRGB')
        self.assertEqual(convert_material(m.GetPrim(),context(s)).pbr.baseColorTexture.size,(1024,512))
        self.assertEqual(convert_material(m.GetPrim(),context(s,texture_size=0)).pbr.baseColorTexture.size,(2048,1024))
        tex.GetInput('file').Set(Sdf.AssetPath('missing.png'))
        ctx=context(s); convert_material(m.GetPrim(),ctx)
        self.assertTrue(any(d['code']=='texture' for d in ctx.report['diagnostics']))
        with self.assertRaises(ValueError): convert_material(m.GetPrim(),context(s,strict=True))

    def test_graph_cycle_and_unknown_surface(self):
        s,m,_,sh=fixture(self)
        graph=UsdShade.NodeGraph.Define(s,'/Graph')
        out=graph.CreateOutput('loop',Sdf.ValueTypeNames.Float3)
        out.ConnectToSource(graph.ConnectableAPI(),'loop')
        sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).ConnectToSource(graph.ConnectableAPI(),'loop')
        ctx=context(s); convert_material(m.GetPrim(),ctx)
        self.assertTrue(any(d['code']=='shader' for d in ctx.report['diagnostics']))
        sh.CreateIdAttr('UnsupportedMDL')
        ctx=context(s); convert_material(m.GetPrim(),ctx)
        self.assertTrue(ctx.report['diagnostics'])

    def test_single_udim_and_multitile_diagnostic(self):
        s,m,_,sh=fixture(self)
        tex,_=texture(s,sh,'diffuseColor',[[[255,0,0]]],colorspace='sRGB')
        directory=Path(s.GetRootLayer().realPath).parent
        (directory/'diffuseColor.png').rename(directory/'map.1001.png')
        tex.GetInput('file').Set(Sdf.AssetPath('map.<UDIM>.png'))
        data=convert_material(m.GetPrim(),context(s))
        np.testing.assert_array_equal(np.asarray(data.pbr.baseColorTexture)[0,0,:3],[255,0,0])
        Image.new('RGB',(1,1)).save(directory/'map.1002.png')
        ctx=context(s); convert_material(m.GetPrim(),ctx)
        self.assertTrue(any(d['code']=='texture' for d in ctx.report['diagnostics']))

    def test_single_udim_requires_uvs_inside_tile(self):
        s,m,_,sh=fixture(self)
        tex,_=texture(s,sh,'diffuseColor',[[[255,0,0]]],colorspace='sRGB')
        directory=Path(s.GetRootLayer().realPath).parent
        (directory/'diffuseColor.png').rename(directory/'map.1001.png')
        tex.GetInput('file').Set(Sdf.AssetPath('map.<UDIM>.png'))
        UsdGeom.PrimvarsAPI(m).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,'vertex').Set([(0,0),(2,0),(0,1)])
        ctx=context(s); part=convert_geometry(next(iter_occurrences(ctx)),ctx)[0]
        attach_material(part,convert_material(m.GetPrim(),ctx),ctx)
        self.assertTrue(any(d['code']=='uv' for d in ctx.report['diagnostics']))

    def test_display_color_fallback_and_no_mutation(self):
        s=make_stage(self); m=author_mesh(s,'/Mesh',[[0,0,0],[1,0,0],[0,1,0]],[[0,1,2]])
        m.CreateDisplayColorPrimvar('constant').Set([(1,0,0)])
        ctx=context(s); part=convert_geometry(next(iter_occurrences(ctx)),ctx)[0]
        attached=attach_material(part,convert_material(m.GetPrim(),ctx),ctx)
        np.testing.assert_array_equal(attached.visual.vertex_colors if attached.visual.kind=='vertex'
                                     else attached.visual.material.baseColorFactor[:3],[255,0,0] if attached.visual.kind!='vertex' else [[255,0,0,255]]*3)
        self.assertEqual(part.mesh.visual.kind,'vertex')


if __name__=='__main__': unittest.main()
