"""Fixed ruler face and three USD geometry pointers.

Only the pointers move. The face texture never changes after spawn. Live values
come from Runtime's physical link forces and integrated output encoder.
"""
from __future__ import annotations

from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

_FACE = Path(__file__).resolve().parent / 'source_assets' / 'recorder_fixed_scales.png'
_X0, _X1 = 375, 945
_ROWS = (130, 290, 449)
_W, _H = 1024, 512


def _font(size):
    return ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', size)


def _face():
    im = Image.new('RGB', (_W, _H), (9, 25, 35))
    d = ImageDraw.Draw(im)
    d.text((30, 5), 'JOINT PLAY  |  LOAD / UNLOAD', font=_font(37), fill=(235, 244, 244))
    labels = (('LEFT FORCE', 'N'), ('RIGHT FORCE', 'N'), ('OUTPUT ANGLE', 'deg'))
    for row, (name, unit) in enumerate(labels):
        y = _ROWS[row]
        d.text((28, y-38), name, font=_font(34), fill=(240, 244, 244))
        d.text((26, y+5), unit, font=_font(37), fill=(240, 244, 244))
        d.line((_X0, y+19, _X1, y+19), fill=(240, 244, 244), width=5)
        for i in range(11):
            x = round(_X0 + (_X1-_X0)*i/10)
            d.line((x, y+9, x, y+30 if i % 5 == 0 else y+23),
                   fill=(220, 232, 232), width=3)
        if row < 2:
            a = _X0 + (_X1-_X0)*.6
            b = _X0 + (_X1-_X0)*.9
            d.rectangle((a, y+36, b, y+43), fill=(34, 212, 126))
            d.text((_X0-13, y-41), '0', font=_font(31), fill=(225, 238, 238))
            d.text(((_X0+_X1)//2-13, y-41), '1', font=_font(31), fill=(225, 238, 238))
            d.text((_X1-13, y-41), '2', font=_font(31), fill=(225, 238, 238))
        else:
            for frac, text in ((0, '-5'), (.5, '0'), (1, '+5')):
                x = round(_X0 + (_X1-_X0)*frac)
                d.text((x-26, y-41), text, font=_font(31), fill=(225, 238, 238))
    return im


def _to_local(px, py):
    return (-.148 + .296*px/_W, -.071 + .142*(1-py/_H))


def _texture(stage, prim_path):
    from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt
    quad = UsdGeom.Mesh.Define(stage, f'{prim_path}/readout_visual/screen_quad')
    quad.CreatePointsAttr(Vt.Vec3fArray([
        Gf.Vec3f(-.148, -.071, .0518), Gf.Vec3f(.148, -.071, .0518),
        Gf.Vec3f(.148, .071, .0518), Gf.Vec3f(-.148, .071, .0518),
    ]))
    quad.CreateFaceVertexCountsAttr([4])
    quad.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    quad.CreateSubdivisionSchemeAttr('none')
    uv = UsdGeom.PrimvarsAPI(quad).CreatePrimvar('st', Sdf.ValueTypeNames.TexCoord2fArray,
                                                UsdGeom.Tokens.faceVarying)
    uv.Set(Vt.Vec2fArray([Gf.Vec2f(0,0), Gf.Vec2f(1,0),
                          Gf.Vec2f(1,1), Gf.Vec2f(0,1)]))
    material = UsdShade.Material.Define(stage, f'{prim_path}/Looks/readout')
    surface = UsdShade.Shader.Define(stage, f'{material.GetPath()}/surface')
    surface.CreateIdAttr('UsdPreviewSurface')
    surface.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.22)
    tex = UsdShade.Shader.Define(stage, f'{material.GetPath()}/texture')
    tex.CreateIdAttr('UsdUVTexture')
    tex.CreateInput('file', Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(_FACE)))
    tex.CreateOutput('rgb', Sdf.ValueTypeNames.Float3)
    st = UsdShade.Shader.Define(stage, f'{material.GetPath()}/st')
    st.CreateIdAttr('UsdPrimvarReader_float2')
    st.CreateInput('varname', Sdf.ValueTypeNames.Token).Set('st')
    st.CreateOutput('result', Sdf.ValueTypeNames.Float2)
    tex.CreateInput('st', Sdf.ValueTypeNames.Float2).ConnectToSource(st.ConnectableAPI(), 'result')
    surface.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), 'rgb')
    surface.CreateInput('emissiveColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), 'rgb')
    material.CreateSurfaceOutput().ConnectToSource(surface.ConnectableAPI(), 'surface')
    UsdShade.MaterialBindingAPI.Apply(quad.GetPrim()).Bind(material)


def _pointer(stage, prim_path, row, color):
    from pxr import Gf, Sdf, UsdGeom, UsdShade
    path = f'{prim_path}/readout_visual/pointer_{row}'
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.)
    x, y = _to_local(_X0 if row < 2 else (_X0+_X1)/2, _ROWS[row]+18)
    cube.AddTranslateOp().Set(Gf.Vec3d(x,y,.0533))
    cube.AddScaleOp().Set(Gf.Vec3f(.006,.020,.002))
    mat = UsdShade.Material.Define(stage, f'{prim_path}/Looks/pointer_{row}')
    shader = UsdShade.Shader.Define(stage, f'{mat.GetPath()}/surface')
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput('emissiveColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(mat)


def spawn(stage, prim_path, spec):
    from pxr import UsdGeom
    spec.meta['prim_path'] = prim_path
    UsdGeom.Xform.Define(stage, f'{prim_path}/readout_visual')
    _FACE.parent.mkdir(parents=True, exist_ok=True)
    if not _FACE.exists():
        _face().save(_FACE)
    _texture(stage, prim_path)
    for row, color in enumerate(((1.,.16,.10),(1.,.16,.10),(.12,.85,1.))):
        _pointer(stage, prim_path, row, color)


def _move(stage, prim_path, row, value):
    from pxr import Gf
    frac = max(0.,min(1., value/2. if row < 2 else (value+5.)/10.))
    x, y = _to_local(_X0+(_X1-_X0)*frac, _ROWS[row]+18)
    attr = stage.GetPrimAtPath(f'{prim_path}/readout_visual/pointer_{row}').GetAttribute('xformOp:translate')
    if not attr:
        raise RuntimeError(f'missing recorder pointer {row}')
    attr.Set(Gf.Vec3d(x,y,.0533))


def update(stage, prim_path, left_n, right_n, encoder_count, step):
    if step % 2:
        return
    angle_deg = encoder_count*360/4096
    for row, value in enumerate((left_n,right_n,angle_deg)):
        _move(stage, prim_path, row, value)


def publish_synthetic(stage, prim_path, left_n, right_n, angle_deg, token):
    """Move visual pointers only for isolated renderer diagnosis."""
    for row, value in enumerate((left_n,right_n,angle_deg)):
        _move(stage, prim_path, row, value)
    return {'left_n':left_n,'right_n':right_n,'angle_deg':angle_deg}
