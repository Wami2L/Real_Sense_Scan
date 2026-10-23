"""
Herramientas offline para escenas 3D Gaussian Splatting (plan 15): leer, analizar, limpiar y reducir PLY
de 3DGS antes de llevarlos a Unity (gsplat) en Quest 2 y PC VR.

Los archivos pueden pesar varios GB: todo se procesa por bloques sobre un memmap, nunca entero en memoria.
"""

from .ply import SH_C0, SplatPly, sigmoid, write_ply  # noqa: F401
