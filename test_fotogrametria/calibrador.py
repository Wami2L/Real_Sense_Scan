"""
Compatibilidad del estudio de calibración (5005).

El motor de alineación está en live3d/alignment.py y es el mismo que usa el visor
(5002, /api/dual/auto_align). Aquí solo quedan alias para el código del estudio.
"""

from live3d.alignment import RGBDView, align_pair, draw_matches, public_result  # noqa: F401
from live3d.registration import euler_translation_to_matrix, matrix_to_euler_translation  # noqa: F401


class AlineadorVirtual:
    """Envoltorio fino sobre live3d.alignment.align_pair."""

    def align(self, view1, view2, prior_T=None):
        return align_pair(view1, view2, prior_T=prior_T)
