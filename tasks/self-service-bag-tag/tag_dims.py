"""Single source for the simulated tag's discretisation (shared by scene, score and probes).

The stock is one 50.8 x 533 mm thermal bag-tag strip. It is modelled as SEGMENTS rigid panels joined by
elastic hinges (a discretised beam, k = EI / L per hinge). Total length, width, mass and bending
stiffness EI do not depend on SEGMENTS; printed features keep their positions along the strip.
"""
SEGMENTS = 14
TAG_LENGTH = .533
SEGMENT_LENGTH = TAG_LENGTH / SEGMENTS
TAG_WIDTH = .0508


def panel_at(distance_from_leading_end_m):
    """Index of the panel whose centre is nearest a position along the strip."""
    return min(SEGMENTS - 1, max(0, int(round(distance_from_leading_end_m / SEGMENT_LENGTH - .5))))


# Barcodes (front face only) at 80 mm and 293 mm from the leading end; yellow mark at 453 mm.
BARCODE_PANELS = (panel_at(.080), panel_at(.293))
MARK_PANEL = panel_at(.453)
