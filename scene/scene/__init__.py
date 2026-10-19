"""Rookery scene package.

Pattern (from Phase 2): **Builder**. ``scene.py`` turns a ``SceneSpec`` of named
parameters (trays, cells, pitch, shelf gap, exit tray) into MJCF under
``generated/``. Why: the MJCF is never hand-edited (rule 7); changing the farm
means changing arguments and regenerating, and the output is byte-identical
for the same arguments.
"""
