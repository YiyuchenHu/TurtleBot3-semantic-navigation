"""Pluggable grounding backends.

Each backend implements load(), info(), and
ground(image_bytes, query) -> [{"bbox_xyxy", "score", "label"}, ...].
"""

from __future__ import annotations


def get_backend(name: str, **kwargs):
    # Imported lazily so the mock backend doesn't require torch.
    if name == "mock":
        from .mock import MockColorBackend
        return MockColorBackend(**kwargs)
    if name == "locate_anything":
        from .locate_anything import LocateAnythingBackend
        return LocateAnythingBackend(**kwargs)
    raise ValueError(f"unknown backend '{name}' (expected 'locate_anything' or 'mock')")
