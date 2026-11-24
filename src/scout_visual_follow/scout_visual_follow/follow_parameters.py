"""Shared startup defaults and constraints for visual approach control."""
from math import isfinite


DEFAULT_APPROACH = dict(stop_distance=0.5, resume_distance=1.0, linear_gain=1.0)
DEFAULT_MIN_DEPTH = 0.2
DEFAULT_MAX_DEPTH = 4.0


def resolve_approach(overrides, *, min_depth=DEFAULT_MIN_DEPTH, max_depth=DEFAULT_MAX_DEPTH):
    values = {}
    for name, default in DEFAULT_APPROACH.items():
        try:
            value = float(overrides.get(name, default))
        except (TypeError, ValueError, OverflowError):
            raise ValueError(f'{name} must be a finite number') from None
        if not isfinite(value):
            raise ValueError(f'{name} must be a finite number')
        values[name] = value

    if not 0 < min_depth < values['stop_distance'] < values['resume_distance'] < max_depth < float('inf'):
        raise ValueError('Require 0 < min_depth < stop_distance < resume_distance < max_depth '
                         f'(min_depth={min_depth:g} m, max_depth={max_depth:g} m)')
    if values['linear_gain'] <= 0:
        raise ValueError('linear_gain must be greater than 0')
    return values
