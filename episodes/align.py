"""Time alignment for episode export. No ROS imports, unit-testable."""

import numpy as np


def latest_at_or_before(t_src, t_query):
    """Index of the last source sample with t <= query, or -1 if none.
    Used for observations: never uses data from the future."""
    return np.searchsorted(t_src, t_query, side="right") - 1


def first_at_or_after(t_src, t_query):
    """Index of the first source sample with t >= query, or len(t_src) if
    none. Used for actions: the command issued in response to what was seen."""
    return np.searchsorted(t_src, t_query, side="left")
