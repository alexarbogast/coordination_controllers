import numpy as np

def linear_path(start, end, vel):
    diff = end - start
    path_len = np.linalg.norm(diff)
    u = diff / path_len
    dur = path_len / vel

    f = lambda t: start + t * diff
    f_dot = lambda _: vel * u
    return f, f_dot, dur


def hypotrochoid(scaling):
    # https://www.desmos.com/calculator/r3ltep03hx
    f = lambda t: scaling * np.array(
        [
            2 * np.cos(t) + 4.5 * np.cos(2 / 3 * t),
            2 * np.sin(t) - 4.5 * np.sin(2 / 3 * t),
            0,
        ]
    )
    f_dot = lambda t: scaling * np.array(
        [
            -2 * np.sin(t) - 3 * np.sin(2 / 3 * t),
            2 * np.cos(t) - 3 * np.cos(2 / 3 * t),
            0,
        ]
    )
    return f, f_dot
