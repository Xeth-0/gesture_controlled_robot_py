"""
Mapping for the different mov't commands to their cmd/vel translations.
"""

SPEED = 0.8


MAPPING = {
    "Forward": {
        'linear': (SPEED, 0.0, 0.0),
        'angular': (0.0)
    },
    "Go": {
        'linear': (SPEED, 0.0, 0.0),
        'angular': (0.0)
    },
    "Backward": {
        'linear': (-SPEED, 0.0, 0.0),
        'angular': (0.0)
    },
    "Left": {
        'linear': (0.0, 0.0, 0.0),
        'angular': (SPEED)
    },
    "Right": {
        'linear': (0.0, 0.0, 0.0),
        'angular': (-SPEED)
    },
    "Stop": {
        'linear': (0.0, 0.0, 0.0),
        'angular': (0.0)
    }
}