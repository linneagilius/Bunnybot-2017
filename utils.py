"""Standard utilies"""


def clamp(value, max, min):
    """Clamp value within a range"""
    return max if value > max else min if value < min else value
