"""Station-level controls that are NOT the screener.

The screener is one component of a station, not the station. These run at
different times, on different information:

    screener/   before execution, per candidate, from a camera estimate.
                Purely kinematic -- it never computes a force.
    station/    during execution, on the robot, from its own sensors.

Keeping them in one package implied the force trip was part of the screener.
It is not: it is what bounds the cost of the screener being wrong.
"""
