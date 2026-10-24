"""utilities for introspecting services"""

import enum


class ServiceIntrospectionState(enum.IntEnum):
    OFF = 0

    METADATA = 1

    CONTENTS = 2
