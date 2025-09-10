from enum import Enum


class LOC_TYPE(Enum):
    INTERSECTION='intersection'
    T_JUNCTION='t_junction'
    UNSIGNALIZED_MIN_JUNCTION='unsignalized_minor_junction'


junction_map = {
    LOC_TYPE.INTERSECTION: [189],
    LOC_TYPE.T_JUNCTION: [23, 532, 468, 719],
    LOC_TYPE.UNSIGNALIZED_MIN_JUNCTION: [134, 895, 664]
}