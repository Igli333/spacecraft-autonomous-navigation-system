class Analysis:
    def __init__(
            self,
            range_,
            closing_rate,
            lateral_offset,
            inside_corridor,
            constraints_violated,
            relPos_H,
            relVel_H
    ):
        self.range = range_
        self.closing_rate = closing_rate
        self.lateral_offset = lateral_offset
        self.inside_corridor = inside_corridor
        self.constraints_violated = constraints_violated
        self.relPos_H = relPos_H
        self.relVel_H = relVel_H