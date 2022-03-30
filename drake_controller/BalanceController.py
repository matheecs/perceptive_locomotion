from pydrake.all import (
    LeafSystem,
)


class BalanceController(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        print("Please create the Balance Controller")
        state_index = self.DeclareDiscreteState(12)
        self.DeclareStateOutputPort("torque", state_index)
        self.DeclarePeriodicDiscreteUpdate(period_sec=0.01, offset_sec=0.0)
