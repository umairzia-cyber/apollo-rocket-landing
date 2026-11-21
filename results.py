from dataclasses import dataclass, asdict



@dataclass
class SimulationResult:
    scenario_name: str
    initial_position: tuple
    initial_velocity: tuple
    initial_theta: float
    initial_omega: float
    initial_fuel: float

    status: str
    horizontal_error: float
    horizontal_velocity: float
    vertical_velocity: float
    touchdown_theta: float
    touchdown_omega: float
    fuel_burned: float
    flight_time: float

    def to_dict(self):
        return asdict(self)