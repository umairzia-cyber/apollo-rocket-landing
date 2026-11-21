from dataclasses import dataclass


@dataclass
class RocketState:
    position: tuple       
    velocity: tuple       
    theta: float          
    omega: float          
    fuel: float           
    dry_mass: float       

    @property
    def mass(self):
        """Total rocket mass in kilograms."""
        return self.dry_mass + self.fuel
