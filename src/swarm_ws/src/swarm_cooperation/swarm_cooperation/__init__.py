"""Cooperative mapping for the car and ACG720."""
from pkgutil import extend_path

# A source checkout may be first on sys.path while the extensions live in the ROS overlay.
__path__ = extend_path(__path__, __name__)
