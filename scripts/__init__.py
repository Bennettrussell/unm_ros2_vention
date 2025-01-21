

__version__ = "0.0.1"

# Optionally, import classes/functions so users can do:
#   from unm_ros2_vention import VentionController
#   from unm_ros2_vention import vention_node_main
# etc.

# For example, you can import from the wrapper:
try:
    from .vention_api_wrapper import (
        VentionController,
        DEFAULT_IP_ADDRESS,
        # plus any other items you want to expose at package level
    )
except ImportError:
    # If for some reason the wrapper isn't found, skip or log a warning
    VentionController = None
    DEFAULT_IP_ADDRESS = None

# If you want to give direct access to the nodes' main functions, you can do so:
try:
    from .vention_node import main as vention_node_main
except ImportError:
    vention_node_main = None

try:
    from .vention_reset import main as vention_reset_main
except ImportError:
    vention_reset_main = None

__all__ = [
    "__version__",
    "VentionController",
    "DEFAULT_IP_ADDRESS",
    "vention_node_main",
    "vention_reset_main",
]
