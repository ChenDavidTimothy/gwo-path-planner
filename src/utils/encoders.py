import json
import numpy as np

class NumpyEncoder(json.JSONEncoder):
    """JSON encoder that handles NumPy arrays."""
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)