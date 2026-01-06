from tensorflow.lite.python.interpreter import Interpreter
import numpy as np

def load_interpreter(model_path):
    interpreter = Interpreter(model_path=model_path)
    interpreter.allocate_tensors()
    in_det = interpreter.get_input_details()[0]
    out_det = interpreter.get_output_details()[0]
    return interpreter, in_det, out_det

def run_inference(interpreter, in_det, out_det, data):
    """
    img28: float32 [0,1] shape (28,28,1)
    Supports float32 and int8 models.
    Returns: probs float32 (10,)
    """
    x = data
    if in_det["dtype"] == np.float32:
        x_in = x.astype(np.float32)
    elif in_det["dtype"] == np.int8:
        scale, zero = in_det["quantization"]
        x_in = (x / scale + zero).astype(np.int8)
    else:
        raise ValueError("Unsupported input dtype", in_det["dtype"])

    x_in = np.expand_dims(x_in, 0)  # (1,28,28,1)
    interpreter.set_tensor(in_det["index"], x_in)
    interpreter.invoke()
    y = interpreter.get_tensor(out_det["index"])

    if out_det["dtype"] == np.int8:
        s, z = out_det["quantization"]
        y = (y.astype(np.float32) - z) * s

    return y[0].astype(np.float32), y