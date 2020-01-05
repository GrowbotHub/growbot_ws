#!/usr/bin/python3

# Usage: 
# $ cat test.jpg | ./classifier_cli.py

from PIL import Image
import glob, os
import numpy as np
from time import time
import sys
from io import BytesIO
import tensorflow as tf
import json


MODEL_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../data/radishes.tflite')


def main():
  result = {}

  # Load model
  interpreter = tf.lite.Interpreter(model_path=MODEL_PATH)
  interpreter.allocate_tensors()
  input_details = interpreter.get_input_details()
  output_details = interpreter.get_output_details()

  # Prepare image
  im = Image.open(BytesIO(sys.stdin.buffer.read()))
  im = im.resize(input_details[0]['shape'][1:3])
  im = np.array(im).reshape(input_details[0]['shape'])

  # Classify
  tic = time()
  interpreter.set_tensor(input_details[0]['index'], im.astype(np.float32))
  interpreter.invoke()
  toc = time()
  result['time'] = round(toc - tic, 3) * 1000

  # Print the result
  output_data = interpreter.get_tensor(output_details[0]['index'])
  result['classification'] = float(output_data)

  print(json.dumps(result))


if __name__ == "__main__":
  main()
