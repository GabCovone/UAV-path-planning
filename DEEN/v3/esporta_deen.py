import onnx
import os
currpath = os.path.dirname(os.path.abspath(__file__))

# Carica il modello frammentato (si tirerà dietro automaticamente i dati esterni)
model = onnx.load(os.path.join(currpath,"deen_addestrato_v2.onnx"))

# Salva tutto in un unico file monolitico disattivando i dati esterni
onnx.save_model(model, os.path.join(currpath,"deen_standalone.onnx"), save_as_external_data=False)