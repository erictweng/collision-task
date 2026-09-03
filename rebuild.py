import sys, pickle, time
sys.path.insert(0,".")
from screener.evalrun import build_dataset
from screener.heldout import generate_B
t=time.time()
dsA=build_dataset(n_layouts=32, per_layout=40, gen="A")
pickle.dump(dsA, open("out/ds_A.pkl","wb")); print("A done %.0fs"%(time.time()-t), flush=True)
dsB=build_dataset(n_layouts=16, per_layout=40, gen="B", generator=generate_B)
pickle.dump(dsB, open("out/ds_B.pkl","wb")); print("B done %.0fs"%(time.time()-t), flush=True)
