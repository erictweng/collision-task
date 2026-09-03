"""Build one chunk of labelled truth. Chunked because each shell call here is
time-limited and background jobs do not survive it; merge_chunks.py reassembles."""
import sys, pickle
sys.path.insert(0, ".")
from screener.evalrun import build_dataset
from screener.heldout import generate_B

seed0, n, gen, out = int(sys.argv[1]), int(sys.argv[2]), sys.argv[3], sys.argv[4]
ds = build_dataset(n_layouts=n, per_layout=40, seed0=seed0, gen=gen,
                   generator=generate_B if gen == "B" else None)
pickle.dump(ds, open(out, "wb"))
