import sys, pickle
sys.path.insert(0, ".")
import warnings; warnings.filterwarnings("ignore")
from screener.dual import build_dual_dataset
seed0, n, out = int(sys.argv[1]), int(sys.argv[2]), sys.argv[3]
pickle.dump(build_dual_dataset(n_layouts=n, per_layout=24, seed0=seed0), open(out, "wb"))
