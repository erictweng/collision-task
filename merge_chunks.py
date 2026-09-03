import sys, pickle, glob
parts = [pickle.load(open(f, "rb")) for f in sorted(glob.glob(sys.argv[1]))]
m = dict(layouts={}, motions=[], records=[], outcomes={}, sim_rate=0.0)
for p in parts:
    m["layouts"].update(p["layouts"]); m["motions"] += p["motions"]
    m["records"] += p["records"]; m["outcomes"].update(p["outcomes"])
m["sim_rate"] = sum(p["sim_rate"] for p in parts) / len(parts)
pickle.dump(m, open(sys.argv[2], "wb"))
print("merged %d parts -> %d motions, %d layouts, sim_rate %.1f/s"
      % (len(parts), len(m["records"]), len(m["layouts"]), m["sim_rate"]))
