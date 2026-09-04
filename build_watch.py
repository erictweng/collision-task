"""Build out/watch.html -- self-contained animated review of the screener."""
import json, pathlib

DATA = json.load(open("out/watch_data.json"))
TMPL = pathlib.Path("watch_template.html").read_text()
out = pathlib.Path("out/watch.html")
out.write_text(TMPL.replace("__DATA__", json.dumps(DATA)))
print("wrote", out, out.stat().st_size // 1024, "KB")
