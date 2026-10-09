# Pre-build: gzip ../bench/web/index.html — the SAME UI as the USB tool — into
# src/index_html.h so the robot serves it itself (one upload, no filesystem
# image). Rewritten only when it changes.
import gzip
import os

Import("env")  # noqa: F821  (provided by PlatformIO)

proj = env.subst("$PROJECT_DIR")  # noqa: F821
src = os.path.join(proj, "..", "bench", "web", "index.html")   # one UI for USB and Wi-Fi
dst = os.path.join(proj, "src", "index_html.h")

with open(src, "rb") as f:
    raw = f.read()
gz = gzip.compress(raw, compresslevel=9, mtime=0)

lines = []
for i in range(0, len(gz), 24):
    lines.append("    " + ",".join(str(b) for b in gz[i:i + 24]) + ",")
body = (
    "// AUTO-GENERATED from bench/web/index.html by tools/embed_html.py. Do not edit.\n"
    "#pragma once\n"
    "#include <pgmspace.h>\n"
    "#include <stddef.h>\n"
    "static const unsigned char INDEX_HTML_GZ[] PROGMEM = {\n"
    + "\n".join(lines)
    + "\n};\n"
    "static const size_t INDEX_HTML_GZ_LEN = %d;   // raw %d bytes\n" % (len(gz), len(raw))
)

old = None
if os.path.exists(dst):
    with open(dst, "r", encoding="utf-8") as f:
        old = f.read()
if old != body:
    with open(dst, "w", encoding="utf-8", newline="\n") as f:
        f.write(body)
    print("embed_html: index.html %d bytes -> %d gz" % (len(raw), len(gz)))
