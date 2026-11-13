#!/usr/bin/env python3
# Genera src/pagina_html.h da pagina/pagina.html.
# Da rilanciare ogni volta che si modifica pagina.html:  python3 genera_pagina.py
from pathlib import Path

qui = Path(__file__).resolve().parent
html = (qui / "pagina" / "pagina.html").read_text(encoding="utf-8")
assert ')PAGINA"' not in html
uscita = (
    "// FILE GENERATO da genera_pagina.py partendo da pagina/pagina.html: non modificarlo a mano.\n"
    "#pragma once\n"
    "static const char PAGINA_HTML[] = R\"PAGINA(" + html + ")PAGINA\";\n"
)
(qui / "src" / "pagina_html.h").write_text(uscita, encoding="utf-8")
print("Scritto src/pagina_html.h (%d byte di pagina)" % len(html.encode("utf-8")))
