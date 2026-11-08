#!/usr/bin/env python3
"""Build xiao_fpv_rc/index_html.h from xiao_fpv_rc/web/index.html.
Run after editing the web page:  python3 firmware/tools/embed_html.py
"""
import os
here = os.path.dirname(os.path.abspath(__file__))
src = os.path.join(here, '..', 'xiao_fpv_rc', 'web', 'index.html')
dst = os.path.join(here, '..', 'xiao_fpv_rc', 'index_html.h')
html = open(src, encoding='utf-8').read()
delim = 'HTML_PAGE'
assert ')' + delim + '"' not in html
with open(dst, 'w', encoding='utf-8', newline='\n') as f:
    f.write('// Generated from web/index.html by tools/embed_html.py - edit the .html, then run the script\n')
    f.write('#pragma once\n\n')
    f.write('const char INDEX_HTML[] = R"' + delim + '(' + html + ')' + delim + '";\n')
print('written', dst, len(html.encode('utf-8')), 'bytes')
