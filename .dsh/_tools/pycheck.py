"""Syntax-only pre-check for the Windows-side mirror.

Compiles each file in memory (no __pycache__, so nothing extra gets scp'd into
the VM) and prints a compact, line-numbered diagnostic. Exit 0 = all clean.

Usage: python .dsh/_tools/pycheck.py <file> [<file> ...]
"""

import pathlib
import sys

bad = 0
for name in sys.argv[1:]:
    path = pathlib.Path(name)
    try:
        src = path.read_text(encoding='utf-8')
    except OSError as exc:
        bad = 1
        print('READ_FAIL {}: {}'.format(name, exc))
        continue
    try:
        compile(src, str(path), 'exec')
    except SyntaxError as exc:
        bad = 1
        print('SYNTAX_FAIL {}:{}: {}'.format(path, exc.lineno, exc.msg))
        text = (exc.text or '').rstrip()
        if text:
            print('    {}'.format(text))
            if exc.offset:
                print('    {}^'.format(' ' * (exc.offset - 1)))

print('LOCAL_SYNTAX=' + ('FAIL' if bad else 'ok'))
sys.exit(bad)
