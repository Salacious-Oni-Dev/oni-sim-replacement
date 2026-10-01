#!/usr/bin/env python3
"""Generate the framework's `SimTunable` enum from sim/tunables.def.

    gen_tunables_cs.py <tunables.def> <out.cs>           write the enum
    gen_tunables_cs.py <tunables.def> <out.cs> --check   exit 1 if <out.cs> is not what
                                                         this would write

tunables.def is the only place a tunable is defined, and a row's id is its position in the
file, so the managed enum is generated rather than typed: one member per row, in row order,
valued by that position, carrying the row's doc line, group and origin. The framework commits
the output (OniFramework/SimTunable.cs), because its build does not have this repository;
`--check` is how a change to the rows is caught before it ships a stale enum, and the
framework's self-test compares the enum against SIM_ExtTunableDescribe by name, both ways, on
the DLL that is actually loaded.

The output is a pure function of tunables.def.
"""
import re
import sys


def split_args(body):
    """Top-level comma split of a macro argument list, respecting string literals."""
    args, depth, cur, i = [], 0, [], 0
    while i < len(body):
        ch = body[i]
        if ch == '"':
            j = i + 1
            while body[j] != '"':
                j += 2 if body[j] == '\\' else 1
            cur.append(body[i:j + 1])
            i = j + 1
            continue
        if ch in '([':
            depth += 1
        elif ch in ')]':
            depth -= 1
        if ch == ',' and depth == 0:
            args.append(''.join(cur).strip())
            cur = []
        else:
            cur.append(ch)
        i += 1
    args.append(''.join(cur).strip())
    return args


def rows(text):
    # Comments first, so a `TUNABLE(` inside one is not a row. Strings in this file hold no `//`.
    text = re.sub(r'//[^\n]*', '', text)
    out = []
    for m in re.finditer(r'\bTUNABLE(_AT)?\(', text):
        start = m.end()
        depth, i = 1, start
        while depth:
            ch = text[i]
            if ch == '"':
                i += 1
                while text[i] != '"':
                    i += 2 if text[i] == '\\' else 1
            elif ch == '(':
                depth += 1
            elif ch == ')':
                depth -= 1
            i += 1
        args = split_args(text[start:i - 1])
        at = m.group(1) is not None
        want = 10 if at else 9
        if len(args) != want:
            sys.exit(f"tunables.def: a row near {args[0]!r} has {len(args)} arguments, not {want}")
        name = args[0]
        typ = args[3] if at else args[2]
        group, origin = args[-3], args[-2]
        doc = ''.join(re.findall(r'"((?:[^"\\]|\\.)*)"', args[-1]))
        out.append((name, typ, group, origin, doc))
    return out


def xml(s):
    return s.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')


def render(table):
    kinds = {'float': 'float', 'int32_t': 'int', 'double': 'double'}
    lines = [
        "// GENERATED from the SimDLL's sim/tunables.def by its tools/gen_tunables_cs.py. DO NOT",
        "// EDIT: change the row there and regenerate. A member's value is its row's position, which",
        "// is the tunable's id on the wire (kSetTunable) and in every SIM_ExtTunable* export. Rows",
        "// are append-only, so an id never changes meaning. SimTunables is the API that uses these.",
        "namespace OniFramework",
        "{",
        "\t/// <summary>",
        "\t/// One of the SimDLL's live-tunable numbers. Read and set through <see cref=\"SimTunables\"/>.",
        f"\t/// {len(table)} rows, generated from the SimDLL's own table.",
        "\t/// </summary>",
        "\tpublic enum SimTunable",
        "\t{",
    ]
    for i, (name, typ, group, origin, doc) in enumerate(table):
        lines.append(f"\t\t/// <summary>{xml(doc)}</summary>")
        lines.append(f"\t\t/// <remarks>{kinds[typ]}; group {group}; origin {origin}.</remarks>")
        lines.append(f"\t\t{name} = {i},")
    lines += ["\t}", "}", ""]
    return "\n".join(lines)


def main():
    if len(sys.argv) not in (3, 4) or (len(sys.argv) == 4 and sys.argv[3] != '--check'):
        sys.exit(__doc__)
    def_path, out_path = sys.argv[1:3]
    table = rows(open(def_path, encoding='utf-8').read())
    names = [r[0] for r in table]
    if len(set(names)) != len(names):
        sys.exit("tunables.def: two rows share a name")
    text = render(table)
    if len(sys.argv) == 4:
        try:
            have = open(out_path, encoding='utf-8', newline='').read()
        except FileNotFoundError:
            have = None
        if have != text:
            sys.exit(f"{out_path} is stale: regenerate it from {def_path}")
        print(f"{out_path}: up to date ({len(table)} rows)")
        return
    with open(out_path, 'w', encoding='utf-8', newline='') as f:
        f.write(text)
    print(f"{out_path}: {len(table)} rows")


main()
