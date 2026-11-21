# 96_clean_names.py - strips the " (1)" suffix Fusion adds to rebuilt components (deleted names stay in undo history).
# Renames each component to its base name unless that name is still taken by another component.  Run after 90 (joints keep references).
import re
import aegis_cad as A

def run(_context: str):
    if not A.doc_ok():
        return
    comps = list(A.design().allComponents)
    taken = set(c.name for c in comps)
    done, left = 0, []
    for c in comps:
        base = re.sub(r' \(\d+\)$', '', c.name)
        if base != c.name:
            if base in taken:
                left.append(c.name); continue
            try:
                taken.discard(c.name); c.name = base; taken.add(base); done += 1
            except Exception as e:
                left.append('%s (%s)' % (c.name, e))
    print('renamed %d components; left: %s' % (done, left))
