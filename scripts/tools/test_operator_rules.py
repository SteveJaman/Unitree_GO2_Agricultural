import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'nodes'))
import traversability_core as tc

D = tc.decide
def V(name, label='step_over', fragile=False):
    return {'name': name, 'label': label, 'fragile': fragile}
R = {'avoid': ['laptop', 'glass'], 'safe': ['cable', 'power strip'], 'unknown': 'vlm'}
S = dict(R, unknown='strict')
cases = [
    ('tall object is avoided whatever the lists say', D(0.12, V('cable'), 0.08, True, R), 'avoid'),
    ('avoid list beats a step_over verdict', D(0.03, V('Laptop computer'), 0.08, True, R), 'avoid'),
    ('safe list beats a fragile verdict', D(0.03, V('power strip', 'avoid', True), 0.08, True, R), 'traversable'),
    ('unlisted, VLM says step_over', D(0.03, V('rug'), 0.08, True, R), 'traversable'),
    ('unlisted, VLM says fragile', D(0.03, V('mug', 'step_over', True), 0.08, True, R), 'avoid'),
    ('verdict still pending stays caution', D(0.03, None, 0.08, True, R), 'caution'),
    ('no rules gives the old behaviour', D(0.03, V('laptop'), 0.08, True, None), 'traversable'),
    ('strict: unlisted name is avoided', D(0.03, V('rug'), 0.08, True, S), 'avoid'),
    ('strict: safe-listed name is crossed', D(0.03, V('cable'), 0.08, True, S), 'traversable'),
    ('VLM off: lists have no effect', D(0.03, None, 0.08, False, R), 'traversable'),
]
bad = 0
for name, got, want in cases:
    ok = got == want
    bad += not ok
    print(('PASS ' if ok else 'FAIL ') + name + ('' if ok else f'  (got {got}, want {want})'))
print(f'{len(cases) - bad}/{len(cases)} passed')
sys.exit(1 if bad else 0)
