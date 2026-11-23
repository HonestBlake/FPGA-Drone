"""Independent integer dot-product reference for the supplied, unscaled model."""
from pathlib import Path
import random

HERE = Path(__file__).resolve().parent
MODEL = HERE.parents[1] / 'Drone' / 'NPU'

def signed(value, bits):
    return value - (1 << bits) if value & (1 << (bits - 1)) else value

weights = [signed(int(x, 16), 16) for x in (MODEL / 'weights.mem').read_text().split()]
biases = [signed(int(x, 16), 32) for x in (MODEL / 'biases.mem').read_text().split()]
assert len(weights) == 320 and len(biases) == 5
rng = random.Random(20261120)
scenes = [[4000 if weight > 0 else 20 for weight in weights[j*64:(j+1)*64]] for j in range(5)]
scenes += [[0]*64, [65535]*64, [1200+i for i in range(64)]]
scenes += [[rng.randrange(20, 4001) for _ in range(64)] for _ in range(12)]
lines = ['// Independent signed-integer dot products of the original model.']
classes = set()
for k, pixels in enumerate(scenes):
    scores = [biases[j] + sum(pixels[i]*weights[j*64+i] for i in range(64)) for j in range(5)]
    assert all(-(1 << 31) <= s < (1 << 31) for s in scores)
    prediction = max(range(5), key=lambda j: scores[j])
    classes.add(prediction)
    lines += [f'vectors[{k*64+i}]=16\'d{v};' for i, v in enumerate(pixels)]
    lines += [f'expected_scores[{k*5+j}]=32\'h{s & 0xffffffff:08x};' for j, s in enumerate(scores)]
    lines += [f'expected_class[{k}]=3\'d{prediction};']
assert classes == set(range(5)), classes
(HERE / 'model_vectors.vh').write_text('\n'.join(lines)+'\n')
(HERE / 'vector_count.vh').write_text(f'localparam VECTOR_COUNT={len(scenes)};\n')
print(f'Generated {len(scenes)} reference frames; all five classes covered.')
