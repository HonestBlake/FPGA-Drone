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
accepted_count = 0
def decision(scores):
    prediction = max(range(5), key=lambda j: scores[j])
    ordered = sorted(scores, reverse=True)
    top, second = ordered[:2]
    gap = top-second
    confident = top > second and top >= 4096 and gap >= 4096 and gap*4 >= abs(top)
    return prediction, confident

def evaluate(pixels):
    scores = [biases[j] + sum(pixels[i]*weights[j*64+i] for i in range(64)) for j in range(5)]
    assert all(-(1 << 31) <= s < (1 << 31) for s in scores)
    return scores

for k, pixels in enumerate(scenes):
    scores = evaluate(pixels)
    prediction, confident = decision(scores)
    accepted_count += confident
    classes.add(prediction)
    lines += [f'vectors[{k*64+i}]=16\'d{v};' for i, v in enumerate(pixels)]
    lines += [f'expected_scores[{k*5+j}]=32\'h{s & 0xffffffff:08x};' for j, s in enumerate(scores)]
    lines += [f'expected_class[{k}]=3\'d{prediction};']
    lines += [f'expected_confident[{k}]=1\'b{int(confident)};']
assert classes == set(range(5)), classes
(HERE / 'model_vectors.vh').write_text('\n'.join(lines)+'\n')
(HERE / 'vector_count.vh').write_text(f'localparam VECTOR_COUNT={len(scenes)};\n')
constants = ['// Independent model-specific reference values. Regenerate after a model update.']
strong = next(p for p in scenes if decision(evaluate(p))[1])
weak = next((p for p in scenes if not decision(evaluate(p))[1]), None)
if weak is None:
    # Find the boundary between two class-winning scenes without assuming the
    # newly trained model labels a particular hand-picked image as ambiguous.
    left = scenes[0]
    left_class = decision(evaluate(left))[0]
    right = next(p for p in scenes if decision(evaluate(p))[0] != left_class)
    low, high = 0, 65536
    for _ in range(16):
        mid = (low+high)//2
        pixels = [(a*(65536-mid)+b*mid)//65536 for a,b in zip(left,right)]
        winner, confident = decision(evaluate(pixels))
        if not confident:
            weak = pixels
            break
        if winner == left_class:
            low = mid
        else:
            high = mid
assert weak is not None, 'Need an ambiguous reference frame for confidence tests'
for name, pixels in [('ACCEPTED_FRAME',strong),('REJECTED_FRAME',weak)]:
    packed = ''.join(f'{value:04x}' for value in reversed(pixels))
    winner, confident = decision(evaluate(pixels))
    constants += [f"localparam [1023:0] {name}_PIXELS=1024'h{packed};",
                  f"localparam [2:0] {name}_CLASS=3'd{winner};",
                  f"localparam {name}_CONFIDENT=1'b{int(confident)};"]
for name, value in [('BIAS_FIRST', biases[0]), ('BIAS_LAST', biases[4])]:
    constants += [f"localparam signed [31:0] {name}=32'h{value & 0xffffffff:08x};"]
for name, value in [('WEIGHT_ROW0_PIXEL27', weights[27]), ('WEIGHT_ROW1_PIXEL27', weights[64+27])]:
    constants += [f"localparam signed [15:0] {name}=16'h{value & 0xffff:04x};"]
for name, pixels in [('LAST_PIXEL', [0]*63+[2000]), ('UNIFORM_1000', [1000]*64), ('RAMP', [1200+i for i in range(64)])]:
    scores = evaluate(pixels)
    winner, confident = decision(scores)
    constants += [f"localparam [2:0] {name}_CLASS=3'd{winner};",
                  f"localparam {name}_CONFIDENT=1'b{int(confident)};"]
    for j, score in enumerate(scores):
        constants += [f"localparam signed [31:0] {name}_SCORE{j}=32'h{score & 0xffffffff:08x};"]
(HERE / 'model_constants.vh').write_text('\n'.join(constants)+'\n')
print(f'Generated {len(scenes)} reference frames; all five raw classes covered; {accepted_count} meet confidence thresholds.')
