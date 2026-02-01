# Controller Performance Benchmarks

Micro-benchmarks for `CrazyflieController.compute` to gauge how throughput scales with the number of parallel environments on CPU/GPU.

## Run

From the repository root:

```bash
# Default attitude mode
python perf/benchmark_controller.py --envs 1 4 8 16 --steps 500 --device cpu

# Speed mode
python perf/benchmark_controller.py --mode velocity --envs 8 32 64 --steps 1000 --device cpu

# Position mode
python perf/benchmark_controller.py --mode position --envs 4 16 64 --steps 1000 --device cpu

# CUDA large-scale examples
python perf/benchmark_controller.py --mode attitude --envs 512 8192 32768 124000 --steps 2000 --warmup 200 --device cuda
python perf/benchmark_controller.py --mode velocity --envs 512 8192 32768 124000 --steps 2000 --warmup 200 --device cuda
python perf/benchmark_controller.py --mode position --envs 512 8192 32768 124000 --steps 2000 --warmup 200 --device cuda
```

Common flags:
- `--envs`: space-separated list of environment counts to test (default: `1 8 32 128`)
- `--steps`: timed iterations per case (default: `1000`)
- `--warmup`: warmup iterations per case to prime kernels/caches (default: `100`)
- `--device`: `cpu` or `cuda` (defaults to `cuda` when available)
- `--attitude-dt`, `--position-dt`: optionally override controller update periods (seconds)
- `--mode`: control mode to benchmark: `attitude` (default), `velocity`, or `position`

## Output

Each row reports throughput for one env count:

```
Running on device: cuda | mode: attitude
CUDA device: NVIDIA GeForce RTX 3090
envs |   steps/s | envSteps/s |  ms/step |  us/env-step
-------------------------------------------------------
  512 |      638.2 |      326.7k |    1.567 |       3.06
 8192 |      678.3 |     5556.7k |    1.474 |       0.18
32768 |      664.1 |    21761.1k |    1.506 |       0.05
124000 |      645.7 |    80060.7k |    1.549 |       0.01
```

Values are wall-clock measurements; GPU runs call `torch.cuda.synchronize()` before timing to avoid skew from asynchronous launches.
