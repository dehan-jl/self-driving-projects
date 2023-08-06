# WSL conda libstd configuration
Something with the way conda/wsl/libstd interacts causes issues, this fixes it.
```bash
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6
```