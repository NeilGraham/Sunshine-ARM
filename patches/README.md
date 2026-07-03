# Local patches for third-party submodules

Patches here fix bugs in submodules whose upstreams we do not control. They
must be re-applied after any `git submodule update`, which checks out the
pinned upstream commit and discards working-tree changes.

Apply (idempotent — `--check` first, skip if already applied):

```bash
p=patches/inputtino-ds5-motion-negative-values.patch
git -C third-party/inputtino apply --check "$p" 2>/dev/null \
  && git -C third-party/inputtino apply "$p"
```

`init.sh` (retro-stream-home) and `~/bin/build-sunshine-arm` both do this
automatically before building.

Proper fix: fork the submodule (e.g. NeilGraham/inputtino), land the patch
there, and point `.gitmodules` at the fork — then delete the patch here.

| patch | submodule | what it fixes |
| --- | --- | --- |
| `inputtino-ds5-motion-negative-values.patch` | third-party/inputtino | DS5 gyro/accel negatives saturated to 0 in `to_le_signed` (half the motion range lost) |
