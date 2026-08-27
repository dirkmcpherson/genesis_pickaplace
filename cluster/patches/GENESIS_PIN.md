# Genesis local patch — pin + apply
#
# The simulator is NOT pip-reproducible (CLAUDE.md). It is upstream
#   https://github.com/Genesis-Embodied-AI/Genesis.git @ 31951c3f
# plus the two-file headless-render patch below, which until 2026-08-27 existed ONLY as
# uncommitted working-tree edits on the cluster and the devbox -- i.e. one rm away from lost.
#
# Reproduce exactly:
#   git clone https://github.com/Genesis-Embodied-AI/Genesis.git && cd Genesis
#   git checkout 31951c3f
#   git apply /path/to/genesis_0.2.1_headless_render.patch
#   pip install -e .
#
# Verify: python -c 'import genesis; print(genesis.__version__)' -> 0.2.1
