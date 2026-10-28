#!/usr/bin/env bash
# run_analysis.sh: run analyze.py on the host, or inside the go2-3d image when
# the host has no working matplotlib (e.g. numpy 2 with Ubuntu's matplotlib).
#
#   run_analysis.sh --out docs/images/locomotion --rooms agri_rooms TRIAL_DIR...
set -eo pipefail
HERE="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
if python3 -c 'import matplotlib, numpy' 2>/dev/null; then
    exec python3 "${HERE}/analyze.py" "$@"
fi
# Paths (trial dirs, the --out value) are bound at the same place inside the
# container; other option values (--rooms, --resolution) pass unchanged.
mounts=(-v "${HERE}:${HERE}:ro")
args=()
prev=""
for arg in "$@"; do
    if [[ "${arg}" == -* ]] || { [[ "${prev}" == --* ]] && [ "${prev}" != "--out" ] && [ "${prev}" != "--no-plots" ]; }; then
        args+=("${arg}")
    else
        mkdir -p "${arg}" 2>/dev/null || true
        p="$(realpath -m "${arg}")"
        mounts+=(-v "${p}:${p}")
        args+=("${p}")
    fi
    prev="${arg}"
done
exec docker run --rm -u "$(id -u):$(id -g)" -e MPLCONFIGDIR=/tmp "${mounts[@]}" \
    --entrypoint python3 go2-3d:local "${HERE}/analyze.py" "${args[@]}"
