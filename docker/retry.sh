#!/usr/bin/env bash
#
# retry <command...> — run a build step up to three times.
#
# Why: this image is linux/amd64 and Apple Silicon builds it under Docker
# Desktop's Rosetta emulation, where a process is occasionally killed with
# "Illegal instruction" (SIGILL) for no reason of its own — dpkg's helpers
# during a big apt-get install are the usual victims (docker/for-mac#7255,
# #7533; Docker calls amd64 emulation "best effort"). apt/dpkg and pip both
# keep consistent state across such a crash, so simply running the same
# command again resumes where it stopped instead of throwing away a
# ten-minute layer. On a native amd64 host the first attempt just succeeds.
set -o pipefail
for attempt in 1 2 3; do
    "$@" && exit 0
    echo "retry: attempt $attempt/3 of '$1' failed; repairing dpkg state and retrying" >&2
    dpkg --configure -a >/dev/null 2>&1 || true   # no-op for non-apt commands
    sleep 3
done
echo "retry: giving up on '$*' after 3 attempts" >&2
exit 1
