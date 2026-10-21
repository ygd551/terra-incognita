#!/bin/bash
# Resumable download of a registry image (curl -C -, retries forever), then `docker load`.
# Usage: fetch_image.sh [repo] [tag] [workdir]
set -u
REPO=${1:-nvidia/isaac-sim}; TAG=${2:-4.5.0}; DIR=${3:-$(cd "$(dirname "$0")/.." && pwd)/assets/image_blobs}
mkdir -p "$DIR" && cd "$DIR" || exit 1
ACCEPT='application/vnd.docker.distribution.manifest.v2+json'
token() { curl -s "https://nvcr.io/proxy_auth?scope=repository:$REPO:pull" | python3 -c 'import json,sys;print(json.load(sys.stdin)["token"])'; }

until curl -sf -H "Authorization: Bearer $(token)" -H "Accept: $ACCEPT" \
      "https://nvcr.io/v2/$REPO/manifests/$TAG" -o manifest.v2.json; do sleep 10; done

# config + layers as "digest size" lines
python3 - > blobs.txt <<'EOF'
import json
m = json.load(open("manifest.v2.json"))
for b in [m["config"]] + m["layers"]:
    print(b["digest"].split(":")[1], b["size"])
EOF

while read -r hex size; do
  until [ -f "$hex.ok" ]; do
    curl -sS -L -C - --retry 3 --speed-limit 1024 --speed-time 60 \
      -H "Authorization: Bearer $(token)" -o "$hex" \
      "https://nvcr.io/v2/$REPO/blobs/sha256:$hex"
    have=$(stat -c %s "$hex" 2>/dev/null || echo 0)
    echo "$(date +%T) $hex ${have}/${size}"
    if [ "$have" -ge "$size" ]; then
      if [ "$(sha256sum "$hex" | cut -d' ' -f1)" = "$hex" ]; then touch "$hex.ok"; else echo "bad checksum, redownload $hex"; rm -f "$hex"; fi
    else
      sleep 5
    fi
  done
done < blobs.txt

# legacy docker-archive layout; docker load decompresses gzip layers itself
python3 - "$REPO" "$TAG" <<'EOF'
import json, sys
m = json.load(open("manifest.v2.json"))
h = lambda b: b["digest"].split(":")[1]
json.dump([{"Config": h(m["config"]), "RepoTags": [f"nvcr.io/{sys.argv[1]}:{sys.argv[2]}"],
            "Layers": [h(l) for l in m["layers"]]}], open("manifest.json", "w"))
EOF
tar -c manifest.json $(cut -d' ' -f1 blobs.txt) | docker load && echo "LOADED nvcr.io/$REPO:$TAG"
