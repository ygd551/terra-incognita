#!/bin/bash
# Download the Python wheels of the image (listed in wheels.txt) with resumable curl; complete wheels are kept.
# pip through a slow proxy only reached ~0.6 MB/s; curl gets the full ~5 MB/s.
cd "$(dirname "$0")" && mkdir -p wheels
whole() { python3 -c "import zipfile,sys; zipfile.ZipFile(sys.argv[1]).testzip()" "$1" 2>/dev/null; }
while read -r url; do
  f="wheels/$(basename "${url//%2B/+}")"
  until whole "$f" || { curl -sS -L -C - --retry 5 -o "$f" "$url" && whole "$f"; }; do
    echo "retry $f"; sleep 3
  done
  echo "ok $f"
done < wheels.txt
echo FETCH_DONE
