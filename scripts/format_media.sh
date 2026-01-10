#!/usr/bin/env bash
for f in media/*.gif; do
  ffmpeg -y -i "$f" \
    -vf "scale=-1:320,pad=420:320:(ow-iw)/2:(oh-ih)/2" \
    -loop 0 "${f%.gif}_tmp.gif" && \
  mv "${f%.gif}_tmp.gif" "$f"
done
