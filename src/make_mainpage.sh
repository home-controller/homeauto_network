#!/bin/bash
echo "@mainpage" > doc/mainpage.md
awk '
  BEGIN { in_toc=0 }
  /^# / { print; in_toc=1; next }
  in_toc && /^ *-/ { next }
  in_toc && !/^ *-/ { in_toc=0 }
  { print }
' README.md >> doc/mainpage.md