#!/bin/bash
# Robomimic leg: fetch the Can low-dim files (plan §1, HF repo robomimic/robomimic_datasets) to
# $LAB/robomimic_data/v1.5/can/{ph,mh,mg,paired}/ and record sha256 + bytes in sha256.txt. Data never enter git.
#   ssh pax 'nohup bash ~/genesis_pickaplace/cluster/robomimic/download_data.sh > $LAB/robomimic_download.log 2>&1 &'
set -uo pipefail
LAB=${LAB:-/cluster/tufts/shortlab/jstale02}
D=$LAB/robomimic_data/v1.5/can
HF=https://huggingface.co/datasets/robomimic/robomimic_datasets/resolve/main/v1.5/can
mkdir -p $D/ph $D/mh $D/mg $D/paired
fetch() { local sub=$1 name=$2; local out=$D/$sub/$name
  if [ -s "$out" ] && [ -f "$out.ok" ]; then echo "== $sub/$name present, skip"; return 0; fi
  echo "== $sub/$name $(date -Is)"; rm -f "$out.ok"
  curl -L --retry 5 --retry-delay 10 -C - -o "$out" "$HF/$sub/$name" && touch "$out.ok" || { echo "!! download failed $sub/$name"; return 1; }
}
fetch ph low_dim_v15.hdf5 || exit 1
fetch mh low_dim_v15.hdf5 || exit 1
fetch mg low_dim_sparse_v15.hdf5 || exit 1
fetch paired low_dim_v15.hdf5 || exit 1     # secondary arm PH200+Pbad100 (plan §3); 42 MB
cd $D && ( for f in ph/low_dim_v15.hdf5 mh/low_dim_v15.hdf5 mg/low_dim_sparse_v15.hdf5 paired/low_dim_v15.hdf5; do
  printf '%s  %s  %s\n' "$(sha256sum $f | cut -d" " -f1)" "$(stat -c %s $f)" "$f"; done ) | tee sha256.txt
echo "# download end $(date -Is)"
