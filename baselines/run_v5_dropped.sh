#!/bin/bash
cd /home/j/workspace/genesis_pickaplace
/home/j/workspace/genesis_pickaplace/.venv-eval/bin/python -u baselines/collect_relaxed.py     --outdir baselines/episodes_raw_v5 --uids 236 237 239 244 245 248 252 269 283 287 294 299 300 306 315 328 233 234 246 247 250 255 256 259 262 266 267 275 278 286 293 295 301 303 318 319 320 321 329 330 333 --attempts 8 > baselines/collect_v5_dropped.log 2>&1
echo "[$(date)] v5 dropped-demo collection done" >> baselines/collect_v5_dropped.log
