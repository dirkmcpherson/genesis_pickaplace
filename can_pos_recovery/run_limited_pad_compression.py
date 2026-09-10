"""Replay the frozen hand with independently bounded pad compression travel."""
from pathlib import Path
import sys,argparse,runpy
REPO=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path);p.add_argument('--out',type=Path,required=True);p.add_argument('--preset',type=Path,required=True);p.add_argument('--compression-limit',type=float,choices=[0,.0005,.001],required=True)
a=p.parse_args()
import surface_pad_candidate
from finite_compression_pads import FiniteCompressionPads
surface_pad_candidate.SurfacePads=lambda world,tc,depth:FiniteCompressionPads(world,tc,depth,a.compression_limit)
sys.argv=['run_hand_preset.py',str(a.source),'--out',str(a.out),'--preset',str(a.preset)]
runpy.run_path(str(REPO/'can_pos_recovery/run_hand_preset.py'),run_name='__main__')
