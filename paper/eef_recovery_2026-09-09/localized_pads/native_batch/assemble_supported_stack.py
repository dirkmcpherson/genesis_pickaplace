"""Assemble only exact-version CUDA dependencies into the isolated Torch2.8 overlay."""
from pathlib import Path
from importlib import metadata
from packaging.requirements import Requirement
import json,os,subprocess
D=Path(__file__).resolve().parent
T=Path('/tmp/genesis-cuda28');E=Path('/tmp/genesis-cuda28-extra');OLD=Path('/home/james/workspace/genesis_sim2real/venv/lib/python3.10/site-packages')
assert (T/'torch').exists() and (E/'triton').exists(), 'Wait for both installers to finish'
roots=[T,E,OLD];installed={}
for root in roots:
 for dist in metadata.distributions(path=[str(root)]):
  name=dist.metadata['Name'].lower().replace('_','-')
  installed.setdefault(name,[]).append((root,dist))
torch=next(dist for root,dist in installed['torch'] if root==T)
records=[]
for raw in torch.requires:
 req=Requirement(raw)
 if req.marker and not req.marker.evaluate():continue
 if not (req.name.startswith('nvidia-') or req.name=='triton'):continue
 matches=[(root,dist) for root,dist in installed.get(req.name,[]) if dist.version in req.specifier]
 assert matches,(req.name,str(req.specifier))
 root,dist=matches[0];records.append(dict(name=req.name,version=dist.version,source=str(root)))
 if root==T:continue
 info=Path(dist._path);dest=T/info.name
 if not dest.exists():dest.symlink_to(info,target_is_directory=True)
 if req.name=='triton':
  if not (T/'triton').exists():(T/'triton').symlink_to(root/'triton',target_is_directory=True)
 else:
  parts={f.parts[1] for f in dist.files if len(f.parts)>1 and f.parts[0]=='nvidia' and f.parts[1]!='__init__.py' and not f.parts[1].startswith('__')}
  for name in parts:
   src=root/'nvidia'/name
   if not src.is_dir():continue
   dest=T/'nvidia'/name
   if not dest.exists():dest.symlink_to(src,target_is_directory=True)
probe=subprocess.run(['/tmp/genesis-contact-1.4/bin/python','-c','import torch,genesis; assert torch.__version__.startswith("2.8.0+cu126"); assert torch.cuda.is_available(); print(torch.__version__,genesis.__version__,torch.cuda.get_device_name(0))'],env=dict(os.environ,PYTHONPATH=str(T)),capture_output=True,text=True)
report=dict(torch_version=torch.version,dependencies=records,probe=dict(returncode=probe.returncode,stdout=probe.stdout,stderr=probe.stderr),qualification='Only matching exact-version existing CUDA libraries reused. New Torch/cusparselt/cudnn/nccl/triton packages installed in /tmp. Existing environments untouched.')
(D/'supported_stack.json').write_text(json.dumps(report,indent=2));assert probe.returncode==0,report
print(probe.stdout,flush=True)
