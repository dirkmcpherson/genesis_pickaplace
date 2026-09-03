import wandb, json, sys, re
api = wandb.Api(timeout=240)
SINCE="2026-08-19T00:00:00"
D={}
# RLPD + DP
gp=[]
for r in api.runs("jambotime/genesis_paper", filters={"createdAt":{"$gte":SINCE}}):
    s=r.summary
    gp.append({"name":r.name,"state":r.state,"created":str(r.created_at)[:19],
      "sweep":{k:s.get(k) for k in ["sweep/demoIC","sweep/randomIC","sweep/n","sweep/demoIC_n","sweep/randomIC_n"]},
      "intrain":{k:s.get(k) for k in ["eval_indist/picked","eval_random/picked","eval_indist/n","eval_random/n","eval/train_step","eval/picked","eval/n"]},
      "sweepkeys":[k for k in s.keys() if k.startswith("sweep")],
      "evalkeys":[k for k in s.keys() if k.startswith("eval")],
      "cfg":{k:r.config.get(k) for k in ["seed","steps","demo_dir","pick_shaping","shaped","gamma","arm","ARM","dataset","kind","ckpt","policy"]},
      "step":s.get("_step")})
D["genesis_paper"]=gp
# r2d
r2={"train":[],"eval":[]}
for r in api.runs("jambotime/r2dreamer_genesis", filters={"createdAt":{"$gte":SINCE}}):
    s=r.summary
    if "-eval-step" in r.name:
        r2["eval"].append({"name":r.name,"created":str(r.created_at)[:19],
            "summary":{k:v for k,v in s.items() if not k.startswith("_") and not isinstance(v,dict)},
            "cfg":{k:r.config.get(k) for k in ["ckpt","checkpoint","step","seed","episodes","n","mode"]}})
    else:
        rows=[x for x in r.scan_history(keys=["env_step","eval/picked","eval/mode_flag","episode/train_picked"],page_size=5000)]
        r2["train"].append({"name":r.name,"state":r.state,"created":str(r.created_at)[:19],
            "env_step":s.get("env_step"),"summary":{k:s.get(k) for k in ["eval/picked","episode/train_picked","env_step","eval/mode_flag"]},
            "rows":rows})
D["r2d"]=r2
# dv3
dv={"train":[],"eval":[]}
for r in api.runs("jambotime/dreamer_v3", filters={"createdAt":{"$gte":SINCE}}):
    s=r.summary
    if "-eval-step" in r.name:
        dv["eval"].append({"name":r.name,"created":str(r.created_at)[:19],"state":r.state,
            "summary":{k:v for k,v in s.items() if not k.startswith("_") and not isinstance(v,dict)}})
    else:
        rows=[x for x in r.scan_history(keys=["_step","eval/picked","eval_success_rate","log_picked","train_return","train_success_rate"],page_size=5000)]
        dv["train"].append({"name":r.name,"state":r.state,"created":str(r.created_at)[:19],"step":s.get("_step"),
            "summary":{k:s.get(k) for k in ["eval/picked","eval_success_rate","log_picked","train_return","train_success_rate","eval_rollouts","eval_return"]},
            "keys":sorted([k for k in s.keys() if not k.startswith("_")])[:80],
            "rows":rows})
D["dv3"]=dv
json.dump(D,open("detail.json","w"),indent=1,default=str)
print("ok")
