import wandb, json, sys
api = wandb.Api(timeout=240)
SINCE="2026-08-19T00:00:00"
OUT={"dv3_eval":[], "dv3_train":{}, "r2d_train":{}, "rlpd_train":{}}
def hist(r, key, xkeys=("_step",)):
    try:
        rows=r.history(keys=[key,*xkeys], samples=4000, pandas=False)
        return [[x.get(e) for e in xkeys]+[x.get(key)] for x in rows if x.get(key) is not None]
    except Exception as e:
        try:
            rows=r.history(keys=[key], samples=4000, pandas=False)
            return [[x.get("_step")]+[x.get(key)] for x in rows if x.get(key) is not None]
        except Exception as e2:
            return {"error":str(e2)[:200]}
for r in api.runs("jambotime/dreamer_v3", filters={"createdAt":{"$gte":SINCE}}):
    if "smoke" in r.name: continue
    if "-eval-step" in r.name:
        full={k:(v if not isinstance(v,dict) else "<dict>") for k,v in r.summary.items()}
        try: h=r.history(samples=50, pandas=False)
        except Exception as e: h=[{"error":str(e)[:200]}]
        OUT["dv3_eval"].append({"name":r.name,"created":str(r.created_at)[:19],"summary":full,"hist":h[:20],"config":{k:v for k,v in r.config.items() if not isinstance(v,(dict,list))}})
        print("dv3e",r.name,len(h),[k for k in (h[0].keys() if h else [])][:20],file=sys.stderr)
    else:
        d={"created":str(r.created_at)[:19],"state":r.state}
        for k in ["eval/picked","log_picked","train_return","train_success_rate","eval/contact","log_task_success"]:
            d[k]=hist(r,k)
        OUT["dv3_train"][r.name+"@"+str(r.created_at)[:19]]=d
        print("dv3t",r.name,{k:(len(v) if isinstance(v,list) else v) for k,v in d.items() if k not in("created","state")},file=sys.stderr)
for r in api.runs("jambotime/r2dreamer_genesis", filters={"createdAt":{"$gte":SINCE}}):
    if "-eval-step" in r.name or r.name.endswith("_s0"): continue
    d={"created":str(r.created_at)[:19],"state":r.state,"env_step":r.summary.get("env_step")}
    for k in ["eval/picked","episode/train_picked","eval/mode_flag"]:
        d[k]=hist(r,k,("_step","env_step"))
    OUT["r2d_train"][r.name]=d
    print("r2d",r.name,{k:(len(v) if isinstance(v,list) else v) for k,v in d.items() if k not in("created","state","env_step")},file=sys.stderr)
for r in api.runs("jambotime/genesis_paper", filters={"createdAt":{"$gte":SINCE}}):
    if "RLPD" not in r.name or (r.summary.get("_step") or 0)<100000: continue
    d={"created":str(r.created_at)[:19]}
    for k in ["eval_indist/picked","eval_random/picked","rollout/ep_rew_mean","train/ep_rew_mean","rollout/success_rate","eval/picked"]:
        d[k]=hist(r,k,("_step","eval/train_step"))
    OUT["rlpd_train"][r.name+"@"+str(r.created_at)[:19]]=d
    print("rlpd",r.name,{k:(len(v) if isinstance(v,list) else v) for k,v in d.items() if k!="created"},file=sys.stderr)
json.dump(OUT,open("curves.json","w"),indent=0,default=str)
print("ok")
