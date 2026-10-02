#!/usr/bin/env python3
"""Estimate clock offset using the best RTT of a persistent hotspot SSH session."""
import argparse
import json
import shlex
import subprocess
import time

parser=argparse.ArgumentParser()
parser.add_argument('--host',default='pi')
args=parser.parse_args()
code='import sys,time\nfor line in sys.stdin:\n print(time.time_ns(),flush=True)'
process=subprocess.Popen(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8',args.host,'python3 -u -c '+shlex.quote(code)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
try:
    samples=[]
    for _ in range(12):
        before=time.time_ns();process.stdin.write('tick\n');process.stdin.flush()
        response=process.stdout.readline()
        after=time.time_ns()
        if not response:raise RuntimeError('Remote clock probe exited')
        remote=int(response);samples.append({'rtt_ms':(after-before)/1e6,'offset_ms':(remote-(before+after)/2)/1e6})
    best=min(samples,key=lambda x:x['rtt_ms'])
    best['uncertainty_ms']=best['rtt_ms']/2
    best['accepted']=abs(best['offset_ms'])+best['uncertainty_ms']<50
    print(json.dumps(best,indent=2))
    if not best['accepted']:raise SystemExit(1)
finally:
    process.stdin.close()
    try:process.wait(timeout=5)
    except subprocess.TimeoutExpired:process.terminate()
