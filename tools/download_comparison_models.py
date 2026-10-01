#!/usr/bin/env python3
"""Download pinned public comparison weights; do not upload camera photographs."""
import hashlib
import json
import subprocess
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]

def main():
    sources=json.loads((ROOT/'validation/hf-model-sources.json').read_text())['models']
    out=ROOT/'log/hf-models';out.mkdir(parents=True,exist_ok=True)
    for item in sources:
        name=item.get('local_file',item['file'])
        if Path(name).name!=name or not name.endswith('.pt'):raise ValueError('Expected a simple weight filename')
        target=out/name
        if target.exists() and hashlib.sha256(target.read_bytes()).hexdigest()==item['sha256']:continue
        url=f"https://huggingface.co/{item['repo']}/resolve/{item['revision']}/{item['file']}?download=true"
        subprocess.run(['curl','-fsSL','--retry','2','--connect-timeout','10','--max-time','180',url,'-o',str(target)],check=True)
        if hashlib.sha256(target.read_bytes()).hexdigest()!=item['sha256']:raise RuntimeError(f'Hash mismatch: {name}')
        print('Downloaded and verified',name,flush=True)

if __name__=='__main__':main()
