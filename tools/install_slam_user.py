"""Extract signed-APT-resolved Humble packages into a user-owned runtime prefix.

No sudo, dpkg installation, apt-source changes, or shell-profile changes.
Run on Ubuntu 22.04 with the configured ROS Humble apt repository.
"""
import concurrent.futures
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import urllib.request

root = Path.home()/'.local/share/micromouse_slam'
cache = root/'archives'
cache.mkdir(parents=True, exist_ok=True)
resolved = subprocess.check_output([
    'apt-get', '--print-uris', '--yes', '--download-only', '--no-install-recommends',
    '-o', 'Debug::NoLocking=1', 'install', 'ros-humble-slam-toolbox'], text=True)
records = []
for line in resolved.splitlines():
    if not line.startswith("'"): continue
    url, name, size, digest = shlex.split(line)
    records.append(dict(url=url, name=name, size=int(size), digest=digest))
if not records:
    raise SystemExit('No downloadable packages: use the installed system slam_toolbox if present.')
print(f'Downloading {len(records)} packages into {root}', flush=True)

def download(rec):
    target = cache/rec['name']
    kind, expected = rec['digest'].split(':', 1)
    kind = {'MD5Sum':'md5', 'SHA256':'sha256', 'SHA512':'sha512'}.get(kind, kind.lower())
    def valid():
        return target.is_file() and target.stat().st_size == rec['size'] and hashlib.new(kind.lower(), target.read_bytes()).hexdigest() == expected
    if not valid():
        part = target.with_suffix('.part')
        request = urllib.request.Request(rec['url'], headers={'User-Agent':'APT-micromouse-local-runtime'})
        with urllib.request.urlopen(request, timeout=90) as resp, part.open('wb') as out:
            while data := resp.read(1024*1024): out.write(data)
        part.replace(target)
    if not valid(): raise RuntimeError('Archive size/checksum mismatch: '+str(target))
    return target

with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
    packages = list(pool.map(download, records))
for package in packages:
    subprocess.run(['dpkg-deb', '-x', str(package), str(root)], check=True)
prefix = root/'opt/ros/humble'
(root/'setup.bash').write_text(
    f'export AMENT_PREFIX_PATH={shlex.quote(str(prefix))}:${{AMENT_PREFIX_PATH:-}}\n'
    f'export LD_LIBRARY_PATH={shlex.quote(str(prefix/"lib"))}:{shlex.quote(str(root/"usr/lib"))}:{shlex.quote(str(root/"usr/lib/x86_64-linux-gnu"))}:${{LD_LIBRARY_PATH:-}}\n'
    f'export PYTHONPATH={shlex.quote(str(prefix/"local/lib/python3.10/dist-packages"))}:${{PYTHONPATH:-}}\n')
(root/'runtime_manifest.json').write_text(json.dumps(records, indent=2))
print('User runtime ready: '+str(prefix), flush=True)
