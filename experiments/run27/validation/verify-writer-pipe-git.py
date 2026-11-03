from pathlib import Path
import hashlib
import json
import subprocess

root = Path(__file__).resolve().parents[2]
directory = root/'experiments/writer-pipe-20261102'
files = [p for p in directory.rglob('*') if p.is_file()]
for path in files:
    relative = path.relative_to(root).as_posix()
    assert subprocess.check_output(['git', 'show', 'HEAD:'+relative], cwd=root) == path.read_bytes(), relative
entries = (directory/'SHA256SUMS').read_text().splitlines()
for line in entries:
    digest, relative = line.split('  ', 1)
    assert hashlib.sha256((directory/relative).read_bytes()).hexdigest() == digest, relative
head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip()
remote = subprocess.check_output(['git', 'rev-parse', 'origin/codex/limo-settled-inspection'], cwd=root, text=True).strip()
dirty = subprocess.check_output(['git', 'status', '--porcelain'], cwd=root, text=True)
assert head == remote and not dirty
report = dict(head=head, origin_head=remote, files_equal_git=len(files), sha_entries=len(entries), clean=True)
destination = root/'outputs/setup/writer-pipe-git-verification.json'
with destination.open('x') as stream: json.dump(report, stream, indent=2)
print(json.dumps(report))
