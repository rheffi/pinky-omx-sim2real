"""Copy three explicitly selected regular files from the provided Pinky2 archive.

Never executes archive contents or extracts paths. Creates a new destination only.
"""
import argparse
import hashlib
import json
from pathlib import Path
import tarfile

ROOT = 'pinky_diagnostics_20261022_115032_robot2_check/'
FILES = {
    'pinky_map_new.yaml': ('maps/pinky_map_new.yaml','3f1a1f1c2f3dc28167194c7c43a953b2bb949d3a72c0f4f668521a73d01c13dd'),
    'pinky_map_new.pgm': ('maps/pinky_map_new.pgm','3693a6f7275226cd8c72a764f98a2fc13fcd02ee1eb8d4731a790548a5930aa5'),
    'pinky2_saved_nav2_params.yaml': ('configs/pinky_navigation/params/nav2_params.yaml',None),
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    blobs, provenance = {}, {}
    with tarfile.open(args.archive) as archive:
        for name,(member_name,expected) in FILES.items():
            member = archive.getmember(ROOT+member_name)
            if not member.isfile() or member.size > 1024*1024:
                raise ValueError('Unexpected member type or size: '+member.name)
            blob = archive.extractfile(member).read()
            digest = hashlib.sha256(blob).hexdigest()
            if expected and digest != expected:
                raise ValueError('Baseline hash mismatch: '+name)
            blobs[name] = blob
            provenance[name] = {'member':member.name,'sha256':digest,'bytes':len(blob)}
    args.output.mkdir(parents=True,exist_ok=False)
    for name,blob in blobs.items():
        (args.output/name).write_bytes(blob)
    provenance = {'archive':args.archive.name,'archive_sha256':hashlib.sha256(args.archive.read_bytes()).hexdigest(),
                  'files':provenance,'runtime_override_applied':False,
                  'warning':'Saved baseline, not simulation config. Does not include rear basket footprint. Map-to-layout alignment unmeasured.'}
    (args.output/'provenance.json').write_text(json.dumps(provenance,indent=2,ensure_ascii=False),encoding='utf-8')
    print('Imported 3 verified baseline files; original archive unchanged.')


if __name__ == '__main__':
    main()
