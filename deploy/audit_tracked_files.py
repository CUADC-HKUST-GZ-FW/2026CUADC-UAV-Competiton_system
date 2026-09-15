#!/usr/bin/env python3
"""Hash a tracked-file list against a local checkout or deployed device roots."""

import argparse
import hashlib
from pathlib import Path


def parse_mapping(values):
    mappings = {}
    for value in values:
        key, separator, path = value.partition('=')
        if not separator or not key or not path:
            raise ValueError(f'invalid mapping: {value!r}')
        mappings[key.rstrip('/')] = Path(path)
    return mappings


def resolve_path(relative_path, repo_root, root_mappings, exact_mappings):
    if relative_path in exact_mappings:
        return exact_mappings[relative_path]
    for prefix, root in sorted(
        root_mappings.items(), key=lambda item: len(item[0]), reverse=True
    ):
        marker = prefix + '/'
        if relative_path.startswith(marker):
            return root / relative_path[len(marker):]
    return repo_root / relative_path


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--tracked-list', required=True, type=Path)
    parser.add_argument('--repo-root', type=Path, default=Path.cwd())
    parser.add_argument('--root-map', action='append', default=[])
    parser.add_argument('--exact-map', action='append', default=[])
    args = parser.parse_args()

    root_mappings = parse_mapping(args.root_map)
    exact_mappings = parse_mapping(args.exact_map)
    tracked = args.tracked_list.read_text(encoding='utf-8').splitlines()

    for relative_path in tracked:
        if not relative_path:
            continue
        source = resolve_path(
            relative_path,
            args.repo_root,
            root_mappings,
            exact_mappings,
        )
        if not source.is_file():
            print(f'MISSING\t0\t{relative_path}')
            continue
        print(f'{sha256_file(source)}\t{source.stat().st_size}\t{relative_path}')


if __name__ == '__main__':
    main()
