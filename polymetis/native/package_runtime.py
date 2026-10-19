"""Freeze a verified ELF dependency closure; never alter an existing environment."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess


SYSTEM = re.compile(r"^(?:ld-linux.*|lib(?:c|m|pthread|dl|rt|resolv|util)\.so(?:\..*)?)$")


def digest(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def closure(binary, library_path):
    env = dict(os.environ, LD_LIBRARY_PATH=library_path)
    env.pop("LD_PRELOAD", None)
    result = subprocess.run(["ldd", "-r", str(binary)], env=env, capture_output=True, text=True, check=True)
    if "not found" in result.stdout or "undefined symbol" in result.stdout + result.stderr:
        raise RuntimeError("Incomplete or incompatible dependency closure: " + result.stdout + result.stderr)
    dependencies = {}
    for line in result.stdout.splitlines():
        match = re.match(r"\s*(\S+) => (/\S+) \(", line)
        if match:
            name, filename = match.groups()
            resolved = Path(filename).resolve()
            dynamic = subprocess.check_output(["readelf", "-d", str(resolved)], text=True)
            soname = re.search(r"\(SONAME\).*\[(.*?)\]", dynamic)
            # Some vendor packages (e.g. MKL) use an unversioned DT_NEEDED.
            # A versioned dependency must always match its exact SONAME.
            unversioned_link = name.endswith(".so") and soname and soname[1].startswith(name + ".")
            if soname and soname[1] != name and not unversioned_link:
                raise RuntimeError("SONAME mismatch: {} resolves to {} ({})".format(name, resolved, soname[1]))
            dependencies[name] = resolved
    return dependencies


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", action="append", type=Path, required=True)
    parser.add_argument("--library-path", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--source-revision", required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "bin").mkdir()
    (args.output / "lib").mkdir()
    dependencies, files, system = {}, {}, {}
    for binary in args.binary:
        for name, source in closure(binary, args.library_path).items():
            if name in dependencies and digest(source) != digest(dependencies[name]):
                raise RuntimeError("Conflicting library implementations: " + name)
            dependencies[name] = source
        target = args.output / "bin" / binary.name
        shutil.copy2(binary, target)
        files[str(target.relative_to(args.output))] = {"sha256": digest(target), "source": str(binary)}
    for name, source in sorted(dependencies.items()):
        if SYSTEM.match(name):
            system[name] = {"path": str(source), "sha256": digest(source)}
        else:
            target = args.output / "lib" / name
            shutil.copy2(source, target)
            files[str(target.relative_to(args.output))] = {"sha256": digest(target), "source": str(source)}
    # MKL CPU dispatch libraries are loaded with dlopen, not listed in
    # DT_NEEDED. Freeze the dispatch set from the SAME package as mkl_core.
    mkl_core = dependencies.get("libmkl_core.so")
    if mkl_core:
        patterns = ("libmkl_avx*.so.*", "libmkl_def.so.*", "libmkl_mc*.so.*", "libmkl_vml_*.so.*")
        plugins = {path for pattern in patterns for path in mkl_core.parent.glob(pattern)}
        if not plugins:
            raise RuntimeError("MKL core is present but its CPU dispatch libraries are missing")
        for source in sorted(plugins):
            target = args.output / "lib" / source.name
            shutil.copy2(source, target)
            files[str(target.relative_to(args.output))] = {"sha256": digest(target), "source": str(source),
                                                         "load": "MKL CPU dispatch"}
    # Re-resolve using ONLY the new bundle, and reject any accidental fallback.
    for binary in args.binary:
        for name, resolved in closure(args.output / "bin" / binary.name, str(args.output / "lib")).items():
            if not SYSTEM.match(name) and resolved.parent != (args.output / "lib").resolve():
                raise RuntimeError("Dependency escaped the isolated bundle: " + str(resolved))
    manifest = {"schema": "polymetis.runtime/v1", "source_revision": args.source_revision,
                "files": files, "system_libraries": system}
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({"bundle": str(args.output), "files": len(files), "soname_and_relocations_verified": True}))


if __name__ == "__main__":
    main()
