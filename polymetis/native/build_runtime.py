"""Build in a NEW directory. No installation, service restart, or robot access."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys


def tree_hash(root):
    value = hashlib.sha256()
    for path in sorted(root.rglob("*")):
        if not path.is_file() or ".git" in path.relative_to(root).parts or "__pycache__" in path.parts:
            continue
        value.update(path.relative_to(root).as_posix().encode() + b"\0" + hashlib.sha256(path.read_bytes()).digest())
    return value.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for argument in ("deps-prefix", "model-prefix", "libfranka-source", "ruckig-source", "output"):
        parser.add_argument("--" + argument, type=Path, required=True)
    parser.add_argument("--jobs", type=int, default=2)
    args = parser.parse_args()
    if not 1 <= args.jobs <= 8:
        parser.error("jobs must be in [1, 8]")
    root = Path(__file__).resolve().parents[1]
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ)
    for key in ("LD_LIBRARY_PATH", "LD_PRELOAD", "PYTHONPATH", "CMAKE_PREFIX_PATH"):
        env.pop(key, None)
    deps = args.deps_prefix.resolve()
    model_prefix = args.model_prefix.resolve()
    model_lib = model_prefix / "lib/x86_64-linux-gnu"
    if not model_lib.is_dir():
        model_lib = model_prefix / "lib"
    libfranka = output / "libfranka-source"
    shutil.copytree(args.libfranka_source, libfranka, ignore=shutil.ignore_patterns(".git"))
    cmake_file = libfranka / "CMakeLists.txt"
    cmake = cmake_file.read_text()
    if "LIBFRANKA_ARCHIVE_VERSION" not in cmake:
        cmake = cmake.replace("set_version_from_git(PACKAGE_VERSION PACKAGE_TAG)", '''if(DEFINED LIBFRANKA_ARCHIVE_VERSION)
  # Official source archives do not contain .git. The build manifest records
  # the archive tag and digest; do not fabricate Git history for packaging.
  set(PACKAGE_VERSION "${LIBFRANKA_ARCHIVE_VERSION}")
  set(PACKAGE_TAG "${LIBFRANKA_ARCHIVE_VERSION}-archive")
else()
  set_version_from_git(PACKAGE_VERSION PACKAGE_TAG)
endif()''')
        cmake_file.write_text(cmake)
    lock = json.loads((root / "native/dependencies.lock.json").read_text())
    if tree_hash(libfranka) != lock["libfranka"]["tree_sha256_after_archive_patch"]:
        raise RuntimeError("libfranka/common sources differ from the reviewed dependency lock")
    if tree_hash(args.ruckig_source) != lock["ruckig"]["tree_sha256"]:
        raise RuntimeError("Ruckig source differs from the reviewed dependency lock")
    commands = []

    def run(command, library_path=None):
        command = [str(item) for item in command]
        commands.append(command)
        child = dict(env)
        if library_path:
            child["LD_LIBRARY_PATH"] = library_path
        subprocess.run(command, env=child, check=True)

    def build(name, source, prefix, definitions, library_path=None):
        directory = output / (name + "-build")
        run(["cmake", "-S", source, "-B", directory, "-DCMAKE_BUILD_TYPE=Release",
             "-DCMAKE_INSTALL_PREFIX=" + str(prefix)] + ["-D" + key + "=" + str(value) for key, value in definitions.items()], library_path)
        run(["cmake", "--build", directory, "-j", args.jobs], library_path)
        run(["ctest", "--test-dir", directory, "--output-on-failure"], library_path)
        run(["cmake", "--install", directory], library_path)

    run([deps / "bin/python", "-m", "unittest", "discover", "-s", root / "native",
         "-p", "test_prepare_station_model.py"], str(deps / "lib"))
    franka_prefix = output / "franka-runtime"
    build("franka", libfranka, franka_prefix, {
        "CMAKE_PREFIX_PATH": model_prefix, "BUILD_TESTS": "OFF", "BUILD_EXAMPLES": "OFF",
        "LIBFRANKA_ARCHIVE_VERSION": lock["libfranka"]["version"]})
    native_libs = ":".join(map(str, [franka_prefix / "lib", model_lib, deps / "lib"]))
    build("native", root / "native", output / "native-runtime", {
        "POLYMETIS_DEPS_PREFIX": deps, "POLYMETIS_MODEL_LIBDIR": model_lib,
        "POLYMETIS_FRANKA_VERSION": lock["libfranka"]["version"],
        "FRANKA_MODEL_TEST_URDF": libfranka / "test/fr3.urdf",
        "Franka_DIR": franka_prefix / "lib/cmake/Franka"}, native_libs)
    torch_dir = subprocess.check_output([str(deps / "bin/python"), "-c",
                                        "import torch; print(torch.utils.cmake_prefix_path + '/Torch')"], env=env, text=True).strip()
    build("servo", root / "servo", output / "server-runtime", {
        "RUCKIG_SOURCE_DIR": args.ruckig_source.resolve(), "BUILD_TORCH_SERVO": "ON", "Torch_DIR": torch_dir})
    build("server", root / "server", output / "server-runtime", {
        "POLYMETIS_DEPS_PREFIX": deps, "POLYMETIS_SERVO_LIBRARY": output / "servo-build/libpolymetis_joint_servo.so",
        "Torch_DIR": torch_dir, "pinocchio_DIR": deps / "lib/cmake/pinocchio"}, str(deps / "lib"))
    (output / "build-commands.json").write_text(json.dumps(commands, indent=2) + "\n")
    source_revision = "source-tree-sha256:" + tree_hash(root)
    torch_lib = Path(torch_dir).parents[2] / "lib"
    server_libs = ":".join(map(str, [output / "server-runtime/lib", torch_lib, deps / "lib"]))
    for role, names, libraries in (
        ("native", ["franka_panda_client", "franka_hand_client", "inspect_franka"], native_libs),
        ("server", ["run_server"], server_libs),
    ):
        command = [sys.executable, root / "native/package_runtime.py", "--output", output / (role + "-bundle"),
                   "--library-path", libraries, "--source-revision", source_revision]
        for name in names:
            command += ["--binary", output / (role + "-runtime/bin") / name]
        run(command)
    test_env = dict(env, CONDA_PREFIX=str(deps),
                    POLYMETIS_TORCH_LIB_DIR=str(output / "server-bundle/lib"),
                    LD_LIBRARY_PATH=str(output / "server-bundle/lib"),
                    PYTHONPATH=str(output / "server-build") + ":" + str(root / "python"))
    subprocess.run([str(deps / "bin/python"), str(root / "servo/test_server_roundtrip.py"),
                    "--server", str(output / "server-bundle/bin/run_server"),
                    "--servo-library", str(output / "server-bundle/lib/libpolymetis_joint_servo.so"),
                    "--urdf", str(root / "data/franka_fr3/fr3_arm.urdf"),
                    "--output", str(output / "server-roundtrip.json")], env=test_env, check=True)
    (output / "source-manifest.json").write_text(json.dumps({"source_revision": source_revision,
        "dependencies": lock, "commands": commands}, indent=2) + "\n")
    print("Build, dependency isolation and synthetic-feedback roundtrip passed. Hardware commissioning remains required.")


if __name__ == "__main__":
    main()
