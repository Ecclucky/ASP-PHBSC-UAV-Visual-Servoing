#!/usr/bin/env python3
"""Static release checks; no ROS, hardware access, or controller simulation.

Run from any directory: python3 scripts/validate_workspace.py
Requires PyYAML. This checks the shipped paper configuration, not arbitrary
retuned experiment configurations. It does not replace a ROS/catkin build.
"""

import argparse
import math
from pathlib import Path
import re
import shlex
import subprocess
import sys
import xml.etree.ElementTree as ET

try:
    import yaml
except ImportError:
    sys.exit("PyYAML is required: python3 -m pip install PyYAML")


PACKAGES = {
    "asp_phbsc", "motion_cap_to_odom", "px4ctrl", "record_curves",
    "quadrotor_msgs", "uav_utils", "visual_target", "vrpn_client_ros",
}
EXTERNAL_LAUNCH_PACKAGES = {"mavros", "topic_tools"}
SOURCE_SUFFIXES = {".h", ".hpp", ".c", ".cc", ".cpp"}


class ValidationError(Exception):
    pass


def require(condition, message):
    if not condition:
        raise ValidationError(message)


def read(path):
    return path.read_text(encoding="utf-8-sig")


class UniqueKeyLoader(yaml.SafeLoader):
    """Reject silently overwritten YAML keys, including nested keys."""


def unique_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        require(isinstance(key, str), "YAML configuration keys must be strings")
        require(key not in result, "duplicate YAML key: " + key)
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping
)


def load_yaml(path):
    value = yaml.load(read(path), Loader=UniqueKeyLoader)
    require(isinstance(value, dict), str(path) + ": expected a YAML mapping")
    return value


def match_value(actual, expected, location):
    if isinstance(expected, dict):
        require(isinstance(actual, dict), location + ": expected mapping")
        require(set(actual) == set(expected), location + ": unexpected or missing keys")
        for key in expected:
            match_value(actual[key], expected[key], location + "/" + key)
    elif isinstance(expected, list):
        require(isinstance(actual, list) and len(actual) == len(expected),
                location + ": incorrect list length")
        for index, value in enumerate(expected):
            match_value(actual[index], value, location + "/" + str(index))
    else:
        if type(expected) is float:
            valid = type(actual) in (int, float) and math.isfinite(actual)
            valid = valid and math.isclose(actual, expected, rel_tol=0, abs_tol=1e-12)
        else:
            valid = type(actual) is type(expected) and actual == expected
        require(valid, "{}: expected {!r}, found {!r}".format(location, expected, actual))


def check_configuration(root, require_model=False):
    directory = root / "src/asp_phbsc/config"
    common = load_yaml(directory / "common.yaml")
    model_keys = {"disturbance_dynamics", "disturbance_output", "augmented_observer_gain"}
    require(model_keys <= set(common), "common.yaml lacks explicit augmented-model fields")
    match_value({key: value for key, value in common.items() if key not in model_keys}, {
        "control_period_s": 0.01, "measurement_period_s": 0.05,
        "feedback_gain": [4.0, 2.0], "paper_observer_gain": [-0.6, -6.6],
        "delta_0": 0.02, "delta_1": 0.02, "measurement_delay_steps": 10,
        "max_trigger_interval_steps": 20, "prediction_steps": 100,
        "dos": {"enabled": False, "attack_windows_steps": []},
    }, "common.yaml")
    aw = common["disturbance_dynamics"]
    cw = common["disturbance_output"]
    gain = common["augmented_observer_gain"]
    model_ready = not (aw == [] and cw == [] and gain == [])
    if not model_ready:
        require(not require_model,
                "the manuscript omits experimental A_w, C_w and the full augmented L; fill the three model fields before ROS startup")
        print("PENDING: experimental A_w, C_w and augmented L are unspecified; ROS startup is intentionally disabled")
    else:
        require(isinstance(aw, list) and len(aw) > 0, "disturbance_dynamics must be a nonempty matrix")
        dimension = len(aw)
        require(all(isinstance(row, list) and len(row) == dimension for row in aw),
                "disturbance_dynamics must be square")
        require(isinstance(cw, list) and len(cw) == dimension,
                "disturbance_output length must equal the disturbance state dimension")
        require(isinstance(gain, list) and len(gain) == dimension + 2,
                "augmented_observer_gain length must be 2 + disturbance state dimension")
        values = [value for row in aw for value in row] + cw + gain
        require(all(type(value) in (int, float) and math.isfinite(value) for value in values),
                "augmented model contains a nonnumeric or nonfinite value")
    demo = load_yaml(directory / "synthetic_demo.yaml")
    require(set(demo) == {"dos"} and isinstance(demo["dos"], dict),
            "synthetic_demo.yaml must only override the dos mapping")
    require(set(demo["dos"]) == {"enabled", "attack_windows_steps"},
            "synthetic_demo.yaml: unexpected or missing dos keys")
    require(demo["dos"]["enabled"] is True, "synthetic demo must explicitly enable DoS")
    windows = demo["dos"]["attack_windows_steps"]
    require(isinstance(windows, list) and windows, "synthetic demo needs explicit attack windows")
    for case_name, period, bound, wind in (
        ("case1", 8.0, 60, False), ("case2", 4.5, 40, True)
    ):
        case = load_yaml(directory / (case_name + ".yaml"))
        match_value(case, {
            "target_radius_m": 0.67, "target_period_s": period,
            "max_consecutive_dos_steps": bound, "wind_disturbance": wind,
        }, case_name + ".yaml")
        require(common["prediction_steps"] >= common["max_trigger_interval_steps"] + bound,
                case_name + ": prediction buffer is shorter than the configured bound")
        previous_end = -1
        run_start = -1
        for window in windows:
            require(isinstance(window, list) and len(window) == 2
                    and all(type(value) is int for value in window),
                    "attack windows must be pairs of integer control steps")
            start, end = window
            require(0 < start < end <= 2147483647,
                    "attack window must satisfy 0 < start < end; the paper assumes successful reception at step 0")
            require(start >= previous_end, "attack windows overlap or are not sorted")
            if start != previous_end:
                run_start = start
            require(end - run_start <= bound,
                    case_name + ": adjacent attack windows exceed the consecutive DoS bound")
            previous_end = end


def cmake_commands(path, project_name):
    # The repository uses literal targets and source lists. This is a static
    # checker for those declarations, not a general CMake interpreter.
    source = re.sub(r"#.*", "", read(path))
    source = source.replace("${PROJECT_NAME}", project_name)
    return [(name.lower(), shlex.split(body)) for name, body in
            re.findall(r"(?ms)^\s*(\w+)\s*\((.*?)\)", source)]


def discover_packages(root):
    packages = {}
    for manifest in sorted((root / "src").rglob("package.xml")):
        document = ET.parse(manifest).getroot()
        name = document.findtext("name")
        require(name and name not in packages, "missing or duplicate package name: " + str(manifest))
        require(manifest.parent.name == name, "package directory/name mismatch: " + str(manifest))
        dependencies = {entry.text.strip() for entry in document
                        if entry.tag == "depend" or entry.tag.endswith("_depend")}
        build_dependencies = {entry.text.strip() for entry in document
                              if entry.tag in {"depend", "build_depend"}}
        runtime_dependencies = {entry.text.strip() for entry in document
                                if entry.tag in {"depend", "exec_depend", "run_depend"}}
        commands = cmake_commands(manifest.parent / "CMakeLists.txt", name)
        require(any(command == "project" and args and args[0] == name
                    for command, args in commands), name + ": CMake project/package mismatch")
        require("catkin" in dependencies, name + ": missing catkin build-tool dependency")
        targets = set()
        components = set()
        for command, args in commands:
            if command == "find_package" and args[:1] == ["catkin"] and "COMPONENTS" in args:
                components.update(args[args.index("COMPONENTS") + 1:])
            if command in {"add_executable", "add_library", "catkin_add_gtest"}:
                require(args, name + ": empty CMake target declaration")
                if command != "add_library":
                    targets.add(args[0])
                for source in args[1:]:
                    if Path(source).suffix in SOURCE_SUFFIXES:
                        require((manifest.parent / source).is_file(),
                                name + ": missing target source " + source)
            if command == "catkin_install_python" and args[:1] == ["PROGRAMS"]:
                for script in args[1:args.index("DESTINATION")]:
                    require((manifest.parent / script).is_file(), name + ": missing script " + script)
                    targets.add(Path(script).name)
        require(components <= build_dependencies,
                name + ": CMake catkin dependencies missing from package.xml: "
                + ", ".join(sorted(components - build_dependencies)))
        if (manifest.parent / "launch").is_dir():
            require(any(command == "install" and args[:1] == ["DIRECTORY"]
                        and any(value.rstrip("/") == "launch" for value in args[1:])
                        for command, args in commands), name + ": launch directory is not installed")
        packages[name] = {"path": manifest.parent, "dependencies": dependencies,
                          "runtime_dependencies": runtime_dependencies,
                          "components": components, "targets": targets, "commands": commands}
    require(set(packages) == PACKAGES,
            "expected exactly the eight retained ROS packages; missing={}, extra={}".format(
                sorted(PACKAGES - set(packages)), sorted(set(packages) - PACKAGES)))
    for name, package in packages.items():
        for source in package["path"].rglob("*"):
            if source.is_file() and source.suffix in SOURCE_SUFFIXES:
                headers = re.findall(r'^\s*#\s*include\s*[<"]([^/>"]+)/', read(source), re.M)
                for dependency in set(headers) & PACKAGES - {name}:
                    require(dependency in package["dependencies"] and dependency in package["components"],
                            "{} includes {} headers without both manifest/CMake dependencies".format(source, dependency))
    return packages


FIND = re.compile(r"\$\(find\s+([\w-]+)\)")
ARG = re.compile(r"\$\(arg\s+([\w-]+)\)")


def check_launches(root, packages):
    for owner, package in packages.items():
        for launch in sorted(package["path"].rglob("*.launch")):
            document = ET.parse(launch).getroot()
            require(document.tag == "launch", str(launch) + ": invalid launch root")
            defaults = {arg.attrib["name"]: arg.get("default", arg.get("value", ""))
                        for arg in document.findall("arg")}
            for element in document.iter():
                references = set()
                for value in element.attrib.values():
                    references.update(FIND.findall(value))
                    require(set(ARG.findall(value)) <= set(defaults),
                            str(launch) + ": undeclared launch argument in " + value)
                if element.tag == "node":
                    references.add(element.get("pkg", ""))
                    node_package = element.get("pkg")
                    if node_package in packages:
                        require(element.get("type") in packages[node_package]["targets"],
                                str(launch) + ": unresolved node target " + str(element.attrib))
                for dependency in references:
                    require(dependency in packages or dependency in EXTERNAL_LAUNCH_PACKAGES,
                            str(launch) + ": unknown/removed package " + dependency)
                    require(dependency == owner or dependency in package["runtime_dependencies"],
                            str(launch) + ": package.xml lacks runtime dependency " + dependency)
                file_value = element.get("file")
                if file_value is None:
                    continue
                if any(name in EXTERNAL_LAUNCH_PACKAGES for name in FIND.findall(file_value)):
                    continue  # External ROS packages are not installed by this offline check.
                for case_name in ("case1", "case2"):
                    arguments = dict(defaults, case=case_name)
                    resolved = FIND.sub(lambda match: str(packages[match.group(1)]["path"]), file_value)
                    resolved = ARG.sub(lambda match: arguments.get(match.group(1), ""), resolved)
                    require("$(" not in resolved, str(launch) + ": unsupported file substitution " + resolved)
                    target = Path(resolved)
                    if not target.is_absolute():
                        target = launch.parent / target
                    require(target.is_file(), str(launch) + ": referenced file does not exist: " + resolved)
                    if element.tag == "include":
                        accepted = {arg.get("name") for arg in ET.parse(target).getroot().findall("arg")}
                        passed = {arg.get("name") for arg in element.findall("arg")}
                        require(passed <= accepted, str(launch) + ": unknown forwarded arguments "
                                + ", ".join(sorted(passed - accepted)))
    common = "$(find asp_phbsc)/config/common.yaml"
    controller = ET.parse(root / "src/asp_phbsc/launch/controller.launch").getroot()
    experiment = ET.parse(root / "src/asp_phbsc/launch/experiment.launch").getroot()
    for document, node_type in ((controller, "asp_phbsc_node"), (experiment, "visual_target_node")):
        nodes = [node for node in document.iter("node") if node.get("type") == node_type]
        require(len(nodes) == 1 and any(param.get("file") == common for param in nodes[0].findall("rosparam")),
                node_type + ": must load the shared control/measurement configuration")


def check_messages(packages):
    package = packages["quadrotor_msgs"]
    generated = set()
    message_dependencies = set()
    for command, args in package["commands"]:
        if command == "add_message_files":
            generated.update(value for value in args if value.endswith(".msg"))
        elif command == "generate_messages" and "DEPENDENCIES" in args:
            message_dependencies.update(args[args.index("DEPENDENCIES") + 1:])
    require(generated, "quadrotor_msgs: no generated messages declared")
    primitives = {"bool", "byte", "char", "int8", "uint8", "int16", "uint16", "int32", "uint32",
                  "int64", "uint64", "float32", "float64", "string", "time", "duration"}
    for message in generated:
        source = package["path"] / "msg" / message
        require(source.is_file(), "missing generated message: " + message)
        for line_number, line in enumerate(read(source).splitlines(), 1):
            fields = line.split("#", 1)[0].split()
            if not fields:
                continue
            field_type = fields[0].split("[", 1)[0]
            if field_type in primitives:
                continue
            if field_type == "Header":
                dependency, nested = "std_msgs", "Header"
            elif "/" in field_type:
                dependency, nested = field_type.split("/", 1)
            else:
                dependency, nested = "quadrotor_msgs", field_type
            location = "{}:{}".format(source, line_number)
            if dependency == "quadrotor_msgs":
                require(nested + ".msg" in generated,
                        location + ": nested message is not generated: " + nested)
            else:
                require(dependency in message_dependencies and dependency in package["dependencies"],
                        location + ": undeclared message dependency: " + dependency)
    for owner in packages.values():
        for source in owner["path"].rglob("*"):
            if source.is_file() and source.suffix in SOURCE_SUFFIXES:
                for header in re.findall(r'#\s*include\s*[<"]quadrotor_msgs/([^>"]+)[>"]', read(source)):
                    if (package["path"] / "include/quadrotor_msgs" / header).is_file():
                        continue  # Hand-written encoder/decoder/utility headers.
                    require(Path(header).suffix == ".h" and Path(header).with_suffix(".msg").name in generated,
                            str(source) + ": header has no generated .msg: " + header)


def release_files(root):
    # In a Git checkout, inspect tracked and nonignored files. A local catkin
    # build may coexist with source without making its ignored output a release.
    if (root / ".git").exists():
        result = subprocess.run(["git", "-C", str(root), "ls-files", "--cached", "--others",
                                 "--exclude-standard", "-z"], capture_output=True, check=True)
        return [root / name.decode("utf-8") for name in result.stdout.split(b"\0") if name]
    return [path for path in root.rglob("*") if path.is_file()]


def check_release_tree(root):
    forbidden_dirs = {"build", "devel", "install", "logs", ".catkin_tools", "__pycache__", ".vscode", ".idea"}
    forbidden_suffixes = {".pyc", ".pyo", ".o", ".obj", ".so", ".a", ".exe", ".dll", ".pdb",
                          ".bag", ".mcap", ".ulg", ".csv", ".zip", ".log"}
    legacy = re.compile(r"(?i)\b(?:cmake_utils|phbsc_event|phbsc|iibdo|aic|pid_ctrl|mpc_ctrl|rl_ctrl)\b")
    for path in release_files(root):
        relative = path.relative_to(root)
        require(not (set(relative.parts) & forbidden_dirs) and path.suffix.lower() not in forbidden_suffixes
                and not path.name.endswith("~") and path.name != "CMakeCache.txt",
                "generated/local artifact in release tree: " + str(relative))
        if path.name in {"package.xml", "CMakeLists.txt"} or path.suffix in {".launch", ".sh"}:
            source = read(path)
            if path.name == "package.xml":
                manifest = ET.fromstring(source)
                source = "\n".join(entry.text or "" for entry in manifest
                                   if entry.tag == "name" or entry.tag == "depend"
                                   or entry.tag.endswith("_depend"))
            elif path.suffix == ".launch":
                source = re.sub(r"<!--.*?-->", "", source, flags=re.S)
            source = re.sub(r"(?m)#.*$", "", source)
            match = legacy.search(source)
            require(match is None, "legacy package reference in {}: {}".format(relative, match.group() if match else ""))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--require-model", action="store_true",
                        help="also reject the intentionally empty experimental disturbance model")
    args = parser.parse_args()
    root = args.workspace.resolve()
    try:
        check_configuration(root, args.require_model)
        print("PASS: paper parameters, case metadata, and synthetic attack windows")
        packages = discover_packages(root)
        print("PASS: eight package manifests, CMake targets, source paths, and internal dependencies")
        check_launches(root, packages)
        print("PASS: launch targets, declared runtime dependencies, files, and shared configuration")
        check_messages(packages)
        print("PASS: quadrotor_msgs includes resolve to declared messages or source headers")
        check_release_tree(root)
        print("PASS: no legacy build/launch references or release artifacts")
    except (ValidationError, OSError, ValueError, ET.ParseError, yaml.YAMLError,
            subprocess.SubprocessError) as error:
        print("FAIL: " + str(error), file=sys.stderr)
        return 1
    print("Static checks passed. ROS compilation, flight behavior, and manuscript results are not verified.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
