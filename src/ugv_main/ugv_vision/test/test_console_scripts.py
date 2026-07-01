import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_DIR = ROOT / "ugv_vision"


def _console_scripts():
    setup_tree = ast.parse((ROOT / "setup.py").read_text(), filename=str(ROOT / "setup.py"))
    for node in ast.walk(setup_tree):
        if not isinstance(node, ast.Call):
            continue
        if not getattr(node.func, "id", "") == "setup":
            continue
        for keyword in node.keywords:
            if keyword.arg != "entry_points" or not isinstance(keyword.value, ast.Dict):
                continue
            for key, value in zip(keyword.value.keys, keyword.value.values):
                if not isinstance(key, ast.Constant) or key.value != "console_scripts":
                    continue
                if isinstance(value, ast.List):
                    for item in value.elts:
                        if isinstance(item, ast.Constant) and isinstance(item.value, str):
                            yield item.value


def test_console_script_targets_have_modules_and_main():
    missing = []
    for entry in _console_scripts():
        _, target = entry.split("=", 1)
        module_name, function_name = target.strip().split(":", 1)
        relative_module = module_name.removeprefix("ugv_vision.").replace(".", "/") + ".py"
        module_path = PACKAGE_DIR / relative_module
        if not module_path.exists():
            missing.append(f"{entry}: missing module {module_path.relative_to(ROOT)}")
            continue
        tree = ast.parse(module_path.read_text(), filename=str(module_path))
        has_function = any(isinstance(node, ast.FunctionDef) and node.name == function_name for node in tree.body)
        if not has_function:
            missing.append(f"{entry}: missing function {function_name}")
    assert not missing, "\n".join(missing)
