### Fixed: the package ships a `py.typed` marker, so type checkers read its annotations

The wheel carried no PEP 561 marker, so a type checker treated every import
from `strands_robots` as `Any` even though the package annotates its public
surface and checks itself with `disallow_untyped_defs`. A downstream project
running `mypy --strict` got `import-untyped` errors for `SimEngine`, `Policy`
and `HardwareDriver`, and a subclass of any of them went unchecked: a wrong
return type or a misspelt override passed silently. `strands_robots/py.typed`
now ships at the package root (the whole package is typed, not partial), and
the project declares the `Typing :: Typed` classifier.
