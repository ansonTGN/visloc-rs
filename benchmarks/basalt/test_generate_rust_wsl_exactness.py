import importlib.util
import unittest
from pathlib import Path


SCRIPT = Path(__file__).with_name("generate_rust_wsl_exactness.py")
SPEC = importlib.util.spec_from_file_location("generate_rust_wsl_exactness", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class RustWslExactnessSourceClosureTests(unittest.TestCase):
    def test_example_cli_is_part_of_tracked_source_closure(self) -> None:
        self.assertIn(
            "examples/basalt_euroc_vio_demo.rs", MODULE.TRACKED_SOURCE_PATHS
        )


if __name__ == "__main__":
    unittest.main()
