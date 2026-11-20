# Firmware and hardware work

Read [the design-stage BOM](hardware/BOM.md) before changing channel assignments, endpoint defaults, force targets or supply assumptions. The [profile trace](hardware/profile-trace.md) identifies unresolved decisions for the current CAD.

Run the host checks from the repository root with `python firmware/tools/run_core_tests.py` and `python firmware/tools/check_app_syntax.py`. The second command compiles against declaration stubs. A target build needs the real ESP32 dependencies. Keep physical validation in a separate, device-specific campaign following [the verification procedure](verification/README.md).

Track specification and implementation work in GitHub Issues. Include the exact revision and identify which of host, target-build and physical validation produced each result.
