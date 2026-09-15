# Contributing

Thank you for improving the Go2 adapter. Open an issue before large API or dataset-schema changes.

1. Fork the repository and create a focused branch.
2. Install with `pip install -e '.[test]'`.
3. Run `ruff check .` and `pytest`.
4. Add hardware-free tests for behavior changes.
5. Describe the Go2 model, firmware, SDK revision, network topology, camera firmware, and physical
   safety setup for any hardware result. Never label mock-only results as hardware validation.

Keep the default six-element state and three-element action backward compatible. New sensors should
be optional. Any motion-path change needs tests proving clipping and stop behavior.

By participating, you agree to follow [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md) and license your
contribution under Apache-2.0.

