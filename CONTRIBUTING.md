# Contributing to Spiderbot

The official project repository is
[ERC-BPGC/SpiderBot](https://github.com/ERC-BPGC/SpiderBot).
Please report Spiderbot bugs and discuss proposed features through its
[issue tracker](https://github.com/ERC-BPGC/SpiderBot/issues).

## Workflow

1. Fork the club repository and create a feature branch.
2. Follow the installation instructions in README.md.
3. Make your changes and run `make check` and the relevant tests.
4. Add user-facing changes to the Upcoming version section in
   `docs/source/changelog.rst`, under Added, Changed, or Fixed.
5. Run `make test` before opening a pull request against the club's `main`.

Keep hardware-specific calibration and policy assumptions documented.
The package retains the `mjlab` name for compatibility; this fork is not
published as a replacement for the upstream mjlab package on PyPI.

## License and upstream attribution

Contributions are licensed under Apache-2.0. Preserve existing copyright and
third-party license notices. See LICENSE and README_mjlab.md for the underlying
framework's attribution and citation.
