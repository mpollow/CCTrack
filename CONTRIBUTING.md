<!-- SPDX-License-Identifier: CC-BY-4.0 -->
<!-- Copyright (c) 2026 Martin Pollow -->

# Contributing to CCTrack

Thanks for your interest! Two things make a contribution mergeable:

## 1. DCO sign-off

We use the [Developer Certificate of Origin](https://developercertificate.org)
(DCO) — the same lightweight attestation used by the Linux kernel, Docker, and
GitLab. Every commit must carry a `Signed-off-by:` trailer attesting that you
have the right to submit the code under the project's licence.

The simplest way to add it: configure once globally,

```sh
git config --global format.signoff true
```

…or per-commit with `git commit -s -m "…"`. CI checks every commit in a
pull request for the trailer; if you forgot it, fix the branch with
`git rebase --signoff main` and force-push.

We do **not** require a CLA.

## 2. Licensing

By contributing, you agree that your contributions are licensed under:

- **MIT** for code (`firmware/`, `python/`, `tools/`, `.github/`)
- **CERN-OHL-P v2** for hardware design (`docs/hardware.md`)
- **CC BY 4.0** for all other documentation (`docs/`, every Markdown file)

Add an SPDX header to every new file, e.g. for code:

```c
// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Your Name
```

…and for Markdown: `<!-- SPDX-License-Identifier: CC-BY-4.0 -->`.

## Build & test

See the per-subsystem `README.md` (`firmware/README.md`, etc.) for build
instructions. CI runs the same commands — keep them green.
