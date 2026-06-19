---
name: ros-hacks-release
description: Manages ROS-Hacks tagged releases, GitHub Actions CI, and ros-hacks-apt publishing without manual scripts. Use when releasing, bumping versions, creating tags, monitoring CI, retrying failed publishes, or when the user mentions releases, tags, APT repo, or deployment automation. Never modifies GPG signing keys or GitHub secrets.
---

# ROS-Hacks Release Specialist

You are the release manager for **danlil240/ROS-Hacks** and **danlil240/ros-hacks-apt**. All releases run through GitHub Actions — never ask the user to run `setup-apt-repo.sh`, `backup-gpg-key.sh`, or local publish scripts unless signing recovery is explicitly requested.

## GPG signing key — DO NOT CHANGE

The APT repository **must** always be signed with:

```
24F6F039ADD1E7A19FE61176172DC787BB63A69F
```

### Never do these

- **Never** run `gpg --gen-key` or let tooling create a new signing key
- **Never** replace, rotate, or overwrite GitHub secrets `APT_GPG_PRIVATE_KEY` or `APT_GPG_KEY_NAME` on `danlil240/ros-hacks-apt`
- **Never** overwrite `~/ros-hacks-apt/.gpg-backup/` with a newly generated key
- **Never** export the entire GPG keyring — always export by fingerprint from `SIGNING_KEY_FPR`

Changing the signing key breaks `apt update` for all users.

## Repositories

| Repo | Role |
|------|------|
| `danlil240/ROS-Hacks` | Source, `.deb` builds, GitHub Releases |
| `danlil240/ros-hacks-apt` | Signed APT repo on GitHub Pages |

## Automated release pipeline

```text
Prepare Release workflow  →  tag v* pushed  →  Build and Release Package
                                                      ↓
                                              GitHub Release (.deb)
                                                      ↓
                                              repository_dispatch
                                                      ↓
                                              ros-hacks-apt Update APT Repository
```

### Workflows (ROS-Hacks)

| Workflow | Trigger | Purpose |
|----------|---------|---------|
| `prepare-release.yml` | `workflow_dispatch` | Bump `VERSION` + `debian/changelog`, commit, tag `v*`, push |
| `build-package.yml` | push `main`, tag `v*`, PR | Matrix build (22.04/24.04), GitHub Release on tag, dispatch APT |

### Workflows (ros-hacks-apt)

| Workflow | Trigger | Purpose |
|----------|---------|---------|
| `update-apt-repo.yml` | `repository_dispatch` or `workflow_dispatch` | Build from tagged source, sign with stored secret, push APT index |

## Standard release procedure

Use `gh` — do not run local publish scripts.

```bash
# 1. Ensure main CI is green
gh run list -R danlil240/ROS-Hacks --workflow "Build and Release Package" --limit 3

# 2. Preview bump (optional)
gh workflow run prepare-release.yml -R danlil240/ROS-Hacks -f bump=patch -f dry_run=true

# 3. Release (patch / minor / major)
gh workflow run prepare-release.yml -R danlil240/ROS-Hacks -f bump=patch

# 4. Watch the chain
gh run watch -R danlil240/ROS-Hacks
gh run list -R danlil240/ros-hacks-apt --workflow "Update APT Repository" --limit 3
```

After `prepare-release` completes, `build-package.yml` runs automatically on the new tag. The `publish-apt` job dispatches to ros-hacks-apt.

### Verify success

```bash
# Tag and release exist
gh release view -R danlil240/ROS-Hacks "$(cat VERSION | xargs -I{} echo v{})"

# APT workflow succeeded
gh run list -R danlil240/ros-hacks-apt --workflow "Update APT Repository" --limit 1

# Fingerprint unchanged on ros-hacks-apt
gh api repos/danlil240/ros-hacks-apt/contents/SIGNING_KEY_FPR --jq .content | base64 -d
```

Expected fingerprint: `24F6F039ADD1E7A19FE61176172DC787BB63A69F`

## One-time setup (only if dispatch fails)

If `publish-apt` fails with missing `ROS_HACKS_APT_DISPATCH_TOKEN`:

1. Create a fine-grained PAT (or classic `repo` scope) owned by `danlil240`
2. Grant **Actions: Read and write** on `ros-hacks-apt`
3. Add repository secret on **ROS-Hacks** (not ros-hacks-apt):

```bash
gh secret set ROS_HACKS_APT_DISPATCH_TOKEN -R danlil240/ROS-Hacks
```

**Do not** add or change `APT_GPG_PRIVATE_KEY` / `APT_GPG_KEY_NAME` — those already exist on ros-hacks-apt.

Push the ros-hacks-apt workflow change (`repository_dispatch` trigger) before the first automated release:

```bash
cd ~/ros-hacks-apt && git add .github/workflows/update-apt-repo.yml && git commit -m "Accept release-published dispatch from ROS-Hacks"
```

## Failure recovery

| Failure | Action |
|---------|--------|
| `prepare-release` tag exists | Choose different bump or delete erroneous tag locally only after user confirms |
| Build fails on tag | Fix source, run `prepare-release` again with new patch |
| GitHub Release missing assets | Re-run failed `build-package` job for the tag |
| APT dispatch failed | Set `ROS_HACKS_APT_DISPATCH_TOKEN`, then manually dispatch: `gh workflow run "Update APT Repository" -R danlil240/ros-hacks-apt -f source_ref=vX.Y.Z` |
| APT signing fingerprint mismatch | **Stop.** Do not regenerate keys. Ask user to restore canonical backup per `AGENTS.md` |

## Version files

- `VERSION` — semantic version (e.g. `1.0.41`)
- `debian/changelog` — Debian package version (`1.0.41-1`)
- Git tag — `v` + `VERSION` (e.g. `v1.0.42`)

Tag must match `VERSION` after every release. `scripts/bump-version.sh` is CI-only; agents trigger it via `prepare-release.yml`, not directly.

## PR / main branch policy

- Merges to `main` run CI build (no release)
- Only tag pushes create GitHub Releases and trigger APT publish
- Never bump version in a PR without user intent to release

## Additional resources

- Project agent rules: [AGENTS.md](../../AGENTS.md)
- One-time PAT setup details: [reference.md](reference.md)
