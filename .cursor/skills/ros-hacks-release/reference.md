# ROS-Hacks release reference

## Fine-grained PAT for `ROS_HACKS_APT_DISPATCH_TOKEN`

Create at: https://github.com/settings/tokens?type=beta

| Field | Value |
|-------|-------|
| Resource owner | `danlil240` |
| Repository access | Only `ros-hacks-apt` |
| Permissions | Actions: Read and write, Metadata: Read |

Store on **danlil240/ROS-Hacks** as `ROS_HACKS_APT_DISPATCH_TOKEN`.

This token only triggers workflows. It does **not** hold the GPG private key.

## Manual APT retry (fallback)

```bash
gh workflow run "Update APT Repository" \
  -R danlil240/ros-hacks-apt \
  -f source_repo=danlil240/ROS-Hacks \
  -f source_ref=v1.0.42
```

## CI monitoring shortcuts

```bash
# All recent ROS-Hacks runs
gh run list -R danlil240/ROS-Hacks --limit 10

# Failed runs only
gh run list -R danlil240/ROS-Hacks --status failure --limit 5

# Watch active run
gh run watch -R danlil240/ROS-Hacks <run-id>
```

## Canonical signing key recovery

Only when user explicitly requests signing recovery:

```bash
./scripts/backup-gpg-key.sh restore   # after copying backup from external drive
```

Then re-dispatch APT update — never `setup-apt-repo.sh` key generation paths.
