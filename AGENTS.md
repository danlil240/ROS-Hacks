# Agent instructions — ROS-Hacks

## APT signing key — DO NOT CHANGE

The ros-hacks-apt repository **must** always be signed with:

```
24F6F039ADD1E7A19FE61176172DC787BB63A69F
```

### Never do these

- **Never** run `gpg --gen-key` or let `setup-apt-repo.sh` create a new signing key
- **Never** replace GitHub secrets `APT_GPG_PRIVATE_KEY` or `APT_GPG_KEY_NAME` on `danlil240/ros-hacks-apt`
- **Never** overwrite `~/ros-hacks-apt/.gpg-backup/` with a newly generated key
- **Never** export the entire GPG keyring — always export by fingerprint from `SIGNING_KEY_FPR`

### Publishing APT packages

Prefer the GitHub Actions workflow on `danlil240/ros-hacks-apt` (uses stored secrets).

For local publishes, restore the canonical key first:

```bash
./scripts/backup-gpg-key.sh restore   # after copying backup from external drive
./scripts/setup-apt-repo.sh update  # re-sign only
```

Changing the signing key breaks `apt update` for all users.
