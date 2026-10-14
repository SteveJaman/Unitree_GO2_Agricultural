# Troubleshooting

Every failure mode encountered during development, with the actual fix.

Sections are self-contained. Search for the error message you see.

---

## Session / Infrastructure

### `GO2_CONTAINER: Start the project Docker container first`

`mapping.sh` expects a running container. `wifi_nav.sh` creates it.

```bash
cd ~/files/go2-nav2-wifi
./setup-lidar3d.sh --laptop