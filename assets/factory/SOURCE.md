# Factory Backdrop Asset Provenance

`Factory.usd` and `SubUSDs/` in this directory are vendored from NVIDIA's
**USD Explorer Sample Assets Pack** — a real factory-floor scene (factory
shell/building, a Kuka robot arm, car lift, safety gates, vehicle hangers,
part racks), not one of Isaac Sim's bundled warehouse environments.

- **Source**: [Downloadable Asset Packs](https://docs.omniverse.nvidia.com/usd/latest/usd_content_samples/downloadable_packs.html)
  → "USD Explorer Sample Assets Pack"
- **Download URL**: `https://d4i3qtqj3r0z5.cloudfront.net/USD_Explorer_Sample_NVD%4010011.zip`
  (~524MB zip)
- **Package**: `USD_Explorer_Sample_NVD`, version `10011`, commit
  `744960a41e6c83864c88879f402bd1ff1622fe51` (per the zip's own
  `PACKAGE-INFO.yaml`)
- **License**: NVIDIA Omniverse License Agreement (content-pack terms), not
  open source —
  https://docs.omniverse.nvidia.com/platform/latest/common/NVIDIA_Omniverse_License_Agreement.html
  (full text in the zip's `PACKAGE-LICENSES/USD_Explorer_Sample_NVD-license.txt`)

## What's vendored here vs. what's excluded

The zip's `Usd_Explorer/Samples/Examples/2023_2/Factory/` folder contains:

- `Factory.usd` + `SubUSDs/` — the actual composed scene (USD + textures +
  materials referenced by `Factory.usd`). **Vendored here.**
- `Source_Files/` (~400MB) — the original CAD files this scene was
  authored from (Revit `.rvt`, STEP, SolidWorks `.SLDPRT`, JT, Visual
  Components `.vcmx`) plus per-format USD re-exports. Not needed to load
  the scene in Isaac Sim. **Not vendored.**
- `.thumbs/` — cosmetic preview PNGs. **Not vendored.**

## Not committed to git

`Factory.usd` and `SubUSDs/` (~404MB) are gitignored (see the repo's
top-level `.gitignore`) — too large to vendor in git. To restore them after
a fresh clone:

```bash
curl -fL -o /tmp/usd_explorer_sample.zip \
  "https://d4i3qtqj3r0z5.cloudfront.net/USD_Explorer_Sample_NVD%4010011.zip"
unzip -q /tmp/usd_explorer_sample.zip \
  "Usd_Explorer/Samples/Examples/2023_2/Factory/Factory.usd" \
  "Usd_Explorer/Samples/Examples/2023_2/Factory/SubUSDs/*" \
  -x "*/.thumbs/*" \
  -d /tmp/usd_explorer_extract
mv /tmp/usd_explorer_extract/Usd_Explorer/Samples/Examples/2023_2/Factory/Factory.usd assets/factory/
mv /tmp/usd_explorer_extract/Usd_Explorer/Samples/Examples/2023_2/Factory/SubUSDs assets/factory/
```

Referenced from `configs/scene/table_layout.yaml`'s `factory.backdrop_usd`
(a path relative to the repo root) and loaded by
`scripts/build_scene.py`'s `build_factory()`.
