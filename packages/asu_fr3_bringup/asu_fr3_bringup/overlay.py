"""Config/model overlay: baked defaults plus optional user files.

The image bakes its defaults in $ASU_FR3_DIR (default /opt/asu_fr3) as `config/` and `models/`.
A user folder mounted at $ASU_FR3_OVERRIDE (default /ws_override) with the same layout wins, file by
file. `build_effective()` links both into one folder (default /tmp/asu_fr3_effective) so that xacro
includes, the MuJoCo scene and its meshes all see an ordinary directory, and returns where each
file came from.
"""
import filecmp
import os
import shutil
from pathlib import Path

KINDS = ("config", "models")


class Effective:
    def __init__(self, root, sources, baked, override):
        self.root = Path(root)
        # {"models/scene.xml": "baked" | "same" | "override"}: only in the image, mounted but identical to the
        # baked copy, or mounted and different (or new).
        self.sources = sources
        self.baked = Path(baked)
        self.override = Path(override)

    def file(self, kind, name):
        """Path of `name` inside the effective `kind` folder; raises if it does not exist."""
        path = self.root / kind / name
        if not path.exists():
            raise FileNotFoundError(
                f"{kind}/{name} not found in {self.baked}/{kind} or {self.override}/{kind}"
            )
        return path

    @property
    def models(self):
        return self.root / "models"

    def source_of(self, kind, name):
        return self.sources.get(f"{kind}/{name}", "missing")

    def label(self, kind, name):
        src = self.source_of(kind, name)
        return "override" if src == "override" else ("baked" if src == "baked" else "mounted, same as baked")

    def report(self, in_use=None):
        """Short text for the startup log. `in_use` maps a label to (kind, name)."""
        lines = [f"[asu_fr3] config overlay: baked={self.baked}  override={self.override}"]
        for kind in KINDS:
            names = [k for k in self.sources if k.startswith(kind + "/")]
            n_over = sum(1 for k in names if self.sources[k] == "override")
            n_same = sum(1 for k in names if self.sources[k] == "same")
            lines.append(
                f"[asu_fr3]   {kind}: {len(names)} files, {n_over} differ from the baked default, "
                f"{n_same} mounted but identical"
            )
        for k, v in sorted(self.sources.items()):
            if v == "override":
                lines.append(f"[asu_fr3]   override: {k}")
        for label, (kind, name) in (in_use or {}).items():
            lines.append(f"[asu_fr3]   using {label}: {kind}/{name} [{self.label(kind, name)}]")
        return "\n".join(lines)


def build_effective(baked=None, override=None, effective=None):
    baked = Path(baked or os.environ.get("ASU_FR3_DIR", "/opt/asu_fr3"))
    override = Path(override or os.environ.get("ASU_FR3_OVERRIDE", "/ws_override"))
    effective = Path(effective or os.environ.get("ASU_FR3_EFFECTIVE", "/tmp/asu_fr3_effective"))
    if not any((baked / k).is_dir() for k in KINDS) and not any((override / k).is_dir() for k in KINDS):
        raise FileNotFoundError(f"no config/ or models/ in {baked} or {override}")
    if effective.exists():
        shutil.rmtree(effective)
    sources = {}
    for layer, root in (("baked", baked), ("override", override)):   # override is linked last, so it wins
        for kind in KINDS:
            base = root / kind
            if not base.is_dir():
                continue
            for path in sorted(base.rglob("*")):
                if path.is_dir() or "__pycache__" in path.parts or path.suffix == ".pyc":
                    continue   # python caches (e.g. from running a script on the host) are not config
                rel = path.relative_to(root)
                dest = effective / rel
                dest.parent.mkdir(parents=True, exist_ok=True)
                if dest.is_symlink() or dest.exists():
                    dest.unlink()
                dest.symlink_to(path.resolve())
                if layer == "override":
                    twin = baked / rel
                    layer_name = "same" if twin.is_file() and filecmp.cmp(twin, path, shallow=False) else "override"
                else:
                    layer_name = layer
                sources[rel.as_posix()] = layer_name
    return Effective(effective, sources, baked, override)


_cached = None


def get_effective():
    """Build once per process and print the report (launch files call this at generation time)."""
    global _cached
    if _cached is None:
        _cached = build_effective()
    return _cached
