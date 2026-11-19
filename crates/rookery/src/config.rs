//! `rookery.toml` (row 10.4): one file says which farm to run, so the same
//! binary runs one tray or the rack from configuration alone.
//!
//! ```toml
//! scene = "scene/generated/rack3.xml"   # its metadata is the .json beside it
//! seed = 7
//! detector = "oracle"                   # the only one until Phase 11
//! max_physics = 900.0                   # seconds of physics a run may use
//! trays = 3                             # optional: must match the scene
//! molt_rate = 1.0                       # 3: about an eyestalk-ablated cycle
//! ```
//!
//! Every field has a default, so an empty file is the one-tray farm.
//! [`Config::validate`] checks the file against the scene at start-up: the
//! scene loads, its shelf windows fit the mast (row 10.3), and `trays`, when
//! given, matches. A mismatch stops the run with a message, before any physics.

use std::path::{Path, PathBuf};

use mast::{SceneMeta, ShelfWindows};
use rookery_core::DetectorKind;
use serde::Deserialize;

#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Config {
    pub scene: PathBuf,
    pub seed: u64,
    pub detector: String,
    pub max_physics: f64,
    pub trays: Option<usize>,
    /// Molt rate multiplier (row 12.2): 3 is about an eyestalk-ablated cycle.
    pub molt_rate: f64,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            scene: PathBuf::from("scene/generated/rookery.xml"),
            seed: 7,
            detector: "oracle".into(),
            max_physics: 900.0,
            trays: None,
            molt_rate: 1.0,
        }
    }
}

impl Config {
    /// Read and parse `path`. A relative `scene` is taken from the file's
    /// directory, so a config works from wherever it is run.
    pub fn load(path: &Path) -> Result<Config, String> {
        let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
        let mut c: Config =
            toml::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))?;
        if c.scene.is_relative() {
            c.scene = path.parent().unwrap_or(Path::new(".")).join(&c.scene);
        }
        Ok(c)
    }

    /// Check the config against its scene. Returns the scene metadata.
    pub fn validate(&self) -> Result<SceneMeta, String> {
        let _: DetectorKind = self.detector.parse()?;
        if !(self.molt_rate.is_finite() && self.molt_rate > 0.0) {
            return Err(format!(
                "molt_rate must be a positive number, got {}",
                self.molt_rate
            ));
        }
        if !(self.max_physics.is_finite() && self.max_physics > 0.0) {
            return Err(format!(
                "max_physics must be a positive number of seconds, got {}",
                self.max_physics
            ));
        }
        let json = self.scene.with_extension("json");
        let meta = SceneMeta::load(&json).map_err(|e| format!("{}: {e}", json.display()))?;
        ShelfWindows::from_meta(&meta).map_err(|e| format!("{}: {e}", json.display()))?;
        if let Some(t) = self.trays
            && t != meta.trays
        {
            return Err(format!(
                "the config says trays = {t}, but {} has {}; regenerate the scene or fix the config",
                self.scene.display(),
                meta.trays
            ));
        }
        Ok(meta)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn repo(rel: &str) -> PathBuf {
        PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../..")).join(rel)
    }

    fn write(name: &str, text: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("rookery-config-{name}-{}", std::process::id()));
        std::fs::create_dir_all(&d).unwrap();
        let p = d.join("rookery.toml");
        std::fs::write(&p, text).unwrap();
        p
    }

    #[test]
    fn an_empty_file_is_the_one_tray_farm() {
        let c: Config = toml::from_str("").unwrap();
        assert_eq!(c, Config::default());
    }

    #[test]
    fn the_rack_config_validates_against_its_scene() {
        let p = write(
            "rack",
            &format!(
                "scene = {:?}\ntrays = 3\nseed = 11\n",
                repo("scene/generated/rack3.xml")
            ),
        );
        let c = Config::load(&p).unwrap();
        assert_eq!(c.validate().unwrap().trays, 3);
        assert_eq!(c.seed, 11);
    }

    #[test]
    fn mismatches_and_unknowns_are_refused_with_a_message() {
        let p = write(
            "mismatch",
            &format!(
                "scene = {:?}\ntrays = 3\n",
                repo("scene/generated/rookery.xml")
            ),
        );
        let e = Config::load(&p).unwrap().validate().unwrap_err();
        assert!(e.contains("trays = 3") && e.contains("has 1"), "{e}");
        let p = write(
            "lidar",
            &format!(
                "scene = {:?}\ndetector = \"lidar\"\n",
                repo("scene/generated/rookery.xml")
            ),
        );
        assert!(
            Config::load(&p)
                .unwrap()
                .validate()
                .unwrap_err()
                .contains("oracle")
        );
        let p = write("typo", "sede = 3\n");
        assert!(Config::load(&p).unwrap_err().contains("unknown field"));
        let p = write("relative", "scene = \"nope.xml\"\n");
        let c = Config::load(&p).unwrap();
        assert!(c.scene.is_absolute() || c.scene.starts_with(std::env::temp_dir()));
        assert!(c.validate().is_err());
    }
}
