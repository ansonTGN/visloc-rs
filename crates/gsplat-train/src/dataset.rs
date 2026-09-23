//! A COLMAP scene with its images, split into train and eval views.
//!
//! Expected layout (what `colmap image_undistorter` writes, plus a text model):
//!
//! ```text
//! <root>/images/<name>          one image per registered view
//! <root>/sparse/0/cameras.txt   (or sparse/, text or binary)
//! ```
//!
//! Views are sorted by image name and every `eval_every`-th one (index 0, N,
//! 2N, ...) is held out, matching brush's and the Inria code's split so scores
//! are comparable. Cameras must be pinhole: undistort first.

use std::path::{Path, PathBuf};

use visloc_gsplat_core::camera::CameraView;
use visloc_gsplat_core::colmap_scene::{
    load_colmap_scene, ColmapSceneError, DEFAULT_SEED_LOG_SCALE,
};
use visloc_gsplat_core::gaussian::Scene;

/// Errors from loading a dataset.
#[derive(Debug, thiserror::Error)]
pub enum DatasetError {
    #[error("no COLMAP model under {0} (looked in sparse/0 and sparse)")]
    NoModel(PathBuf),
    #[error(transparent)]
    Colmap(#[from] ColmapSceneError),
    #[error("image {path}: {source}")]
    Image {
        path: PathBuf,
        source: image::ImageError,
    },
    #[error("image {path} is {got_w}x{got_h}, camera expects {want_w}x{want_h}")]
    SizeMismatch {
        path: PathBuf,
        got_w: u32,
        got_h: u32,
        want_w: u32,
        want_h: u32,
    },
}

/// One posed image.
#[derive(Debug, Clone)]
pub struct View {
    pub name: String,
    pub camera: CameraView,
    pub image_path: PathBuf,
}

/// A loaded dataset: SfM points as the initial scene, plus the view split.
#[derive(Debug, Clone)]
pub struct Dataset {
    /// Seed gaussians from the COLMAP points (isotropic, default log-scale).
    pub init: Scene,
    pub train: Vec<View>,
    pub eval: Vec<View>,
}

/// Load `<root>` (see module docs). `eval_every = None` keeps every view for
/// training.
pub fn load_colmap_dataset(
    root: impl AsRef<Path>,
    eval_every: Option<usize>,
) -> Result<Dataset, DatasetError> {
    let root = root.as_ref();
    let model_dir = [root.join("sparse").join("0"), root.join("sparse")]
        .into_iter()
        .find(|d| d.join("cameras.txt").exists() || d.join("cameras.bin").exists())
        .ok_or_else(|| DatasetError::NoModel(root.to_path_buf()))?;
    let colmap = load_colmap_scene(&model_dir, DEFAULT_SEED_LOG_SCALE)?;

    let mut views: Vec<View> = colmap
        .image_names
        .iter()
        .zip(colmap.views)
        .map(|(name, camera)| View {
            name: name.clone(),
            camera,
            image_path: root.join("images").join(name),
        })
        .collect();
    views.sort_by(|a, b| a.name.cmp(&b.name));

    let mut train = Vec::new();
    let mut eval = Vec::new();
    for (i, v) in views.into_iter().enumerate() {
        match eval_every {
            Some(n) if n > 0 && i % n == 0 => eval.push(v),
            _ => train.push(v),
        }
    }
    Ok(Dataset {
        init: colmap.scene,
        train,
        eval,
    })
}

/// Load a view's image as row-major RGB in `[0, 1]`, checking it matches the
/// camera's resolution.
pub fn load_view_rgb(view: &View) -> Result<Vec<[f32; 3]>, DatasetError> {
    let img = image::open(&view.image_path)
        .map_err(|source| DatasetError::Image {
            path: view.image_path.clone(),
            source,
        })?
        .to_rgb8();
    let (w, h) = img.dimensions();
    let (want_w, want_h) = (view.camera.camera.width, view.camera.camera.height);
    if (w, h) != (want_w, want_h) {
        return Err(DatasetError::SizeMismatch {
            path: view.image_path.clone(),
            got_w: w,
            got_h: h,
            want_w,
            want_h,
        });
    }
    Ok(img
        .pixels()
        .map(|p| {
            [
                p[0] as f32 / 255.0,
                p[1] as f32 / 255.0,
                p[2] as f32 / 255.0,
            ]
        })
        .collect())
}
