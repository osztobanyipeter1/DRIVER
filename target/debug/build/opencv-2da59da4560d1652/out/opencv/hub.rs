#[cfg(ocvrs_has_module_calib3d)]
include!(concat!(env!("OUT_DIR"), "/opencv/calib3d.rs"));
#[cfg(ocvrs_has_module_core)]
include!(concat!(env!("OUT_DIR"), "/opencv/core.rs"));
#[cfg(ocvrs_has_module_features2d)]
include!(concat!(env!("OUT_DIR"), "/opencv/features2d.rs"));
#[cfg(ocvrs_has_module_flann)]
include!(concat!(env!("OUT_DIR"), "/opencv/flann.rs"));
#[cfg(ocvrs_has_module_highgui)]
include!(concat!(env!("OUT_DIR"), "/opencv/highgui.rs"));
#[cfg(ocvrs_has_module_imgproc)]
include!(concat!(env!("OUT_DIR"), "/opencv/imgproc.rs"));
pub mod types {
include!(concat!(env!("OUT_DIR"), "/opencv/types.rs"));
}
#[doc(hidden)]
pub mod sys {
include!(concat!(env!("OUT_DIR"), "/opencv/sys.rs"));
}
pub mod hub_prelude {
	#[cfg(ocvrs_has_module_calib3d)]
	pub use super::calib3d::prelude::*;
	#[cfg(ocvrs_has_module_core)]
	pub use super::core::prelude::*;
	#[cfg(ocvrs_has_module_features2d)]
	pub use super::features2d::prelude::*;
	#[cfg(ocvrs_has_module_flann)]
	pub use super::flann::prelude::*;
	#[cfg(ocvrs_has_module_highgui)]
	pub use super::highgui::prelude::*;
	#[cfg(ocvrs_has_module_imgproc)]
	pub use super::imgproc::prelude::*;
}
