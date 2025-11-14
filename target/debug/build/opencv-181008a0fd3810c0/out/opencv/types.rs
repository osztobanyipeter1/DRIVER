
#[cfg(ocvrs_has_module_calib3d)]
mod calib3d_types {
	use crate::{mod_prelude::*, core, types, sys};

	pub type PtrOfLMSolver = core::Ptr<crate::calib3d::LMSolver>;
	
	ptr_extern! { crate::calib3d::LMSolver,
		cv_PtrLcv_LMSolverG_delete, cv_PtrLcv_LMSolverG_getInnerPtr_const, cv_PtrLcv_LMSolverG_getInnerPtrMut
	}
	
	impl core::Ptr<crate::calib3d::LMSolver> {
		#[inline] pub fn as_raw_PtrOfLMSolver(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfLMSolver(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::calib3d::LMSolverTraitConst for core::Ptr<crate::calib3d::LMSolver> {
		#[inline] fn as_raw_LMSolver(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::calib3d::LMSolverTrait for core::Ptr<crate::calib3d::LMSolver> {
		#[inline] fn as_raw_mut_LMSolver(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl core::AlgorithmTraitConst for core::Ptr<crate::calib3d::LMSolver> {
		#[inline] fn as_raw_Algorithm(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::AlgorithmTrait for core::Ptr<crate::calib3d::LMSolver> {
		#[inline] fn as_raw_mut_Algorithm(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfLMSolver, core::Ptr<core::Algorithm>, cv_PtrLcv_LMSolverG_to_PtrOfAlgorithm }
	
	impl std::fmt::Debug for core::Ptr<crate::calib3d::LMSolver> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfLMSolver")
				.finish()
		}
	}
	
	pub type PtrOfLMSolver_Callback = core::Ptr<crate::calib3d::LMSolver_Callback>;
	
	ptr_extern! { crate::calib3d::LMSolver_Callback,
		cv_PtrLcv_LMSolver_CallbackG_delete, cv_PtrLcv_LMSolver_CallbackG_getInnerPtr_const, cv_PtrLcv_LMSolver_CallbackG_getInnerPtrMut
	}
	
	impl core::Ptr<crate::calib3d::LMSolver_Callback> {
		#[inline] pub fn as_raw_PtrOfLMSolver_Callback(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfLMSolver_Callback(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::calib3d::LMSolver_CallbackTraitConst for core::Ptr<crate::calib3d::LMSolver_Callback> {
		#[inline] fn as_raw_LMSolver_Callback(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::calib3d::LMSolver_CallbackTrait for core::Ptr<crate::calib3d::LMSolver_Callback> {
		#[inline] fn as_raw_mut_LMSolver_Callback(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl std::fmt::Debug for core::Ptr<crate::calib3d::LMSolver_Callback> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfLMSolver_Callback")
				.finish()
		}
	}
	
	pub type PtrOfStereoBM = core::Ptr<crate::calib3d::StereoBM>;
	
	ptr_extern! { crate::calib3d::StereoBM,
		cv_PtrLcv_StereoBMG_delete, cv_PtrLcv_StereoBMG_getInnerPtr_const, cv_PtrLcv_StereoBMG_getInnerPtrMut
	}
	
	impl core::Ptr<crate::calib3d::StereoBM> {
		#[inline] pub fn as_raw_PtrOfStereoBM(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfStereoBM(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::calib3d::StereoBMTraitConst for core::Ptr<crate::calib3d::StereoBM> {
		#[inline] fn as_raw_StereoBM(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::calib3d::StereoBMTrait for core::Ptr<crate::calib3d::StereoBM> {
		#[inline] fn as_raw_mut_StereoBM(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl core::AlgorithmTraitConst for core::Ptr<crate::calib3d::StereoBM> {
		#[inline] fn as_raw_Algorithm(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::AlgorithmTrait for core::Ptr<crate::calib3d::StereoBM> {
		#[inline] fn as_raw_mut_Algorithm(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfStereoBM, core::Ptr<core::Algorithm>, cv_PtrLcv_StereoBMG_to_PtrOfAlgorithm }
	
	impl crate::calib3d::StereoMatcherTraitConst for core::Ptr<crate::calib3d::StereoBM> {
		#[inline] fn as_raw_StereoMatcher(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::calib3d::StereoMatcherTrait for core::Ptr<crate::calib3d::StereoBM> {
		#[inline] fn as_raw_mut_StereoMatcher(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfStereoBM, core::Ptr<crate::calib3d::StereoMatcher>, cv_PtrLcv_StereoBMG_to_PtrOfStereoMatcher }
	
	impl std::fmt::Debug for core::Ptr<crate::calib3d::StereoBM> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfStereoBM")
				.finish()
		}
	}
	
	pub type PtrOfStereoMatcher = core::Ptr<crate::calib3d::StereoMatcher>;
	
	ptr_extern! { crate::calib3d::StereoMatcher,
		cv_PtrLcv_StereoMatcherG_delete, cv_PtrLcv_StereoMatcherG_getInnerPtr_const, cv_PtrLcv_StereoMatcherG_getInnerPtrMut
	}
	
	impl core::Ptr<crate::calib3d::StereoMatcher> {
		#[inline] pub fn as_raw_PtrOfStereoMatcher(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfStereoMatcher(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::calib3d::StereoMatcherTraitConst for core::Ptr<crate::calib3d::StereoMatcher> {
		#[inline] fn as_raw_StereoMatcher(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::calib3d::StereoMatcherTrait for core::Ptr<crate::calib3d::StereoMatcher> {
		#[inline] fn as_raw_mut_StereoMatcher(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl core::AlgorithmTraitConst for core::Ptr<crate::calib3d::StereoMatcher> {
		#[inline] fn as_raw_Algorithm(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::AlgorithmTrait for core::Ptr<crate::calib3d::StereoMatcher> {
		#[inline] fn as_raw_mut_Algorithm(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfStereoMatcher, core::Ptr<core::Algorithm>, cv_PtrLcv_StereoMatcherG_to_PtrOfAlgorithm }
	
	impl std::fmt::Debug for core::Ptr<crate::calib3d::StereoMatcher> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfStereoMatcher")
				.finish()
		}
	}
	
	pub type PtrOfStereoSGBM = core::Ptr<crate::calib3d::StereoSGBM>;
	
	ptr_extern! { crate::calib3d::StereoSGBM,
		cv_PtrLcv_StereoSGBMG_delete, cv_PtrLcv_StereoSGBMG_getInnerPtr_const, cv_PtrLcv_StereoSGBMG_getInnerPtrMut
	}
	
	impl core::Ptr<crate::calib3d::StereoSGBM> {
		#[inline] pub fn as_raw_PtrOfStereoSGBM(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfStereoSGBM(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::calib3d::StereoSGBMTraitConst for core::Ptr<crate::calib3d::StereoSGBM> {
		#[inline] fn as_raw_StereoSGBM(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::calib3d::StereoSGBMTrait for core::Ptr<crate::calib3d::StereoSGBM> {
		#[inline] fn as_raw_mut_StereoSGBM(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl core::AlgorithmTraitConst for core::Ptr<crate::calib3d::StereoSGBM> {
		#[inline] fn as_raw_Algorithm(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::AlgorithmTrait for core::Ptr<crate::calib3d::StereoSGBM> {
		#[inline] fn as_raw_mut_Algorithm(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfStereoSGBM, core::Ptr<core::Algorithm>, cv_PtrLcv_StereoSGBMG_to_PtrOfAlgorithm }
	
	impl crate::calib3d::StereoMatcherTraitConst for core::Ptr<crate::calib3d::StereoSGBM> {
		#[inline] fn as_raw_StereoMatcher(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::calib3d::StereoMatcherTrait for core::Ptr<crate::calib3d::StereoSGBM> {
		#[inline] fn as_raw_mut_StereoMatcher(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfStereoSGBM, core::Ptr<crate::calib3d::StereoMatcher>, cv_PtrLcv_StereoSGBMG_to_PtrOfStereoMatcher }
	
	impl std::fmt::Debug for core::Ptr<crate::calib3d::StereoSGBM> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfStereoSGBM")
				.finish()
		}
	}
	
}
#[cfg(ocvrs_has_module_calib3d)]
pub use calib3d_types::*;

#[cfg(ocvrs_has_module_core)]
mod core_types {
	use crate::{mod_prelude::*, core, types, sys};

	impl core::GpuMat_AllocatorTraitConst for types::AbstractRefMut<'static, core::GpuMat_Allocator> {
		#[inline] fn as_raw_GpuMat_Allocator(&self) -> extern_send!(Self) { self.as_raw() }
	}
	
	impl core::GpuMat_AllocatorTrait for types::AbstractRefMut<'static, core::GpuMat_Allocator> {
		#[inline] fn as_raw_mut_GpuMat_Allocator(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl core::MatOpTraitConst for types::AbstractRefMut<'static, core::MatOp> {
		#[inline] fn as_raw_MatOp(&self) -> extern_send!(Self) { self.as_raw() }
	}
	
	impl core::MatOpTrait for types::AbstractRefMut<'static, core::MatOp> {
		#[inline] fn as_raw_mut_MatOp(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	pub type PtrOfAlgorithm = core::Ptr<core::Algorithm>;
	
	ptr_extern! { core::Algorithm,
		cv_PtrLcv_AlgorithmG_delete, cv_PtrLcv_AlgorithmG_getInnerPtr_const, cv_PtrLcv_AlgorithmG_getInnerPtrMut
	}
	
	ptr_extern_ctor! { core::Algorithm, cv_PtrLcv_AlgorithmG_new_const_Algorithm }
	impl core::Ptr<core::Algorithm> {
		#[inline] pub fn as_raw_PtrOfAlgorithm(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfAlgorithm(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl core::AlgorithmTraitConst for core::Ptr<core::Algorithm> {
		#[inline] fn as_raw_Algorithm(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::AlgorithmTrait for core::Ptr<core::Algorithm> {
		#[inline] fn as_raw_mut_Algorithm(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl std::fmt::Debug for core::Ptr<core::Algorithm> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfAlgorithm")
				.finish()
		}
	}
	
	pub type PtrOfConjGradSolver = core::Ptr<core::ConjGradSolver>;
	
	ptr_extern! { core::ConjGradSolver,
		cv_PtrLcv_ConjGradSolverG_delete, cv_PtrLcv_ConjGradSolverG_getInnerPtr_const, cv_PtrLcv_ConjGradSolverG_getInnerPtrMut
	}
	
	impl core::Ptr<core::ConjGradSolver> {
		#[inline] pub fn as_raw_PtrOfConjGradSolver(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfConjGradSolver(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl core::ConjGradSolverTraitConst for core::Ptr<core::ConjGradSolver> {
		#[inline] fn as_raw_ConjGradSolver(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::ConjGradSolverTrait for core::Ptr<core::ConjGradSolver> {
		#[inline] fn as_raw_mut_ConjGradSolver(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl core::AlgorithmTraitConst for core::Ptr<core::ConjGradSolver> {
		#[inline] fn as_raw_Algorithm(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::AlgorithmTrait for core::Ptr<core::ConjGradSolver> {
		#[inline] fn as_raw_mut_Algorithm(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfConjGradSolver, core::Ptr<core::Algorithm>, cv_PtrLcv_ConjGradSolverG_to_PtrOfAlgorithm }
	
	impl core::MinProblemSolverTraitConst for core::Ptr<core::ConjGradSolver> {
		#[inline] fn as_raw_MinProblemSolver(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::MinProblemSolverTrait for core::Ptr<core::ConjGradSolver> {
		#[inline] fn as_raw_mut_MinProblemSolver(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfConjGradSolver, core::Ptr<core::MinProblemSolver>, cv_PtrLcv_ConjGradSolverG_to_PtrOfMinProblemSolver }
	
	impl std::fmt::Debug for core::Ptr<core::ConjGradSolver> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfConjGradSolver")
				.finish()
		}
	}
	
	pub type PtrOfDownhillSolver = core::Ptr<core::DownhillSolver>;
	
	ptr_extern! { core::DownhillSolver,
		cv_PtrLcv_DownhillSolverG_delete, cv_PtrLcv_DownhillSolverG_getInnerPtr_const, cv_PtrLcv_DownhillSolverG_getInnerPtrMut
	}
	
	impl core::Ptr<core::DownhillSolver> {
		#[inline] pub fn as_raw_PtrOfDownhillSolver(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfDownhillSolver(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl core::DownhillSolverTraitConst for core::Ptr<core::DownhillSolver> {
		#[inline] fn as_raw_DownhillSolver(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::DownhillSolverTrait for core::Ptr<core::DownhillSolver> {
		#[inline] fn as_raw_mut_DownhillSolver(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl core::AlgorithmTraitConst for core::Ptr<core::DownhillSolver> {
		#[inline] fn as_raw_Algorithm(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::AlgorithmTrait for core::Ptr<core::DownhillSolver> {
		#[inline] fn as_raw_mut_Algorithm(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfDownhillSolver, core::Ptr<core::Algorithm>, cv_PtrLcv_DownhillSolverG_to_PtrOfAlgorithm }
	
	impl core::MinProblemSolverTraitConst for core::Ptr<core::DownhillSolver> {
		#[inline] fn as_raw_MinProblemSolver(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::MinProblemSolverTrait for core::Ptr<core::DownhillSolver> {
		#[inline] fn as_raw_mut_MinProblemSolver(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfDownhillSolver, core::Ptr<core::MinProblemSolver>, cv_PtrLcv_DownhillSolverG_to_PtrOfMinProblemSolver }
	
	impl std::fmt::Debug for core::Ptr<core::DownhillSolver> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfDownhillSolver")
				.finish()
		}
	}
	
	pub type PtrOfFileStorage = core::Ptr<core::FileStorage>;
	
	ptr_extern! { core::FileStorage,
		cv_PtrLcv_FileStorageG_delete, cv_PtrLcv_FileStorageG_getInnerPtr_const, cv_PtrLcv_FileStorageG_getInnerPtrMut
	}
	
	ptr_extern_ctor! { core::FileStorage, cv_PtrLcv_FileStorageG_new_const_FileStorage }
	impl core::Ptr<core::FileStorage> {
		#[inline] pub fn as_raw_PtrOfFileStorage(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfFileStorage(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl core::FileStorageTraitConst for core::Ptr<core::FileStorage> {
		#[inline] fn as_raw_FileStorage(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::FileStorageTrait for core::Ptr<core::FileStorage> {
		#[inline] fn as_raw_mut_FileStorage(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl std::fmt::Debug for core::Ptr<core::FileStorage> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfFileStorage")
				.field("state", &core::FileStorageTraitConst::state(self))
				.field("elname", &core::FileStorageTraitConst::elname(self))
				.finish()
		}
	}
	
	pub type PtrOfFormatted = core::Ptr<core::Formatted>;
	
	ptr_extern! { core::Formatted,
		cv_PtrLcv_FormattedG_delete, cv_PtrLcv_FormattedG_getInnerPtr_const, cv_PtrLcv_FormattedG_getInnerPtrMut
	}
	
	impl core::Ptr<core::Formatted> {
		#[inline] pub fn as_raw_PtrOfFormatted(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfFormatted(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl core::FormattedTraitConst for core::Ptr<core::Formatted> {
		#[inline] fn as_raw_Formatted(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::FormattedTrait for core::Ptr<core::Formatted> {
		#[inline] fn as_raw_mut_Formatted(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl std::fmt::Debug for core::Ptr<core::Formatted> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfFormatted")
				.finish()
		}
	}
	
	pub type PtrOfFormatter = core::Ptr<core::Formatter>;
	
	ptr_extern! { core::Formatter,
		cv_PtrLcv_FormatterG_delete, cv_PtrLcv_FormatterG_getInnerPtr_const, cv_PtrLcv_FormatterG_getInnerPtrMut
	}
	
	impl core::Ptr<core::Formatter> {
		#[inline] pub fn as_raw_PtrOfFormatter(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfFormatter(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl core::FormatterTraitConst for core::Ptr<core::Formatter> {
		#[inline] fn as_raw_Formatter(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::FormatterTrait for core::Ptr<core::Formatter> {
		#[inline] fn as_raw_mut_Formatter(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl std::fmt::Debug for core::Ptr<core::Formatter> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfFormatter")
				.finish()
		}
	}
	
	pub type PtrOfGpuMat_Allocator = core::Ptr<core::GpuMat_Allocator>;
	
	ptr_extern! { core::GpuMat_Allocator,
		cv_PtrLcv_cuda_GpuMat_AllocatorG_delete, cv_PtrLcv_cuda_GpuMat_AllocatorG_getInnerPtr_const, cv_PtrLcv_cuda_GpuMat_AllocatorG_getInnerPtrMut
	}
	
	impl core::Ptr<core::GpuMat_Allocator> {
		#[inline] pub fn as_raw_PtrOfGpuMat_Allocator(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfGpuMat_Allocator(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl core::GpuMat_AllocatorTraitConst for core::Ptr<core::GpuMat_Allocator> {
		#[inline] fn as_raw_GpuMat_Allocator(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::GpuMat_AllocatorTrait for core::Ptr<core::GpuMat_Allocator> {
		#[inline] fn as_raw_mut_GpuMat_Allocator(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl std::fmt::Debug for core::Ptr<core::GpuMat_Allocator> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfGpuMat_Allocator")
				.finish()
		}
	}
	
	pub type PtrOfMinProblemSolver = core::Ptr<core::MinProblemSolver>;
	
	ptr_extern! { core::MinProblemSolver,
		cv_PtrLcv_MinProblemSolverG_delete, cv_PtrLcv_MinProblemSolverG_getInnerPtr_const, cv_PtrLcv_MinProblemSolverG_getInnerPtrMut
	}
	
	impl core::Ptr<core::MinProblemSolver> {
		#[inline] pub fn as_raw_PtrOfMinProblemSolver(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfMinProblemSolver(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl core::MinProblemSolverTraitConst for core::Ptr<core::MinProblemSolver> {
		#[inline] fn as_raw_MinProblemSolver(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::MinProblemSolverTrait for core::Ptr<core::MinProblemSolver> {
		#[inline] fn as_raw_mut_MinProblemSolver(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl core::AlgorithmTraitConst for core::Ptr<core::MinProblemSolver> {
		#[inline] fn as_raw_Algorithm(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::AlgorithmTrait for core::Ptr<core::MinProblemSolver> {
		#[inline] fn as_raw_mut_Algorithm(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfMinProblemSolver, core::Ptr<core::Algorithm>, cv_PtrLcv_MinProblemSolverG_to_PtrOfAlgorithm }
	
	impl std::fmt::Debug for core::Ptr<core::MinProblemSolver> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfMinProblemSolver")
				.finish()
		}
	}
	
	pub type PtrOfMinProblemSolver_Function = core::Ptr<core::MinProblemSolver_Function>;
	
	ptr_extern! { core::MinProblemSolver_Function,
		cv_PtrLcv_MinProblemSolver_FunctionG_delete, cv_PtrLcv_MinProblemSolver_FunctionG_getInnerPtr_const, cv_PtrLcv_MinProblemSolver_FunctionG_getInnerPtrMut
	}
	
	impl core::Ptr<core::MinProblemSolver_Function> {
		#[inline] pub fn as_raw_PtrOfMinProblemSolver_Function(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfMinProblemSolver_Function(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl core::MinProblemSolver_FunctionTraitConst for core::Ptr<core::MinProblemSolver_Function> {
		#[inline] fn as_raw_MinProblemSolver_Function(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::MinProblemSolver_FunctionTrait for core::Ptr<core::MinProblemSolver_Function> {
		#[inline] fn as_raw_mut_MinProblemSolver_Function(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl std::fmt::Debug for core::Ptr<core::MinProblemSolver_Function> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfMinProblemSolver_Function")
				.finish()
		}
	}
	
	pub type PtrOfOriginalClassName = core::Ptr<core::OriginalClassName>;
	
	ptr_extern! { core::OriginalClassName,
		cv_PtrLcv_utils_nested_OriginalClassNameG_delete, cv_PtrLcv_utils_nested_OriginalClassNameG_getInnerPtr_const, cv_PtrLcv_utils_nested_OriginalClassNameG_getInnerPtrMut
	}
	
	ptr_extern_ctor! { core::OriginalClassName, cv_PtrLcv_utils_nested_OriginalClassNameG_new_const_OriginalClassName }
	impl core::Ptr<core::OriginalClassName> {
		#[inline] pub fn as_raw_PtrOfOriginalClassName(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfOriginalClassName(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl core::OriginalClassNameTraitConst for core::Ptr<core::OriginalClassName> {
		#[inline] fn as_raw_OriginalClassName(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::OriginalClassNameTrait for core::Ptr<core::OriginalClassName> {
		#[inline] fn as_raw_mut_OriginalClassName(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl std::fmt::Debug for core::Ptr<core::OriginalClassName> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfOriginalClassName")
				.finish()
		}
	}
	
	pub type PtrOff32 = core::Ptr<f32>;
	
	ptr_extern! { f32,
		cv_PtrLfloatG_delete, cv_PtrLfloatG_getInnerPtr_const, cv_PtrLfloatG_getInnerPtrMut
	}
	
	ptr_extern_ctor! { f32, cv_PtrLfloatG_new_const_float }
	impl core::Ptr<f32> {
		#[inline] pub fn as_raw_PtrOff32(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOff32(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	pub type VectorOfDMatch = core::Vector<core::DMatch>;
	
	impl core::Vector<core::DMatch> {
		pub fn as_raw_VectorOfDMatch(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfDMatch(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::DMatch,
		std_vectorLcv_DMatchG_new_const, std_vectorLcv_DMatchG_delete,
		std_vectorLcv_DMatchG_len_const, std_vectorLcv_DMatchG_isEmpty_const,
		std_vectorLcv_DMatchG_capacity_const, std_vectorLcv_DMatchG_shrinkToFit,
		std_vectorLcv_DMatchG_reserve_size_t, std_vectorLcv_DMatchG_remove_size_t,
		std_vectorLcv_DMatchG_swap_size_t_size_t, std_vectorLcv_DMatchG_clear,
		std_vectorLcv_DMatchG_get_const_size_t, std_vectorLcv_DMatchG_set_size_t_const_DMatch,
		std_vectorLcv_DMatchG_push_const_DMatch, std_vectorLcv_DMatchG_insert_size_t_const_DMatch,
	}
	vector_copy_non_bool! { core::DMatch,
		std_vectorLcv_DMatchG_data_const, std_vectorLcv_DMatchG_dataMut, cv_fromSlice_const_const_DMatchX_size_t,
		std_vectorLcv_DMatchG_clone_const,
	}
	
	pub type VectorOfGpuMat = core::Vector<core::GpuMat>;
	
	impl core::Vector<core::GpuMat> {
		pub fn as_raw_VectorOfGpuMat(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfGpuMat(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::GpuMat,
		std_vectorLcv_cuda_GpuMatG_new_const, std_vectorLcv_cuda_GpuMatG_delete,
		std_vectorLcv_cuda_GpuMatG_len_const, std_vectorLcv_cuda_GpuMatG_isEmpty_const,
		std_vectorLcv_cuda_GpuMatG_capacity_const, std_vectorLcv_cuda_GpuMatG_shrinkToFit,
		std_vectorLcv_cuda_GpuMatG_reserve_size_t, std_vectorLcv_cuda_GpuMatG_remove_size_t,
		std_vectorLcv_cuda_GpuMatG_swap_size_t_size_t, std_vectorLcv_cuda_GpuMatG_clear,
		std_vectorLcv_cuda_GpuMatG_get_const_size_t, std_vectorLcv_cuda_GpuMatG_set_size_t_const_GpuMat,
		std_vectorLcv_cuda_GpuMatG_push_const_GpuMat, std_vectorLcv_cuda_GpuMatG_insert_size_t_const_GpuMat,
	}
	vector_non_copy_or_bool! { clone core::GpuMat }
	
	pub type VectorOfKeyPoint = core::Vector<core::KeyPoint>;
	
	impl core::Vector<core::KeyPoint> {
		pub fn as_raw_VectorOfKeyPoint(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfKeyPoint(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::KeyPoint,
		std_vectorLcv_KeyPointG_new_const, std_vectorLcv_KeyPointG_delete,
		std_vectorLcv_KeyPointG_len_const, std_vectorLcv_KeyPointG_isEmpty_const,
		std_vectorLcv_KeyPointG_capacity_const, std_vectorLcv_KeyPointG_shrinkToFit,
		std_vectorLcv_KeyPointG_reserve_size_t, std_vectorLcv_KeyPointG_remove_size_t,
		std_vectorLcv_KeyPointG_swap_size_t_size_t, std_vectorLcv_KeyPointG_clear,
		std_vectorLcv_KeyPointG_get_const_size_t, std_vectorLcv_KeyPointG_set_size_t_const_KeyPoint,
		std_vectorLcv_KeyPointG_push_const_KeyPoint, std_vectorLcv_KeyPointG_insert_size_t_const_KeyPoint,
	}
	vector_non_copy_or_bool! { clone core::KeyPoint }
	
	pub type VectorOfMat = core::Vector<core::Mat>;
	
	impl core::Vector<core::Mat> {
		pub fn as_raw_VectorOfMat(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfMat(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::Mat,
		std_vectorLcv_MatG_new_const, std_vectorLcv_MatG_delete,
		std_vectorLcv_MatG_len_const, std_vectorLcv_MatG_isEmpty_const,
		std_vectorLcv_MatG_capacity_const, std_vectorLcv_MatG_shrinkToFit,
		std_vectorLcv_MatG_reserve_size_t, std_vectorLcv_MatG_remove_size_t,
		std_vectorLcv_MatG_swap_size_t_size_t, std_vectorLcv_MatG_clear,
		std_vectorLcv_MatG_get_const_size_t, std_vectorLcv_MatG_set_size_t_const_Mat,
		std_vectorLcv_MatG_push_const_Mat, std_vectorLcv_MatG_insert_size_t_const_Mat,
	}
	vector_non_copy_or_bool! { clone core::Mat }
	
	pub type VectorOfPlatformInfo = core::Vector<core::PlatformInfo>;
	
	impl core::Vector<core::PlatformInfo> {
		pub fn as_raw_VectorOfPlatformInfo(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfPlatformInfo(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::PlatformInfo,
		std_vectorLcv_ocl_PlatformInfoG_new_const, std_vectorLcv_ocl_PlatformInfoG_delete,
		std_vectorLcv_ocl_PlatformInfoG_len_const, std_vectorLcv_ocl_PlatformInfoG_isEmpty_const,
		std_vectorLcv_ocl_PlatformInfoG_capacity_const, std_vectorLcv_ocl_PlatformInfoG_shrinkToFit,
		std_vectorLcv_ocl_PlatformInfoG_reserve_size_t, std_vectorLcv_ocl_PlatformInfoG_remove_size_t,
		std_vectorLcv_ocl_PlatformInfoG_swap_size_t_size_t, std_vectorLcv_ocl_PlatformInfoG_clear,
		std_vectorLcv_ocl_PlatformInfoG_get_const_size_t, std_vectorLcv_ocl_PlatformInfoG_set_size_t_const_PlatformInfo,
		std_vectorLcv_ocl_PlatformInfoG_push_const_PlatformInfo, std_vectorLcv_ocl_PlatformInfoG_insert_size_t_const_PlatformInfo,
	}
	vector_non_copy_or_bool! { core::PlatformInfo }
	
	pub type VectorOfPoint = core::Vector<core::Point>;
	
	impl core::Vector<core::Point> {
		pub fn as_raw_VectorOfPoint(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfPoint(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::Point,
		std_vectorLcv_PointG_new_const, std_vectorLcv_PointG_delete,
		std_vectorLcv_PointG_len_const, std_vectorLcv_PointG_isEmpty_const,
		std_vectorLcv_PointG_capacity_const, std_vectorLcv_PointG_shrinkToFit,
		std_vectorLcv_PointG_reserve_size_t, std_vectorLcv_PointG_remove_size_t,
		std_vectorLcv_PointG_swap_size_t_size_t, std_vectorLcv_PointG_clear,
		std_vectorLcv_PointG_get_const_size_t, std_vectorLcv_PointG_set_size_t_const_Point,
		std_vectorLcv_PointG_push_const_Point, std_vectorLcv_PointG_insert_size_t_const_Point,
	}
	vector_copy_non_bool! { core::Point,
		std_vectorLcv_PointG_data_const, std_vectorLcv_PointG_dataMut, cv_fromSlice_const_const_PointX_size_t,
		std_vectorLcv_PointG_clone_const,
	}
	
	impl core::ToInputArray for core::Vector<core::Point> {
		#[inline]
		fn input_array(&self) -> Result<core::_InputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_PointG_inputArray_const(self.as_raw_VectorOfPoint(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToOutputArray for core::Vector<core::Point> {
		#[inline]
		fn output_array(&mut self) -> Result<core::_OutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_PointG_outputArray(self.as_raw_mut_VectorOfPoint(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_OutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToInputOutputArray for core::Vector<core::Point> {
		#[inline]
		fn input_output_array(&mut self) -> Result<core::_InputOutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_PointG_inputOutputArray(self.as_raw_mut_VectorOfPoint(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputOutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	input_output_array_ref_forward! { core::Vector<core::Point> }
	
	pub type VectorOfPoint2d = core::Vector<core::Point2d>;
	
	impl core::Vector<core::Point2d> {
		pub fn as_raw_VectorOfPoint2d(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfPoint2d(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::Point2d,
		std_vectorLcv_Point2dG_new_const, std_vectorLcv_Point2dG_delete,
		std_vectorLcv_Point2dG_len_const, std_vectorLcv_Point2dG_isEmpty_const,
		std_vectorLcv_Point2dG_capacity_const, std_vectorLcv_Point2dG_shrinkToFit,
		std_vectorLcv_Point2dG_reserve_size_t, std_vectorLcv_Point2dG_remove_size_t,
		std_vectorLcv_Point2dG_swap_size_t_size_t, std_vectorLcv_Point2dG_clear,
		std_vectorLcv_Point2dG_get_const_size_t, std_vectorLcv_Point2dG_set_size_t_const_Point2d,
		std_vectorLcv_Point2dG_push_const_Point2d, std_vectorLcv_Point2dG_insert_size_t_const_Point2d,
	}
	vector_copy_non_bool! { core::Point2d,
		std_vectorLcv_Point2dG_data_const, std_vectorLcv_Point2dG_dataMut, cv_fromSlice_const_const_Point2dX_size_t,
		std_vectorLcv_Point2dG_clone_const,
	}
	
	impl core::ToInputArray for core::Vector<core::Point2d> {
		#[inline]
		fn input_array(&self) -> Result<core::_InputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Point2dG_inputArray_const(self.as_raw_VectorOfPoint2d(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToOutputArray for core::Vector<core::Point2d> {
		#[inline]
		fn output_array(&mut self) -> Result<core::_OutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Point2dG_outputArray(self.as_raw_mut_VectorOfPoint2d(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_OutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToInputOutputArray for core::Vector<core::Point2d> {
		#[inline]
		fn input_output_array(&mut self) -> Result<core::_InputOutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Point2dG_inputOutputArray(self.as_raw_mut_VectorOfPoint2d(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputOutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	input_output_array_ref_forward! { core::Vector<core::Point2d> }
	
	pub type VectorOfPoint2f = core::Vector<core::Point2f>;
	
	impl core::Vector<core::Point2f> {
		pub fn as_raw_VectorOfPoint2f(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfPoint2f(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::Point2f,
		std_vectorLcv_Point2fG_new_const, std_vectorLcv_Point2fG_delete,
		std_vectorLcv_Point2fG_len_const, std_vectorLcv_Point2fG_isEmpty_const,
		std_vectorLcv_Point2fG_capacity_const, std_vectorLcv_Point2fG_shrinkToFit,
		std_vectorLcv_Point2fG_reserve_size_t, std_vectorLcv_Point2fG_remove_size_t,
		std_vectorLcv_Point2fG_swap_size_t_size_t, std_vectorLcv_Point2fG_clear,
		std_vectorLcv_Point2fG_get_const_size_t, std_vectorLcv_Point2fG_set_size_t_const_Point2f,
		std_vectorLcv_Point2fG_push_const_Point2f, std_vectorLcv_Point2fG_insert_size_t_const_Point2f,
	}
	vector_copy_non_bool! { core::Point2f,
		std_vectorLcv_Point2fG_data_const, std_vectorLcv_Point2fG_dataMut, cv_fromSlice_const_const_Point2fX_size_t,
		std_vectorLcv_Point2fG_clone_const,
	}
	
	impl core::ToInputArray for core::Vector<core::Point2f> {
		#[inline]
		fn input_array(&self) -> Result<core::_InputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Point2fG_inputArray_const(self.as_raw_VectorOfPoint2f(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToOutputArray for core::Vector<core::Point2f> {
		#[inline]
		fn output_array(&mut self) -> Result<core::_OutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Point2fG_outputArray(self.as_raw_mut_VectorOfPoint2f(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_OutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToInputOutputArray for core::Vector<core::Point2f> {
		#[inline]
		fn input_output_array(&mut self) -> Result<core::_InputOutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Point2fG_inputOutputArray(self.as_raw_mut_VectorOfPoint2f(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputOutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	input_output_array_ref_forward! { core::Vector<core::Point2f> }
	
	pub type VectorOfPoint3d = core::Vector<core::Point3d>;
	
	impl core::Vector<core::Point3d> {
		pub fn as_raw_VectorOfPoint3d(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfPoint3d(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::Point3d,
		std_vectorLcv_Point3dG_new_const, std_vectorLcv_Point3dG_delete,
		std_vectorLcv_Point3dG_len_const, std_vectorLcv_Point3dG_isEmpty_const,
		std_vectorLcv_Point3dG_capacity_const, std_vectorLcv_Point3dG_shrinkToFit,
		std_vectorLcv_Point3dG_reserve_size_t, std_vectorLcv_Point3dG_remove_size_t,
		std_vectorLcv_Point3dG_swap_size_t_size_t, std_vectorLcv_Point3dG_clear,
		std_vectorLcv_Point3dG_get_const_size_t, std_vectorLcv_Point3dG_set_size_t_const_Point3d,
		std_vectorLcv_Point3dG_push_const_Point3d, std_vectorLcv_Point3dG_insert_size_t_const_Point3d,
	}
	vector_copy_non_bool! { core::Point3d,
		std_vectorLcv_Point3dG_data_const, std_vectorLcv_Point3dG_dataMut, cv_fromSlice_const_const_Point3dX_size_t,
		std_vectorLcv_Point3dG_clone_const,
	}
	
	impl core::ToInputArray for core::Vector<core::Point3d> {
		#[inline]
		fn input_array(&self) -> Result<core::_InputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Point3dG_inputArray_const(self.as_raw_VectorOfPoint3d(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToOutputArray for core::Vector<core::Point3d> {
		#[inline]
		fn output_array(&mut self) -> Result<core::_OutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Point3dG_outputArray(self.as_raw_mut_VectorOfPoint3d(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_OutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToInputOutputArray for core::Vector<core::Point3d> {
		#[inline]
		fn input_output_array(&mut self) -> Result<core::_InputOutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Point3dG_inputOutputArray(self.as_raw_mut_VectorOfPoint3d(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputOutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	input_output_array_ref_forward! { core::Vector<core::Point3d> }
	
	pub type VectorOfPoint3f = core::Vector<core::Point3f>;
	
	impl core::Vector<core::Point3f> {
		pub fn as_raw_VectorOfPoint3f(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfPoint3f(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::Point3f,
		std_vectorLcv_Point3fG_new_const, std_vectorLcv_Point3fG_delete,
		std_vectorLcv_Point3fG_len_const, std_vectorLcv_Point3fG_isEmpty_const,
		std_vectorLcv_Point3fG_capacity_const, std_vectorLcv_Point3fG_shrinkToFit,
		std_vectorLcv_Point3fG_reserve_size_t, std_vectorLcv_Point3fG_remove_size_t,
		std_vectorLcv_Point3fG_swap_size_t_size_t, std_vectorLcv_Point3fG_clear,
		std_vectorLcv_Point3fG_get_const_size_t, std_vectorLcv_Point3fG_set_size_t_const_Point3f,
		std_vectorLcv_Point3fG_push_const_Point3f, std_vectorLcv_Point3fG_insert_size_t_const_Point3f,
	}
	vector_copy_non_bool! { core::Point3f,
		std_vectorLcv_Point3fG_data_const, std_vectorLcv_Point3fG_dataMut, cv_fromSlice_const_const_Point3fX_size_t,
		std_vectorLcv_Point3fG_clone_const,
	}
	
	impl core::ToInputArray for core::Vector<core::Point3f> {
		#[inline]
		fn input_array(&self) -> Result<core::_InputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Point3fG_inputArray_const(self.as_raw_VectorOfPoint3f(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToOutputArray for core::Vector<core::Point3f> {
		#[inline]
		fn output_array(&mut self) -> Result<core::_OutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Point3fG_outputArray(self.as_raw_mut_VectorOfPoint3f(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_OutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToInputOutputArray for core::Vector<core::Point3f> {
		#[inline]
		fn input_output_array(&mut self) -> Result<core::_InputOutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Point3fG_inputOutputArray(self.as_raw_mut_VectorOfPoint3f(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputOutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	input_output_array_ref_forward! { core::Vector<core::Point3f> }
	
	pub type VectorOfPoint3i = core::Vector<core::Point3i>;
	
	impl core::Vector<core::Point3i> {
		pub fn as_raw_VectorOfPoint3i(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfPoint3i(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::Point3i,
		std_vectorLcv_Point3iG_new_const, std_vectorLcv_Point3iG_delete,
		std_vectorLcv_Point3iG_len_const, std_vectorLcv_Point3iG_isEmpty_const,
		std_vectorLcv_Point3iG_capacity_const, std_vectorLcv_Point3iG_shrinkToFit,
		std_vectorLcv_Point3iG_reserve_size_t, std_vectorLcv_Point3iG_remove_size_t,
		std_vectorLcv_Point3iG_swap_size_t_size_t, std_vectorLcv_Point3iG_clear,
		std_vectorLcv_Point3iG_get_const_size_t, std_vectorLcv_Point3iG_set_size_t_const_Point3i,
		std_vectorLcv_Point3iG_push_const_Point3i, std_vectorLcv_Point3iG_insert_size_t_const_Point3i,
	}
	vector_copy_non_bool! { core::Point3i,
		std_vectorLcv_Point3iG_data_const, std_vectorLcv_Point3iG_dataMut, cv_fromSlice_const_const_Point3iX_size_t,
		std_vectorLcv_Point3iG_clone_const,
	}
	
	impl core::ToInputArray for core::Vector<core::Point3i> {
		#[inline]
		fn input_array(&self) -> Result<core::_InputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Point3iG_inputArray_const(self.as_raw_VectorOfPoint3i(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToOutputArray for core::Vector<core::Point3i> {
		#[inline]
		fn output_array(&mut self) -> Result<core::_OutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Point3iG_outputArray(self.as_raw_mut_VectorOfPoint3i(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_OutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToInputOutputArray for core::Vector<core::Point3i> {
		#[inline]
		fn input_output_array(&mut self) -> Result<core::_InputOutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Point3iG_inputOutputArray(self.as_raw_mut_VectorOfPoint3i(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputOutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	input_output_array_ref_forward! { core::Vector<core::Point3i> }
	
	pub type VectorOfRange = core::Vector<core::Range>;
	
	impl core::Vector<core::Range> {
		pub fn as_raw_VectorOfRange(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfRange(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::Range,
		std_vectorLcv_RangeG_new_const, std_vectorLcv_RangeG_delete,
		std_vectorLcv_RangeG_len_const, std_vectorLcv_RangeG_isEmpty_const,
		std_vectorLcv_RangeG_capacity_const, std_vectorLcv_RangeG_shrinkToFit,
		std_vectorLcv_RangeG_reserve_size_t, std_vectorLcv_RangeG_remove_size_t,
		std_vectorLcv_RangeG_swap_size_t_size_t, std_vectorLcv_RangeG_clear,
		std_vectorLcv_RangeG_get_const_size_t, std_vectorLcv_RangeG_set_size_t_const_Range,
		std_vectorLcv_RangeG_push_const_Range, std_vectorLcv_RangeG_insert_size_t_const_Range,
	}
	vector_non_copy_or_bool! { core::Range }
	
	pub type VectorOfRect = core::Vector<core::Rect>;
	
	impl core::Vector<core::Rect> {
		pub fn as_raw_VectorOfRect(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfRect(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::Rect,
		std_vectorLcv_RectG_new_const, std_vectorLcv_RectG_delete,
		std_vectorLcv_RectG_len_const, std_vectorLcv_RectG_isEmpty_const,
		std_vectorLcv_RectG_capacity_const, std_vectorLcv_RectG_shrinkToFit,
		std_vectorLcv_RectG_reserve_size_t, std_vectorLcv_RectG_remove_size_t,
		std_vectorLcv_RectG_swap_size_t_size_t, std_vectorLcv_RectG_clear,
		std_vectorLcv_RectG_get_const_size_t, std_vectorLcv_RectG_set_size_t_const_Rect,
		std_vectorLcv_RectG_push_const_Rect, std_vectorLcv_RectG_insert_size_t_const_Rect,
	}
	vector_copy_non_bool! { core::Rect,
		std_vectorLcv_RectG_data_const, std_vectorLcv_RectG_dataMut, cv_fromSlice_const_const_RectX_size_t,
		std_vectorLcv_RectG_clone_const,
	}
	
	impl core::ToInputArray for core::Vector<core::Rect> {
		#[inline]
		fn input_array(&self) -> Result<core::_InputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_RectG_inputArray_const(self.as_raw_VectorOfRect(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToOutputArray for core::Vector<core::Rect> {
		#[inline]
		fn output_array(&mut self) -> Result<core::_OutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_RectG_outputArray(self.as_raw_mut_VectorOfRect(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_OutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToInputOutputArray for core::Vector<core::Rect> {
		#[inline]
		fn input_output_array(&mut self) -> Result<core::_InputOutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_RectG_inputOutputArray(self.as_raw_mut_VectorOfRect(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputOutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	input_output_array_ref_forward! { core::Vector<core::Rect> }
	
	pub type VectorOfRotatedRect = core::Vector<core::RotatedRect>;
	
	impl core::Vector<core::RotatedRect> {
		pub fn as_raw_VectorOfRotatedRect(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfRotatedRect(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::RotatedRect,
		std_vectorLcv_RotatedRectG_new_const, std_vectorLcv_RotatedRectG_delete,
		std_vectorLcv_RotatedRectG_len_const, std_vectorLcv_RotatedRectG_isEmpty_const,
		std_vectorLcv_RotatedRectG_capacity_const, std_vectorLcv_RotatedRectG_shrinkToFit,
		std_vectorLcv_RotatedRectG_reserve_size_t, std_vectorLcv_RotatedRectG_remove_size_t,
		std_vectorLcv_RotatedRectG_swap_size_t_size_t, std_vectorLcv_RotatedRectG_clear,
		std_vectorLcv_RotatedRectG_get_const_size_t, std_vectorLcv_RotatedRectG_set_size_t_const_RotatedRect,
		std_vectorLcv_RotatedRectG_push_const_RotatedRect, std_vectorLcv_RotatedRectG_insert_size_t_const_RotatedRect,
	}
	vector_copy_non_bool! { core::RotatedRect,
		std_vectorLcv_RotatedRectG_data_const, std_vectorLcv_RotatedRectG_dataMut, cv_fromSlice_const_const_RotatedRectX_size_t,
		std_vectorLcv_RotatedRectG_clone_const,
	}
	
	pub type VectorOfString = core::Vector<String>;
	
	impl core::Vector<String> {
		pub fn as_raw_VectorOfString(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfString(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { String,
		std_vectorLcv_StringG_new_const, std_vectorLcv_StringG_delete,
		std_vectorLcv_StringG_len_const, std_vectorLcv_StringG_isEmpty_const,
		std_vectorLcv_StringG_capacity_const, std_vectorLcv_StringG_shrinkToFit,
		std_vectorLcv_StringG_reserve_size_t, std_vectorLcv_StringG_remove_size_t,
		std_vectorLcv_StringG_swap_size_t_size_t, std_vectorLcv_StringG_clear,
		std_vectorLcv_StringG_get_const_size_t, std_vectorLcv_StringG_set_size_t_const_String,
		std_vectorLcv_StringG_push_const_String, std_vectorLcv_StringG_insert_size_t_const_String,
	}
	vector_non_copy_or_bool! { String }
	
	pub type VectorOfUMat = core::Vector<core::UMat>;
	
	impl core::Vector<core::UMat> {
		pub fn as_raw_VectorOfUMat(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfUMat(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::UMat,
		std_vectorLcv_UMatG_new_const, std_vectorLcv_UMatG_delete,
		std_vectorLcv_UMatG_len_const, std_vectorLcv_UMatG_isEmpty_const,
		std_vectorLcv_UMatG_capacity_const, std_vectorLcv_UMatG_shrinkToFit,
		std_vectorLcv_UMatG_reserve_size_t, std_vectorLcv_UMatG_remove_size_t,
		std_vectorLcv_UMatG_swap_size_t_size_t, std_vectorLcv_UMatG_clear,
		std_vectorLcv_UMatG_get_const_size_t, std_vectorLcv_UMatG_set_size_t_const_UMat,
		std_vectorLcv_UMatG_push_const_UMat, std_vectorLcv_UMatG_insert_size_t_const_UMat,
	}
	vector_non_copy_or_bool! { clone core::UMat }
	
	pub type VectorOfVec2d = core::Vector<core::Vec2d>;
	
	impl core::Vector<core::Vec2d> {
		pub fn as_raw_VectorOfVec2d(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfVec2d(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::Vec2d,
		std_vectorLcv_Vec2dG_new_const, std_vectorLcv_Vec2dG_delete,
		std_vectorLcv_Vec2dG_len_const, std_vectorLcv_Vec2dG_isEmpty_const,
		std_vectorLcv_Vec2dG_capacity_const, std_vectorLcv_Vec2dG_shrinkToFit,
		std_vectorLcv_Vec2dG_reserve_size_t, std_vectorLcv_Vec2dG_remove_size_t,
		std_vectorLcv_Vec2dG_swap_size_t_size_t, std_vectorLcv_Vec2dG_clear,
		std_vectorLcv_Vec2dG_get_const_size_t, std_vectorLcv_Vec2dG_set_size_t_const_Vec2d,
		std_vectorLcv_Vec2dG_push_const_Vec2d, std_vectorLcv_Vec2dG_insert_size_t_const_Vec2d,
	}
	vector_copy_non_bool! { core::Vec2d,
		std_vectorLcv_Vec2dG_data_const, std_vectorLcv_Vec2dG_dataMut, cv_fromSlice_const_const_Vec2dX_size_t,
		std_vectorLcv_Vec2dG_clone_const,
	}
	
	impl core::ToInputArray for core::Vector<core::Vec2d> {
		#[inline]
		fn input_array(&self) -> Result<core::_InputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Vec2dG_inputArray_const(self.as_raw_VectorOfVec2d(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToOutputArray for core::Vector<core::Vec2d> {
		#[inline]
		fn output_array(&mut self) -> Result<core::_OutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Vec2dG_outputArray(self.as_raw_mut_VectorOfVec2d(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_OutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToInputOutputArray for core::Vector<core::Vec2d> {
		#[inline]
		fn input_output_array(&mut self) -> Result<core::_InputOutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Vec2dG_inputOutputArray(self.as_raw_mut_VectorOfVec2d(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputOutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	input_output_array_ref_forward! { core::Vector<core::Vec2d> }
	
	pub type VectorOfVec2f = core::Vector<core::Vec2f>;
	
	impl core::Vector<core::Vec2f> {
		pub fn as_raw_VectorOfVec2f(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfVec2f(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::Vec2f,
		std_vectorLcv_Vec2fG_new_const, std_vectorLcv_Vec2fG_delete,
		std_vectorLcv_Vec2fG_len_const, std_vectorLcv_Vec2fG_isEmpty_const,
		std_vectorLcv_Vec2fG_capacity_const, std_vectorLcv_Vec2fG_shrinkToFit,
		std_vectorLcv_Vec2fG_reserve_size_t, std_vectorLcv_Vec2fG_remove_size_t,
		std_vectorLcv_Vec2fG_swap_size_t_size_t, std_vectorLcv_Vec2fG_clear,
		std_vectorLcv_Vec2fG_get_const_size_t, std_vectorLcv_Vec2fG_set_size_t_const_Vec2f,
		std_vectorLcv_Vec2fG_push_const_Vec2f, std_vectorLcv_Vec2fG_insert_size_t_const_Vec2f,
	}
	vector_copy_non_bool! { core::Vec2f,
		std_vectorLcv_Vec2fG_data_const, std_vectorLcv_Vec2fG_dataMut, cv_fromSlice_const_const_Vec2fX_size_t,
		std_vectorLcv_Vec2fG_clone_const,
	}
	
	impl core::ToInputArray for core::Vector<core::Vec2f> {
		#[inline]
		fn input_array(&self) -> Result<core::_InputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Vec2fG_inputArray_const(self.as_raw_VectorOfVec2f(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToOutputArray for core::Vector<core::Vec2f> {
		#[inline]
		fn output_array(&mut self) -> Result<core::_OutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Vec2fG_outputArray(self.as_raw_mut_VectorOfVec2f(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_OutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToInputOutputArray for core::Vector<core::Vec2f> {
		#[inline]
		fn input_output_array(&mut self) -> Result<core::_InputOutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Vec2fG_inputOutputArray(self.as_raw_mut_VectorOfVec2f(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputOutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	input_output_array_ref_forward! { core::Vector<core::Vec2f> }
	
	pub type VectorOfVec3d = core::Vector<core::Vec3d>;
	
	impl core::Vector<core::Vec3d> {
		pub fn as_raw_VectorOfVec3d(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfVec3d(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::Vec3d,
		std_vectorLcv_Vec3dG_new_const, std_vectorLcv_Vec3dG_delete,
		std_vectorLcv_Vec3dG_len_const, std_vectorLcv_Vec3dG_isEmpty_const,
		std_vectorLcv_Vec3dG_capacity_const, std_vectorLcv_Vec3dG_shrinkToFit,
		std_vectorLcv_Vec3dG_reserve_size_t, std_vectorLcv_Vec3dG_remove_size_t,
		std_vectorLcv_Vec3dG_swap_size_t_size_t, std_vectorLcv_Vec3dG_clear,
		std_vectorLcv_Vec3dG_get_const_size_t, std_vectorLcv_Vec3dG_set_size_t_const_Vec3d,
		std_vectorLcv_Vec3dG_push_const_Vec3d, std_vectorLcv_Vec3dG_insert_size_t_const_Vec3d,
	}
	vector_copy_non_bool! { core::Vec3d,
		std_vectorLcv_Vec3dG_data_const, std_vectorLcv_Vec3dG_dataMut, cv_fromSlice_const_const_Vec3dX_size_t,
		std_vectorLcv_Vec3dG_clone_const,
	}
	
	impl core::ToInputArray for core::Vector<core::Vec3d> {
		#[inline]
		fn input_array(&self) -> Result<core::_InputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Vec3dG_inputArray_const(self.as_raw_VectorOfVec3d(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToOutputArray for core::Vector<core::Vec3d> {
		#[inline]
		fn output_array(&mut self) -> Result<core::_OutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Vec3dG_outputArray(self.as_raw_mut_VectorOfVec3d(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_OutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToInputOutputArray for core::Vector<core::Vec3d> {
		#[inline]
		fn input_output_array(&mut self) -> Result<core::_InputOutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Vec3dG_inputOutputArray(self.as_raw_mut_VectorOfVec3d(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputOutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	input_output_array_ref_forward! { core::Vector<core::Vec3d> }
	
	pub type VectorOfVec3f = core::Vector<core::Vec3f>;
	
	impl core::Vector<core::Vec3f> {
		pub fn as_raw_VectorOfVec3f(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfVec3f(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::Vec3f,
		std_vectorLcv_Vec3fG_new_const, std_vectorLcv_Vec3fG_delete,
		std_vectorLcv_Vec3fG_len_const, std_vectorLcv_Vec3fG_isEmpty_const,
		std_vectorLcv_Vec3fG_capacity_const, std_vectorLcv_Vec3fG_shrinkToFit,
		std_vectorLcv_Vec3fG_reserve_size_t, std_vectorLcv_Vec3fG_remove_size_t,
		std_vectorLcv_Vec3fG_swap_size_t_size_t, std_vectorLcv_Vec3fG_clear,
		std_vectorLcv_Vec3fG_get_const_size_t, std_vectorLcv_Vec3fG_set_size_t_const_Vec3f,
		std_vectorLcv_Vec3fG_push_const_Vec3f, std_vectorLcv_Vec3fG_insert_size_t_const_Vec3f,
	}
	vector_copy_non_bool! { core::Vec3f,
		std_vectorLcv_Vec3fG_data_const, std_vectorLcv_Vec3fG_dataMut, cv_fromSlice_const_const_Vec3fX_size_t,
		std_vectorLcv_Vec3fG_clone_const,
	}
	
	impl core::ToInputArray for core::Vector<core::Vec3f> {
		#[inline]
		fn input_array(&self) -> Result<core::_InputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Vec3fG_inputArray_const(self.as_raw_VectorOfVec3f(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToOutputArray for core::Vector<core::Vec3f> {
		#[inline]
		fn output_array(&mut self) -> Result<core::_OutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Vec3fG_outputArray(self.as_raw_mut_VectorOfVec3f(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_OutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToInputOutputArray for core::Vector<core::Vec3f> {
		#[inline]
		fn input_output_array(&mut self) -> Result<core::_InputOutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Vec3fG_inputOutputArray(self.as_raw_mut_VectorOfVec3f(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputOutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	input_output_array_ref_forward! { core::Vector<core::Vec3f> }
	
	pub type VectorOfVec4f = core::Vector<core::Vec4f>;
	
	impl core::Vector<core::Vec4f> {
		pub fn as_raw_VectorOfVec4f(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfVec4f(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::Vec4f,
		std_vectorLcv_Vec4fG_new_const, std_vectorLcv_Vec4fG_delete,
		std_vectorLcv_Vec4fG_len_const, std_vectorLcv_Vec4fG_isEmpty_const,
		std_vectorLcv_Vec4fG_capacity_const, std_vectorLcv_Vec4fG_shrinkToFit,
		std_vectorLcv_Vec4fG_reserve_size_t, std_vectorLcv_Vec4fG_remove_size_t,
		std_vectorLcv_Vec4fG_swap_size_t_size_t, std_vectorLcv_Vec4fG_clear,
		std_vectorLcv_Vec4fG_get_const_size_t, std_vectorLcv_Vec4fG_set_size_t_const_Vec4f,
		std_vectorLcv_Vec4fG_push_const_Vec4f, std_vectorLcv_Vec4fG_insert_size_t_const_Vec4f,
	}
	vector_copy_non_bool! { core::Vec4f,
		std_vectorLcv_Vec4fG_data_const, std_vectorLcv_Vec4fG_dataMut, cv_fromSlice_const_const_Vec4fX_size_t,
		std_vectorLcv_Vec4fG_clone_const,
	}
	
	impl core::ToInputArray for core::Vector<core::Vec4f> {
		#[inline]
		fn input_array(&self) -> Result<core::_InputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Vec4fG_inputArray_const(self.as_raw_VectorOfVec4f(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToOutputArray for core::Vector<core::Vec4f> {
		#[inline]
		fn output_array(&mut self) -> Result<core::_OutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Vec4fG_outputArray(self.as_raw_mut_VectorOfVec4f(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_OutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToInputOutputArray for core::Vector<core::Vec4f> {
		#[inline]
		fn input_output_array(&mut self) -> Result<core::_InputOutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Vec4fG_inputOutputArray(self.as_raw_mut_VectorOfVec4f(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputOutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	input_output_array_ref_forward! { core::Vector<core::Vec4f> }
	
	pub type VectorOfVec4i = core::Vector<core::Vec4i>;
	
	impl core::Vector<core::Vec4i> {
		pub fn as_raw_VectorOfVec4i(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfVec4i(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::Vec4i,
		std_vectorLcv_Vec4iG_new_const, std_vectorLcv_Vec4iG_delete,
		std_vectorLcv_Vec4iG_len_const, std_vectorLcv_Vec4iG_isEmpty_const,
		std_vectorLcv_Vec4iG_capacity_const, std_vectorLcv_Vec4iG_shrinkToFit,
		std_vectorLcv_Vec4iG_reserve_size_t, std_vectorLcv_Vec4iG_remove_size_t,
		std_vectorLcv_Vec4iG_swap_size_t_size_t, std_vectorLcv_Vec4iG_clear,
		std_vectorLcv_Vec4iG_get_const_size_t, std_vectorLcv_Vec4iG_set_size_t_const_Vec4i,
		std_vectorLcv_Vec4iG_push_const_Vec4i, std_vectorLcv_Vec4iG_insert_size_t_const_Vec4i,
	}
	vector_copy_non_bool! { core::Vec4i,
		std_vectorLcv_Vec4iG_data_const, std_vectorLcv_Vec4iG_dataMut, cv_fromSlice_const_const_Vec4iX_size_t,
		std_vectorLcv_Vec4iG_clone_const,
	}
	
	impl core::ToInputArray for core::Vector<core::Vec4i> {
		#[inline]
		fn input_array(&self) -> Result<core::_InputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Vec4iG_inputArray_const(self.as_raw_VectorOfVec4i(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToOutputArray for core::Vector<core::Vec4i> {
		#[inline]
		fn output_array(&mut self) -> Result<core::_OutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Vec4iG_outputArray(self.as_raw_mut_VectorOfVec4i(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_OutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToInputOutputArray for core::Vector<core::Vec4i> {
		#[inline]
		fn input_output_array(&mut self) -> Result<core::_InputOutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Vec4iG_inputOutputArray(self.as_raw_mut_VectorOfVec4i(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputOutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	input_output_array_ref_forward! { core::Vector<core::Vec4i> }
	
	pub type VectorOfVec6f = core::Vector<core::Vec6f>;
	
	impl core::Vector<core::Vec6f> {
		pub fn as_raw_VectorOfVec6f(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfVec6f(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::Vec6f,
		std_vectorLcv_Vec6fG_new_const, std_vectorLcv_Vec6fG_delete,
		std_vectorLcv_Vec6fG_len_const, std_vectorLcv_Vec6fG_isEmpty_const,
		std_vectorLcv_Vec6fG_capacity_const, std_vectorLcv_Vec6fG_shrinkToFit,
		std_vectorLcv_Vec6fG_reserve_size_t, std_vectorLcv_Vec6fG_remove_size_t,
		std_vectorLcv_Vec6fG_swap_size_t_size_t, std_vectorLcv_Vec6fG_clear,
		std_vectorLcv_Vec6fG_get_const_size_t, std_vectorLcv_Vec6fG_set_size_t_const_Vec6f,
		std_vectorLcv_Vec6fG_push_const_Vec6f, std_vectorLcv_Vec6fG_insert_size_t_const_Vec6f,
	}
	vector_copy_non_bool! { core::Vec6f,
		std_vectorLcv_Vec6fG_data_const, std_vectorLcv_Vec6fG_dataMut, cv_fromSlice_const_const_Vec6fX_size_t,
		std_vectorLcv_Vec6fG_clone_const,
	}
	
	impl core::ToInputArray for core::Vector<core::Vec6f> {
		#[inline]
		fn input_array(&self) -> Result<core::_InputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Vec6fG_inputArray_const(self.as_raw_VectorOfVec6f(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToOutputArray for core::Vector<core::Vec6f> {
		#[inline]
		fn output_array(&mut self) -> Result<core::_OutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Vec6fG_outputArray(self.as_raw_mut_VectorOfVec6f(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_OutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToInputOutputArray for core::Vector<core::Vec6f> {
		#[inline]
		fn input_output_array(&mut self) -> Result<core::_InputOutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLcv_Vec6fG_inputOutputArray(self.as_raw_mut_VectorOfVec6f(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputOutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	input_output_array_ref_forward! { core::Vector<core::Vec6f> }
	
	pub type VectorOfVectorOfDMatch = core::Vector<core::Vector<core::DMatch>>;
	
	impl core::Vector<core::Vector<core::DMatch>> {
		pub fn as_raw_VectorOfVectorOfDMatch(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfVectorOfDMatch(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::Vector<core::DMatch>,
		std_vectorLstd_vectorLcv_DMatchGG_new_const, std_vectorLstd_vectorLcv_DMatchGG_delete,
		std_vectorLstd_vectorLcv_DMatchGG_len_const, std_vectorLstd_vectorLcv_DMatchGG_isEmpty_const,
		std_vectorLstd_vectorLcv_DMatchGG_capacity_const, std_vectorLstd_vectorLcv_DMatchGG_shrinkToFit,
		std_vectorLstd_vectorLcv_DMatchGG_reserve_size_t, std_vectorLstd_vectorLcv_DMatchGG_remove_size_t,
		std_vectorLstd_vectorLcv_DMatchGG_swap_size_t_size_t, std_vectorLstd_vectorLcv_DMatchGG_clear,
		std_vectorLstd_vectorLcv_DMatchGG_get_const_size_t, std_vectorLstd_vectorLcv_DMatchGG_set_size_t_const_vectorLDMatchG,
		std_vectorLstd_vectorLcv_DMatchGG_push_const_vectorLDMatchG, std_vectorLstd_vectorLcv_DMatchGG_insert_size_t_const_vectorLDMatchG,
	}
	vector_non_copy_or_bool! { clone core::Vector<core::DMatch> }
	
	pub type VectorOfVectorOfKeyPoint = core::Vector<core::Vector<core::KeyPoint>>;
	
	impl core::Vector<core::Vector<core::KeyPoint>> {
		pub fn as_raw_VectorOfVectorOfKeyPoint(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfVectorOfKeyPoint(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::Vector<core::KeyPoint>,
		std_vectorLstd_vectorLcv_KeyPointGG_new_const, std_vectorLstd_vectorLcv_KeyPointGG_delete,
		std_vectorLstd_vectorLcv_KeyPointGG_len_const, std_vectorLstd_vectorLcv_KeyPointGG_isEmpty_const,
		std_vectorLstd_vectorLcv_KeyPointGG_capacity_const, std_vectorLstd_vectorLcv_KeyPointGG_shrinkToFit,
		std_vectorLstd_vectorLcv_KeyPointGG_reserve_size_t, std_vectorLstd_vectorLcv_KeyPointGG_remove_size_t,
		std_vectorLstd_vectorLcv_KeyPointGG_swap_size_t_size_t, std_vectorLstd_vectorLcv_KeyPointGG_clear,
		std_vectorLstd_vectorLcv_KeyPointGG_get_const_size_t, std_vectorLstd_vectorLcv_KeyPointGG_set_size_t_const_vectorLKeyPointG,
		std_vectorLstd_vectorLcv_KeyPointGG_push_const_vectorLKeyPointG, std_vectorLstd_vectorLcv_KeyPointGG_insert_size_t_const_vectorLKeyPointG,
	}
	vector_non_copy_or_bool! { clone core::Vector<core::KeyPoint> }
	
	pub type VectorOfVectorOfPoint = core::Vector<core::Vector<core::Point>>;
	
	impl core::Vector<core::Vector<core::Point>> {
		pub fn as_raw_VectorOfVectorOfPoint(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfVectorOfPoint(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::Vector<core::Point>,
		std_vectorLstd_vectorLcv_PointGG_new_const, std_vectorLstd_vectorLcv_PointGG_delete,
		std_vectorLstd_vectorLcv_PointGG_len_const, std_vectorLstd_vectorLcv_PointGG_isEmpty_const,
		std_vectorLstd_vectorLcv_PointGG_capacity_const, std_vectorLstd_vectorLcv_PointGG_shrinkToFit,
		std_vectorLstd_vectorLcv_PointGG_reserve_size_t, std_vectorLstd_vectorLcv_PointGG_remove_size_t,
		std_vectorLstd_vectorLcv_PointGG_swap_size_t_size_t, std_vectorLstd_vectorLcv_PointGG_clear,
		std_vectorLstd_vectorLcv_PointGG_get_const_size_t, std_vectorLstd_vectorLcv_PointGG_set_size_t_const_vectorLPointG,
		std_vectorLstd_vectorLcv_PointGG_push_const_vectorLPointG, std_vectorLstd_vectorLcv_PointGG_insert_size_t_const_vectorLPointG,
	}
	vector_non_copy_or_bool! { clone core::Vector<core::Point> }
	
	impl core::ToInputArray for core::Vector<core::Vector<core::Point>> {
		#[inline]
		fn input_array(&self) -> Result<core::_InputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLstd_vectorLcv_PointGG_inputArray_const(self.as_raw_VectorOfVectorOfPoint(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToOutputArray for core::Vector<core::Vector<core::Point>> {
		#[inline]
		fn output_array(&mut self) -> Result<core::_OutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLstd_vectorLcv_PointGG_outputArray(self.as_raw_mut_VectorOfVectorOfPoint(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_OutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToInputOutputArray for core::Vector<core::Vector<core::Point>> {
		#[inline]
		fn input_output_array(&mut self) -> Result<core::_InputOutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLstd_vectorLcv_PointGG_inputOutputArray(self.as_raw_mut_VectorOfVectorOfPoint(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputOutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	input_output_array_ref_forward! { core::Vector<core::Vector<core::Point>> }
	
	pub type VectorOfVectorOfPoint2f = core::Vector<core::Vector<core::Point2f>>;
	
	impl core::Vector<core::Vector<core::Point2f>> {
		pub fn as_raw_VectorOfVectorOfPoint2f(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfVectorOfPoint2f(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::Vector<core::Point2f>,
		std_vectorLstd_vectorLcv_Point2fGG_new_const, std_vectorLstd_vectorLcv_Point2fGG_delete,
		std_vectorLstd_vectorLcv_Point2fGG_len_const, std_vectorLstd_vectorLcv_Point2fGG_isEmpty_const,
		std_vectorLstd_vectorLcv_Point2fGG_capacity_const, std_vectorLstd_vectorLcv_Point2fGG_shrinkToFit,
		std_vectorLstd_vectorLcv_Point2fGG_reserve_size_t, std_vectorLstd_vectorLcv_Point2fGG_remove_size_t,
		std_vectorLstd_vectorLcv_Point2fGG_swap_size_t_size_t, std_vectorLstd_vectorLcv_Point2fGG_clear,
		std_vectorLstd_vectorLcv_Point2fGG_get_const_size_t, std_vectorLstd_vectorLcv_Point2fGG_set_size_t_const_vectorLPoint2fG,
		std_vectorLstd_vectorLcv_Point2fGG_push_const_vectorLPoint2fG, std_vectorLstd_vectorLcv_Point2fGG_insert_size_t_const_vectorLPoint2fG,
	}
	vector_non_copy_or_bool! { clone core::Vector<core::Point2f> }
	
	impl core::ToInputArray for core::Vector<core::Vector<core::Point2f>> {
		#[inline]
		fn input_array(&self) -> Result<core::_InputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLstd_vectorLcv_Point2fGG_inputArray_const(self.as_raw_VectorOfVectorOfPoint2f(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToOutputArray for core::Vector<core::Vector<core::Point2f>> {
		#[inline]
		fn output_array(&mut self) -> Result<core::_OutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLstd_vectorLcv_Point2fGG_outputArray(self.as_raw_mut_VectorOfVectorOfPoint2f(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_OutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToInputOutputArray for core::Vector<core::Vector<core::Point2f>> {
		#[inline]
		fn input_output_array(&mut self) -> Result<core::_InputOutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLstd_vectorLcv_Point2fGG_inputOutputArray(self.as_raw_mut_VectorOfVectorOfPoint2f(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputOutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	input_output_array_ref_forward! { core::Vector<core::Vector<core::Point2f>> }
	
	pub type VectorOfVectorOfPoint3d = core::Vector<core::Vector<core::Point3d>>;
	
	impl core::Vector<core::Vector<core::Point3d>> {
		pub fn as_raw_VectorOfVectorOfPoint3d(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfVectorOfPoint3d(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::Vector<core::Point3d>,
		std_vectorLstd_vectorLcv_Point3dGG_new_const, std_vectorLstd_vectorLcv_Point3dGG_delete,
		std_vectorLstd_vectorLcv_Point3dGG_len_const, std_vectorLstd_vectorLcv_Point3dGG_isEmpty_const,
		std_vectorLstd_vectorLcv_Point3dGG_capacity_const, std_vectorLstd_vectorLcv_Point3dGG_shrinkToFit,
		std_vectorLstd_vectorLcv_Point3dGG_reserve_size_t, std_vectorLstd_vectorLcv_Point3dGG_remove_size_t,
		std_vectorLstd_vectorLcv_Point3dGG_swap_size_t_size_t, std_vectorLstd_vectorLcv_Point3dGG_clear,
		std_vectorLstd_vectorLcv_Point3dGG_get_const_size_t, std_vectorLstd_vectorLcv_Point3dGG_set_size_t_const_vectorLPoint3dG,
		std_vectorLstd_vectorLcv_Point3dGG_push_const_vectorLPoint3dG, std_vectorLstd_vectorLcv_Point3dGG_insert_size_t_const_vectorLPoint3dG,
	}
	vector_non_copy_or_bool! { clone core::Vector<core::Point3d> }
	
	impl core::ToInputArray for core::Vector<core::Vector<core::Point3d>> {
		#[inline]
		fn input_array(&self) -> Result<core::_InputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLstd_vectorLcv_Point3dGG_inputArray_const(self.as_raw_VectorOfVectorOfPoint3d(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToOutputArray for core::Vector<core::Vector<core::Point3d>> {
		#[inline]
		fn output_array(&mut self) -> Result<core::_OutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLstd_vectorLcv_Point3dGG_outputArray(self.as_raw_mut_VectorOfVectorOfPoint3d(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_OutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToInputOutputArray for core::Vector<core::Vector<core::Point3d>> {
		#[inline]
		fn input_output_array(&mut self) -> Result<core::_InputOutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLstd_vectorLcv_Point3dGG_inputOutputArray(self.as_raw_mut_VectorOfVectorOfPoint3d(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputOutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	input_output_array_ref_forward! { core::Vector<core::Vector<core::Point3d>> }
	
	pub type VectorOfVectorOfPoint3f = core::Vector<core::Vector<core::Point3f>>;
	
	impl core::Vector<core::Vector<core::Point3f>> {
		pub fn as_raw_VectorOfVectorOfPoint3f(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfVectorOfPoint3f(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::Vector<core::Point3f>,
		std_vectorLstd_vectorLcv_Point3fGG_new_const, std_vectorLstd_vectorLcv_Point3fGG_delete,
		std_vectorLstd_vectorLcv_Point3fGG_len_const, std_vectorLstd_vectorLcv_Point3fGG_isEmpty_const,
		std_vectorLstd_vectorLcv_Point3fGG_capacity_const, std_vectorLstd_vectorLcv_Point3fGG_shrinkToFit,
		std_vectorLstd_vectorLcv_Point3fGG_reserve_size_t, std_vectorLstd_vectorLcv_Point3fGG_remove_size_t,
		std_vectorLstd_vectorLcv_Point3fGG_swap_size_t_size_t, std_vectorLstd_vectorLcv_Point3fGG_clear,
		std_vectorLstd_vectorLcv_Point3fGG_get_const_size_t, std_vectorLstd_vectorLcv_Point3fGG_set_size_t_const_vectorLPoint3fG,
		std_vectorLstd_vectorLcv_Point3fGG_push_const_vectorLPoint3fG, std_vectorLstd_vectorLcv_Point3fGG_insert_size_t_const_vectorLPoint3fG,
	}
	vector_non_copy_or_bool! { clone core::Vector<core::Point3f> }
	
	impl core::ToInputArray for core::Vector<core::Vector<core::Point3f>> {
		#[inline]
		fn input_array(&self) -> Result<core::_InputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLstd_vectorLcv_Point3fGG_inputArray_const(self.as_raw_VectorOfVectorOfPoint3f(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToOutputArray for core::Vector<core::Vector<core::Point3f>> {
		#[inline]
		fn output_array(&mut self) -> Result<core::_OutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLstd_vectorLcv_Point3fGG_outputArray(self.as_raw_mut_VectorOfVectorOfPoint3f(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_OutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToInputOutputArray for core::Vector<core::Vector<core::Point3f>> {
		#[inline]
		fn input_output_array(&mut self) -> Result<core::_InputOutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLstd_vectorLcv_Point3fGG_inputOutputArray(self.as_raw_mut_VectorOfVectorOfPoint3f(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputOutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	input_output_array_ref_forward! { core::Vector<core::Vector<core::Point3f>> }
	
	pub type VectorOfVectorOfPoint3i = core::Vector<core::Vector<core::Point3i>>;
	
	impl core::Vector<core::Vector<core::Point3i>> {
		pub fn as_raw_VectorOfVectorOfPoint3i(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfVectorOfPoint3i(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::Vector<core::Point3i>,
		std_vectorLstd_vectorLcv_Point3iGG_new_const, std_vectorLstd_vectorLcv_Point3iGG_delete,
		std_vectorLstd_vectorLcv_Point3iGG_len_const, std_vectorLstd_vectorLcv_Point3iGG_isEmpty_const,
		std_vectorLstd_vectorLcv_Point3iGG_capacity_const, std_vectorLstd_vectorLcv_Point3iGG_shrinkToFit,
		std_vectorLstd_vectorLcv_Point3iGG_reserve_size_t, std_vectorLstd_vectorLcv_Point3iGG_remove_size_t,
		std_vectorLstd_vectorLcv_Point3iGG_swap_size_t_size_t, std_vectorLstd_vectorLcv_Point3iGG_clear,
		std_vectorLstd_vectorLcv_Point3iGG_get_const_size_t, std_vectorLstd_vectorLcv_Point3iGG_set_size_t_const_vectorLPoint3iG,
		std_vectorLstd_vectorLcv_Point3iGG_push_const_vectorLPoint3iG, std_vectorLstd_vectorLcv_Point3iGG_insert_size_t_const_vectorLPoint3iG,
	}
	vector_non_copy_or_bool! { clone core::Vector<core::Point3i> }
	
	impl core::ToInputArray for core::Vector<core::Vector<core::Point3i>> {
		#[inline]
		fn input_array(&self) -> Result<core::_InputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLstd_vectorLcv_Point3iGG_inputArray_const(self.as_raw_VectorOfVectorOfPoint3i(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToOutputArray for core::Vector<core::Vector<core::Point3i>> {
		#[inline]
		fn output_array(&mut self) -> Result<core::_OutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLstd_vectorLcv_Point3iGG_outputArray(self.as_raw_mut_VectorOfVectorOfPoint3i(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_OutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToInputOutputArray for core::Vector<core::Vector<core::Point3i>> {
		#[inline]
		fn input_output_array(&mut self) -> Result<core::_InputOutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLstd_vectorLcv_Point3iGG_inputOutputArray(self.as_raw_mut_VectorOfVectorOfPoint3i(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputOutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	input_output_array_ref_forward! { core::Vector<core::Vector<core::Point3i>> }
	
	pub type VectorOfVectorOfVec3f = core::Vector<core::Vector<core::Vec3f>>;
	
	impl core::Vector<core::Vector<core::Vec3f>> {
		pub fn as_raw_VectorOfVectorOfVec3f(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfVectorOfVec3f(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::Vector<core::Vec3f>,
		std_vectorLstd_vectorLcv_Vec3fGG_new_const, std_vectorLstd_vectorLcv_Vec3fGG_delete,
		std_vectorLstd_vectorLcv_Vec3fGG_len_const, std_vectorLstd_vectorLcv_Vec3fGG_isEmpty_const,
		std_vectorLstd_vectorLcv_Vec3fGG_capacity_const, std_vectorLstd_vectorLcv_Vec3fGG_shrinkToFit,
		std_vectorLstd_vectorLcv_Vec3fGG_reserve_size_t, std_vectorLstd_vectorLcv_Vec3fGG_remove_size_t,
		std_vectorLstd_vectorLcv_Vec3fGG_swap_size_t_size_t, std_vectorLstd_vectorLcv_Vec3fGG_clear,
		std_vectorLstd_vectorLcv_Vec3fGG_get_const_size_t, std_vectorLstd_vectorLcv_Vec3fGG_set_size_t_const_vectorLVec3fG,
		std_vectorLstd_vectorLcv_Vec3fGG_push_const_vectorLVec3fG, std_vectorLstd_vectorLcv_Vec3fGG_insert_size_t_const_vectorLVec3fG,
	}
	vector_non_copy_or_bool! { clone core::Vector<core::Vec3f> }
	
	impl core::ToInputArray for core::Vector<core::Vector<core::Vec3f>> {
		#[inline]
		fn input_array(&self) -> Result<core::_InputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLstd_vectorLcv_Vec3fGG_inputArray_const(self.as_raw_VectorOfVectorOfVec3f(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToOutputArray for core::Vector<core::Vector<core::Vec3f>> {
		#[inline]
		fn output_array(&mut self) -> Result<core::_OutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLstd_vectorLcv_Vec3fGG_outputArray(self.as_raw_mut_VectorOfVectorOfVec3f(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_OutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToInputOutputArray for core::Vector<core::Vector<core::Vec3f>> {
		#[inline]
		fn input_output_array(&mut self) -> Result<core::_InputOutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLstd_vectorLcv_Vec3fGG_inputOutputArray(self.as_raw_mut_VectorOfVectorOfVec3f(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputOutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	input_output_array_ref_forward! { core::Vector<core::Vector<core::Vec3f>> }
	
	pub type VectorOfVectorOfc_char = core::Vector<core::Vector<c_char>>;
	
	impl core::Vector<core::Vector<c_char>> {
		pub fn as_raw_VectorOfVectorOfc_char(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfVectorOfc_char(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	pub type VectorOfVectorOff64 = core::Vector<core::Vector<f64>>;
	
	impl core::Vector<core::Vector<f64>> {
		pub fn as_raw_VectorOfVectorOff64(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfVectorOff64(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::Vector<f64>,
		std_vectorLstd_vectorLdoubleGG_new_const, std_vectorLstd_vectorLdoubleGG_delete,
		std_vectorLstd_vectorLdoubleGG_len_const, std_vectorLstd_vectorLdoubleGG_isEmpty_const,
		std_vectorLstd_vectorLdoubleGG_capacity_const, std_vectorLstd_vectorLdoubleGG_shrinkToFit,
		std_vectorLstd_vectorLdoubleGG_reserve_size_t, std_vectorLstd_vectorLdoubleGG_remove_size_t,
		std_vectorLstd_vectorLdoubleGG_swap_size_t_size_t, std_vectorLstd_vectorLdoubleGG_clear,
		std_vectorLstd_vectorLdoubleGG_get_const_size_t, std_vectorLstd_vectorLdoubleGG_set_size_t_const_vectorLdoubleG,
		std_vectorLstd_vectorLdoubleGG_push_const_vectorLdoubleG, std_vectorLstd_vectorLdoubleGG_insert_size_t_const_vectorLdoubleG,
	}
	vector_non_copy_or_bool! { clone core::Vector<f64> }
	
	impl core::ToInputArray for core::Vector<core::Vector<f64>> {
		#[inline]
		fn input_array(&self) -> Result<core::_InputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLstd_vectorLdoubleGG_inputArray_const(self.as_raw_VectorOfVectorOff64(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToOutputArray for core::Vector<core::Vector<f64>> {
		#[inline]
		fn output_array(&mut self) -> Result<core::_OutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLstd_vectorLdoubleGG_outputArray(self.as_raw_mut_VectorOfVectorOff64(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_OutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToInputOutputArray for core::Vector<core::Vector<f64>> {
		#[inline]
		fn input_output_array(&mut self) -> Result<core::_InputOutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLstd_vectorLdoubleGG_inputOutputArray(self.as_raw_mut_VectorOfVectorOff64(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputOutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	input_output_array_ref_forward! { core::Vector<core::Vector<f64>> }
	
	pub type VectorOfVectorOfi32 = core::Vector<core::Vector<i32>>;
	
	impl core::Vector<core::Vector<i32>> {
		pub fn as_raw_VectorOfVectorOfi32(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfVectorOfi32(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::Vector<i32>,
		std_vectorLstd_vectorLintGG_new_const, std_vectorLstd_vectorLintGG_delete,
		std_vectorLstd_vectorLintGG_len_const, std_vectorLstd_vectorLintGG_isEmpty_const,
		std_vectorLstd_vectorLintGG_capacity_const, std_vectorLstd_vectorLintGG_shrinkToFit,
		std_vectorLstd_vectorLintGG_reserve_size_t, std_vectorLstd_vectorLintGG_remove_size_t,
		std_vectorLstd_vectorLintGG_swap_size_t_size_t, std_vectorLstd_vectorLintGG_clear,
		std_vectorLstd_vectorLintGG_get_const_size_t, std_vectorLstd_vectorLintGG_set_size_t_const_vectorLintG,
		std_vectorLstd_vectorLintGG_push_const_vectorLintG, std_vectorLstd_vectorLintGG_insert_size_t_const_vectorLintG,
	}
	vector_non_copy_or_bool! { clone core::Vector<i32> }
	
	impl core::ToInputArray for core::Vector<core::Vector<i32>> {
		#[inline]
		fn input_array(&self) -> Result<core::_InputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLstd_vectorLintGG_inputArray_const(self.as_raw_VectorOfVectorOfi32(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToOutputArray for core::Vector<core::Vector<i32>> {
		#[inline]
		fn output_array(&mut self) -> Result<core::_OutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLstd_vectorLintGG_outputArray(self.as_raw_mut_VectorOfVectorOfi32(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_OutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToInputOutputArray for core::Vector<core::Vector<i32>> {
		#[inline]
		fn input_output_array(&mut self) -> Result<core::_InputOutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLstd_vectorLintGG_inputOutputArray(self.as_raw_mut_VectorOfVectorOfi32(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputOutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	input_output_array_ref_forward! { core::Vector<core::Vector<i32>> }
	
	pub type VectorOfVectorOfi8 = core::Vector<core::Vector<i8>>;
	
	impl core::Vector<core::Vector<i8>> {
		pub fn as_raw_VectorOfVectorOfi8(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfVectorOfi8(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::Vector<i8>,
		std_vectorLstd_vectorLsigned_charGG_new_const, std_vectorLstd_vectorLsigned_charGG_delete,
		std_vectorLstd_vectorLsigned_charGG_len_const, std_vectorLstd_vectorLsigned_charGG_isEmpty_const,
		std_vectorLstd_vectorLsigned_charGG_capacity_const, std_vectorLstd_vectorLsigned_charGG_shrinkToFit,
		std_vectorLstd_vectorLsigned_charGG_reserve_size_t, std_vectorLstd_vectorLsigned_charGG_remove_size_t,
		std_vectorLstd_vectorLsigned_charGG_swap_size_t_size_t, std_vectorLstd_vectorLsigned_charGG_clear,
		std_vectorLstd_vectorLsigned_charGG_get_const_size_t, std_vectorLstd_vectorLsigned_charGG_set_size_t_const_vectorLsigned_charG,
		std_vectorLstd_vectorLsigned_charGG_push_const_vectorLsigned_charG, std_vectorLstd_vectorLsigned_charGG_insert_size_t_const_vectorLsigned_charG,
	}
	vector_non_copy_or_bool! { clone core::Vector<i8> }
	
	pub type VectorOfVectorOfu8 = core::Vector<core::Vector<u8>>;
	
	impl core::Vector<core::Vector<u8>> {
		pub fn as_raw_VectorOfVectorOfu8(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfVectorOfu8(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { core::Vector<u8>,
		std_vectorLstd_vectorLunsigned_charGG_new_const, std_vectorLstd_vectorLunsigned_charGG_delete,
		std_vectorLstd_vectorLunsigned_charGG_len_const, std_vectorLstd_vectorLunsigned_charGG_isEmpty_const,
		std_vectorLstd_vectorLunsigned_charGG_capacity_const, std_vectorLstd_vectorLunsigned_charGG_shrinkToFit,
		std_vectorLstd_vectorLunsigned_charGG_reserve_size_t, std_vectorLstd_vectorLunsigned_charGG_remove_size_t,
		std_vectorLstd_vectorLunsigned_charGG_swap_size_t_size_t, std_vectorLstd_vectorLunsigned_charGG_clear,
		std_vectorLstd_vectorLunsigned_charGG_get_const_size_t, std_vectorLstd_vectorLunsigned_charGG_set_size_t_const_vectorLunsigned_charG,
		std_vectorLstd_vectorLunsigned_charGG_push_const_vectorLunsigned_charG, std_vectorLstd_vectorLunsigned_charGG_insert_size_t_const_vectorLunsigned_charG,
	}
	vector_non_copy_or_bool! { clone core::Vector<u8> }
	
	impl core::ToInputArray for core::Vector<core::Vector<u8>> {
		#[inline]
		fn input_array(&self) -> Result<core::_InputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLstd_vectorLunsigned_charGG_inputArray_const(self.as_raw_VectorOfVectorOfu8(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToOutputArray for core::Vector<core::Vector<u8>> {
		#[inline]
		fn output_array(&mut self) -> Result<core::_OutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLstd_vectorLunsigned_charGG_outputArray(self.as_raw_mut_VectorOfVectorOfu8(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_OutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToInputOutputArray for core::Vector<core::Vector<u8>> {
		#[inline]
		fn input_output_array(&mut self) -> Result<core::_InputOutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLstd_vectorLunsigned_charGG_inputOutputArray(self.as_raw_mut_VectorOfVectorOfu8(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputOutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	input_output_array_ref_forward! { core::Vector<core::Vector<u8>> }
	
	pub type VectorOfbool = core::Vector<bool>;
	
	impl core::Vector<bool> {
		pub fn as_raw_VectorOfbool(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfbool(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { bool,
		std_vectorLboolG_new_const, std_vectorLboolG_delete,
		std_vectorLboolG_len_const, std_vectorLboolG_isEmpty_const,
		std_vectorLboolG_capacity_const, std_vectorLboolG_shrinkToFit,
		std_vectorLboolG_reserve_size_t, std_vectorLboolG_remove_size_t,
		std_vectorLboolG_swap_size_t_size_t, std_vectorLboolG_clear,
		std_vectorLboolG_get_const_size_t, std_vectorLboolG_set_size_t_const_bool,
		std_vectorLboolG_push_const_bool, std_vectorLboolG_insert_size_t_const_bool,
	}
	vector_non_copy_or_bool! { clone bool }
	
	pub type VectorOfc_char = core::Vector<c_char>;
	
	impl core::Vector<c_char> {
		pub fn as_raw_VectorOfc_char(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfc_char(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	pub type VectorOff32 = core::Vector<f32>;
	
	impl core::Vector<f32> {
		pub fn as_raw_VectorOff32(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOff32(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { f32,
		std_vectorLfloatG_new_const, std_vectorLfloatG_delete,
		std_vectorLfloatG_len_const, std_vectorLfloatG_isEmpty_const,
		std_vectorLfloatG_capacity_const, std_vectorLfloatG_shrinkToFit,
		std_vectorLfloatG_reserve_size_t, std_vectorLfloatG_remove_size_t,
		std_vectorLfloatG_swap_size_t_size_t, std_vectorLfloatG_clear,
		std_vectorLfloatG_get_const_size_t, std_vectorLfloatG_set_size_t_const_float,
		std_vectorLfloatG_push_const_float, std_vectorLfloatG_insert_size_t_const_float,
	}
	vector_copy_non_bool! { f32,
		std_vectorLfloatG_data_const, std_vectorLfloatG_dataMut, cv_fromSlice_const_const_floatX_size_t,
		std_vectorLfloatG_clone_const,
	}
	
	impl core::ToInputArray for core::Vector<f32> {
		#[inline]
		fn input_array(&self) -> Result<core::_InputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLfloatG_inputArray_const(self.as_raw_VectorOff32(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToOutputArray for core::Vector<f32> {
		#[inline]
		fn output_array(&mut self) -> Result<core::_OutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLfloatG_outputArray(self.as_raw_mut_VectorOff32(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_OutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToInputOutputArray for core::Vector<f32> {
		#[inline]
		fn input_output_array(&mut self) -> Result<core::_InputOutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLfloatG_inputOutputArray(self.as_raw_mut_VectorOff32(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputOutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	input_output_array_ref_forward! { core::Vector<f32> }
	
	pub type VectorOff64 = core::Vector<f64>;
	
	impl core::Vector<f64> {
		pub fn as_raw_VectorOff64(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOff64(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { f64,
		std_vectorLdoubleG_new_const, std_vectorLdoubleG_delete,
		std_vectorLdoubleG_len_const, std_vectorLdoubleG_isEmpty_const,
		std_vectorLdoubleG_capacity_const, std_vectorLdoubleG_shrinkToFit,
		std_vectorLdoubleG_reserve_size_t, std_vectorLdoubleG_remove_size_t,
		std_vectorLdoubleG_swap_size_t_size_t, std_vectorLdoubleG_clear,
		std_vectorLdoubleG_get_const_size_t, std_vectorLdoubleG_set_size_t_const_double,
		std_vectorLdoubleG_push_const_double, std_vectorLdoubleG_insert_size_t_const_double,
	}
	vector_copy_non_bool! { f64,
		std_vectorLdoubleG_data_const, std_vectorLdoubleG_dataMut, cv_fromSlice_const_const_doubleX_size_t,
		std_vectorLdoubleG_clone_const,
	}
	
	impl core::ToInputArray for core::Vector<f64> {
		#[inline]
		fn input_array(&self) -> Result<core::_InputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLdoubleG_inputArray_const(self.as_raw_VectorOff64(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToOutputArray for core::Vector<f64> {
		#[inline]
		fn output_array(&mut self) -> Result<core::_OutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLdoubleG_outputArray(self.as_raw_mut_VectorOff64(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_OutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToInputOutputArray for core::Vector<f64> {
		#[inline]
		fn input_output_array(&mut self) -> Result<core::_InputOutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLdoubleG_inputOutputArray(self.as_raw_mut_VectorOff64(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputOutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	input_output_array_ref_forward! { core::Vector<f64> }
	
	pub type VectorOfi32 = core::Vector<i32>;
	
	impl core::Vector<i32> {
		pub fn as_raw_VectorOfi32(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfi32(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { i32,
		std_vectorLintG_new_const, std_vectorLintG_delete,
		std_vectorLintG_len_const, std_vectorLintG_isEmpty_const,
		std_vectorLintG_capacity_const, std_vectorLintG_shrinkToFit,
		std_vectorLintG_reserve_size_t, std_vectorLintG_remove_size_t,
		std_vectorLintG_swap_size_t_size_t, std_vectorLintG_clear,
		std_vectorLintG_get_const_size_t, std_vectorLintG_set_size_t_const_int,
		std_vectorLintG_push_const_int, std_vectorLintG_insert_size_t_const_int,
	}
	vector_copy_non_bool! { i32,
		std_vectorLintG_data_const, std_vectorLintG_dataMut, cv_fromSlice_const_const_intX_size_t,
		std_vectorLintG_clone_const,
	}
	
	impl core::ToInputArray for core::Vector<i32> {
		#[inline]
		fn input_array(&self) -> Result<core::_InputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLintG_inputArray_const(self.as_raw_VectorOfi32(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToOutputArray for core::Vector<i32> {
		#[inline]
		fn output_array(&mut self) -> Result<core::_OutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLintG_outputArray(self.as_raw_mut_VectorOfi32(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_OutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToInputOutputArray for core::Vector<i32> {
		#[inline]
		fn input_output_array(&mut self) -> Result<core::_InputOutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLintG_inputOutputArray(self.as_raw_mut_VectorOfi32(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputOutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	input_output_array_ref_forward! { core::Vector<i32> }
	
	pub type VectorOfi8 = core::Vector<i8>;
	
	impl core::Vector<i8> {
		pub fn as_raw_VectorOfi8(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfi8(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { i8,
		std_vectorLsigned_charG_new_const, std_vectorLsigned_charG_delete,
		std_vectorLsigned_charG_len_const, std_vectorLsigned_charG_isEmpty_const,
		std_vectorLsigned_charG_capacity_const, std_vectorLsigned_charG_shrinkToFit,
		std_vectorLsigned_charG_reserve_size_t, std_vectorLsigned_charG_remove_size_t,
		std_vectorLsigned_charG_swap_size_t_size_t, std_vectorLsigned_charG_clear,
		std_vectorLsigned_charG_get_const_size_t, std_vectorLsigned_charG_set_size_t_const_signed_char,
		std_vectorLsigned_charG_push_const_signed_char, std_vectorLsigned_charG_insert_size_t_const_signed_char,
	}
	vector_copy_non_bool! { i8,
		std_vectorLsigned_charG_data_const, std_vectorLsigned_charG_dataMut, cv_fromSlice_const_const_signed_charX_size_t,
		std_vectorLsigned_charG_clone_const,
	}
	
	pub type VectorOfsize_t = core::Vector<size_t>;
	
	impl core::Vector<size_t> {
		pub fn as_raw_VectorOfsize_t(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfsize_t(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { size_t,
		std_vectorLsize_tG_new_const, std_vectorLsize_tG_delete,
		std_vectorLsize_tG_len_const, std_vectorLsize_tG_isEmpty_const,
		std_vectorLsize_tG_capacity_const, std_vectorLsize_tG_shrinkToFit,
		std_vectorLsize_tG_reserve_size_t, std_vectorLsize_tG_remove_size_t,
		std_vectorLsize_tG_swap_size_t_size_t, std_vectorLsize_tG_clear,
		std_vectorLsize_tG_get_const_size_t, std_vectorLsize_tG_set_size_t_const_size_t,
		std_vectorLsize_tG_push_const_size_t, std_vectorLsize_tG_insert_size_t_const_size_t,
	}
	vector_copy_non_bool! { size_t,
		std_vectorLsize_tG_data_const, std_vectorLsize_tG_dataMut, cv_fromSlice_const_const_size_tX_size_t,
		std_vectorLsize_tG_clone_const,
	}
	
	pub type VectorOfu8 = core::Vector<u8>;
	
	impl core::Vector<u8> {
		pub fn as_raw_VectorOfu8(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfu8(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { u8,
		std_vectorLunsigned_charG_new_const, std_vectorLunsigned_charG_delete,
		std_vectorLunsigned_charG_len_const, std_vectorLunsigned_charG_isEmpty_const,
		std_vectorLunsigned_charG_capacity_const, std_vectorLunsigned_charG_shrinkToFit,
		std_vectorLunsigned_charG_reserve_size_t, std_vectorLunsigned_charG_remove_size_t,
		std_vectorLunsigned_charG_swap_size_t_size_t, std_vectorLunsigned_charG_clear,
		std_vectorLunsigned_charG_get_const_size_t, std_vectorLunsigned_charG_set_size_t_const_unsigned_char,
		std_vectorLunsigned_charG_push_const_unsigned_char, std_vectorLunsigned_charG_insert_size_t_const_unsigned_char,
	}
	vector_copy_non_bool! { u8,
		std_vectorLunsigned_charG_data_const, std_vectorLunsigned_charG_dataMut, cv_fromSlice_const_const_unsigned_charX_size_t,
		std_vectorLunsigned_charG_clone_const,
	}
	
	impl core::ToInputArray for core::Vector<u8> {
		#[inline]
		fn input_array(&self) -> Result<core::_InputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLunsigned_charG_inputArray_const(self.as_raw_VectorOfu8(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToOutputArray for core::Vector<u8> {
		#[inline]
		fn output_array(&mut self) -> Result<core::_OutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLunsigned_charG_outputArray(self.as_raw_mut_VectorOfu8(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_OutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	impl core::ToInputOutputArray for core::Vector<u8> {
		#[inline]
		fn input_output_array(&mut self) -> Result<core::_InputOutputArray> {
			return_send!(via ocvrs_return);
			unsafe { sys::std_vectorLunsigned_charG_inputOutputArray(self.as_raw_mut_VectorOfu8(), ocvrs_return.as_mut_ptr()) };
			return_receive!(unsafe ocvrs_return => ret);
			let ret = ret.into_result()?;
			let ret = unsafe { core::_InputOutputArray::opencv_from_extern(ret) };
			Ok(ret)
		}
		
	}
	
	input_output_array_ref_forward! { core::Vector<u8> }
	
}
#[cfg(ocvrs_has_module_core)]
pub use core_types::*;

#[cfg(ocvrs_has_module_features2d)]
mod features2d_types {
	use crate::{mod_prelude::*, core, types, sys};

	pub type PtrOfAKAZE = core::Ptr<crate::features2d::AKAZE>;
	
	ptr_extern! { crate::features2d::AKAZE,
		cv_PtrLcv_AKAZEG_delete, cv_PtrLcv_AKAZEG_getInnerPtr_const, cv_PtrLcv_AKAZEG_getInnerPtrMut
	}
	
	impl core::Ptr<crate::features2d::AKAZE> {
		#[inline] pub fn as_raw_PtrOfAKAZE(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfAKAZE(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::features2d::AKAZETraitConst for core::Ptr<crate::features2d::AKAZE> {
		#[inline] fn as_raw_AKAZE(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::features2d::AKAZETrait for core::Ptr<crate::features2d::AKAZE> {
		#[inline] fn as_raw_mut_AKAZE(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl core::AlgorithmTraitConst for core::Ptr<crate::features2d::AKAZE> {
		#[inline] fn as_raw_Algorithm(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::AlgorithmTrait for core::Ptr<crate::features2d::AKAZE> {
		#[inline] fn as_raw_mut_Algorithm(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfAKAZE, core::Ptr<core::Algorithm>, cv_PtrLcv_AKAZEG_to_PtrOfAlgorithm }
	
	impl crate::features2d::Feature2DTraitConst for core::Ptr<crate::features2d::AKAZE> {
		#[inline] fn as_raw_Feature2D(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::features2d::Feature2DTrait for core::Ptr<crate::features2d::AKAZE> {
		#[inline] fn as_raw_mut_Feature2D(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfAKAZE, core::Ptr<crate::features2d::Feature2D>, cv_PtrLcv_AKAZEG_to_PtrOfFeature2D }
	
	impl std::fmt::Debug for core::Ptr<crate::features2d::AKAZE> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfAKAZE")
				.finish()
		}
	}
	
	pub type PtrOfAffineFeature = core::Ptr<crate::features2d::AffineFeature>;
	
	ptr_extern! { crate::features2d::AffineFeature,
		cv_PtrLcv_AffineFeatureG_delete, cv_PtrLcv_AffineFeatureG_getInnerPtr_const, cv_PtrLcv_AffineFeatureG_getInnerPtrMut
	}
	
	impl core::Ptr<crate::features2d::AffineFeature> {
		#[inline] pub fn as_raw_PtrOfAffineFeature(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfAffineFeature(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::features2d::AffineFeatureTraitConst for core::Ptr<crate::features2d::AffineFeature> {
		#[inline] fn as_raw_AffineFeature(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::features2d::AffineFeatureTrait for core::Ptr<crate::features2d::AffineFeature> {
		#[inline] fn as_raw_mut_AffineFeature(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl core::AlgorithmTraitConst for core::Ptr<crate::features2d::AffineFeature> {
		#[inline] fn as_raw_Algorithm(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::AlgorithmTrait for core::Ptr<crate::features2d::AffineFeature> {
		#[inline] fn as_raw_mut_Algorithm(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfAffineFeature, core::Ptr<core::Algorithm>, cv_PtrLcv_AffineFeatureG_to_PtrOfAlgorithm }
	
	impl crate::features2d::Feature2DTraitConst for core::Ptr<crate::features2d::AffineFeature> {
		#[inline] fn as_raw_Feature2D(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::features2d::Feature2DTrait for core::Ptr<crate::features2d::AffineFeature> {
		#[inline] fn as_raw_mut_Feature2D(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfAffineFeature, core::Ptr<crate::features2d::Feature2D>, cv_PtrLcv_AffineFeatureG_to_PtrOfFeature2D }
	
	impl std::fmt::Debug for core::Ptr<crate::features2d::AffineFeature> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfAffineFeature")
				.finish()
		}
	}
	
	pub type PtrOfAgastFeatureDetector = core::Ptr<crate::features2d::AgastFeatureDetector>;
	
	ptr_extern! { crate::features2d::AgastFeatureDetector,
		cv_PtrLcv_AgastFeatureDetectorG_delete, cv_PtrLcv_AgastFeatureDetectorG_getInnerPtr_const, cv_PtrLcv_AgastFeatureDetectorG_getInnerPtrMut
	}
	
	impl core::Ptr<crate::features2d::AgastFeatureDetector> {
		#[inline] pub fn as_raw_PtrOfAgastFeatureDetector(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfAgastFeatureDetector(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::features2d::AgastFeatureDetectorTraitConst for core::Ptr<crate::features2d::AgastFeatureDetector> {
		#[inline] fn as_raw_AgastFeatureDetector(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::features2d::AgastFeatureDetectorTrait for core::Ptr<crate::features2d::AgastFeatureDetector> {
		#[inline] fn as_raw_mut_AgastFeatureDetector(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl core::AlgorithmTraitConst for core::Ptr<crate::features2d::AgastFeatureDetector> {
		#[inline] fn as_raw_Algorithm(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::AlgorithmTrait for core::Ptr<crate::features2d::AgastFeatureDetector> {
		#[inline] fn as_raw_mut_Algorithm(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfAgastFeatureDetector, core::Ptr<core::Algorithm>, cv_PtrLcv_AgastFeatureDetectorG_to_PtrOfAlgorithm }
	
	impl crate::features2d::Feature2DTraitConst for core::Ptr<crate::features2d::AgastFeatureDetector> {
		#[inline] fn as_raw_Feature2D(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::features2d::Feature2DTrait for core::Ptr<crate::features2d::AgastFeatureDetector> {
		#[inline] fn as_raw_mut_Feature2D(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfAgastFeatureDetector, core::Ptr<crate::features2d::Feature2D>, cv_PtrLcv_AgastFeatureDetectorG_to_PtrOfFeature2D }
	
	impl std::fmt::Debug for core::Ptr<crate::features2d::AgastFeatureDetector> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfAgastFeatureDetector")
				.finish()
		}
	}
	
	pub type PtrOfBFMatcher = core::Ptr<crate::features2d::BFMatcher>;
	
	ptr_extern! { crate::features2d::BFMatcher,
		cv_PtrLcv_BFMatcherG_delete, cv_PtrLcv_BFMatcherG_getInnerPtr_const, cv_PtrLcv_BFMatcherG_getInnerPtrMut
	}
	
	ptr_extern_ctor! { crate::features2d::BFMatcher, cv_PtrLcv_BFMatcherG_new_const_BFMatcher }
	impl core::Ptr<crate::features2d::BFMatcher> {
		#[inline] pub fn as_raw_PtrOfBFMatcher(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfBFMatcher(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::features2d::BFMatcherTraitConst for core::Ptr<crate::features2d::BFMatcher> {
		#[inline] fn as_raw_BFMatcher(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::features2d::BFMatcherTrait for core::Ptr<crate::features2d::BFMatcher> {
		#[inline] fn as_raw_mut_BFMatcher(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl core::AlgorithmTraitConst for core::Ptr<crate::features2d::BFMatcher> {
		#[inline] fn as_raw_Algorithm(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::AlgorithmTrait for core::Ptr<crate::features2d::BFMatcher> {
		#[inline] fn as_raw_mut_Algorithm(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfBFMatcher, core::Ptr<core::Algorithm>, cv_PtrLcv_BFMatcherG_to_PtrOfAlgorithm }
	
	impl crate::features2d::DescriptorMatcherTraitConst for core::Ptr<crate::features2d::BFMatcher> {
		#[inline] fn as_raw_DescriptorMatcher(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::features2d::DescriptorMatcherTrait for core::Ptr<crate::features2d::BFMatcher> {
		#[inline] fn as_raw_mut_DescriptorMatcher(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfBFMatcher, core::Ptr<crate::features2d::DescriptorMatcher>, cv_PtrLcv_BFMatcherG_to_PtrOfDescriptorMatcher }
	
	impl std::fmt::Debug for core::Ptr<crate::features2d::BFMatcher> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfBFMatcher")
				.finish()
		}
	}
	
	pub type PtrOfBRISK = core::Ptr<crate::features2d::BRISK>;
	
	ptr_extern! { crate::features2d::BRISK,
		cv_PtrLcv_BRISKG_delete, cv_PtrLcv_BRISKG_getInnerPtr_const, cv_PtrLcv_BRISKG_getInnerPtrMut
	}
	
	ptr_extern_ctor! { crate::features2d::BRISK, cv_PtrLcv_BRISKG_new_const_BRISK }
	impl core::Ptr<crate::features2d::BRISK> {
		#[inline] pub fn as_raw_PtrOfBRISK(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfBRISK(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::features2d::BRISKTraitConst for core::Ptr<crate::features2d::BRISK> {
		#[inline] fn as_raw_BRISK(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::features2d::BRISKTrait for core::Ptr<crate::features2d::BRISK> {
		#[inline] fn as_raw_mut_BRISK(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl core::AlgorithmTraitConst for core::Ptr<crate::features2d::BRISK> {
		#[inline] fn as_raw_Algorithm(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::AlgorithmTrait for core::Ptr<crate::features2d::BRISK> {
		#[inline] fn as_raw_mut_Algorithm(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfBRISK, core::Ptr<core::Algorithm>, cv_PtrLcv_BRISKG_to_PtrOfAlgorithm }
	
	impl crate::features2d::Feature2DTraitConst for core::Ptr<crate::features2d::BRISK> {
		#[inline] fn as_raw_Feature2D(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::features2d::Feature2DTrait for core::Ptr<crate::features2d::BRISK> {
		#[inline] fn as_raw_mut_Feature2D(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfBRISK, core::Ptr<crate::features2d::Feature2D>, cv_PtrLcv_BRISKG_to_PtrOfFeature2D }
	
	impl std::fmt::Debug for core::Ptr<crate::features2d::BRISK> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfBRISK")
				.finish()
		}
	}
	
	pub type PtrOfDescriptorMatcher = core::Ptr<crate::features2d::DescriptorMatcher>;
	
	ptr_extern! { crate::features2d::DescriptorMatcher,
		cv_PtrLcv_DescriptorMatcherG_delete, cv_PtrLcv_DescriptorMatcherG_getInnerPtr_const, cv_PtrLcv_DescriptorMatcherG_getInnerPtrMut
	}
	
	impl core::Ptr<crate::features2d::DescriptorMatcher> {
		#[inline] pub fn as_raw_PtrOfDescriptorMatcher(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfDescriptorMatcher(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::features2d::DescriptorMatcherTraitConst for core::Ptr<crate::features2d::DescriptorMatcher> {
		#[inline] fn as_raw_DescriptorMatcher(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::features2d::DescriptorMatcherTrait for core::Ptr<crate::features2d::DescriptorMatcher> {
		#[inline] fn as_raw_mut_DescriptorMatcher(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl core::AlgorithmTraitConst for core::Ptr<crate::features2d::DescriptorMatcher> {
		#[inline] fn as_raw_Algorithm(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::AlgorithmTrait for core::Ptr<crate::features2d::DescriptorMatcher> {
		#[inline] fn as_raw_mut_Algorithm(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfDescriptorMatcher, core::Ptr<core::Algorithm>, cv_PtrLcv_DescriptorMatcherG_to_PtrOfAlgorithm }
	
	impl std::fmt::Debug for core::Ptr<crate::features2d::DescriptorMatcher> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfDescriptorMatcher")
				.finish()
		}
	}
	
	pub type PtrOfFastFeatureDetector = core::Ptr<crate::features2d::FastFeatureDetector>;
	
	ptr_extern! { crate::features2d::FastFeatureDetector,
		cv_PtrLcv_FastFeatureDetectorG_delete, cv_PtrLcv_FastFeatureDetectorG_getInnerPtr_const, cv_PtrLcv_FastFeatureDetectorG_getInnerPtrMut
	}
	
	impl core::Ptr<crate::features2d::FastFeatureDetector> {
		#[inline] pub fn as_raw_PtrOfFastFeatureDetector(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfFastFeatureDetector(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::features2d::FastFeatureDetectorTraitConst for core::Ptr<crate::features2d::FastFeatureDetector> {
		#[inline] fn as_raw_FastFeatureDetector(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::features2d::FastFeatureDetectorTrait for core::Ptr<crate::features2d::FastFeatureDetector> {
		#[inline] fn as_raw_mut_FastFeatureDetector(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl core::AlgorithmTraitConst for core::Ptr<crate::features2d::FastFeatureDetector> {
		#[inline] fn as_raw_Algorithm(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::AlgorithmTrait for core::Ptr<crate::features2d::FastFeatureDetector> {
		#[inline] fn as_raw_mut_Algorithm(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfFastFeatureDetector, core::Ptr<core::Algorithm>, cv_PtrLcv_FastFeatureDetectorG_to_PtrOfAlgorithm }
	
	impl crate::features2d::Feature2DTraitConst for core::Ptr<crate::features2d::FastFeatureDetector> {
		#[inline] fn as_raw_Feature2D(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::features2d::Feature2DTrait for core::Ptr<crate::features2d::FastFeatureDetector> {
		#[inline] fn as_raw_mut_Feature2D(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfFastFeatureDetector, core::Ptr<crate::features2d::Feature2D>, cv_PtrLcv_FastFeatureDetectorG_to_PtrOfFeature2D }
	
	impl std::fmt::Debug for core::Ptr<crate::features2d::FastFeatureDetector> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfFastFeatureDetector")
				.finish()
		}
	}
	
	pub type PtrOfFeature2D = core::Ptr<crate::features2d::Feature2D>;
	
	ptr_extern! { crate::features2d::Feature2D,
		cv_PtrLcv_Feature2DG_delete, cv_PtrLcv_Feature2DG_getInnerPtr_const, cv_PtrLcv_Feature2DG_getInnerPtrMut
	}
	
	ptr_extern_ctor! { crate::features2d::Feature2D, cv_PtrLcv_Feature2DG_new_const_Feature2D }
	impl core::Ptr<crate::features2d::Feature2D> {
		#[inline] pub fn as_raw_PtrOfFeature2D(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfFeature2D(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::features2d::Feature2DTraitConst for core::Ptr<crate::features2d::Feature2D> {
		#[inline] fn as_raw_Feature2D(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::features2d::Feature2DTrait for core::Ptr<crate::features2d::Feature2D> {
		#[inline] fn as_raw_mut_Feature2D(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl core::AlgorithmTraitConst for core::Ptr<crate::features2d::Feature2D> {
		#[inline] fn as_raw_Algorithm(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::AlgorithmTrait for core::Ptr<crate::features2d::Feature2D> {
		#[inline] fn as_raw_mut_Algorithm(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfFeature2D, core::Ptr<core::Algorithm>, cv_PtrLcv_Feature2DG_to_PtrOfAlgorithm }
	
	impl std::fmt::Debug for core::Ptr<crate::features2d::Feature2D> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfFeature2D")
				.finish()
		}
	}
	
	pub type PtrOfFlannBasedMatcher = core::Ptr<crate::features2d::FlannBasedMatcher>;
	
	ptr_extern! { crate::features2d::FlannBasedMatcher,
		cv_PtrLcv_FlannBasedMatcherG_delete, cv_PtrLcv_FlannBasedMatcherG_getInnerPtr_const, cv_PtrLcv_FlannBasedMatcherG_getInnerPtrMut
	}
	
	ptr_extern_ctor! { crate::features2d::FlannBasedMatcher, cv_PtrLcv_FlannBasedMatcherG_new_const_FlannBasedMatcher }
	impl core::Ptr<crate::features2d::FlannBasedMatcher> {
		#[inline] pub fn as_raw_PtrOfFlannBasedMatcher(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfFlannBasedMatcher(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::features2d::FlannBasedMatcherTraitConst for core::Ptr<crate::features2d::FlannBasedMatcher> {
		#[inline] fn as_raw_FlannBasedMatcher(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::features2d::FlannBasedMatcherTrait for core::Ptr<crate::features2d::FlannBasedMatcher> {
		#[inline] fn as_raw_mut_FlannBasedMatcher(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl core::AlgorithmTraitConst for core::Ptr<crate::features2d::FlannBasedMatcher> {
		#[inline] fn as_raw_Algorithm(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::AlgorithmTrait for core::Ptr<crate::features2d::FlannBasedMatcher> {
		#[inline] fn as_raw_mut_Algorithm(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfFlannBasedMatcher, core::Ptr<core::Algorithm>, cv_PtrLcv_FlannBasedMatcherG_to_PtrOfAlgorithm }
	
	impl crate::features2d::DescriptorMatcherTraitConst for core::Ptr<crate::features2d::FlannBasedMatcher> {
		#[inline] fn as_raw_DescriptorMatcher(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::features2d::DescriptorMatcherTrait for core::Ptr<crate::features2d::FlannBasedMatcher> {
		#[inline] fn as_raw_mut_DescriptorMatcher(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfFlannBasedMatcher, core::Ptr<crate::features2d::DescriptorMatcher>, cv_PtrLcv_FlannBasedMatcherG_to_PtrOfDescriptorMatcher }
	
	impl std::fmt::Debug for core::Ptr<crate::features2d::FlannBasedMatcher> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfFlannBasedMatcher")
				.finish()
		}
	}
	
	pub type PtrOfGFTTDetector = core::Ptr<crate::features2d::GFTTDetector>;
	
	ptr_extern! { crate::features2d::GFTTDetector,
		cv_PtrLcv_GFTTDetectorG_delete, cv_PtrLcv_GFTTDetectorG_getInnerPtr_const, cv_PtrLcv_GFTTDetectorG_getInnerPtrMut
	}
	
	impl core::Ptr<crate::features2d::GFTTDetector> {
		#[inline] pub fn as_raw_PtrOfGFTTDetector(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfGFTTDetector(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::features2d::GFTTDetectorTraitConst for core::Ptr<crate::features2d::GFTTDetector> {
		#[inline] fn as_raw_GFTTDetector(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::features2d::GFTTDetectorTrait for core::Ptr<crate::features2d::GFTTDetector> {
		#[inline] fn as_raw_mut_GFTTDetector(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl core::AlgorithmTraitConst for core::Ptr<crate::features2d::GFTTDetector> {
		#[inline] fn as_raw_Algorithm(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::AlgorithmTrait for core::Ptr<crate::features2d::GFTTDetector> {
		#[inline] fn as_raw_mut_Algorithm(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfGFTTDetector, core::Ptr<core::Algorithm>, cv_PtrLcv_GFTTDetectorG_to_PtrOfAlgorithm }
	
	impl crate::features2d::Feature2DTraitConst for core::Ptr<crate::features2d::GFTTDetector> {
		#[inline] fn as_raw_Feature2D(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::features2d::Feature2DTrait for core::Ptr<crate::features2d::GFTTDetector> {
		#[inline] fn as_raw_mut_Feature2D(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfGFTTDetector, core::Ptr<crate::features2d::Feature2D>, cv_PtrLcv_GFTTDetectorG_to_PtrOfFeature2D }
	
	impl std::fmt::Debug for core::Ptr<crate::features2d::GFTTDetector> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfGFTTDetector")
				.finish()
		}
	}
	
	pub type PtrOfKAZE = core::Ptr<crate::features2d::KAZE>;
	
	ptr_extern! { crate::features2d::KAZE,
		cv_PtrLcv_KAZEG_delete, cv_PtrLcv_KAZEG_getInnerPtr_const, cv_PtrLcv_KAZEG_getInnerPtrMut
	}
	
	impl core::Ptr<crate::features2d::KAZE> {
		#[inline] pub fn as_raw_PtrOfKAZE(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfKAZE(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::features2d::KAZETraitConst for core::Ptr<crate::features2d::KAZE> {
		#[inline] fn as_raw_KAZE(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::features2d::KAZETrait for core::Ptr<crate::features2d::KAZE> {
		#[inline] fn as_raw_mut_KAZE(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl core::AlgorithmTraitConst for core::Ptr<crate::features2d::KAZE> {
		#[inline] fn as_raw_Algorithm(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::AlgorithmTrait for core::Ptr<crate::features2d::KAZE> {
		#[inline] fn as_raw_mut_Algorithm(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfKAZE, core::Ptr<core::Algorithm>, cv_PtrLcv_KAZEG_to_PtrOfAlgorithm }
	
	impl crate::features2d::Feature2DTraitConst for core::Ptr<crate::features2d::KAZE> {
		#[inline] fn as_raw_Feature2D(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::features2d::Feature2DTrait for core::Ptr<crate::features2d::KAZE> {
		#[inline] fn as_raw_mut_Feature2D(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfKAZE, core::Ptr<crate::features2d::Feature2D>, cv_PtrLcv_KAZEG_to_PtrOfFeature2D }
	
	impl std::fmt::Debug for core::Ptr<crate::features2d::KAZE> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfKAZE")
				.finish()
		}
	}
	
	pub type PtrOfMSER = core::Ptr<crate::features2d::MSER>;
	
	ptr_extern! { crate::features2d::MSER,
		cv_PtrLcv_MSERG_delete, cv_PtrLcv_MSERG_getInnerPtr_const, cv_PtrLcv_MSERG_getInnerPtrMut
	}
	
	impl core::Ptr<crate::features2d::MSER> {
		#[inline] pub fn as_raw_PtrOfMSER(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfMSER(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::features2d::MSERTraitConst for core::Ptr<crate::features2d::MSER> {
		#[inline] fn as_raw_MSER(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::features2d::MSERTrait for core::Ptr<crate::features2d::MSER> {
		#[inline] fn as_raw_mut_MSER(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl core::AlgorithmTraitConst for core::Ptr<crate::features2d::MSER> {
		#[inline] fn as_raw_Algorithm(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::AlgorithmTrait for core::Ptr<crate::features2d::MSER> {
		#[inline] fn as_raw_mut_Algorithm(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfMSER, core::Ptr<core::Algorithm>, cv_PtrLcv_MSERG_to_PtrOfAlgorithm }
	
	impl crate::features2d::Feature2DTraitConst for core::Ptr<crate::features2d::MSER> {
		#[inline] fn as_raw_Feature2D(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::features2d::Feature2DTrait for core::Ptr<crate::features2d::MSER> {
		#[inline] fn as_raw_mut_Feature2D(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfMSER, core::Ptr<crate::features2d::Feature2D>, cv_PtrLcv_MSERG_to_PtrOfFeature2D }
	
	impl std::fmt::Debug for core::Ptr<crate::features2d::MSER> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfMSER")
				.finish()
		}
	}
	
	pub type PtrOfORB = core::Ptr<crate::features2d::ORB>;
	
	ptr_extern! { crate::features2d::ORB,
		cv_PtrLcv_ORBG_delete, cv_PtrLcv_ORBG_getInnerPtr_const, cv_PtrLcv_ORBG_getInnerPtrMut
	}
	
	impl core::Ptr<crate::features2d::ORB> {
		#[inline] pub fn as_raw_PtrOfORB(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfORB(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::features2d::ORBTraitConst for core::Ptr<crate::features2d::ORB> {
		#[inline] fn as_raw_ORB(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::features2d::ORBTrait for core::Ptr<crate::features2d::ORB> {
		#[inline] fn as_raw_mut_ORB(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl core::AlgorithmTraitConst for core::Ptr<crate::features2d::ORB> {
		#[inline] fn as_raw_Algorithm(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::AlgorithmTrait for core::Ptr<crate::features2d::ORB> {
		#[inline] fn as_raw_mut_Algorithm(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfORB, core::Ptr<core::Algorithm>, cv_PtrLcv_ORBG_to_PtrOfAlgorithm }
	
	impl crate::features2d::Feature2DTraitConst for core::Ptr<crate::features2d::ORB> {
		#[inline] fn as_raw_Feature2D(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::features2d::Feature2DTrait for core::Ptr<crate::features2d::ORB> {
		#[inline] fn as_raw_mut_Feature2D(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfORB, core::Ptr<crate::features2d::Feature2D>, cv_PtrLcv_ORBG_to_PtrOfFeature2D }
	
	impl std::fmt::Debug for core::Ptr<crate::features2d::ORB> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfORB")
				.finish()
		}
	}
	
	pub type PtrOfSIFT = core::Ptr<crate::features2d::SIFT>;
	
	ptr_extern! { crate::features2d::SIFT,
		cv_PtrLcv_SIFTG_delete, cv_PtrLcv_SIFTG_getInnerPtr_const, cv_PtrLcv_SIFTG_getInnerPtrMut
	}
	
	ptr_extern_ctor! { crate::features2d::SIFT, cv_PtrLcv_SIFTG_new_const_SIFT }
	impl core::Ptr<crate::features2d::SIFT> {
		#[inline] pub fn as_raw_PtrOfSIFT(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfSIFT(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::features2d::SIFTTraitConst for core::Ptr<crate::features2d::SIFT> {
		#[inline] fn as_raw_SIFT(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::features2d::SIFTTrait for core::Ptr<crate::features2d::SIFT> {
		#[inline] fn as_raw_mut_SIFT(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl core::AlgorithmTraitConst for core::Ptr<crate::features2d::SIFT> {
		#[inline] fn as_raw_Algorithm(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::AlgorithmTrait for core::Ptr<crate::features2d::SIFT> {
		#[inline] fn as_raw_mut_Algorithm(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfSIFT, core::Ptr<core::Algorithm>, cv_PtrLcv_SIFTG_to_PtrOfAlgorithm }
	
	impl crate::features2d::Feature2DTraitConst for core::Ptr<crate::features2d::SIFT> {
		#[inline] fn as_raw_Feature2D(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::features2d::Feature2DTrait for core::Ptr<crate::features2d::SIFT> {
		#[inline] fn as_raw_mut_Feature2D(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfSIFT, core::Ptr<crate::features2d::Feature2D>, cv_PtrLcv_SIFTG_to_PtrOfFeature2D }
	
	impl std::fmt::Debug for core::Ptr<crate::features2d::SIFT> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfSIFT")
				.finish()
		}
	}
	
	pub type PtrOfSimpleBlobDetector = core::Ptr<crate::features2d::SimpleBlobDetector>;
	
	ptr_extern! { crate::features2d::SimpleBlobDetector,
		cv_PtrLcv_SimpleBlobDetectorG_delete, cv_PtrLcv_SimpleBlobDetectorG_getInnerPtr_const, cv_PtrLcv_SimpleBlobDetectorG_getInnerPtrMut
	}
	
	ptr_extern_ctor! { crate::features2d::SimpleBlobDetector, cv_PtrLcv_SimpleBlobDetectorG_new_const_SimpleBlobDetector }
	impl core::Ptr<crate::features2d::SimpleBlobDetector> {
		#[inline] pub fn as_raw_PtrOfSimpleBlobDetector(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfSimpleBlobDetector(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::features2d::SimpleBlobDetectorTraitConst for core::Ptr<crate::features2d::SimpleBlobDetector> {
		#[inline] fn as_raw_SimpleBlobDetector(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::features2d::SimpleBlobDetectorTrait for core::Ptr<crate::features2d::SimpleBlobDetector> {
		#[inline] fn as_raw_mut_SimpleBlobDetector(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl core::AlgorithmTraitConst for core::Ptr<crate::features2d::SimpleBlobDetector> {
		#[inline] fn as_raw_Algorithm(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::AlgorithmTrait for core::Ptr<crate::features2d::SimpleBlobDetector> {
		#[inline] fn as_raw_mut_Algorithm(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfSimpleBlobDetector, core::Ptr<core::Algorithm>, cv_PtrLcv_SimpleBlobDetectorG_to_PtrOfAlgorithm }
	
	impl crate::features2d::Feature2DTraitConst for core::Ptr<crate::features2d::SimpleBlobDetector> {
		#[inline] fn as_raw_Feature2D(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::features2d::Feature2DTrait for core::Ptr<crate::features2d::SimpleBlobDetector> {
		#[inline] fn as_raw_mut_Feature2D(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfSimpleBlobDetector, core::Ptr<crate::features2d::Feature2D>, cv_PtrLcv_SimpleBlobDetectorG_to_PtrOfFeature2D }
	
	impl std::fmt::Debug for core::Ptr<crate::features2d::SimpleBlobDetector> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfSimpleBlobDetector")
				.finish()
		}
	}
	
}
#[cfg(ocvrs_has_module_features2d)]
pub use features2d_types::*;

#[cfg(ocvrs_has_module_flann)]
mod flann_types {
	use crate::{mod_prelude::*, core, types, sys};

	pub type PtrOfAutotunedIndexParams = core::Ptr<crate::flann::AutotunedIndexParams>;
	
	ptr_extern! { crate::flann::AutotunedIndexParams,
		cv_PtrLcv_flann_AutotunedIndexParamsG_delete, cv_PtrLcv_flann_AutotunedIndexParamsG_getInnerPtr_const, cv_PtrLcv_flann_AutotunedIndexParamsG_getInnerPtrMut
	}
	
	ptr_extern_ctor! { crate::flann::AutotunedIndexParams, cv_PtrLcv_flann_AutotunedIndexParamsG_new_const_AutotunedIndexParams }
	impl core::Ptr<crate::flann::AutotunedIndexParams> {
		#[inline] pub fn as_raw_PtrOfAutotunedIndexParams(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfAutotunedIndexParams(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::flann::AutotunedIndexParamsTraitConst for core::Ptr<crate::flann::AutotunedIndexParams> {
		#[inline] fn as_raw_AutotunedIndexParams(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::flann::AutotunedIndexParamsTrait for core::Ptr<crate::flann::AutotunedIndexParams> {
		#[inline] fn as_raw_mut_AutotunedIndexParams(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl crate::flann::IndexParamsTraitConst for core::Ptr<crate::flann::AutotunedIndexParams> {
		#[inline] fn as_raw_IndexParams(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::flann::IndexParamsTrait for core::Ptr<crate::flann::AutotunedIndexParams> {
		#[inline] fn as_raw_mut_IndexParams(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfAutotunedIndexParams, core::Ptr<crate::flann::IndexParams>, cv_PtrLcv_flann_AutotunedIndexParamsG_to_PtrOfIndexParams }
	
	impl std::fmt::Debug for core::Ptr<crate::flann::AutotunedIndexParams> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfAutotunedIndexParams")
				.finish()
		}
	}
	
	pub type PtrOfCompositeIndexParams = core::Ptr<crate::flann::CompositeIndexParams>;
	
	ptr_extern! { crate::flann::CompositeIndexParams,
		cv_PtrLcv_flann_CompositeIndexParamsG_delete, cv_PtrLcv_flann_CompositeIndexParamsG_getInnerPtr_const, cv_PtrLcv_flann_CompositeIndexParamsG_getInnerPtrMut
	}
	
	ptr_extern_ctor! { crate::flann::CompositeIndexParams, cv_PtrLcv_flann_CompositeIndexParamsG_new_const_CompositeIndexParams }
	impl core::Ptr<crate::flann::CompositeIndexParams> {
		#[inline] pub fn as_raw_PtrOfCompositeIndexParams(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfCompositeIndexParams(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::flann::CompositeIndexParamsTraitConst for core::Ptr<crate::flann::CompositeIndexParams> {
		#[inline] fn as_raw_CompositeIndexParams(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::flann::CompositeIndexParamsTrait for core::Ptr<crate::flann::CompositeIndexParams> {
		#[inline] fn as_raw_mut_CompositeIndexParams(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl crate::flann::IndexParamsTraitConst for core::Ptr<crate::flann::CompositeIndexParams> {
		#[inline] fn as_raw_IndexParams(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::flann::IndexParamsTrait for core::Ptr<crate::flann::CompositeIndexParams> {
		#[inline] fn as_raw_mut_IndexParams(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfCompositeIndexParams, core::Ptr<crate::flann::IndexParams>, cv_PtrLcv_flann_CompositeIndexParamsG_to_PtrOfIndexParams }
	
	impl std::fmt::Debug for core::Ptr<crate::flann::CompositeIndexParams> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfCompositeIndexParams")
				.finish()
		}
	}
	
	pub type PtrOfHierarchicalClusteringIndexParams = core::Ptr<crate::flann::HierarchicalClusteringIndexParams>;
	
	ptr_extern! { crate::flann::HierarchicalClusteringIndexParams,
		cv_PtrLcv_flann_HierarchicalClusteringIndexParamsG_delete, cv_PtrLcv_flann_HierarchicalClusteringIndexParamsG_getInnerPtr_const, cv_PtrLcv_flann_HierarchicalClusteringIndexParamsG_getInnerPtrMut
	}
	
	ptr_extern_ctor! { crate::flann::HierarchicalClusteringIndexParams, cv_PtrLcv_flann_HierarchicalClusteringIndexParamsG_new_const_HierarchicalClusteringIndexParams }
	impl core::Ptr<crate::flann::HierarchicalClusteringIndexParams> {
		#[inline] pub fn as_raw_PtrOfHierarchicalClusteringIndexParams(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfHierarchicalClusteringIndexParams(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::flann::HierarchicalClusteringIndexParamsTraitConst for core::Ptr<crate::flann::HierarchicalClusteringIndexParams> {
		#[inline] fn as_raw_HierarchicalClusteringIndexParams(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::flann::HierarchicalClusteringIndexParamsTrait for core::Ptr<crate::flann::HierarchicalClusteringIndexParams> {
		#[inline] fn as_raw_mut_HierarchicalClusteringIndexParams(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl crate::flann::IndexParamsTraitConst for core::Ptr<crate::flann::HierarchicalClusteringIndexParams> {
		#[inline] fn as_raw_IndexParams(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::flann::IndexParamsTrait for core::Ptr<crate::flann::HierarchicalClusteringIndexParams> {
		#[inline] fn as_raw_mut_IndexParams(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfHierarchicalClusteringIndexParams, core::Ptr<crate::flann::IndexParams>, cv_PtrLcv_flann_HierarchicalClusteringIndexParamsG_to_PtrOfIndexParams }
	
	impl std::fmt::Debug for core::Ptr<crate::flann::HierarchicalClusteringIndexParams> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfHierarchicalClusteringIndexParams")
				.finish()
		}
	}
	
	pub type PtrOfIndexParams = core::Ptr<crate::flann::IndexParams>;
	
	ptr_extern! { crate::flann::IndexParams,
		cv_PtrLcv_flann_IndexParamsG_delete, cv_PtrLcv_flann_IndexParamsG_getInnerPtr_const, cv_PtrLcv_flann_IndexParamsG_getInnerPtrMut
	}
	
	ptr_extern_ctor! { crate::flann::IndexParams, cv_PtrLcv_flann_IndexParamsG_new_const_IndexParams }
	impl core::Ptr<crate::flann::IndexParams> {
		#[inline] pub fn as_raw_PtrOfIndexParams(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfIndexParams(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::flann::IndexParamsTraitConst for core::Ptr<crate::flann::IndexParams> {
		#[inline] fn as_raw_IndexParams(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::flann::IndexParamsTrait for core::Ptr<crate::flann::IndexParams> {
		#[inline] fn as_raw_mut_IndexParams(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl std::fmt::Debug for core::Ptr<crate::flann::IndexParams> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfIndexParams")
				.finish()
		}
	}
	
	pub type PtrOfKDTreeIndexParams = core::Ptr<crate::flann::KDTreeIndexParams>;
	
	ptr_extern! { crate::flann::KDTreeIndexParams,
		cv_PtrLcv_flann_KDTreeIndexParamsG_delete, cv_PtrLcv_flann_KDTreeIndexParamsG_getInnerPtr_const, cv_PtrLcv_flann_KDTreeIndexParamsG_getInnerPtrMut
	}
	
	ptr_extern_ctor! { crate::flann::KDTreeIndexParams, cv_PtrLcv_flann_KDTreeIndexParamsG_new_const_KDTreeIndexParams }
	impl core::Ptr<crate::flann::KDTreeIndexParams> {
		#[inline] pub fn as_raw_PtrOfKDTreeIndexParams(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfKDTreeIndexParams(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::flann::KDTreeIndexParamsTraitConst for core::Ptr<crate::flann::KDTreeIndexParams> {
		#[inline] fn as_raw_KDTreeIndexParams(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::flann::KDTreeIndexParamsTrait for core::Ptr<crate::flann::KDTreeIndexParams> {
		#[inline] fn as_raw_mut_KDTreeIndexParams(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl crate::flann::IndexParamsTraitConst for core::Ptr<crate::flann::KDTreeIndexParams> {
		#[inline] fn as_raw_IndexParams(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::flann::IndexParamsTrait for core::Ptr<crate::flann::KDTreeIndexParams> {
		#[inline] fn as_raw_mut_IndexParams(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfKDTreeIndexParams, core::Ptr<crate::flann::IndexParams>, cv_PtrLcv_flann_KDTreeIndexParamsG_to_PtrOfIndexParams }
	
	impl std::fmt::Debug for core::Ptr<crate::flann::KDTreeIndexParams> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfKDTreeIndexParams")
				.finish()
		}
	}
	
	pub type PtrOfKMeansIndexParams = core::Ptr<crate::flann::KMeansIndexParams>;
	
	ptr_extern! { crate::flann::KMeansIndexParams,
		cv_PtrLcv_flann_KMeansIndexParamsG_delete, cv_PtrLcv_flann_KMeansIndexParamsG_getInnerPtr_const, cv_PtrLcv_flann_KMeansIndexParamsG_getInnerPtrMut
	}
	
	ptr_extern_ctor! { crate::flann::KMeansIndexParams, cv_PtrLcv_flann_KMeansIndexParamsG_new_const_KMeansIndexParams }
	impl core::Ptr<crate::flann::KMeansIndexParams> {
		#[inline] pub fn as_raw_PtrOfKMeansIndexParams(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfKMeansIndexParams(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::flann::KMeansIndexParamsTraitConst for core::Ptr<crate::flann::KMeansIndexParams> {
		#[inline] fn as_raw_KMeansIndexParams(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::flann::KMeansIndexParamsTrait for core::Ptr<crate::flann::KMeansIndexParams> {
		#[inline] fn as_raw_mut_KMeansIndexParams(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl crate::flann::IndexParamsTraitConst for core::Ptr<crate::flann::KMeansIndexParams> {
		#[inline] fn as_raw_IndexParams(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::flann::IndexParamsTrait for core::Ptr<crate::flann::KMeansIndexParams> {
		#[inline] fn as_raw_mut_IndexParams(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfKMeansIndexParams, core::Ptr<crate::flann::IndexParams>, cv_PtrLcv_flann_KMeansIndexParamsG_to_PtrOfIndexParams }
	
	impl std::fmt::Debug for core::Ptr<crate::flann::KMeansIndexParams> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfKMeansIndexParams")
				.finish()
		}
	}
	
	pub type PtrOfLinearIndexParams = core::Ptr<crate::flann::LinearIndexParams>;
	
	ptr_extern! { crate::flann::LinearIndexParams,
		cv_PtrLcv_flann_LinearIndexParamsG_delete, cv_PtrLcv_flann_LinearIndexParamsG_getInnerPtr_const, cv_PtrLcv_flann_LinearIndexParamsG_getInnerPtrMut
	}
	
	ptr_extern_ctor! { crate::flann::LinearIndexParams, cv_PtrLcv_flann_LinearIndexParamsG_new_const_LinearIndexParams }
	impl core::Ptr<crate::flann::LinearIndexParams> {
		#[inline] pub fn as_raw_PtrOfLinearIndexParams(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfLinearIndexParams(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::flann::LinearIndexParamsTraitConst for core::Ptr<crate::flann::LinearIndexParams> {
		#[inline] fn as_raw_LinearIndexParams(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::flann::LinearIndexParamsTrait for core::Ptr<crate::flann::LinearIndexParams> {
		#[inline] fn as_raw_mut_LinearIndexParams(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl crate::flann::IndexParamsTraitConst for core::Ptr<crate::flann::LinearIndexParams> {
		#[inline] fn as_raw_IndexParams(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::flann::IndexParamsTrait for core::Ptr<crate::flann::LinearIndexParams> {
		#[inline] fn as_raw_mut_IndexParams(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfLinearIndexParams, core::Ptr<crate::flann::IndexParams>, cv_PtrLcv_flann_LinearIndexParamsG_to_PtrOfIndexParams }
	
	impl std::fmt::Debug for core::Ptr<crate::flann::LinearIndexParams> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfLinearIndexParams")
				.finish()
		}
	}
	
	pub type PtrOfLshIndexParams = core::Ptr<crate::flann::LshIndexParams>;
	
	ptr_extern! { crate::flann::LshIndexParams,
		cv_PtrLcv_flann_LshIndexParamsG_delete, cv_PtrLcv_flann_LshIndexParamsG_getInnerPtr_const, cv_PtrLcv_flann_LshIndexParamsG_getInnerPtrMut
	}
	
	ptr_extern_ctor! { crate::flann::LshIndexParams, cv_PtrLcv_flann_LshIndexParamsG_new_const_LshIndexParams }
	impl core::Ptr<crate::flann::LshIndexParams> {
		#[inline] pub fn as_raw_PtrOfLshIndexParams(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfLshIndexParams(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::flann::LshIndexParamsTraitConst for core::Ptr<crate::flann::LshIndexParams> {
		#[inline] fn as_raw_LshIndexParams(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::flann::LshIndexParamsTrait for core::Ptr<crate::flann::LshIndexParams> {
		#[inline] fn as_raw_mut_LshIndexParams(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl crate::flann::IndexParamsTraitConst for core::Ptr<crate::flann::LshIndexParams> {
		#[inline] fn as_raw_IndexParams(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::flann::IndexParamsTrait for core::Ptr<crate::flann::LshIndexParams> {
		#[inline] fn as_raw_mut_IndexParams(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfLshIndexParams, core::Ptr<crate::flann::IndexParams>, cv_PtrLcv_flann_LshIndexParamsG_to_PtrOfIndexParams }
	
	impl std::fmt::Debug for core::Ptr<crate::flann::LshIndexParams> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfLshIndexParams")
				.finish()
		}
	}
	
	pub type PtrOfSavedIndexParams = core::Ptr<crate::flann::SavedIndexParams>;
	
	ptr_extern! { crate::flann::SavedIndexParams,
		cv_PtrLcv_flann_SavedIndexParamsG_delete, cv_PtrLcv_flann_SavedIndexParamsG_getInnerPtr_const, cv_PtrLcv_flann_SavedIndexParamsG_getInnerPtrMut
	}
	
	ptr_extern_ctor! { crate::flann::SavedIndexParams, cv_PtrLcv_flann_SavedIndexParamsG_new_const_SavedIndexParams }
	impl core::Ptr<crate::flann::SavedIndexParams> {
		#[inline] pub fn as_raw_PtrOfSavedIndexParams(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfSavedIndexParams(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::flann::SavedIndexParamsTraitConst for core::Ptr<crate::flann::SavedIndexParams> {
		#[inline] fn as_raw_SavedIndexParams(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::flann::SavedIndexParamsTrait for core::Ptr<crate::flann::SavedIndexParams> {
		#[inline] fn as_raw_mut_SavedIndexParams(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl crate::flann::IndexParamsTraitConst for core::Ptr<crate::flann::SavedIndexParams> {
		#[inline] fn as_raw_IndexParams(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::flann::IndexParamsTrait for core::Ptr<crate::flann::SavedIndexParams> {
		#[inline] fn as_raw_mut_IndexParams(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfSavedIndexParams, core::Ptr<crate::flann::IndexParams>, cv_PtrLcv_flann_SavedIndexParamsG_to_PtrOfIndexParams }
	
	impl std::fmt::Debug for core::Ptr<crate::flann::SavedIndexParams> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfSavedIndexParams")
				.finish()
		}
	}
	
	pub type PtrOfSearchParams = core::Ptr<crate::flann::SearchParams>;
	
	ptr_extern! { crate::flann::SearchParams,
		cv_PtrLcv_flann_SearchParamsG_delete, cv_PtrLcv_flann_SearchParamsG_getInnerPtr_const, cv_PtrLcv_flann_SearchParamsG_getInnerPtrMut
	}
	
	ptr_extern_ctor! { crate::flann::SearchParams, cv_PtrLcv_flann_SearchParamsG_new_const_SearchParams }
	impl core::Ptr<crate::flann::SearchParams> {
		#[inline] pub fn as_raw_PtrOfSearchParams(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfSearchParams(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::flann::SearchParamsTraitConst for core::Ptr<crate::flann::SearchParams> {
		#[inline] fn as_raw_SearchParams(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::flann::SearchParamsTrait for core::Ptr<crate::flann::SearchParams> {
		#[inline] fn as_raw_mut_SearchParams(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl crate::flann::IndexParamsTraitConst for core::Ptr<crate::flann::SearchParams> {
		#[inline] fn as_raw_IndexParams(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::flann::IndexParamsTrait for core::Ptr<crate::flann::SearchParams> {
		#[inline] fn as_raw_mut_IndexParams(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfSearchParams, core::Ptr<crate::flann::IndexParams>, cv_PtrLcv_flann_SearchParamsG_to_PtrOfIndexParams }
	
	impl std::fmt::Debug for core::Ptr<crate::flann::SearchParams> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfSearchParams")
				.finish()
		}
	}
	
	pub type VectorOfFlannIndexType = core::Vector<crate::flann::FlannIndexType>;
	
	impl core::Vector<crate::flann::FlannIndexType> {
		pub fn as_raw_VectorOfFlannIndexType(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOfFlannIndexType(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { crate::flann::FlannIndexType,
		std_vectorLcv_flann_FlannIndexTypeG_new_const, std_vectorLcv_flann_FlannIndexTypeG_delete,
		std_vectorLcv_flann_FlannIndexTypeG_len_const, std_vectorLcv_flann_FlannIndexTypeG_isEmpty_const,
		std_vectorLcv_flann_FlannIndexTypeG_capacity_const, std_vectorLcv_flann_FlannIndexTypeG_shrinkToFit,
		std_vectorLcv_flann_FlannIndexTypeG_reserve_size_t, std_vectorLcv_flann_FlannIndexTypeG_remove_size_t,
		std_vectorLcv_flann_FlannIndexTypeG_swap_size_t_size_t, std_vectorLcv_flann_FlannIndexTypeG_clear,
		std_vectorLcv_flann_FlannIndexTypeG_get_const_size_t, std_vectorLcv_flann_FlannIndexTypeG_set_size_t_const_FlannIndexType,
		std_vectorLcv_flann_FlannIndexTypeG_push_const_FlannIndexType, std_vectorLcv_flann_FlannIndexTypeG_insert_size_t_const_FlannIndexType,
	}
	vector_copy_non_bool! { crate::flann::FlannIndexType,
		std_vectorLcv_flann_FlannIndexTypeG_data_const, std_vectorLcv_flann_FlannIndexTypeG_dataMut, cv_fromSlice_const_const_FlannIndexTypeX_size_t,
		std_vectorLcv_flann_FlannIndexTypeG_clone_const,
	}
	
	pub type VectorOffeature_index = core::Vector<crate::flann::feature_index>;
	
	impl core::Vector<crate::flann::feature_index> {
		pub fn as_raw_VectorOffeature_index(&self) -> extern_send!(Self) { self.as_raw() }
		pub fn as_raw_mut_VectorOffeature_index(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	vector_extern! { crate::flann::feature_index,
		std_vectorLcvflann_lsh_FeatureIndexG_new_const, std_vectorLcvflann_lsh_FeatureIndexG_delete,
		std_vectorLcvflann_lsh_FeatureIndexG_len_const, std_vectorLcvflann_lsh_FeatureIndexG_isEmpty_const,
		std_vectorLcvflann_lsh_FeatureIndexG_capacity_const, std_vectorLcvflann_lsh_FeatureIndexG_shrinkToFit,
		std_vectorLcvflann_lsh_FeatureIndexG_reserve_size_t, std_vectorLcvflann_lsh_FeatureIndexG_remove_size_t,
		std_vectorLcvflann_lsh_FeatureIndexG_swap_size_t_size_t, std_vectorLcvflann_lsh_FeatureIndexG_clear,
		std_vectorLcvflann_lsh_FeatureIndexG_get_const_size_t, std_vectorLcvflann_lsh_FeatureIndexG_set_size_t_const_FeatureIndex,
		std_vectorLcvflann_lsh_FeatureIndexG_push_const_FeatureIndex, std_vectorLcvflann_lsh_FeatureIndexG_insert_size_t_const_FeatureIndex,
	}
	vector_copy_non_bool! { crate::flann::feature_index,
		std_vectorLcvflann_lsh_FeatureIndexG_data_const, std_vectorLcvflann_lsh_FeatureIndexG_dataMut, cv_fromSlice_const_const_FeatureIndexX_size_t,
		std_vectorLcvflann_lsh_FeatureIndexG_clone_const,
	}
	
}
#[cfg(ocvrs_has_module_flann)]
pub use flann_types::*;

#[cfg(ocvrs_has_module_imgproc)]
mod imgproc_types {
	use crate::{mod_prelude::*, core, types, sys};

	pub type PtrOfCLAHE = core::Ptr<crate::imgproc::CLAHE>;
	
	ptr_extern! { crate::imgproc::CLAHE,
		cv_PtrLcv_CLAHEG_delete, cv_PtrLcv_CLAHEG_getInnerPtr_const, cv_PtrLcv_CLAHEG_getInnerPtrMut
	}
	
	impl core::Ptr<crate::imgproc::CLAHE> {
		#[inline] pub fn as_raw_PtrOfCLAHE(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfCLAHE(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::imgproc::CLAHETraitConst for core::Ptr<crate::imgproc::CLAHE> {
		#[inline] fn as_raw_CLAHE(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::imgproc::CLAHETrait for core::Ptr<crate::imgproc::CLAHE> {
		#[inline] fn as_raw_mut_CLAHE(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl core::AlgorithmTraitConst for core::Ptr<crate::imgproc::CLAHE> {
		#[inline] fn as_raw_Algorithm(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::AlgorithmTrait for core::Ptr<crate::imgproc::CLAHE> {
		#[inline] fn as_raw_mut_Algorithm(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfCLAHE, core::Ptr<core::Algorithm>, cv_PtrLcv_CLAHEG_to_PtrOfAlgorithm }
	
	impl std::fmt::Debug for core::Ptr<crate::imgproc::CLAHE> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfCLAHE")
				.finish()
		}
	}
	
	pub type PtrOfGeneralizedHough = core::Ptr<crate::imgproc::GeneralizedHough>;
	
	ptr_extern! { crate::imgproc::GeneralizedHough,
		cv_PtrLcv_GeneralizedHoughG_delete, cv_PtrLcv_GeneralizedHoughG_getInnerPtr_const, cv_PtrLcv_GeneralizedHoughG_getInnerPtrMut
	}
	
	impl core::Ptr<crate::imgproc::GeneralizedHough> {
		#[inline] pub fn as_raw_PtrOfGeneralizedHough(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfGeneralizedHough(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::imgproc::GeneralizedHoughTraitConst for core::Ptr<crate::imgproc::GeneralizedHough> {
		#[inline] fn as_raw_GeneralizedHough(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::imgproc::GeneralizedHoughTrait for core::Ptr<crate::imgproc::GeneralizedHough> {
		#[inline] fn as_raw_mut_GeneralizedHough(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl core::AlgorithmTraitConst for core::Ptr<crate::imgproc::GeneralizedHough> {
		#[inline] fn as_raw_Algorithm(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::AlgorithmTrait for core::Ptr<crate::imgproc::GeneralizedHough> {
		#[inline] fn as_raw_mut_Algorithm(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfGeneralizedHough, core::Ptr<core::Algorithm>, cv_PtrLcv_GeneralizedHoughG_to_PtrOfAlgorithm }
	
	impl std::fmt::Debug for core::Ptr<crate::imgproc::GeneralizedHough> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfGeneralizedHough")
				.finish()
		}
	}
	
	pub type PtrOfGeneralizedHoughBallard = core::Ptr<crate::imgproc::GeneralizedHoughBallard>;
	
	ptr_extern! { crate::imgproc::GeneralizedHoughBallard,
		cv_PtrLcv_GeneralizedHoughBallardG_delete, cv_PtrLcv_GeneralizedHoughBallardG_getInnerPtr_const, cv_PtrLcv_GeneralizedHoughBallardG_getInnerPtrMut
	}
	
	impl core::Ptr<crate::imgproc::GeneralizedHoughBallard> {
		#[inline] pub fn as_raw_PtrOfGeneralizedHoughBallard(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfGeneralizedHoughBallard(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::imgproc::GeneralizedHoughBallardTraitConst for core::Ptr<crate::imgproc::GeneralizedHoughBallard> {
		#[inline] fn as_raw_GeneralizedHoughBallard(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::imgproc::GeneralizedHoughBallardTrait for core::Ptr<crate::imgproc::GeneralizedHoughBallard> {
		#[inline] fn as_raw_mut_GeneralizedHoughBallard(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl core::AlgorithmTraitConst for core::Ptr<crate::imgproc::GeneralizedHoughBallard> {
		#[inline] fn as_raw_Algorithm(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::AlgorithmTrait for core::Ptr<crate::imgproc::GeneralizedHoughBallard> {
		#[inline] fn as_raw_mut_Algorithm(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfGeneralizedHoughBallard, core::Ptr<core::Algorithm>, cv_PtrLcv_GeneralizedHoughBallardG_to_PtrOfAlgorithm }
	
	impl crate::imgproc::GeneralizedHoughTraitConst for core::Ptr<crate::imgproc::GeneralizedHoughBallard> {
		#[inline] fn as_raw_GeneralizedHough(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::imgproc::GeneralizedHoughTrait for core::Ptr<crate::imgproc::GeneralizedHoughBallard> {
		#[inline] fn as_raw_mut_GeneralizedHough(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfGeneralizedHoughBallard, core::Ptr<crate::imgproc::GeneralizedHough>, cv_PtrLcv_GeneralizedHoughBallardG_to_PtrOfGeneralizedHough }
	
	impl std::fmt::Debug for core::Ptr<crate::imgproc::GeneralizedHoughBallard> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfGeneralizedHoughBallard")
				.finish()
		}
	}
	
	pub type PtrOfGeneralizedHoughGuil = core::Ptr<crate::imgproc::GeneralizedHoughGuil>;
	
	ptr_extern! { crate::imgproc::GeneralizedHoughGuil,
		cv_PtrLcv_GeneralizedHoughGuilG_delete, cv_PtrLcv_GeneralizedHoughGuilG_getInnerPtr_const, cv_PtrLcv_GeneralizedHoughGuilG_getInnerPtrMut
	}
	
	impl core::Ptr<crate::imgproc::GeneralizedHoughGuil> {
		#[inline] pub fn as_raw_PtrOfGeneralizedHoughGuil(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfGeneralizedHoughGuil(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::imgproc::GeneralizedHoughGuilTraitConst for core::Ptr<crate::imgproc::GeneralizedHoughGuil> {
		#[inline] fn as_raw_GeneralizedHoughGuil(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::imgproc::GeneralizedHoughGuilTrait for core::Ptr<crate::imgproc::GeneralizedHoughGuil> {
		#[inline] fn as_raw_mut_GeneralizedHoughGuil(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl core::AlgorithmTraitConst for core::Ptr<crate::imgproc::GeneralizedHoughGuil> {
		#[inline] fn as_raw_Algorithm(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::AlgorithmTrait for core::Ptr<crate::imgproc::GeneralizedHoughGuil> {
		#[inline] fn as_raw_mut_Algorithm(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfGeneralizedHoughGuil, core::Ptr<core::Algorithm>, cv_PtrLcv_GeneralizedHoughGuilG_to_PtrOfAlgorithm }
	
	impl crate::imgproc::GeneralizedHoughTraitConst for core::Ptr<crate::imgproc::GeneralizedHoughGuil> {
		#[inline] fn as_raw_GeneralizedHough(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::imgproc::GeneralizedHoughTrait for core::Ptr<crate::imgproc::GeneralizedHoughGuil> {
		#[inline] fn as_raw_mut_GeneralizedHough(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfGeneralizedHoughGuil, core::Ptr<crate::imgproc::GeneralizedHough>, cv_PtrLcv_GeneralizedHoughGuilG_to_PtrOfGeneralizedHough }
	
	impl std::fmt::Debug for core::Ptr<crate::imgproc::GeneralizedHoughGuil> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfGeneralizedHoughGuil")
				.finish()
		}
	}
	
	pub type PtrOfLineSegmentDetector = core::Ptr<crate::imgproc::LineSegmentDetector>;
	
	ptr_extern! { crate::imgproc::LineSegmentDetector,
		cv_PtrLcv_LineSegmentDetectorG_delete, cv_PtrLcv_LineSegmentDetectorG_getInnerPtr_const, cv_PtrLcv_LineSegmentDetectorG_getInnerPtrMut
	}
	
	impl core::Ptr<crate::imgproc::LineSegmentDetector> {
		#[inline] pub fn as_raw_PtrOfLineSegmentDetector(&self) -> extern_send!(Self) { self.as_raw() }
		#[inline] pub fn as_raw_mut_PtrOfLineSegmentDetector(&mut self) -> extern_send!(mut Self) { self.as_raw_mut() }
	}
	
	impl crate::imgproc::LineSegmentDetectorTraitConst for core::Ptr<crate::imgproc::LineSegmentDetector> {
		#[inline] fn as_raw_LineSegmentDetector(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl crate::imgproc::LineSegmentDetectorTrait for core::Ptr<crate::imgproc::LineSegmentDetector> {
		#[inline] fn as_raw_mut_LineSegmentDetector(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	impl core::AlgorithmTraitConst for core::Ptr<crate::imgproc::LineSegmentDetector> {
		#[inline] fn as_raw_Algorithm(&self) -> *const c_void { self.inner_as_raw() }
	}
	
	impl core::AlgorithmTrait for core::Ptr<crate::imgproc::LineSegmentDetector> {
		#[inline] fn as_raw_mut_Algorithm(&mut self) -> *mut c_void { self.inner_as_raw_mut() }
	}
	
	ptr_cast_base! { PtrOfLineSegmentDetector, core::Ptr<core::Algorithm>, cv_PtrLcv_LineSegmentDetectorG_to_PtrOfAlgorithm }
	
	impl std::fmt::Debug for core::Ptr<crate::imgproc::LineSegmentDetector> {
		#[inline]
		fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
			f.debug_struct("PtrOfLineSegmentDetector")
				.finish()
		}
	}
	
}
#[cfg(ocvrs_has_module_imgproc)]
pub use imgproc_types::*;

pub use crate::manual::types::*;
