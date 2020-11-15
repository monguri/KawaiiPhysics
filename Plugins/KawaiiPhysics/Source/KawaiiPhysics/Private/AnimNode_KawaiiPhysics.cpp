#include "AnimNode_KawaiiPhysics.h"

#include "KawaiiPhysics.h"
#include "AnimationRuntime.h"
#include "Animation/AnimInstanceProxy.h"
#include "Curves/CurveFloat.h"
#include "KawaiiPhysicsLimitsDataAsset.h"

TAutoConsoleVariable<int32> CVarEnableOldPhysicsMethodGrayity(TEXT("p.KawaiiPhysics.EnableOldPhysicsMethodGravity"), 0, 
	TEXT("Enables/Disables old physics method for gravity before v1.3.1. This is the setting for the transition period when changing the physical calculation."));
TAutoConsoleVariable<int32> CVarEnableOldPhysicsMethodSphereLimit(TEXT("p.KawaiiPhysics.EnableOldPhysicsMethodSphereLimit"), 0,
	TEXT("Enables/Disables old physics method for sphere limit before v1.3.1. This is the setting for the transition period when changing the physical calculation."));

FAnimNode_KawaiiPhysics::FAnimNode_KawaiiPhysics()
{

}

void FAnimNode_KawaiiPhysics::Initialize_AnyThread(const FAnimationInitializeContext& Context)
{
	FAnimNode_SkeletalControlBase::Initialize_AnyThread(Context);
	FBoneContainer& RequiredBones = Context.AnimInstanceProxy->GetRequiredBones();

	ApplyLimitsDataAsset(RequiredBones);

	if (bUsePhysicsAssetAsLimits)
	{
		const USkeletalMeshComponent* SkeletalMeshComp = Context.AnimInstanceProxy->GetSkelMeshComponent();
		const USkeletalMesh* SkeletalMeshAsset = SkeletalMeshComp->SkeletalMesh;

		const FReferenceSkeleton& SkelMeshRefSkel = SkeletalMeshAsset->RefSkeleton;
		UsePhysicsAssetAsLimits = OverridePhysicsAssetAsLimits ? OverridePhysicsAssetAsLimits : SkeletalMeshComp->GetPhysicsAsset();
	}

	InitializeBoneReferences(RequiredBones);

	ModifyBones.Empty();

	// For Avoiding Zero Divide in the first frame
	DeltaTimeOld = 1.0f / TargetFramerate;

#if WITH_EDITOR
	auto World = Context.AnimInstanceProxy->GetSkelMeshComponent()->GetWorld();
	if (World->WorldType == EWorldType::Editor ||
		World->WorldType == EWorldType::EditorPreview)
	{
		bEditing = true;
	}
#endif
}

void FAnimNode_KawaiiPhysics::CacheBones_AnyThread(const FAnimationCacheBonesContext& Context)
{
	FAnimNode_SkeletalControlBase::CacheBones_AnyThread(Context);

}

void FAnimNode_KawaiiPhysics::UpdateInternal(const FAnimationUpdateContext& Context)
{
	FAnimNode_SkeletalControlBase::UpdateInternal(Context);

	DeltaTime = Context.GetDeltaTime();
}

DECLARE_CYCLE_STAT(TEXT("KawaiiPhysics_Eval"), STAT_KawaiiPhysics_Eval, STATGROUP_Anim);

void FAnimNode_KawaiiPhysics::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
{
	SCOPE_CYCLE_COUNTER(STAT_KawaiiPhysics_Eval);

	check(OutBoneTransforms.Num() == 0);

	const FBoneContainer& BoneContainer = Output.Pose.GetPose().GetBoneContainer();
	FTransform ComponentTransform = Output.AnimInstanceProxy->GetComponentTransform();

#if WITH_EDITOR
	// sync editing on other Nodes
	if (LimitsDataAsset)
	{ 
		ApplyLimitsDataAsset(BoneContainer);
	}
	if (bUsePhysicsAssetAsLimits)
	{
		const USkeletalMeshComponent* SkeletalMeshComp = Output.AnimInstanceProxy->GetSkelMeshComponent();
		const USkeletalMesh* SkeletalMeshAsset = SkeletalMeshComp->SkeletalMesh;

		const FReferenceSkeleton& SkelMeshRefSkel = SkeletalMeshAsset->RefSkeleton;
		UsePhysicsAssetAsLimits = OverridePhysicsAssetAsLimits ? OverridePhysicsAssetAsLimits : SkeletalMeshComp->GetPhysicsAsset();
	}
#endif

	if (!RootBone.IsValidToEvaluate(BoneContainer))
	{
		return;
	}

	if (ModifyBones.Num() == 0)
	{
		InitModifyBones(Output, BoneContainer);
		PreSkelCompTransform = ComponentTransform;
	}

	// Update each parameters and collision
	if (!bInitPhysicsSettings || bUpdatePhysicsSettingsInGame)
	{
		UpdatePhysicsSettingsOfModifyBones();
		
#if WITH_EDITORONLY_DATA
		if (!bEditing)
#endif
		{
			bInitPhysicsSettings = true;
		}
	}
	UpdateSphericalLimits(SphericalLimits, Output, BoneContainer, ComponentTransform);
	UpdateSphericalLimits(SphericalLimitsData, Output, BoneContainer, ComponentTransform);
	UpdateCapsuleLimits(CapsuleLimits, Output, BoneContainer, ComponentTransform);
	UpdateCapsuleLimits(CapsuleLimitsData, Output, BoneContainer, ComponentTransform);
	UpdatePlanerLimits(PlanarLimits,Output, BoneContainer, ComponentTransform);
	UpdatePlanerLimits(PlanarLimitsData, Output, BoneContainer, ComponentTransform);
	for (auto& Bone : ModifyBones)
	{
		if (!Bone.bDummy)
		{
			Bone.UpdatePoseTranform(BoneContainer, Output.Pose);
		}
		else
		{
			auto ParentBone = ModifyBones[Bone.ParentIndex];
			Bone.PoseLocation = ParentBone.PoseLocation + GetBoneForwardVector(ParentBone.PoseRotation) * DummyBoneLength;
			Bone.PoseRotation = ParentBone.PoseRotation;
			Bone.PoseScale = ParentBone.PoseScale;
		}
	}
	

	// Calc SkeletalMeshComponent movement in World Space
	SkelCompMoveVector = ComponentTransform.InverseTransformPosition(PreSkelCompTransform.GetLocation());
	if (SkelCompMoveVector.SizeSquared() > TeleportDistanceThreshold * TeleportDistanceThreshold)
	{
		SkelCompMoveVector = FVector::ZeroVector;
	}

	SkelCompMoveRotation = ComponentTransform.InverseTransformRotation(PreSkelCompTransform.GetRotation());
	if ( TeleportRotationThreshold >= 0 && FMath::RadiansToDegrees( SkelCompMoveRotation.GetAngle() ) > TeleportRotationThreshold )
	{
		SkelCompMoveRotation = FQuat::Identity;
	}

	PreSkelCompTransform = ComponentTransform;

	// Simulate Physics and Apply
	SimulateModifyBones(Output, BoneContainer, ComponentTransform);
	ApplySimuateResult(Output, BoneContainer, OutBoneTransforms);
}

bool FAnimNode_KawaiiPhysics::IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones)
{
	return RootBone.IsValidToEvaluate(RequiredBones);
}

void FAnimNode_KawaiiPhysics::InitializeBoneReferences(const FBoneContainer& RequiredBones)
{
	RootBone.Initialize(RequiredBones);

	for (auto& Bone : ModifyBones)
	{
		Bone.BoneRef.Initialize(RequiredBones);
	}

	for (auto& Sphere : SphericalLimits)
	{
		Sphere.DrivingBone.Initialize(RequiredBones);
	}
	for (auto& Capsule : CapsuleLimits)
	{
		Capsule.DrivingBone.Initialize(RequiredBones);
	}
	for (auto& Planer : PlanarLimits)
	{
		Planer.DrivingBone.Initialize(RequiredBones);
	}

}

DECLARE_CYCLE_STAT(TEXT("KawaiiPhysics_InitModifyBones"), STAT_KawaiiPhysics_InitModifyBones, STATGROUP_Anim);

void FAnimNode_KawaiiPhysics::InitModifyBones(FComponentSpacePoseContext& Output, const FBoneContainer& BoneContainer)
{
	SCOPE_CYCLE_COUNTER(STAT_KawaiiPhysics_InitModifyBones);

	auto Skeleton = BoneContainer.GetSkeletonAsset();
	auto& RefSkeleton = Skeleton->GetReferenceSkeleton();

	ModifyBones.Empty();
	AddModifyBone(Output, BoneContainer, RefSkeleton, RefSkeleton.FindBoneIndex(RootBone.BoneName));
	if (ModifyBones.Num() > 0)
	{
		CalcBoneLength(ModifyBones[0], BoneContainer.GetRefPoseCompactArray());
	}

	if (bUsePhysicsAssetAsShapes && PhysicsAssetAsShapes != nullptr)
	{
		InitModifyBonesPhysicsBodiesSetup(Output, BoneContainer);
	}
}

void FAnimNode_KawaiiPhysics::InitModifyBonesPhysicsBodiesSetup(FComponentSpacePoseContext& Output, const FBoneContainer& BoneContainer)
{
	check(bUsePhysicsAssetAsShapes && PhysicsAssetAsShapes != nullptr);

	for (FKawaiiPhysicsModifyBone& ModifyBone : ModifyBones)
	{
		for (int32 i = 0; i < PhysicsAssetAsShapes->SkeletalBodySetups.Num(); ++i)
		{
			if (!ensure(PhysicsAssetAsShapes->SkeletalBodySetups[i]))
			{
				continue;
			}

			if (PhysicsAssetAsShapes->SkeletalBodySetups[i]->BoneName == ModifyBone.BoneRef.BoneName)
			{
				ModifyBone.PhysicsBodySetup = PhysicsAssetAsShapes->SkeletalBodySetups[i];
				break; // SkeletalBodySetupsの中で骨が一致するものはひとつしかない前提
			}
		}
	}
}

void FAnimNode_KawaiiPhysics::ApplyLimitsDataAsset(const FBoneContainer& RequiredBones)
{
	SphericalLimitsData.Empty();
	CapsuleLimitsData.Empty();
	PlanarLimitsData.Empty();
	if (LimitsDataAsset)
	{
		SphericalLimitsData = LimitsDataAsset->SphericalLimits;
		CapsuleLimitsData = LimitsDataAsset->CapsuleLimits;
		PlanarLimitsData = LimitsDataAsset->PlanarLimits;
	}

	for (auto& Sphere : SphericalLimitsData)
	{
		Sphere.DrivingBone.Initialize(RequiredBones);
	}

	for (auto& Capsule : CapsuleLimitsData)
	{
		Capsule.DrivingBone.Initialize(RequiredBones);
	}
	for (auto& Planer : PlanarLimitsData)
	{
		Planer.DrivingBone.Initialize(RequiredBones);
	}
}

int FAnimNode_KawaiiPhysics::AddModifyBone(FComponentSpacePoseContext& Output, const FBoneContainer& BoneContainer, 
	const FReferenceSkeleton& RefSkeleton, int BoneIndex)
{
	if (BoneIndex < 0 || RefSkeleton.GetNum() < BoneIndex)
	{
		return INDEX_NONE;
	}

	FBoneReference BoneRef;
	BoneRef.BoneName = RefSkeleton.GetBoneName(BoneIndex);

	if (ExcludeBones.Num() > 0 && ExcludeBones.Find(BoneRef) >= 0)
	{
		return INDEX_NONE;
	}

	FKawaiiPhysicsModifyBone NewModifyBone;
	NewModifyBone.BoneRef = BoneRef;
	NewModifyBone.BoneRef.Initialize(BoneContainer);
	if (NewModifyBone.BoneRef.CachedCompactPoseIndex == INDEX_NONE)
	{
		return INDEX_NONE;
	}

	auto& RefBonePoseTransform = Output.Pose.GetComponentSpaceTransform(NewModifyBone.BoneRef.CachedCompactPoseIndex);
	NewModifyBone.Location = RefBonePoseTransform.GetLocation();  
	NewModifyBone.PrevLocation = NewModifyBone.Location;
	NewModifyBone.PoseLocation = NewModifyBone.Location;
	NewModifyBone.PrevRotation = RefBonePoseTransform.GetRotation();
	NewModifyBone.PoseRotation = NewModifyBone.PrevRotation;
	NewModifyBone.PoseScale = RefBonePoseTransform.GetScale3D();
	int ModifyBoneIndex = ModifyBones.Add(NewModifyBone);

	TArray<int32> ChildBoneIndexs;
	CollectChildBones(RefSkeleton, BoneIndex, ChildBoneIndexs);

	if (ChildBoneIndexs.Num() > 0)
	{
		for (auto ChildBoneIndex : ChildBoneIndexs)
		{
			auto ChildModifyBoneIndex = AddModifyBone(Output, BoneContainer, RefSkeleton, ChildBoneIndex);
			if (ChildModifyBoneIndex >= 0)
			{
				ModifyBones[ModifyBoneIndex].ChildIndexs.Add(ChildModifyBoneIndex);
				ModifyBones[ChildModifyBoneIndex].ParentIndex = ModifyBoneIndex;
			}
		}
	}
	else if(DummyBoneLength > 0.0f)
	{
		// Add dummy modify bone
		FKawaiiPhysicsModifyBone DummyModifyBone;
		DummyModifyBone.bDummy = true;
		DummyModifyBone.Location = NewModifyBone.Location + GetBoneForwardVector(NewModifyBone.PrevRotation) * DummyBoneLength;
		DummyModifyBone.PrevLocation = DummyModifyBone.Location;
		DummyModifyBone.PoseLocation = DummyModifyBone.Location;
		DummyModifyBone.PrevRotation = NewModifyBone.PrevRotation;
		DummyModifyBone.PoseRotation = DummyModifyBone.PrevRotation;
		DummyModifyBone.PoseScale = RefBonePoseTransform.GetScale3D();

		int DummyBoneIndex = ModifyBones.Add(DummyModifyBone);
		ModifyBones[ModifyBoneIndex].ChildIndexs.Add(DummyBoneIndex);
		ModifyBones[DummyBoneIndex].ParentIndex = ModifyBoneIndex;
	}


	return ModifyBoneIndex;
}

int32 FAnimNode_KawaiiPhysics::CollectChildBones(const FReferenceSkeleton& RefSkeleton, int32 ParentBoneIndex, TArray<int32> & Children) const
{
	Children.Reset();

	const int32 NumBones = RefSkeleton.GetNum();
	for (int32 ChildIndex = ParentBoneIndex + 1; ChildIndex < NumBones; ChildIndex++)
	{
		if (ParentBoneIndex == RefSkeleton.GetParentIndex(ChildIndex))
		{
			Children.Add(ChildIndex);
		}
	}

	return Children.Num();
}

void FAnimNode_KawaiiPhysics::CalcBoneLength(FKawaiiPhysicsModifyBone& Bone, const TArray<FTransform>& RefBonePose)
{
	if (Bone.ParentIndex < 0)
	{
		Bone.LengthFromRoot = 0.0f;
	}
	else
	{
		if (!Bone.bDummy)
		{
			Bone.LengthFromRoot = ModifyBones[Bone.ParentIndex].LengthFromRoot
				+ RefBonePose[Bone.BoneRef.BoneIndex].GetLocation().Size();
		}
		else
		{
			Bone.LengthFromRoot = ModifyBones[Bone.ParentIndex].LengthFromRoot + DummyBoneLength;
		}
		
		TotalBoneLength = FMath::Max(TotalBoneLength, Bone.LengthFromRoot);
	}

	for (int ChildIndex : Bone.ChildIndexs)
	{
		CalcBoneLength(ModifyBones[ChildIndex], RefBonePose);
	}
}

DECLARE_CYCLE_STAT(TEXT("KawaiiPhysics_UpdatePhysicsSetting"), STAT_KawaiiPhysics_UpdatePhysicsSetting, STATGROUP_Anim);

void FAnimNode_KawaiiPhysics::UpdatePhysicsSettingsOfModifyBones()
{
	for (auto& Bone : ModifyBones)
	{
		SCOPE_CYCLE_COUNTER(STAT_KawaiiPhysics_UpdatePhysicsSetting);

		float LengthRate = Bone.LengthFromRoot / TotalBoneLength;

		// Damping
		Bone.PhysicsSettings.Damping = PhysicsSettings.Damping;
		if (TotalBoneLength > 0 && DampingCurve && DampingCurve->GetCurves().Num() > 0)
		{
			Bone.PhysicsSettings.Damping *= DampingCurve->GetFloatValue(LengthRate);
		}
		Bone.PhysicsSettings.Damping = FMath::Clamp<float>(Bone.PhysicsSettings.Damping, 0.0f, 1.0f);

		// WorldLocationDamping
		Bone.PhysicsSettings.WorldDampingLocation = PhysicsSettings.WorldDampingLocation;
		if (TotalBoneLength > 0 && WorldDampingLocationCurve && WorldDampingLocationCurve->GetCurves().Num() > 0)
		{
			Bone.PhysicsSettings.WorldDampingLocation *= WorldDampingLocationCurve->GetFloatValue(LengthRate);
		}
		Bone.PhysicsSettings.WorldDampingLocation = FMath::Clamp<float>(Bone.PhysicsSettings.WorldDampingLocation, 0.0f, 1.0f);

		// WorldRotationDamping
		Bone.PhysicsSettings.WorldDampingRotation = PhysicsSettings.WorldDampingRotation;
		if (TotalBoneLength > 0 && WorldDampingRotationCurve && WorldDampingRotationCurve->GetCurves().Num() > 0)
		{
			Bone.PhysicsSettings.WorldDampingRotation *= WorldDampingRotationCurve->GetFloatValue(LengthRate);
		}
		Bone.PhysicsSettings.WorldDampingRotation = FMath::Clamp<float>(Bone.PhysicsSettings.WorldDampingRotation, 0.0f, 1.0f);

		// Stiffness
		Bone.PhysicsSettings.Stiffness = PhysicsSettings.Stiffness;
		if (TotalBoneLength > 0 && StiffnessCurve && StiffnessCurve->GetCurves().Num() > 0)
		{
			Bone.PhysicsSettings.Stiffness *= StiffnessCurve->GetFloatValue(LengthRate);
		}
		Bone.PhysicsSettings.Stiffness = FMath::Clamp<float>(Bone.PhysicsSettings.Stiffness, 0.0f, 1.0f);

		// Radius
		Bone.PhysicsSettings.Radius = PhysicsSettings.Radius;
		if (TotalBoneLength > 0 && RadiusCurve && RadiusCurve->GetCurves().Num() > 0)
		{
			Bone.PhysicsSettings.Radius *= RadiusCurve->GetFloatValue(LengthRate);
		}
		Bone.PhysicsSettings.Radius = FMath::Max<float>(Bone.PhysicsSettings.Radius, 0.0f);

		// LimitAngle
		Bone.PhysicsSettings.LimitAngle = PhysicsSettings.LimitAngle;
		if (TotalBoneLength > 0 && LimitAngleCurve && LimitAngleCurve->GetCurves().Num() > 0)
		{
			Bone.PhysicsSettings.LimitAngle *= LimitAngleCurve->GetFloatValue(LengthRate);
		}
		Bone.PhysicsSettings.LimitAngle = FMath::Max<float>(Bone.PhysicsSettings.LimitAngle, 0.0f);
	}
}

DECLARE_CYCLE_STAT(TEXT("KawaiiPhysics_UpdateSphericalLimit"), STAT_KawaiiPhysics_UpdateSphericalLimit, STATGROUP_Anim);

void FAnimNode_KawaiiPhysics::UpdateSphericalLimits(TArray<FSphericalLimit>& Limits, FComponentSpacePoseContext& Output, const FBoneContainer& BoneContainer, FTransform& ComponentTransform)
{
	for (auto& Sphere : Limits)
	{
		SCOPE_CYCLE_COUNTER(STAT_KawaiiPhysics_UpdateSphericalLimit);

		if (Sphere.DrivingBone.BoneIndex >= 0)
		{		
			FCompactPoseBoneIndex CompactPoseIndex = Sphere.DrivingBone.GetCompactPoseIndex(BoneContainer);
			FTransform BoneTransform = Output.Pose.GetComponentSpaceTransform(CompactPoseIndex);

			FAnimationRuntime::ConvertCSTransformToBoneSpace(ComponentTransform, Output.Pose, BoneTransform, CompactPoseIndex, BCS_BoneSpace);
			BoneTransform.SetRotation(Sphere.OffsetRotation.Quaternion() * BoneTransform.GetRotation());
			BoneTransform.AddToTranslation(Sphere.OffsetLocation);

			FAnimationRuntime::ConvertBoneSpaceTransformToCS(ComponentTransform, Output.Pose, BoneTransform, CompactPoseIndex, BCS_BoneSpace);
			Sphere.Location = BoneTransform.GetLocation();
			Sphere.Rotation = BoneTransform.GetRotation();
		}
		else
		{
			Sphere.Location = Sphere.OffsetLocation;
		}
	}
}

DECLARE_CYCLE_STAT(TEXT("KawaiiPhysics_UpdateCapsuleLimit"), STAT_KawaiiPhysics_UpdateCapsuleLimit, STATGROUP_Anim);

void FAnimNode_KawaiiPhysics::UpdateCapsuleLimits(TArray<FCapsuleLimit>& Limits, FComponentSpacePoseContext& Output, const FBoneContainer& BoneContainer, FTransform& ComponentTransform)
{
	for (auto& Capsule : Limits)
	{
		SCOPE_CYCLE_COUNTER(STAT_KawaiiPhysics_UpdateCapsuleLimit);

		if (Capsule.DrivingBone.BoneIndex >= 0)
		{			
			FCompactPoseBoneIndex CompactPoseIndex = Capsule.DrivingBone.GetCompactPoseIndex(BoneContainer);
			FTransform BoneTransform = Output.Pose.GetComponentSpaceTransform(CompactPoseIndex);

			FAnimationRuntime::ConvertCSTransformToBoneSpace(ComponentTransform, Output.Pose, BoneTransform, CompactPoseIndex, BCS_BoneSpace);
			BoneTransform.SetRotation(Capsule.OffsetRotation.Quaternion() * BoneTransform.GetRotation());
			BoneTransform.AddToTranslation(Capsule.OffsetLocation);

			FAnimationRuntime::ConvertBoneSpaceTransformToCS(ComponentTransform, Output.Pose, BoneTransform, CompactPoseIndex, BCS_BoneSpace);
			Capsule.Location = BoneTransform.GetLocation();
			Capsule.Rotation = BoneTransform.GetRotation();
		}
		else
		{			
			Capsule.Location = Capsule.OffsetLocation;
			Capsule.Rotation = Capsule.OffsetRotation.Quaternion();
		}
	}
}

DECLARE_CYCLE_STAT(TEXT("KawaiiPhysics_UpdatePlanerLimit"), STAT_KawaiiPhysics_UpdatePlanerLimit, STATGROUP_Anim);

void FAnimNode_KawaiiPhysics::UpdatePlanerLimits(TArray<FPlanarLimit>& Limits, FComponentSpacePoseContext& Output, const FBoneContainer& BoneContainer, FTransform& ComponentTransform)
{
	for (auto& Planar : Limits)
	{
		SCOPE_CYCLE_COUNTER(STAT_KawaiiPhysics_UpdatePlanerLimit);

		if (Planar.DrivingBone.BoneIndex >= 0)
		{
			FCompactPoseBoneIndex CompactPoseIndex = Planar.DrivingBone.GetCompactPoseIndex(BoneContainer);
			FTransform BoneTransform = Output.Pose.GetComponentSpaceTransform(CompactPoseIndex);

			FAnimationRuntime::ConvertCSTransformToBoneSpace(ComponentTransform, Output.Pose, BoneTransform, CompactPoseIndex, BCS_BoneSpace);
			BoneTransform.SetRotation(Planar.OffsetRotation.Quaternion() * BoneTransform.GetRotation());
			BoneTransform.AddToTranslation(Planar.OffsetLocation);

			FAnimationRuntime::ConvertBoneSpaceTransformToCS(ComponentTransform, Output.Pose, BoneTransform, CompactPoseIndex, BCS_BoneSpace);
			Planar.Location = BoneTransform.GetLocation();
			Planar.Rotation = BoneTransform.GetRotation();
			Planar.Rotation.Normalize();
			Planar.Plane = FPlane(Planar.Location, Planar.Rotation.GetUpVector());
		}
		else
		{
			Planar.Location = Planar.OffsetLocation;
			Planar.Rotation = Planar.OffsetRotation.Quaternion();
			Planar.Rotation.Normalize();
			Planar.Plane = FPlane(Planar.Location, Planar.Rotation.GetUpVector());
		}
	}
}

DECLARE_CYCLE_STAT(TEXT("KawaiiPhysics_SimulatemodifyBones"), STAT_KawaiiPhysics_SimulatemodifyBones, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("KawaiiPhysics_SimulatemodifyBone"), STAT_KawaiiPhysics_SimulatemodifyBone, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("KawaiiPhysics_AdjustBone"), STAT_KawaiiPhysics_AdjustBone, STATGROUP_Anim);
DECLARE_CYCLE_STAT(TEXT("KawaiiPhysics_Wind"), STAT_KawaiiPhysics_Wind, STATGROUP_Anim);

void FAnimNode_KawaiiPhysics::SimulateModifyBones(FComponentSpacePoseContext& Output, const FBoneContainer& BoneContainer, FTransform& ComponentTransform)
{
	SCOPE_CYCLE_COUNTER(STAT_KawaiiPhysics_SimulatemodifyBones);

	if (DeltaTime <= 0.0f)
	{
		return;
	}

	// for wind
	FVector WindDirection;
	float WindSpeed;
	float WindMinGust;
	float WindMaxGust;

	const USkeletalMeshComponent* SkelComp = Output.AnimInstanceProxy->GetSkelMeshComponent();
	const UWorld* World = SkelComp ? SkelComp->GetWorld() : nullptr;
	FSceneInterface* Scene = World && World->Scene ? World->Scene : nullptr;
	const float Exponent = TargetFramerate * DeltaTime;

	//transform gravity to component space
	FVector GravityCS = ComponentTransform.InverseTransformVector(Gravity);

	for (int i = 0; i < ModifyBones.Num(); ++i)
	{
		SCOPE_CYCLE_COUNTER(STAT_KawaiiPhysics_SimulatemodifyBone);

		auto& Bone = ModifyBones[i];
		if (Bone.BoneRef.BoneIndex < 0 && !Bone.bDummy)
		{
			continue;
		}

		if (Bone.ParentIndex < 0)
		{
			Bone.PrevLocation = Bone.Location;
			Bone.Location = Bone.PoseLocation;
			continue;
		}

		auto& ParentBone = ModifyBones[Bone.ParentIndex];
		FVector BonePoseLocation = Bone.PoseLocation;
		FVector ParentBonePoseLocation = ParentBone.PoseLocation;

		// Move using Velocity( = movement amount in pre frame ) and Damping
		{
			FVector Velocity = (Bone.Location - Bone.PrevLocation) / DeltaTimeOld;
			Bone.PrevLocation = Bone.Location;
			Velocity *= (1.0f - Bone.PhysicsSettings.Damping);

			// wind
			if (bEnableWind && Scene)
			{
				SCOPE_CYCLE_COUNTER(STAT_KawaiiPhysics_Wind);

				Scene->GetWindParameters_GameThread(ComponentTransform.TransformPosition(Bone.PoseLocation), WindDirection, WindSpeed, WindMinGust, WindMaxGust);
				WindDirection = ComponentTransform.Inverse().TransformVector(WindDirection);
				FVector WindVelocity = WindDirection * WindSpeed * WindScale;

				// TODO:Migrate if there are more good method (Currently copying AnimDynamics implementation)
				WindVelocity *= FMath::FRandRange(0.0f, 2.0f);

				Velocity += WindVelocity * TargetFramerate;
			}
			Bone.Location += Velocity * DeltaTime;
		}

		// Follow Translation
        Bone.Location += SkelCompMoveVector * (1.0f - Bone.PhysicsSettings.WorldDampingLocation);

		// Follow Rotation
		Bone.Location += (SkelCompMoveRotation.RotateVector(Bone.PrevLocation) - Bone.PrevLocation)
			* (1.0f - Bone.PhysicsSettings.WorldDampingRotation);

		// Gravity
		// TODO:Migrate if there are more good method (Currently copying AnimDynamics implementation)
		if (CVarEnableOldPhysicsMethodGrayity.GetValueOnAnyThread() == 0)
		{
			Bone.Location += 0.5 * GravityCS * DeltaTime * DeltaTime;
		}
		else
		{
			Bone.Location += GravityCS * DeltaTime;
		}
		
		// Pull to Pose Location
		FVector BaseLocation = ParentBone.Location + (BonePoseLocation - ParentBonePoseLocation);
		Bone.Location += (BaseLocation - Bone.Location) *
			(1.0f - FMath::Pow(1.0f - Bone.PhysicsSettings.Stiffness, Exponent));

		// Calculate Rotation before adjusting collision
		ParentBone.Rotation = ParentBone.PoseRotation;
		if (i > 0)
		{
			if (ParentBone.BoneRef.BoneIndex >= 0)
			{
				FVector PoseVector = Bone.PoseLocation - ParentBone.PoseLocation;
				FVector SimulateVector = Bone.Location - ParentBone.Location;

				if (PoseVector.GetSafeNormal() != SimulateVector.GetSafeNormal())
				{
					if (BoneForwardAxis == EBoneForwardAxis::X_Negative || BoneForwardAxis == EBoneForwardAxis::Y_Negative || BoneForwardAxis == EBoneForwardAxis::Z_Negative)
					{
						PoseVector *= -1;
						SimulateVector *= -1;
					}

					FQuat SimulateRotation = FQuat::FindBetweenVectors(PoseVector, SimulateVector) * ParentBone.PoseRotation;
					ParentBone.Rotation = SimulateRotation;
				}
			}
		}

		{
			SCOPE_CYCLE_COUNTER(STAT_KawaiiPhysics_AdjustBone);

			// Adjust by each collisions
			AdjustBySphereCollision(SkelComp, ParentBone, Bone, SphericalLimits);
			AdjustBySphereCollision(SkelComp, ParentBone, Bone, SphericalLimitsData);
			AdjustByCapsuleCollision(SkelComp, ParentBone, Bone, CapsuleLimits);
			AdjustByCapsuleCollision(SkelComp, ParentBone, Bone, CapsuleLimitsData);
			AdjustByPlanerCollision(SkelComp, ParentBone, Bone, PlanarLimits);
			AdjustByPlanerCollision(SkelComp, ParentBone, Bone, PlanarLimitsData);
			AdjustByPhysicsAssetCollision(SkelComp, ParentBone, Bone);

			// Adjust by angle limit
			AdjustByAngleLimit(Output, BoneContainer, ComponentTransform, Bone, ParentBone);

			// Adjust by Planar Constraint
			AdjustByPlanarConstraint(Bone, ParentBone);
		}

		// Restore Bone Length
		float BoneLength = (BonePoseLocation - ParentBonePoseLocation).Size();
		Bone.Location = (Bone.Location - ParentBone.Location).GetSafeNormal() * BoneLength + ParentBone.Location;
	}
	DeltaTimeOld = DeltaTime;
}

void FAnimNode_KawaiiPhysics::AdjustBySphereCollision(const USkeletalMeshComponent* SkeletalMeshComp, FKawaiiPhysicsModifyBone& ParentBone, FKawaiiPhysicsModifyBone& Bone, TArray<FSphericalLimit>& Limits)
{
	if (!bUsePhysicsAssetAsShapes)
	{
		for (auto& Sphere : Limits)
		{
			if (Sphere.Radius <= 0.0f)
			{
				continue;
			}

			float LimitDistance = Bone.PhysicsSettings.Radius + Sphere.Radius;
			if (Sphere.LimitType == ESphericalLimitType::Outer)
			{
				if ((Bone.Location - Sphere.Location).SizeSquared() > LimitDistance * LimitDistance)
				{
					continue;
				}
				else
				{
					Bone.Location += (LimitDistance - (Bone.Location - Sphere.Location).Size())
						* (Bone.Location - Sphere.Location).GetSafeNormal();
				}
			}
			else
			{
				if ((Bone.Location - Sphere.Location).SizeSquared() < LimitDistance * LimitDistance)
				{
					continue;
				}
				else
				{
					if (CVarEnableOldPhysicsMethodSphereLimit.GetValueOnAnyThread() == 0)
					{
						Bone.Location = Sphere.Location + (Sphere.Radius - Bone.PhysicsSettings.Radius) * (Bone.Location - Sphere.Location).GetSafeNormal();
					}
					else
					{
						Bone.Location = Sphere.Location + Sphere.Radius * (Bone.Location - Sphere.Location).GetSafeNormal();
					}
				}
			}
		}
	}
	else
	{
		if (Bone.PhysicsBodySetup != nullptr)
		{
			check(Bone.BoneRef.BoneIndex != INDEX_NONE);
			float Scale = SkeletalMeshComp->GetBoneTransform(Bone.BoneRef.BoneIndex, FTransform::Identity).GetScale3D().GetAbsMax(); // コンポーネント座標でのTransformのスケール
			FVector VectorScale(Scale);

			FTransform BoneTM = FTransform(Bone.Rotation, Bone.Location);

			FKAggregateGeom* AggGeom = &Bone.PhysicsBodySetup->AggGeom;

			for (int32 i = 0; i <AggGeom->SphereElems.Num(); ++i)
			{
				const FKSphereElem& SphereShape = AggGeom->SphereElems[i];

				// FAnimNode_KawaiiPhysics::AdjustBySphereCollision()のESphericalLimitType::Outerのケースを参考にしている
				if (SphereShape.Radius <= 0.0f)
				{
					continue;
				}

				FTransform ElemTM = SphereShape.GetTransform();
				ElemTM.ScaleTranslation(VectorScale);
				ElemTM *= BoneTM;

				FVector SphereShapeLocation = ElemTM.GetLocation();

				for (auto& Sphere : Limits)
				{
					if (Sphere.Radius <= 0.0f)
					{
						continue;
					}

					FVector PushOutVector = FVector::ZeroVector;

					float LimitDistance = SphereShape.Radius + Sphere.Radius;
					if (Sphere.LimitType == ESphericalLimitType::Outer)
					{
						if ((SphereShapeLocation - Sphere.Location).SizeSquared() > LimitDistance * LimitDistance)
						{
							continue;
						}
						else
						{
							PushOutVector = (LimitDistance - (SphereShapeLocation - Sphere.Location).Size())
								* (SphereShapeLocation - Sphere.Location).GetSafeNormal();
						}
					}
					else
					{
						if ((SphereShapeLocation - Sphere.Location).SizeSquared() < LimitDistance * LimitDistance)
						{
							continue;
						}
						else
						{
							if (CVarEnableOldPhysicsMethodSphereLimit.GetValueOnAnyThread() == 0)
							{
								PushOutVector = Sphere.Location + (Sphere.Radius - SphereShape.Radius) * (SphereShapeLocation - Sphere.Location).GetSafeNormal() - SphereShapeLocation;
							}
							else
							{
								PushOutVector = Sphere.Location + Sphere.Radius * (SphereShapeLocation - Sphere.Location).GetSafeNormal() - SphereShapeLocation;
							}
						}
					}

					SphereShapeLocation += PushOutVector;
					// SphereShapeが押し出されたベクトルだけボーンも移動させるという単純な計算
					Bone.Location += PushOutVector;
				}

				// シェイプが骨に複数くっついている場合、すべてを満足する押し出し位置は1イテレーションでは計算できないので、ひとつ押し出しを計算したらそこで打ち切る
				break;
			}

			for (int32 i = 0; i <AggGeom->BoxElems.Num(); ++i)
			{
				FTransform ElemTM = AggGeom->BoxElems[i].GetTransform();
				ElemTM.ScaleTranslation(VectorScale);
				ElemTM *= BoneTM;
				// TODO:
			}

			for (int32 i = 0; i <AggGeom->ConvexElems.Num(); ++i)
			{
				FTransform ElemTM = AggGeom->ConvexElems[i].GetTransform();
				ElemTM.ScaleTranslation(VectorScale);
				ElemTM *= BoneTM;
				// TODO:
			}

			for (int32 i = 0; i <AggGeom->TaperedCapsuleElems.Num(); ++i)
			{
				FTransform ElemTM = AggGeom->TaperedCapsuleElems[i].GetTransform();
				ElemTM.ScaleTranslation(VectorScale);
				ElemTM *= BoneTM;
				// TODO:
			}
		}

		if (ParentBone.PhysicsBodySetup != nullptr)
		{
			// Capsuleの場合はParentBoneがもつカプセルのコリジョン判定によってParentBoneとBoneの位置を押し出す
			float ParentScale = SkeletalMeshComp->GetBoneTransform(ParentBone.BoneRef.BoneIndex, FTransform::Identity).GetScale3D().GetAbsMax(); // コンポーネント座標でのTransformのスケール
			FVector ParentVectorScale(ParentScale);
			FTransform ParentBoneTM = FTransform(ParentBone.Rotation, ParentBone.Location);
			FKAggregateGeom* ParentAggGeom = &ParentBone.PhysicsBodySetup->AggGeom;

			for (int32 i = 0; i <ParentAggGeom->SphylElems.Num(); ++i)
			{
				const FKSphylElem& Capsule = ParentAggGeom->SphylElems[i];

				if (Capsule.Radius <= 0 || Capsule.Length <= 0)
				{
					continue;
				}

				FTransform ElemTM = Capsule.GetTransform();
				ElemTM.ScaleTranslation(ParentVectorScale);
				ElemTM *= ParentBoneTM;

				FVector CapsuleShapeLocation = ElemTM.GetLocation();

				for (auto& Sphere : Limits)
				{
					if (Sphere.Radius <= 0.0f)
					{
						continue;
					}

					FVector PushOutVector = FVector::ZeroVector;

					FVector StartPoint = CapsuleShapeLocation + ElemTM.GetRotation().GetAxisZ() * Capsule.Length * 0.5f;
					FVector EndPoint = CapsuleShapeLocation + ElemTM.GetRotation().GetAxisZ() * Capsule.Length * -0.5f;

					if (Sphere.LimitType == ESphericalLimitType::Outer)
					{
						float LimitDistance = Capsule.Radius + Sphere.Radius;
						float DistSquared = FMath::PointDistToSegmentSquared(Sphere.Location, StartPoint, EndPoint);
						if (DistSquared < LimitDistance * LimitDistance)
						{
							FVector ClosestPoint = FMath::ClosestPointOnSegment(Sphere.Location, StartPoint, EndPoint);
							PushOutVector = (ClosestPoint - Sphere.Location).GetSafeNormal() * LimitDistance - (ClosestPoint - Sphere.Location);
						}
					}
					else // Inner
					{
						float LimitDistance = Sphere.Radius - Capsule.Radius;
						// 以下の押し出し処理だと、Start側の半球がスフィア内に入ってもスフィアのRotation次第でEnd側の半球がスフィアから飛び出すことは考えうるが
						// Rotationまで変えないので、そうなっても放置する
						if (Sphere.Radius * 2.0f > Capsule.Radius * 2.0f + Capsule.Length) // この条件はカプセルがスフィアに入る必要条件。入らない場合は押し出ししない
						{
							float StartPointDistSquared = (StartPoint - Sphere.Location).SizeSquared();
							float EndPointDistSquared = (EndPoint - Sphere.Location).SizeSquared();
							if (StartPointDistSquared >= EndPointDistSquared)
							{
								if (StartPointDistSquared > Sphere.Radius - Capsule.Radius)
								{
									PushOutVector = ((StartPoint - Sphere.Location).GetSafeNormal() * LimitDistance + Sphere.Location) - StartPoint;
								}
							}
							else
							{
								if (EndPointDistSquared > Sphere.Radius - Capsule.Radius)
								{
									PushOutVector = ((EndPoint - Sphere.Location).GetSafeNormal() * LimitDistance + Sphere.Location) - EndPoint;
								}
							}
						}
					}

					CapsuleShapeLocation += PushOutVector;
					// CapsuleShapeが押し出されたベクトルだけボーンも移動させるという単純な計算
					if (ParentBone.ParentIndex >= 0)
					{
						ParentBone.Location += PushOutVector;
					}
					Bone.Location += PushOutVector;
				}

				// シェイプが骨に複数くっついている場合、すべてを満足する押し出し位置は1イテレーションでは計算できないので、ひとつ押し出しを計算したらそこで打ち切る
				break;
			}
		}
	}
}

void FAnimNode_KawaiiPhysics::AdjustByCapsuleCollision(const USkeletalMeshComponent* SkeletalMeshComp, FKawaiiPhysicsModifyBone& ParentBone, FKawaiiPhysicsModifyBone& Bone, TArray<FCapsuleLimit>& Limits)
{
	if (!bUsePhysicsAssetAsShapes)
	{
		for (auto& Capsule : Limits)
		{
			if (Capsule.Radius <= 0 || Capsule.Length <= 0)
			{
				continue;
			}

			FVector StartPoint = Capsule.Location + Capsule.Rotation.GetAxisZ() * Capsule.Length * 0.5f;
			FVector EndPoint = Capsule.Location + Capsule.Rotation.GetAxisZ() * Capsule.Length * -0.5f;
			float DistSquared = FMath::PointDistToSegmentSquared(Bone.Location, StartPoint, EndPoint);

			float LimitDistance = Bone.PhysicsSettings.Radius + Capsule.Radius;
			if (DistSquared < LimitDistance * LimitDistance)
			{
				FVector ClosestPoint = FMath::ClosestPointOnSegment(Bone.Location, StartPoint, EndPoint);
				Bone.Location = ClosestPoint + (Bone.Location - ClosestPoint).GetSafeNormal() * LimitDistance;
			}
		}
	}
	else
	{
		if (Bone.PhysicsBodySetup != nullptr)
		{
			check(Bone.BoneRef.BoneIndex != INDEX_NONE);
			float Scale = SkeletalMeshComp->GetBoneTransform(Bone.BoneRef.BoneIndex, FTransform::Identity).GetScale3D().GetAbsMax(); // コンポーネント座標でのTransformのスケール
			FVector VectorScale(Scale);

			FTransform BoneTM = FTransform(Bone.Rotation, Bone.Location);

			FKAggregateGeom* AggGeom = &Bone.PhysicsBodySetup->AggGeom;

			for (int32 i = 0; i <AggGeom->SphereElems.Num(); ++i)
			{
				const FKSphereElem& SphereShape = AggGeom->SphereElems[i];

				// FAnimNode_KawaiiPhysics::AdjustBySphereCollision()のESphericalLimitType::Outerのケースを参考にしている
				if (SphereShape.Radius <= 0.0f)
				{
					continue;
				}

				FTransform ElemTM = SphereShape.GetTransform();
				ElemTM.ScaleTranslation(VectorScale);
				ElemTM *= BoneTM;

				FVector SphereShapeLocation = ElemTM.GetLocation();

				for (auto& Capsule : Limits)
				{
					if (Capsule.Radius <= 0 || Capsule.Length <= 0)
					{
						continue;
					}

					FVector PushOutVector = FVector::ZeroVector;

					FVector StartPoint = Capsule.Location + Capsule.Rotation.GetAxisZ() * Capsule.Length * 0.5f;
					FVector EndPoint = Capsule.Location + Capsule.Rotation.GetAxisZ() * Capsule.Length * -0.5f;
					float DistSquared = FMath::PointDistToSegmentSquared(SphereShapeLocation, StartPoint, EndPoint);

					float LimitDistance = SphereShape.Radius + Capsule.Radius;
					if (DistSquared < LimitDistance * LimitDistance)
					{
						FVector ClosestPoint = FMath::ClosestPointOnSegment(SphereShapeLocation, StartPoint, EndPoint);
						PushOutVector = ClosestPoint + (SphereShapeLocation - ClosestPoint).GetSafeNormal() * LimitDistance - SphereShapeLocation;
					}

					SphereShapeLocation += PushOutVector;
					// SphereShapeが押し出されたベクトルだけボーンも移動させるという単純な計算
					Bone.Location += PushOutVector;
				}

				// シェイプが骨に複数くっついている場合、すべてを満足する押し出し位置は1イテレーションでは計算できないので、ひとつ押し出しを計算したらそこで打ち切る
				break;
			}

			for (int32 i = 0; i <AggGeom->BoxElems.Num(); ++i)
			{
				FTransform ElemTM = AggGeom->BoxElems[i].GetTransform();
				ElemTM.ScaleTranslation(VectorScale);
				ElemTM *= BoneTM;
				// TODO:
			}

			for (int32 i = 0; i <AggGeom->ConvexElems.Num(); ++i)
			{
				FTransform ElemTM = AggGeom->ConvexElems[i].GetTransform();
				ElemTM.ScaleTranslation(VectorScale);
				ElemTM *= BoneTM;
				// TODO:
			}

			for (int32 i = 0; i <AggGeom->TaperedCapsuleElems.Num(); ++i)
			{
				FTransform ElemTM = AggGeom->TaperedCapsuleElems[i].GetTransform();
				ElemTM.ScaleTranslation(VectorScale);
				ElemTM *= BoneTM;
				// TODO:
			}
		}

		if (ParentBone.PhysicsBodySetup != nullptr)
		{
			// Capsuleの場合はParentBoneがもつカプセルのコリジョン判定によってParentBoneとBoneの位置を押し出す
			float ParentShapeScale = SkeletalMeshComp->GetBoneTransform(ParentBone.BoneRef.BoneIndex, FTransform::Identity).GetScale3D().GetAbsMax(); // コンポーネント座標でのTransformのスケール
			FVector ParentShapeVectorScale(ParentShapeScale);
			FTransform ParentShapeBoneTM = FTransform(ParentBone.Rotation, ParentBone.Location);
			FKAggregateGeom* ParentShapeAggGeom = &ParentBone.PhysicsBodySetup->AggGeom;

			for (int32 i = 0; i <ParentShapeAggGeom->SphylElems.Num(); ++i)
			{
				const FKSphylElem& CapsuleShape = ParentShapeAggGeom->SphylElems[i];

				if (CapsuleShape.Radius <= 0 || CapsuleShape.Length <= 0)
				{
					continue;
				}

				FTransform ElemTM = CapsuleShape.GetTransform();
				ElemTM.ScaleTranslation(ParentShapeVectorScale);
				ElemTM *= ParentShapeBoneTM;

				FVector CapsuleShapeLocation = ElemTM.GetLocation();

				for (auto& Capsule : Limits)
				{
					if (Capsule.Radius <= 0 || Capsule.Length <= 0)
					{
						continue;
					}

					FVector PushOutVector = FVector::ZeroVector;

					FVector CapsuleShapeStartPoint = CapsuleShapeLocation + ElemTM.GetRotation().GetAxisZ() * CapsuleShape.Length * 0.5f;
					FVector CapsuleShapeEndPoint = CapsuleShapeLocation + ElemTM.GetRotation().GetAxisZ() * CapsuleShape.Length * -0.5f;

					FVector CapsuleStartPoint = Capsule.Location + Capsule.Rotation.GetAxisZ() * Capsule.Length * 0.5f;
					FVector CapsuleEndPoint = Capsule.Location + Capsule.Rotation.GetAxisZ() * Capsule.Length * -0.5f;

					FVector CapsuleShapeClosestPoint;
					FVector CapsuleClosestPoint;
					FMath::SegmentDistToSegmentSafe(CapsuleShapeStartPoint, CapsuleShapeEndPoint, CapsuleStartPoint, CapsuleEndPoint, CapsuleShapeClosestPoint, CapsuleClosestPoint);
					float DistSquared = (CapsuleShapeClosestPoint - CapsuleClosestPoint).SizeSquared();

					float LimitDistance = CapsuleShape.Radius + Capsule.Radius;
					if (DistSquared < LimitDistance * LimitDistance)
					{
						PushOutVector = CapsuleClosestPoint + (CapsuleShapeClosestPoint - CapsuleClosestPoint).GetSafeNormal() * LimitDistance - CapsuleShapeClosestPoint;
					}

					CapsuleShapeLocation += PushOutVector;
					// CapsuleShapeが押し出されたベクトルだけボーンも移動させるという単純な計算
					if (ParentBone.ParentIndex >= 0)
					{
						ParentBone.Location += PushOutVector;
					}
					Bone.Location += PushOutVector;
				}

				// シェイプが骨に複数くっついている場合、すべてを満足する押し出し位置は1イテレーションでは計算できないので、ひとつ押し出しを計算したらそこで打ち切る
				break;
			}
		}
	}
}

void FAnimNode_KawaiiPhysics::AdjustByPlanerCollision(const USkeletalMeshComponent* SkeletalMeshComp, FKawaiiPhysicsModifyBone& ParentBone, FKawaiiPhysicsModifyBone& Bone, TArray<FPlanarLimit>& Limits)
{
	if (!bUsePhysicsAssetAsShapes)
	{
		for (auto& Planar : Limits)
		{
			FVector PointOnPlane = FVector::PointPlaneProject(Bone.Location, Planar.Plane);
			float DistSquared = (Bone.Location - PointOnPlane).SizeSquared();

			FVector IntersectionPoint;
			if (DistSquared < Bone.PhysicsSettings.Radius * Bone.PhysicsSettings.Radius ||
				FMath::SegmentPlaneIntersection(Bone.Location, Bone.PrevLocation, Planar.Plane, IntersectionPoint))
			{
				Bone.Location = PointOnPlane + Planar.Rotation.GetUpVector() * Bone.PhysicsSettings.Radius;
				continue;
			}
		}
	}
	else
	{
		if (Bone.PhysicsBodySetup != nullptr)
		{
			check(Bone.BoneRef.BoneIndex != INDEX_NONE);
			float Scale = SkeletalMeshComp->GetBoneTransform(Bone.BoneRef.BoneIndex, FTransform::Identity).GetScale3D().GetAbsMax(); // コンポーネント座標でのTransformのスケール
			FVector VectorScale(Scale);

			FTransform BoneTM = FTransform(Bone.Rotation, Bone.Location);

			FKAggregateGeom* AggGeom = &Bone.PhysicsBodySetup->AggGeom;

			for (int32 i = 0; i <AggGeom->SphereElems.Num(); ++i)
			{
				const FKSphereElem& SphereShape = AggGeom->SphereElems[i];

				// FAnimNode_KawaiiPhysics::AdjustBySphereCollision()のESphericalLimitType::Outerのケースを参考にしている
				if (SphereShape.Radius <= 0.0f)
				{
					continue;
				}

				FTransform ElemTM = SphereShape.GetTransform();
				ElemTM.ScaleTranslation(VectorScale);
				ElemTM *= BoneTM;

				FVector SphereShapeLocation = ElemTM.GetLocation();

				for (auto& Planar : Limits)
				{
					FVector PushOutVector = FVector::ZeroVector;

					FVector PointOnPlane = FVector::PointPlaneProject(SphereShapeLocation, Planar.Plane);
					float DistSquared = (SphereShapeLocation - PointOnPlane).SizeSquared();

					FVector IntersectionPoint;
					if (DistSquared < SphereShape.Radius * SphereShape.Radius ||
						FMath::SegmentPlaneIntersection(SphereShapeLocation, Bone.PrevLocation, Planar.Plane, IntersectionPoint)) // TODO:貫通判定だが、スフィアシェイプの前フレームの位置は記録してないのでとりあえずボーンの前フレームの位置を使っておく
					{
						PushOutVector = PointOnPlane + Planar.Rotation.GetUpVector() * SphereShape.Radius - SphereShapeLocation;
					}

					SphereShapeLocation += PushOutVector;
					// SphereShapeが押し出されたベクトルだけボーンも移動させるという単純な計算
					Bone.Location += PushOutVector;
				}

				// シェイプが骨に複数くっついている場合、すべてを満足する押し出し位置は1イテレーションでは計算できないので、ひとつ押し出しを計算したらそこで打ち切る
				break;
			}

			for (int32 i = 0; i <AggGeom->BoxElems.Num(); ++i)
			{
				FTransform ElemTM = AggGeom->BoxElems[i].GetTransform();
				ElemTM.ScaleTranslation(VectorScale);
				ElemTM *= BoneTM;
				// TODO:
			}

			for (int32 i = 0; i <AggGeom->ConvexElems.Num(); ++i)
			{
				FTransform ElemTM = AggGeom->ConvexElems[i].GetTransform();
				ElemTM.ScaleTranslation(VectorScale);
				ElemTM *= BoneTM;
				// TODO:
			}

			for (int32 i = 0; i <AggGeom->TaperedCapsuleElems.Num(); ++i)
			{
				FTransform ElemTM = AggGeom->TaperedCapsuleElems[i].GetTransform();
				ElemTM.ScaleTranslation(VectorScale);
				ElemTM *= BoneTM;
				// TODO:
			}
		}

		if (ParentBone.PhysicsBodySetup != nullptr)
		{
			// Capsuleの場合はParentBoneがもつカプセルのコリジョン判定によってParentBoneとBoneの位置を押し出す
			float ParentShapeScale = SkeletalMeshComp->GetBoneTransform(ParentBone.BoneRef.BoneIndex, FTransform::Identity).GetScale3D().GetAbsMax(); // コンポーネント座標でのTransformのスケール
			FVector ParentShapeVectorScale(ParentShapeScale);
			FTransform ParentShapeBoneTM = FTransform(ParentBone.Rotation, ParentBone.Location);
			FKAggregateGeom* ParentShapeAggGeom = &ParentBone.PhysicsBodySetup->AggGeom;

			for (int32 i = 0; i <ParentShapeAggGeom->SphylElems.Num(); ++i)
			{
				const FKSphylElem& CapsuleShape = ParentShapeAggGeom->SphylElems[i];

				if (CapsuleShape.Radius <= 0 || CapsuleShape.Length <= 0)
				{
					continue;
				}

				FTransform ElemTM = CapsuleShape.GetTransform();
				ElemTM.ScaleTranslation(ParentShapeVectorScale);
				ElemTM *= ParentShapeBoneTM;

				FVector CapsuleShapeLocation = ElemTM.GetLocation();

				for (auto& Planar : Limits)
				{
					FVector PushOutVector = FVector::ZeroVector;

					FVector CapsuleShapeStartPoint = CapsuleShapeLocation + ElemTM.GetRotation().GetAxisZ() * CapsuleShape.Length * 0.5f;
					FVector CapsuleShapeEndPoint = CapsuleShapeLocation + ElemTM.GetRotation().GetAxisZ() * CapsuleShape.Length * -0.5f;

					FVector StartPointOnPlane = FVector::PointPlaneProject(CapsuleShapeStartPoint, Planar.Plane);
					FVector EndPointOnPlane = FVector::PointPlaneProject(CapsuleShapeEndPoint, Planar.Plane);

					// スフィアのときと違って速度は考慮せず問答無用に押し出す。StartとEndでより潜っている方を押し出す。
					float StartDotProduct = FVector::DotProduct(CapsuleShapeStartPoint - StartPointOnPlane, Planar.Rotation.GetUpVector());
					float EndDotProduct = FVector::DotProduct(CapsuleShapeEndPoint - EndPointOnPlane, Planar.Rotation.GetUpVector());
					if (StartDotProduct < CapsuleShape.Radius || EndDotProduct < CapsuleShape.Radius)
					{
						if (StartDotProduct < EndDotProduct)
						{
							PushOutVector = StartPointOnPlane + (StartPointOnPlane - CapsuleShapeStartPoint).GetSafeNormal() * CapsuleShape.Radius - CapsuleShapeStartPoint;
						}
						else
						{
							PushOutVector = EndPointOnPlane + (EndPointOnPlane - CapsuleShapeEndPoint).GetSafeNormal() * CapsuleShape.Radius - CapsuleShapeEndPoint;
						}
					}

					CapsuleShapeLocation += PushOutVector;
					// CapsuleShapeが押し出されたベクトルだけボーンも移動させるという単純な計算
					if (ParentBone.ParentIndex >= 0)
					{
						ParentBone.Location += PushOutVector;
					}
					Bone.Location += PushOutVector;
				}

				// シェイプが骨に複数くっついている場合、すべてを満足する押し出し位置は1イテレーションでは計算できないので、ひとつ押し出しを計算したらそこで打ち切る
				break;
			}
		}
	}
}

void FAnimNode_KawaiiPhysics::AdjustByPhysicsAssetCollision(const USkeletalMeshComponent* SkeletalMeshComp, FKawaiiPhysicsModifyBone& ParentBone, FKawaiiPhysicsModifyBone& Bone)
{
	if (!bUsePhysicsAssetAsLimits || UsePhysicsAssetAsLimits == nullptr)
	{
		return;
	}

	if (!bUsePhysicsAssetAsShapes)
	{
		for (int32 i = 0; i <UsePhysicsAssetAsLimits->SkeletalBodySetups.Num(); ++i)
		{
			if (!ensure(UsePhysicsAssetAsLimits->SkeletalBodySetups[i]))
			{
				continue;
			}
			int32 BoneIndex = SkeletalMeshComp->GetBoneIndex(UsePhysicsAssetAsLimits->SkeletalBodySetups[i]->BoneName);

			if (BoneIndex != INDEX_NONE)
			{
				FTransform BoneTM = SkeletalMeshComp->GetBoneTransform(BoneIndex, FTransform::Identity); // コンポーネント座標でのTransform
				float Scale = BoneTM.GetScale3D().GetAbsMax();
				FVector VectorScale(Scale);
				BoneTM.RemoveScaling();

				FKAggregateGeom* AggGeom = &UsePhysicsAssetAsLimits->SkeletalBodySetups[i]->AggGeom;

				for (int32 j = 0; j <AggGeom->SphereElems.Num(); ++j)
				{
					const FKSphereElem& Sphere = AggGeom->SphereElems[j];

					// FAnimNode_KawaiiPhysics::AdjustBySphereCollision()のESphericalLimitType::Outerのケースを参考にしている
					if (Sphere.Radius <= 0.0f)
					{
						continue;
					}

					FTransform ElemTM = Sphere.GetTransform();
					ElemTM.ScaleTranslation(VectorScale);
					ElemTM *= BoneTM;

					const FVector& SphereLocation = ElemTM.GetLocation();
					float LimitDistance = Bone.PhysicsSettings.Radius + Sphere.Radius;
					if ((Bone.Location - SphereLocation).SizeSquared() > LimitDistance * LimitDistance)
					{
						continue;
					}
					else
					{
						Bone.Location += (LimitDistance - (Bone.Location - SphereLocation).Size())
							* (Bone.Location - SphereLocation).GetSafeNormal();
					}
				}

				for (int32 j = 0; j <AggGeom->BoxElems.Num(); ++j)
				{
					FTransform ElemTM = AggGeom->BoxElems[j].GetTransform();
					ElemTM.ScaleTranslation(VectorScale);
					ElemTM *= BoneTM;
					// TODO:
				}

				for (int32 j = 0; j <AggGeom->SphylElems.Num(); ++j)
				{
					const FKSphylElem& Capsule = AggGeom->SphylElems[j];

					if (Capsule.Radius <= 0 || Capsule.Length <= 0)
					{
						continue;
					}

					FTransform ElemTM = Capsule.GetTransform();
					ElemTM.ScaleTranslation(VectorScale);
					ElemTM *= BoneTM;

					const FVector& CapsuleLocation = ElemTM.GetLocation();
					FVector StartPoint = CapsuleLocation + ElemTM.GetUnitAxis(EAxis::Type::Z) * Capsule.Length * 0.5f;
					FVector EndPoint = CapsuleLocation + ElemTM.GetUnitAxis(EAxis::Type::Z) * Capsule.Length * -0.5f;
					float DistSquared = FMath::PointDistToSegmentSquared(Bone.Location, StartPoint, EndPoint);

					float LimitDistance = Bone.PhysicsSettings.Radius + Capsule.Radius;
					if (DistSquared < LimitDistance * LimitDistance)
					{
						FVector ClosestPoint = FMath::ClosestPointOnSegment(Bone.Location, StartPoint, EndPoint);
						Bone.Location = ClosestPoint + (Bone.Location - ClosestPoint).GetSafeNormal() * LimitDistance;
					}
				}

				for (int32 j = 0; j <AggGeom->ConvexElems.Num(); ++j)
				{
					FTransform ElemTM = AggGeom->ConvexElems[j].GetTransform();
					ElemTM.ScaleTranslation(VectorScale);
					ElemTM *= BoneTM;
					// TODO:
				}

				for (int32 j = 0; j <AggGeom->TaperedCapsuleElems.Num(); ++j)
				{
					FTransform ElemTM = AggGeom->TaperedCapsuleElems[j].GetTransform();
					ElemTM.ScaleTranslation(VectorScale);
					ElemTM *= BoneTM;
					// TODO:
				}
			}
		}
	}
	else
	{
		if (Bone.PhysicsBodySetup != nullptr)
		{
			check(Bone.BoneRef.BoneIndex != INDEX_NONE);
			float ShapeScale = SkeletalMeshComp->GetBoneTransform(Bone.BoneRef.BoneIndex, FTransform::Identity).GetScale3D().GetAbsMax(); // コンポーネント座標でのTransformのスケール
			FVector ShapeVectorScale(ShapeScale);

			FTransform ShapeBoneTM = FTransform(Bone.Rotation, Bone.Location);

			FKAggregateGeom* ShapeAggGeom = &Bone.PhysicsBodySetup->AggGeom;

			for (int32 i = 0; i <ShapeAggGeom->SphereElems.Num(); ++i)
			{
				const FKSphereElem& SphereShape = ShapeAggGeom->SphereElems[i];

				// FAnimNode_KawaiiPhysics::AdjustBySphereCollision()のESphericalLimitType::Outerのケースを参考にしている
				if (SphereShape.Radius <= 0.0f)
				{
					continue;
				}

				FTransform SphereShapeElemTM = SphereShape.GetTransform();
				SphereShapeElemTM.ScaleTranslation(ShapeVectorScale);
				SphereShapeElemTM *= ShapeBoneTM;

				FVector SphereShapeLocation = SphereShapeElemTM.GetLocation();

				for (int32 j = 0; j <UsePhysicsAssetAsLimits->SkeletalBodySetups.Num(); ++j)
				{
					if (!ensure(UsePhysicsAssetAsLimits->SkeletalBodySetups[j]))
					{
						continue;
					}
					int32 BoneIndex = SkeletalMeshComp->GetBoneIndex(UsePhysicsAssetAsLimits->SkeletalBodySetups[j]->BoneName);

					if (BoneIndex != INDEX_NONE)
					{
						// コリジョンがついてるボーンがシミュレーション対象で押し出しでボーンが動いてコリジョンが動くといたちごっこになるので
						// このフレームでの押し出しはコリジョンの位置に反映させない
						FTransform BoneTM = SkeletalMeshComp->GetBoneTransform(BoneIndex, FTransform::Identity); // コンポーネント座標でのTransform
						float Scale = BoneTM.GetScale3D().GetAbsMax();
						FVector VectorScale(Scale);
						BoneTM.RemoveScaling();

						FKAggregateGeom* AggGeom = &UsePhysicsAssetAsLimits->SkeletalBodySetups[j]->AggGeom;

						for (int32 k = 0; k <AggGeom->SphereElems.Num(); ++k)
						{
							const FKSphereElem& Sphere = AggGeom->SphereElems[k];

							// FAnimNode_KawaiiPhysics::AdjustBySphereCollision()のESphericalLimitType::Outerのケースを参考にしている
							if (Sphere.Radius <= 0.0f)
							{
								continue;
							}

							FTransform ElemTM = Sphere.GetTransform();
							ElemTM.ScaleTranslation(VectorScale);
							ElemTM *= BoneTM;
							const FVector& SphereLocation = ElemTM.GetLocation();

							FVector PushOutVector = FVector::ZeroVector;

							float LimitDistance = SphereShape.Radius + Sphere.Radius;
							if ((SphereShapeLocation - SphereLocation).SizeSquared() > LimitDistance * LimitDistance)
							{
								continue;
							}
							else
							{
								PushOutVector = (LimitDistance - (SphereShapeLocation - SphereLocation).Size())
									* (SphereShapeLocation - SphereLocation).GetSafeNormal();
							}

							SphereShapeLocation += PushOutVector;
							// SphereShapeが押し出されたベクトルだけボーンも移動させるという単純な計算
							// TODO:SphereShapeが骨に対してひとつだけならまだいいが、複数になってくると問題も大きい
							Bone.Location += PushOutVector;
						}

						for (int32 k = 0; k <AggGeom->BoxElems.Num(); ++k)
						{
							FTransform ElemTM = AggGeom->BoxElems[k].GetTransform();
							ElemTM.ScaleTranslation(VectorScale);
							ElemTM *= BoneTM;
							// TODO:
						}

						for (int32 k = 0; k <AggGeom->SphylElems.Num(); ++k)
						{
							const FKSphylElem& Capsule = AggGeom->SphylElems[k];

							if (Capsule.Radius <= 0 || Capsule.Length <= 0)
							{
								continue;
							}

							FTransform ElemTM = Capsule.GetTransform();
							ElemTM.ScaleTranslation(VectorScale);
							ElemTM *= BoneTM;

							FVector PushOutVector = FVector::ZeroVector;

							const FVector& CapsuleLocation = ElemTM.GetLocation();
							FVector StartPoint = CapsuleLocation + ElemTM.GetUnitAxis(EAxis::Type::Z) * Capsule.Length * 0.5f;
							FVector EndPoint = CapsuleLocation + ElemTM.GetUnitAxis(EAxis::Type::Z) * Capsule.Length * -0.5f;
							float DistSquared = FMath::PointDistToSegmentSquared(SphereShapeLocation, StartPoint, EndPoint);

							float LimitDistance = SphereShape.Radius + Capsule.Radius;
							if (DistSquared < LimitDistance * LimitDistance)
							{
								FVector ClosestPoint = FMath::ClosestPointOnSegment(SphereShapeLocation, StartPoint, EndPoint);
								PushOutVector = ClosestPoint + (SphereShapeLocation - ClosestPoint).GetSafeNormal() * LimitDistance - SphereShapeLocation;
							}

							SphereShapeLocation += PushOutVector;
							// SphereShapeが押し出されたベクトルだけボーンも移動させるという単純な計算
							Bone.Location += PushOutVector;
						}

						for (int32 k = 0; k <AggGeom->ConvexElems.Num(); ++k)
						{
							FTransform ElemTM = AggGeom->ConvexElems[k].GetTransform();
							ElemTM.ScaleTranslation(VectorScale);
							ElemTM *= BoneTM;
							// TODO:
						}

						for (int32 k = 0; k <AggGeom->TaperedCapsuleElems.Num(); ++k)
						{
							FTransform ElemTM = AggGeom->TaperedCapsuleElems[k].GetTransform();
							ElemTM.ScaleTranslation(VectorScale);
							ElemTM *= BoneTM;
							// TODO:
						}
					}
				}

				// シェイプが骨に複数くっついている場合、すべてを満足する押し出し位置は1イテレーションでは計算できないので、ひとつ押し出しを計算したらそこで打ち切る
				break;
			}

			for (int32 i = 0; i <ShapeAggGeom->BoxElems.Num(); ++i)
			{
				FTransform SphereShapeElemTM = ShapeAggGeom->BoxElems[i].GetTransform();
				SphereShapeElemTM.ScaleTranslation(ShapeVectorScale);
				SphereShapeElemTM *= ShapeBoneTM;
				// TODO:
			}

			for (int32 i = 0; i <ShapeAggGeom->ConvexElems.Num(); ++i)
			{
				FTransform SphereShapeElemTM = ShapeAggGeom->ConvexElems[i].GetTransform();
				SphereShapeElemTM.ScaleTranslation(ShapeVectorScale);
				SphereShapeElemTM *= ShapeBoneTM;
				// TODO:
			}

			for (int32 i = 0; i <ShapeAggGeom->TaperedCapsuleElems.Num(); ++i)
			{
				FTransform SphereShapeElemTM = ShapeAggGeom->TaperedCapsuleElems[i].GetTransform();
				SphereShapeElemTM.ScaleTranslation(ShapeVectorScale);
				SphereShapeElemTM *= ShapeBoneTM;
				// TODO:
			}
		}

		if (ParentBone.PhysicsBodySetup != nullptr)
		{
			// Capsuleの場合はParentBoneがもつカプセルのコリジョン判定によってParentBoneとBoneの位置を押し出す
			float ParentShapeScale = SkeletalMeshComp->GetBoneTransform(ParentBone.BoneRef.BoneIndex, FTransform::Identity).GetScale3D().GetAbsMax(); // コンポーネント座標でのTransformのスケール
			FVector ParentShapeVectorScale(ParentShapeScale);
			FTransform ParentShapeBoneTM = FTransform(ParentBone.Rotation, ParentBone.Location);
			FKAggregateGeom* ParentShapeAggGeom = &ParentBone.PhysicsBodySetup->AggGeom;

			for (int32 i = 0; i <ParentShapeAggGeom->SphylElems.Num(); ++i)
			{
				const FKSphylElem& CapsuleShape = ParentShapeAggGeom->SphylElems[i];

				if (CapsuleShape.Radius <= 0 || CapsuleShape.Length <= 0)
				{
					continue;
				}

				FTransform CapsuleShapeElemTM = CapsuleShape.GetTransform();
				CapsuleShapeElemTM.ScaleTranslation(ParentShapeVectorScale);
				CapsuleShapeElemTM *= ParentShapeBoneTM;

				FVector CapsuleShapeLocation = CapsuleShapeElemTM.GetLocation();

				for (int32 j = 0; j <UsePhysicsAssetAsLimits->SkeletalBodySetups.Num(); ++j)
				{
					if (!ensure(UsePhysicsAssetAsLimits->SkeletalBodySetups[j]))
					{
						continue;
					}
					int32 BoneIndex = SkeletalMeshComp->GetBoneIndex(UsePhysicsAssetAsLimits->SkeletalBodySetups[j]->BoneName);

					if (BoneIndex != INDEX_NONE)
					{
						// コリジョンがついてるボーンがシミュレーション対象で押し出しでボーンが動いてコリジョンが動くといたちごっこになるので
						// このフレームでの押し出しはコリジョンの位置に反映させない
						FTransform BoneTM = SkeletalMeshComp->GetBoneTransform(BoneIndex, FTransform::Identity); // コンポーネント座標でのTransform
						float Scale = SkeletalMeshComp->GetBoneTransform(BoneIndex, FTransform::Identity).GetScale3D().GetAbsMax();
						FVector VectorScale(Scale);
						BoneTM.RemoveScaling();

						FKAggregateGeom* AggGeom = &UsePhysicsAssetAsLimits->SkeletalBodySetups[j]->AggGeom;

						for (int32 k = 0; k <AggGeom->SphereElems.Num(); ++k)
						{
							const FKSphereElem& Sphere = AggGeom->SphereElems[k];

							// FAnimNode_KawaiiPhysics::AdjustBySphereCollision()のESphericalLimitType::Outerのケースを参考にしている
							if (Sphere.Radius <= 0.0f)
							{
								continue;
							}

							FTransform ElemTM = Sphere.GetTransform();
							ElemTM.ScaleTranslation(VectorScale);
							ElemTM *= BoneTM;
							const FVector& SphereLocation = ElemTM.GetLocation();

							FVector PushOutVector = FVector::ZeroVector;

							FVector StartPoint = CapsuleShapeLocation + ElemTM.GetRotation().GetAxisZ() * CapsuleShape.Length * 0.5f;
							FVector EndPoint = CapsuleShapeLocation + ElemTM.GetRotation().GetAxisZ() * CapsuleShape.Length * -0.5f;

							float LimitDistance = CapsuleShape.Radius + Sphere.Radius;
							float DistSquared = FMath::PointDistToSegmentSquared(SphereLocation, StartPoint, EndPoint);
							if (DistSquared < LimitDistance * LimitDistance)
							{
								FVector ClosestPoint = FMath::ClosestPointOnSegment(SphereLocation, StartPoint, EndPoint);
								PushOutVector = (ClosestPoint - SphereLocation).GetSafeNormal() * LimitDistance - (ClosestPoint - SphereLocation);
							}

							CapsuleShapeLocation += PushOutVector;
							// CapsuleShapeが押し出されたベクトルだけボーンも移動させるという単純な計算
							if (ParentBone.ParentIndex >= 0)
							{
								ParentBone.Location += PushOutVector;
							}
							Bone.Location += PushOutVector;
						}

						for (int32 k = 0; k <AggGeom->BoxElems.Num(); ++k)
						{
							FTransform ElemTM = AggGeom->BoxElems[k].GetTransform();
							ElemTM.ScaleTranslation(VectorScale);
							ElemTM *= BoneTM;
							// TODO:
						}

						for (int32 k = 0; k <AggGeom->SphylElems.Num(); ++k)
						{
							const FKSphylElem& Capsule = AggGeom->SphylElems[k];

							if (Capsule.Radius <= 0 || Capsule.Length <= 0)
							{
								continue;
							}

							FTransform ElemTM = Capsule.GetTransform();
							ElemTM.ScaleTranslation(VectorScale);
							ElemTM *= BoneTM;
							const FVector& CapsuleLocation = ElemTM.GetLocation();

							FVector PushOutVector = FVector::ZeroVector;

							FVector CapsuleShapeStartPoint = CapsuleShapeLocation + CapsuleShapeElemTM.GetRotation().GetAxisZ() * CapsuleShape.Length * 0.5f;
							FVector CapsuleShapeEndPoint = CapsuleShapeLocation + CapsuleShapeElemTM.GetRotation().GetAxisZ() * CapsuleShape.Length * -0.5f;

							FVector CapsuleStartPoint = CapsuleLocation + ElemTM.GetRotation().GetAxisZ() * Capsule.Length * 0.5f;
							FVector CapsuleEndPoint = CapsuleLocation + ElemTM.GetRotation().GetAxisZ() * Capsule.Length * -0.5f;

							FVector CapsuleShapeClosestPoint;
							FVector CapsuleClosestPoint;
							FMath::SegmentDistToSegmentSafe(CapsuleShapeStartPoint, CapsuleShapeEndPoint, CapsuleStartPoint, CapsuleEndPoint, CapsuleShapeClosestPoint, CapsuleClosestPoint);
							float DistSquared = (CapsuleShapeClosestPoint - CapsuleClosestPoint).SizeSquared();

							float LimitDistance = CapsuleShape.Radius + Capsule.Radius;
							if (DistSquared < LimitDistance * LimitDistance)
							{
								PushOutVector = CapsuleClosestPoint + (CapsuleShapeClosestPoint - CapsuleClosestPoint).GetSafeNormal() * LimitDistance - CapsuleShapeClosestPoint;
							}

							CapsuleShapeLocation += PushOutVector;
							// CapsuleShapeが押し出されたベクトルだけボーンも移動させるという単純な計算
							if (ParentBone.ParentIndex >= 0)
							{
								ParentBone.Location += PushOutVector;
							}
							Bone.Location += PushOutVector;
						}

						for (int32 k = 0; k <AggGeom->ConvexElems.Num(); ++k)
						{
							FTransform ElemTM = AggGeom->ConvexElems[k].GetTransform();
							ElemTM.ScaleTranslation(VectorScale);
							ElemTM *= BoneTM;
							// TODO:
						}

						for (int32 k = 0; k <AggGeom->TaperedCapsuleElems.Num(); ++k)
						{
							FTransform ElemTM = AggGeom->TaperedCapsuleElems[k].GetTransform();
							ElemTM.ScaleTranslation(VectorScale);
							ElemTM *= BoneTM;
							// TODO:
						}
					}
				}

				// シェイプが骨に複数くっついている場合、すべてを満足する押し出し位置は1イテレーションでは計算できないので、ひとつ押し出しを計算したらそこで打ち切る
				break;
			}
		}
	}
}

void FAnimNode_KawaiiPhysics::AdjustByAngleLimit(FComponentSpacePoseContext& Output, const FBoneContainer& BoneContainer, FTransform& ComponentTransform, 
	FKawaiiPhysicsModifyBone& Bone, FKawaiiPhysicsModifyBone& ParentBone)
{
	if (Bone.PhysicsSettings.LimitAngle == 0.0f)
	{
		return;
	}

	FVector BoneDir = (Bone.Location - ParentBone.Location).GetSafeNormal();
	FVector PoseDir = (Bone.PoseLocation - ParentBone.PoseLocation).GetSafeNormal();
	FVector Axis = FVector::CrossProduct(PoseDir, BoneDir);
	float Angle = FMath::Atan2(Axis.Size(), FVector::DotProduct(PoseDir, BoneDir));
	float AngleOverLimit= FMath::RadiansToDegrees(Angle) - Bone.PhysicsSettings.LimitAngle;

	if (AngleOverLimit > 0.0f)
	{
		BoneDir = BoneDir.RotateAngleAxis(-AngleOverLimit, Axis);
		Bone.Location = BoneDir * (Bone.Location - ParentBone.Location).Size() + ParentBone.Location;
	}
}

void FAnimNode_KawaiiPhysics::AdjustByPlanarConstraint(FKawaiiPhysicsModifyBone& Bone, FKawaiiPhysicsModifyBone& ParentBone)
{
	FPlane Plane;
	if (PlanarConstraint != EPlanarConstraint::None)
	{
		switch (PlanarConstraint)
		{
		case EPlanarConstraint::X:
			Plane = FPlane(ParentBone.Location, ParentBone.PoseRotation.GetAxisX());
			break;
		case EPlanarConstraint::Y:
			Plane = FPlane(ParentBone.Location, ParentBone.PoseRotation.GetAxisY());
			break;
		case EPlanarConstraint::Z:
			Plane = FPlane(ParentBone.Location, ParentBone.PoseRotation.GetAxisZ());
			break;
		}
		Bone.Location  = FVector::PointPlaneProject(Bone.Location, Plane);
	}
}

void FAnimNode_KawaiiPhysics::ApplySimuateResult(FComponentSpacePoseContext& Output, const FBoneContainer& BoneContainer, TArray<FBoneTransform>& OutBoneTransforms)
{
	for (int i = 0; i < ModifyBones.Num(); ++i)
	{
		OutBoneTransforms.Add(FBoneTransform(ModifyBones[i].BoneRef.GetCompactPoseIndex(BoneContainer), 
			FTransform(ModifyBones[i].PoseRotation, ModifyBones[i].PoseLocation, ModifyBones[i].PoseScale)));
	}	

	for (int i = 1; i < ModifyBones.Num(); ++i)
	{
		FKawaiiPhysicsModifyBone& Bone = ModifyBones[i];
		FKawaiiPhysicsModifyBone& ParentBone = ModifyBones[Bone.ParentIndex];

		if (ParentBone.ChildIndexs.Num() <= 1)
		{
			if (ParentBone.BoneRef.BoneIndex >= 0)
			{
				FVector PoseVector = Bone.PoseLocation - ParentBone.PoseLocation;
				FVector SimulateVector = Bone.Location - ParentBone.Location;

				if (PoseVector.GetSafeNormal() == SimulateVector.GetSafeNormal())
				{
					continue;
				}

				if (BoneForwardAxis == EBoneForwardAxis::X_Negative || BoneForwardAxis == EBoneForwardAxis::Y_Negative || BoneForwardAxis == EBoneForwardAxis::Z_Negative)
				{
					PoseVector *= -1;
					SimulateVector *= -1;
				}

				FQuat SimulateRotation = FQuat::FindBetweenVectors(PoseVector, SimulateVector) * ParentBone.PoseRotation;
				OutBoneTransforms[Bone.ParentIndex].Transform.SetRotation(SimulateRotation);
				ParentBone.PrevRotation = SimulateRotation;
			}
		}

		if (Bone.BoneRef.BoneIndex >= 0 && !Bone.bDummy)
		{
			OutBoneTransforms[i].Transform.SetLocation(Bone.Location);
		}
	}

	OutBoneTransforms.RemoveAll([](const FBoneTransform& BoneTransform) {
		return BoneTransform.BoneIndex < 0;
	});

	// for check in FCSPose<PoseType>::LocalBlendCSBoneTransforms
	OutBoneTransforms.Sort(FCompareBoneTransformIndex());
}