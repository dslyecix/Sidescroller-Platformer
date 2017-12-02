using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using KinematicCharacterController;
using System;

namespace KinematicCharacterController.Walkthrough.PlayerCameraCharacterSetup
{
public class MyCharacterController : BaseCharacterController
{
    public override void BeforeCharacterUpdate(float deltaTime)
    {
    }

    public override void UpdateRotation(ref Quaternion currentRotation, float deltaTime)
    {
    }

    public override void UpdateVelocity(ref Vector3 currentVelocity, float deltaTime)
    {
    }

    public override void AfterCharacterUpdate(float deltaTime)
    {
    }

    public override bool CanBeStableOnCollider(Collider coll)
    {
        return true;
    }

    public override bool IsColliderValidForCollisions(Collider coll)
    {
        return true;
    }

    public override bool MustUpdateGrounding()
    {
        return true;
    }

    public override void OnGroundHit(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint, bool isStableOnHit)
    {
    }

    public override void OnMovementHit(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint, bool isStableOnHit)
    {
    }
}
}