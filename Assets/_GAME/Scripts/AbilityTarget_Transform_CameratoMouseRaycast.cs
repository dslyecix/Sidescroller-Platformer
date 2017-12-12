using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[CreateAssetMenu]
public class AbilityTarget_Transform_CameraToMouseRaycast : AbilityTarget
{

	

	// Determines the target of an Ability according to a
	// raycast from the camera to the mouse position

	public override void FetchTargets()
    {
        abilityTarget = (Transform)FindObjectOfType<PlayerCharacterController>().transform;
    }
}
