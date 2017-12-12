using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class AbilityTarget_Transform_CameratoMouseRaycast : IAbilityTarget
{
    // Determines the target of an Ability according to a
	// raycast from the camera to the mouse position

	public Transform abilityTarget; //TODO: check to see if what we hit has a Unit component (or some layer filtering)

	public void FetchTargets()
    {
        Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
		RaycastHit hit;
		Physics.Raycast(ray, out hit);
		abilityTarget = hit.transform;
    }
}
