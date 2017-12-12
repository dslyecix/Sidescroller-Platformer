using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public abstract class AbilityTarget : ScriptableObject
{
	// Interface for classes that represent the initial place for an ability
	// to be activated towards.  Could be a transform,  Enemy class script, a location,
	// a direction, etc.
	public Transform abilityTarget;

	public abstract void FetchTargets ();
}
