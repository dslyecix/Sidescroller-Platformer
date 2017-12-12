using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public abstract class Ability : ScriptableObject {

	public abstract void CastAbility();

	public abstract void ResolveTargets();

	public abstract void ExecuteBehaviours();
}

