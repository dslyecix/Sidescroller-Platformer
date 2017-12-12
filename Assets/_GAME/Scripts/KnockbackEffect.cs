using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class KnockbackEffect : IAbilityBehaviour
{
    public void Run(List<IAbilityTarget> targets, Transform _caster)
    {
        Debug.Log(_caster.name + "'s Knockback Effect activated!");    
    }
}
