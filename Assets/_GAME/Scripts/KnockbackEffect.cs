using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[CreateAssetMenu]
public class KnockbackEffect : AbilityBehaviour
{
    public override void Run(List<AbilityTarget> targets)
    {
        Debug.Log("Running behaviour");
        foreach (AbilityTarget target in targets)
        {
            PlayerCharacterController character = target.abilityTarget.GetComponent<PlayerCharacterController>();
            
            character.AddVelocity(Random.onUnitSphere * 20f);
        }  
    }
}
