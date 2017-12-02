using UnityEngine;

namespace KinematicCharacterController
{
    public enum MovementCalculationPhase
    {
        BaseVelocity,
        StandingInteractiveRigidbodyVelocity
    }

    public enum RigidbodyInteractionType
    {
        Kinematic,
        SimulatedDynamic
    }

    public enum VelocityProjectionMethod
    {

    }

    public struct CharacterGroundingReport
    {
        public bool FoundAnyGround;
        public bool FoundStableGround;
        public Collider GroundCollider;
        public Vector3 GroundNormal;
    }

    [RequireComponent(typeof(Rigidbody))]
    [RequireComponent(typeof(CapsuleCollider))]
    public class KinematicCharacterMotor : MonoBehaviour
    {
#pragma warning disable 0414
        [Header("Components")]
        public BaseCharacterController CharacterController;
        [ReadOnly]
        public CapsuleCollider CharacterCapsule;
        [ReadOnly]
        public Rigidbody CharacterRigidbody;

        [Header("Component Settings")]
        [Tooltip("Radius of the Character Capsule")]
        public float CharacterRadius = 0.5f;
        [Tooltip("Height of the Character Capsule")]
        public float CharacterHeight = 2f;
        [Tooltip("Physics material of the Character Capsule (Does not affect character movement. Only affects things colliding with it)")]
        public PhysicMaterial CharacterPhysicsMaterial;

        [Header("Motor Parameters")]
        [Tooltip("Distance of the ground probing and ground snapping cast.")]
        public float GroundProbingDistance = 0.2f;
        [Range(0f, 89f)]
        [Tooltip("Maximum slope angle on which the character can be stable")]
        public float MaxStableSlopeAngle = 60f;
        [Tooltip("Maximum height of a step which the character can climb (max value is the character capsule's radius)")]
        public float MaxStepHeight = 0.5f;
        [Tooltip("Maximum velocity magnitude at which the character can consider itself stable when it is moving towards a ledge")]
        public float MaxStableOnLedgeVelocity = 4;
        [Tooltip("Maximum planar distance from the character's center at which the character can consider itself stable when it is standing on a ledge (max value is the character capsule's radius)")]
        public float MaxStableDistanceFromLedge = 5f;
        [Tooltip("A relative mass value used for pushing rigidbodies in \"SimulatedDynamic\" RigidbodyInteractionType")]
        public float DynamicPushForce = 0.2f;
        [Tooltip("How the character interacts with non-kinematic rigidbodies. \"Kinematic\" mode means the character pushes the rigidbodies with infinite force (as a kinematic body would). \"SimulatedDynamic\" pushes the rigidbodies with a simulated mass value.")]
        public RigidbodyInteractionType RigidbodyInteractionType;
        [Tooltip("Determines if the character preserves moving platform velocities when de-grounding from them")]
        public bool PreserveInteractiveRigidbodyMomentum = true;
        [Tooltip("Determines if the character's movement uses the planar constraint")]
        public bool HasPlanarConstraint = false;
        [Tooltip("Defines the plane that the character's movement is constrained on, if HasMovementConstraintPlane is active")]
        public Vector3 PlanarConstraint = Vector3.forward;

        [Header("Quality & Optimizations")]
        [Tooltip("Handles properly detecting grounding status on steps and ledges")]
        public bool StepAndLedgeHandling = true;
        [Tooltip("Handles properly being pushed by and standing on PhysicsMovers or dynamic rigidbodies. Also handles pushing dynamic rigidbodies.")]
        public bool InteractiveRigidbodyHandling = true;
        [Tooltip("Makes sure the character cannot perform a move at all if it would be overlapping with any collidable objects at its destination. Useful for preventing \"tunneling\"")]
        public bool SafeMovement = true;
        
        public Transform CharacterTransform { get; private set; }
        public LayerMask CollidableLayers { get; private set; }
        public bool FoundAnyGround { get; private set; }
        public bool IsStableOnGround { get; private set; }
        public bool WasStableOnGround { get; private set; }
        public RaycastHit GroundProbingHit { get; private set; }
        public Vector3 GroundNormal { get; private set; }
        public Collider GroundCollider { get; private set; }
        public Transform GroundColliderTransform { get; private set; }
        public Rigidbody StableInteractiveRigidbody { get; private set; }
        public Vector3 CharacterUp { get; private set; }
        public Vector3 CharacterForward { get; private set; }
        public Vector3 CharacterRight { get; private set; }

        private bool _solveMovement = true;
        private RaycastHit[] _internalCharacterHits = new RaycastHit[MaxHitsBudget];
        private Collider[] _internalProbedColliders = new Collider[MaxCollisionBudget];
        private Rigidbody[] _rigidbodiesPushedThisUpdate = new Rigidbody[MaxCollisionBudget];
        private Vector3 _characterTransformToCapsuleBottom = Vector3.zero;
        private Vector3 _characterTransformToCapsuleTop = Vector3.zero;
        private Vector3 _characterTransformToCapsuleBottomHemi = Vector3.zero;
        private Vector3 _characterTransformToCapsuleTopHemi = Vector3.zero;
        private float _deltaTime;
        private Vector3 _baseVelocity;
        private Vector3 _stableInteractiveRigidbodyVelocity;
        private Vector3 _initialTransientPosition = Vector3.zero;
        private Quaternion _initialTransientRotation = Quaternion.identity;
        private bool _sweepMovementFoundGround = false;
        private bool _mustUnground = false;
        private Vector3 _velocityAtStartOfMove;
        private Vector3 _previousMovementHitNormal = Vector3.zero;
        private Rigidbody _lastStableInteractiveRigidbody = null;
        private int _rigidbodiesPushedCount = 0;
        private MovementCalculationPhase _movementCalculationPhase;
        private Vector3 _cachedWorldUp = Vector3.up;
        private Vector3 _cachedWorldForward = Vector3.forward;
        private Vector3 _cachedWorldRight = Vector3.right;
        private Vector3 _cachedZeroVector = Vector3.zero;

        // Warning: Don't touch these constants unless you know exactly what you're doing!
        private const int MaxHitsBudget = 32;
        private const int MaxCollisionBudget = 32;
        private const int MaxGroundingSweepIterations = 3;
        private const int MaxMovementSweepIterations = 6;
        private const int MaxSteppingSweepIterations = 3;
        private const int MaxSuccessiveMoveOverlaps = 2;
        private const int DiscreteCollisionIterations = 1;
        private const float CollisionOffset = 0.001f;
        private const float MinimumGroundProbingDistance = 0.002f;
        private const float GroundProbingBackstepDistance = 0.1f;
        private const float SweepProbingBackstepDistance = 0.002f;
        private const float OverlapDetectionPadding = 0.01f;
        private const float SecondaryProbesVertical = 0.1f;
        private const float SecondaryProbesHorizontal = 0.001f;
        private const float MinVelocityMagnitude = 0.01f;
        private const float MaxStepHeightTrim = 0.01f;
        private const float ExtraStepHeightPadding = 0.01f;

        private Vector3 _internalTransientPosition;
        public Vector3 TransientPosition
        {
            get
            {
                return _internalTransientPosition;
            }
            private set
            {
                _internalTransientPosition = value;
            }
        }

        private Quaternion _internalTransientRotation;
        public Quaternion TransientRotation
        {
            get
            {
                return _internalTransientRotation;
            }
            private set
            {
                _internalTransientRotation = value;
                CharacterUp = _internalTransientRotation * _cachedWorldUp;
                CharacterForward = _internalTransientRotation * _cachedWorldForward;
                CharacterRight = _internalTransientRotation * _cachedWorldRight;
            }
        }

        public Vector3 Velocity
        {
            get
            {
                return _baseVelocity + _stableInteractiveRigidbodyVelocity;
            }
        }
        public Vector3 BaseVelocity
        {
            get
            {
                return _baseVelocity;
            }
        }
        public Vector3 StableInteractiveRigidbodyVelocity
        {
            get
            {
                return _stableInteractiveRigidbodyVelocity;
            }
        }

        private Vector3 _velocityForCurrentPhase
        {
            get
            {
                if (_movementCalculationPhase == MovementCalculationPhase.BaseVelocity)
                {
                    return _baseVelocity;
                }
                else if (_movementCalculationPhase == MovementCalculationPhase.StandingInteractiveRigidbodyVelocity)
                {
                    return _stableInteractiveRigidbodyVelocity;
                }
                else
                {
                    return Vector3.zero;
                }
            }
            set
            {
                if (_movementCalculationPhase == MovementCalculationPhase.BaseVelocity)
                {
                    _baseVelocity = value;
                }
                else if (_movementCalculationPhase == MovementCalculationPhase.StandingInteractiveRigidbodyVelocity)
                {
                    _stableInteractiveRigidbodyVelocity = value;
                }
            }
        }
#pragma warning restore 0414 

        private void OnEnable()
        {
            KinematicCharacterSystem.GetOrCreateInstance().KinematicCharacterMotors.Add(this);
        }

        private void OnDisable()
        {
            if (KinematicCharacterSystem.HasInstance())
            {
                KinematicCharacterSystem.GetInstance().KinematicCharacterMotors.Remove(this);
            }
        }

        private void Reset()
        {
            ValidateData();
        }

        private void OnValidate()
        {
            ValidateData();
        }

        [ContextMenu("Remove Component")]
        void HandleRemoveComponent()
        {
            Rigidbody tmpRigidbody = gameObject.GetComponent<Rigidbody>();
            CapsuleCollider tmpCapsule = gameObject.GetComponent<CapsuleCollider>();
            DestroyImmediate(this);
            DestroyImmediate(tmpRigidbody);
            DestroyImmediate(tmpCapsule);
        }

        /// <summary>
        /// Handle validating all required values
        /// </summary>
        public void ValidateData()
        {
            CharacterRigidbody = GetComponent<Rigidbody>();
            CharacterRigidbody.centerOfMass = Vector3.zero;
            CharacterRigidbody.useGravity = false;
            CharacterRigidbody.drag = 0f;
            CharacterRigidbody.angularDrag = 0f;
            CharacterRigidbody.maxAngularVelocity = Mathf.Infinity;
            CharacterRigidbody.maxDepenetrationVelocity = Mathf.Infinity;
            CharacterRigidbody.collisionDetectionMode = CollisionDetectionMode.Discrete;
            CharacterRigidbody.isKinematic = true;
            CharacterRigidbody.constraints = RigidbodyConstraints.None;
            CharacterRigidbody.interpolation = KinematicCharacterSystem.UseInterpolation ? RigidbodyInterpolation.Interpolate : RigidbodyInterpolation.None;

            CharacterCapsule = GetComponent<CapsuleCollider>();
            CharacterRadius = Mathf.Clamp(CharacterRadius, 0f, CharacterHeight * 0.5f);
            CharacterCapsule.isTrigger = false;
            CharacterCapsule.direction = 1;
            CharacterCapsule.sharedMaterial = CharacterPhysicsMaterial;
            SetCapsuleDimensionsAuto(CharacterRadius, CharacterHeight);

            GroundProbingDistance = Mathf.Clamp(GroundProbingDistance, 0f, Mathf.Infinity);
            MaxStepHeight = Mathf.Clamp(MaxStepHeight, 0f, CharacterCapsule.radius - MaxStepHeightTrim);
            MaxStableOnLedgeVelocity = Mathf.Clamp(MaxStableOnLedgeVelocity, 0f, Mathf.Infinity);
            MaxStableDistanceFromLedge = Mathf.Clamp(MaxStableDistanceFromLedge, 0f, CharacterCapsule.radius);
            DynamicPushForce = Mathf.Clamp(DynamicPushForce, 0f, 9999f);

            transform.localScale = Vector3.one;

#if UNITY_EDITOR
            CharacterCapsule.hideFlags = HideFlags.NotEditable;
            CharacterRigidbody.hideFlags = HideFlags.NotEditable;
            if (!Mathf.Approximately(transform.lossyScale.x, 1f) || !Mathf.Approximately(transform.lossyScale.y, 1f) || !Mathf.Approximately(transform.lossyScale.z, 1f))
            {
                Debug.LogError("Character's lossy scale is not (1,1,1). This is not allowed. Make sure the character's transform and its entire parent hierarchy has a (1,1,1) scale.", this.gameObject);
            }
#endif
        }

        /// <summary>
        /// Sets whether or not the KinematicCharacterMotor should be processing collisions and probing logic
        /// </summary>
        public void HandlePhysics(bool movementSolvingActive, bool kinematicCapsuleActive)
        {
            CharacterRigidbody.detectCollisions = kinematicCapsuleActive;
            _solveMovement = movementSolvingActive;
        }

        /// <summary>
        /// Resizes capsule while always keeping the bottom extent flush with transform's position.
        /// </summary>
        public void SetCapsuleDimensionsAuto(float radius, float height)
        {
            SetCapsuleDimensions(radius, height, height * 0.5f);
        }

        /// <summary>
        /// Resizes capsule. ALso caches importand capsule size data
        /// </summary>
        public void SetCapsuleDimensions(float radius, float height, float centerHeight)
        {
            CharacterCapsule.radius = radius;
            CharacterCapsule.height = Mathf.Clamp(height, radius * 2f, height);
            CharacterCapsule.center = new Vector3(0f, centerHeight, 0f);

            _characterTransformToCapsuleBottom = CharacterCapsule.center + (-_cachedWorldUp * (CharacterCapsule.height * 0.5f));
            _characterTransformToCapsuleTop = CharacterCapsule.center + (_cachedWorldUp * (CharacterCapsule.height * 0.5f));
            _characterTransformToCapsuleBottomHemi = CharacterCapsule.center + (-_cachedWorldUp * (CharacterCapsule.height * 0.5f)) + (_cachedWorldUp * CharacterCapsule.radius);
            _characterTransformToCapsuleTopHemi = CharacterCapsule.center + (_cachedWorldUp * (CharacterCapsule.height * 0.5f)) + (-_cachedWorldUp * CharacterCapsule.radius);
        }

        private void Awake()
        {
            CharacterTransform = this.transform;
            ValidateData();

            TransientPosition = CharacterTransform.position;
            TransientRotation = CharacterTransform.rotation;

            // Build CollidableLayers mask
            CollidableLayers = 0;
            for (int i = 0; i < 32; i++)
            {
                if (!Physics.GetIgnoreLayerCollision(this.gameObject.layer, i))
                {
                    CollidableLayers |= (1 << i);
                }
            }

            if(CharacterController)
            {
                CharacterController.SetupCharacterMotor(this);
            }

            SetCapsuleDimensionsAuto(CharacterRadius, CharacterHeight);
        }

        /// <summary>
        /// Update phase 1 is meant to be called after physics movers have calculated their velocities, but
        /// before they have simulated their goal positions/rotations. It is responsible for:
        /// - Initializing all values for update
        /// - Evaluating rotation
        /// - Solving initial collision overlaps
        /// - Ground probing
        /// - Evaluating velocity
        /// - Handle interactions with rigidbodies
        /// </summary>
        public void CharacterUpdatePhase1(float deltaTime)
        {
#if UNITY_EDITOR
            if (!Mathf.Approximately(CharacterTransform.lossyScale.x, 1f) || !Mathf.Approximately(CharacterTransform.lossyScale.y, 1f) || !Mathf.Approximately(CharacterTransform.lossyScale.z, 1f))
            {
                Debug.LogError("Character's lossy scale is not (1,1,1). This is not allowed. Make sure the character's transform and its entire parent hierarchy has a (1,1,1) scale.", this.gameObject);
            }
#endif

            // Before update
            this.CharacterController.BeforeCharacterUpdate(deltaTime);

            #region Initialize values for character pass
            _deltaTime = deltaTime;
            TransientPosition = CharacterTransform.position;
            TransientRotation = CharacterTransform.rotation;
            _initialTransientPosition = TransientPosition;
            _initialTransientRotation = TransientRotation;
            FoundAnyGround = false;
            WasStableOnGround = IsStableOnGround;
            IsStableOnGround = false;
            GroundNormal = CharacterUp;
            GroundCollider = null;
            GroundColliderTransform = null;
            _rigidbodiesPushedCount = 0;
            _movementCalculationPhase = MovementCalculationPhase.BaseVelocity;
            _lastStableInteractiveRigidbody = StableInteractiveRigidbody;
            StableInteractiveRigidbody = null;
            #endregion

            if (_solveMovement)
            {
                #region Resolve initial overlaps
                Vector3 resolutionDirection = _cachedWorldUp;
                float resolutionDistance = 0f;

                for (int collIteration = 0; collIteration < DiscreteCollisionIterations; collIteration++)
                {
                    int nbHits = CharacterCollisionsOverlap(TransientPosition, TransientRotation, _internalProbedColliders);
                    for (int i = 0; i < nbHits; i++)
                    {
                        // Process overlap
                        Transform overlappedTransform = _internalProbedColliders[i].GetComponent<Transform>();
                        if (Physics.ComputePenetration(
                                CharacterCapsule,
                                TransientPosition,
                                TransientRotation,
                                _internalProbedColliders[i],
                                overlappedTransform.position,
                                overlappedTransform.rotation,
                                out resolutionDirection,
                                out resolutionDistance))
                        {
                            // Don't solve collisions against dynamic rigidbodies
                            if (!_internalProbedColliders[i].attachedRigidbody || (_internalProbedColliders[i].attachedRigidbody && _internalProbedColliders[i].attachedRigidbody.isKinematic))
                            {
                                // De-collide
                                Vector3 resolutionMovement = resolutionDirection * (resolutionDistance + CollisionOffset);
                                TransientPosition += resolutionMovement;
                            }
                        }
                    }
                }
                #endregion

                #region Ground Probing and Snapping
                CharacterGroundingReport groundingReport = new CharacterGroundingReport();
                // Handle ungrounding
                if (_mustUnground)
                {
                    TransientPosition += CharacterUp * (MinimumGroundProbingDistance * 1.5f);
                }
                else
                {
                    if (this.CharacterController.MustUpdateGrounding())
                    {
                        ProbeGround(ref _internalTransientPosition, TransientRotation, (WasStableOnGround || _sweepMovementFoundGround) ? GroundProbingDistance : CollisionOffset * 2f, ref groundingReport);
                    }
                }
                FoundAnyGround = groundingReport.FoundAnyGround;
                IsStableOnGround = groundingReport.FoundStableGround;
                GroundNormal = groundingReport.GroundNormal;
                GroundCollider = groundingReport.GroundCollider;
                if (FoundAnyGround)
                {
                    GroundColliderTransform = groundingReport.GroundCollider.GetComponent<Transform>();
                }

                _sweepMovementFoundGround = false;
                _mustUnground = false;
                #endregion
            }

            // Handle velocity
            this.CharacterController.UpdateVelocity(ref _baseVelocity, deltaTime);

            if (_solveMovement && InteractiveRigidbodyHandling)
            {
                #region Interactive Rigidbody Handling 
                // Detect interactive rigidbodies from grounding
                if (IsStableOnGround && GroundCollider.attachedRigidbody)
                {
                    Rigidbody interactiveRigidbody = GetInteractiveRigidbody(GroundCollider);
                    if (interactiveRigidbody)
                    {
                        StableInteractiveRigidbody = interactiveRigidbody;
                    }
                }

                // Conserve momentum when de-stabilized from an interactive rigidbody
                if (PreserveInteractiveRigidbodyMomentum && StableInteractiveRigidbody == null && _lastStableInteractiveRigidbody != null)
                {
                    _baseVelocity += _stableInteractiveRigidbodyVelocity;
                }

                // Process additionnal Velocity from interactive rigidbody
                _stableInteractiveRigidbodyVelocity = _cachedZeroVector;
                if (StableInteractiveRigidbody)
                {
                    _stableInteractiveRigidbodyVelocity = GetTotalVelocityFromInteractiveRigidbody(StableInteractiveRigidbody, TransientPosition, deltaTime);

                    // Rotation from interactive rigidbody
                    Vector3 newForward = Vector3.ProjectOnPlane(Quaternion.Euler(Mathf.Rad2Deg * StableInteractiveRigidbody.angularVelocity * deltaTime) * CharacterForward, CharacterUp).normalized;
                    TransientRotation = Quaternion.LookRotation(newForward, CharacterUp);
                }

                // Cancel out velocity upon landing on an interactive rigidbody
                if (StableInteractiveRigidbody != null && _lastStableInteractiveRigidbody == null)
                {
                    _baseVelocity -= Vector3.ProjectOnPlane(_stableInteractiveRigidbodyVelocity, CharacterUp);
                }
                #endregion

                _movementCalculationPhase = MovementCalculationPhase.StandingInteractiveRigidbodyVelocity;

                #region Calculate character movement from stable interactive rigidbody velocity
                // First, move with stable interactive rigidbody velocity
                if (_stableInteractiveRigidbodyVelocity.sqrMagnitude > 0f)
                {
                    _velocityAtStartOfMove = _stableInteractiveRigidbodyVelocity;
                    _previousMovementHitNormal = _cachedZeroVector;
                    if (!CharacterMove(ref _internalTransientPosition, TransientRotation, (_velocityAtStartOfMove * deltaTime)))
                    {
                        Vector3 tmp = _cachedWorldForward;
                        MultiplyVelocityAndMovementMagnitude(0f, ref tmp);
                    }
                }
                #endregion
            }

            _movementCalculationPhase = MovementCalculationPhase.BaseVelocity;
        }

        /// <summary>
        /// Update phase 2 is meant to be called after physics movers have simulated their goal positions/rotations. It is responsible for:
        /// - Solving additional rigidbody overlaps
        /// - Processing character movement
        /// - Applying character movement instantly for the rest of the simulation to be accurate
        /// </summary>
        public void CharacterUpdatePhase2(float deltaTime)
        {
            if (_solveMovement && InteractiveRigidbodyHandling)
            {
                #region Solve potential stable interactive rigidbody overlap
                if (StableInteractiveRigidbody)
                {
                    float upwardsOffset = CharacterCapsule.height * 0.5f;
                    RaycastHit closestHit;
                    if (CharacterGroundSweep(
                        TransientPosition + (CharacterUp * upwardsOffset),
                        TransientRotation,
                        -CharacterUp,
                        upwardsOffset + CollisionOffset,
                        out closestHit))
                    {
                        if (closestHit.collider == GroundCollider)
                        {
                            float distanceMovedUp = (upwardsOffset - closestHit.distance);
                            TransientPosition = TransientPosition + (CharacterUp * distanceMovedUp) + (CharacterUp * CollisionOffset);
                        }
                    }
                }
                #endregion

                #region Resolve overlaps that could've been caused by physics movers simulation pushing the character
                float tmpRadius = CharacterCapsule.radius;
                float tmpHeight = CharacterCapsule.height;
                CharacterCapsule.radius += CollisionOffset;
                CharacterCapsule.height += CollisionOffset * 2f;
                Vector3 resolutionDirection = _cachedWorldUp;
                float resolutionDistance = 0f;

                for (int collIteration = 0; collIteration < DiscreteCollisionIterations; collIteration++)
                {
                    int nbHits = CharacterCollisionsOverlap(TransientPosition, TransientRotation, _internalProbedColliders);
                    for (int i = 0; i < nbHits; i++)
                    {
                        if (_internalProbedColliders[i].attachedRigidbody)
                        {
                            // Process overlap
                            PhysicsMover physicsMover = _internalProbedColliders[i].attachedRigidbody.gameObject.GetComponent<PhysicsMover>();
                            Transform probedColliderTransform = _internalProbedColliders[i].GetComponent<Transform>();
                            if (physicsMover &&
                                !(StableInteractiveRigidbody && _internalProbedColliders[i].attachedRigidbody == StableInteractiveRigidbody) &&
                                !(_lastStableInteractiveRigidbody && _internalProbedColliders[i].attachedRigidbody == _lastStableInteractiveRigidbody) &&
                                CheckIfColliderValidForCollisions(_internalProbedColliders[i]) &&
                                Physics.ComputePenetration(
                                    CharacterCapsule,
                                    TransientPosition,
                                    TransientRotation,
                                    _internalProbedColliders[i],
                                    probedColliderTransform.position,
                                    probedColliderTransform.rotation,
                                    out resolutionDirection,
                                    out resolutionDistance))
                            {
                                // Handle getting pushed by a kinematic mover (affects velocity as well)
                                if (!IsStableOnNormal(resolutionDirection))
                                {
                                    Vector3 collisionProjectionPlane = IsStableOnGround ? CharacterUp : resolutionDirection;
                                    Vector3 effectiveMoverVelocity = GetTotalVelocityFromInteractiveRigidbody(physicsMover.MoverRigidbody, TransientPosition, deltaTime);
                                    Vector3 additiveVelocity = Vector3.ClampMagnitude(Vector3.Project(_velocityAtStartOfMove, Vector3.Project(effectiveMoverVelocity, collisionProjectionPlane).normalized), effectiveMoverVelocity.magnitude);
                                    _baseVelocity += additiveVelocity;
                                }

                                if (IsStableOnGround)
                                {
                                    resolutionDirection = Vector3.ProjectOnPlane(resolutionDirection, GroundNormal).normalized;
                                }

                                // De-collide
                                Vector3 resolutionMovement = resolutionDirection * resolutionDistance;
                                TransientPosition += resolutionMovement;
                            }
                        }
                    }
                }

                CharacterCapsule.radius = tmpRadius;
                CharacterCapsule.height = tmpHeight;
                #endregion
            }

            #region Calculate Character movement from base velocity      
            if (_baseVelocity.sqrMagnitude > 0f)
            {
                _velocityAtStartOfMove = _baseVelocity;
                if (_solveMovement)
                {
                    _previousMovementHitNormal = _cachedZeroVector;
                    if (!CharacterMove(ref _internalTransientPosition, TransientRotation, (_velocityAtStartOfMove * deltaTime)))
                    {
                        Vector3 tmp = _cachedWorldForward;
                        MultiplyVelocityAndMovementMagnitude(0f, ref tmp);
                    }
                }
                else
                {
                    TransientPosition += _velocityAtStartOfMove * deltaTime;
                }
            }
            #endregion

            // Handle rotation
            this.CharacterController.UpdateRotation(ref _internalTransientRotation, deltaTime);
            TransientRotation = _internalTransientRotation;

            // Handle planar constraint
            if(HasPlanarConstraint)
            {
                TransientPosition = _initialTransientPosition + Vector3.ProjectOnPlane(TransientPosition - _initialTransientPosition, PlanarConstraint.normalized);
            }

            #region Apply final calculated movement instantly
            CharacterRigidbody.position = TransientPosition;
            CharacterRigidbody.rotation = TransientRotation;
            #endregion

            this.CharacterController.AfterCharacterUpdate(deltaTime);
        }

        /// <summary>
        /// Update phase 3 undoes the movement simulation of phase 2 and moves the character with correct interpolation
        /// </summary>
        public void CharacterUpdatePhase3(float deltaTime)
        {
            CharacterRigidbody.position = _initialTransientPosition;
            CharacterRigidbody.rotation = _initialTransientRotation;

            CharacterRigidbody.MovePosition(TransientPosition);
            CharacterRigidbody.MoveRotation(TransientRotation);
        }

        /// <summary>
        /// Determines if motor can be considered stable on given slope normal
        /// </summary>
        private bool IsStableOnNormal(Vector3 normal)
        {
            return Vector3.Angle(CharacterUp, normal) <= MaxStableSlopeAngle;
        }

        /// <summary>
        /// Probes for valid ground and midifies the input transientPosition if ground snapping occurs
        /// </summary>
        public void ProbeGround(ref Vector3 probingPosition, Quaternion atRotation, float probingDistance, ref CharacterGroundingReport groundingReport)
        {
            if (probingDistance < MinimumGroundProbingDistance)
            {
                probingDistance = MinimumGroundProbingDistance;
            }

            int groundSweepsMade = 0;
            RaycastHit groundSweepHit = new RaycastHit();
            bool groundSweepingIsOver = false;
            Vector3 groundSweepPosition = probingPosition;
            Vector3 groundSweepDirection = (atRotation * -_cachedWorldUp);
            float groundProbeDistanceRemaining = probingDistance;
            while (groundProbeDistanceRemaining > 0 && (groundSweepsMade <= MaxGroundingSweepIterations) && !groundSweepingIsOver)
            {
                if (CharacterGroundSweep(
                        groundSweepPosition, // position
                        atRotation, // rotation
                        groundSweepDirection, // direction
                        groundProbeDistanceRemaining, // distance
                        out groundSweepHit)) // hit
                {
                    groundingReport.FoundAnyGround = true;
                    groundingReport.GroundNormal = groundSweepHit.normal;
                    groundingReport.GroundCollider = groundSweepHit.collider;

                    // Found stable ground
                    if (IsStableOnHit(groundSweepHit.collider, groundSweepHit.normal, groundSweepHit.point))
                    {
                        groundingReport.FoundStableGround = true;

                        // Ground snapping
                        Vector3 targetPosition = groundSweepPosition + (groundSweepDirection * groundSweepHit.distance);
                        targetPosition += (-groundSweepDirection * CollisionOffset);
                        MovePositionIfNoOverlaps(ref probingPosition, targetPosition, atRotation, 1);

                        this.CharacterController.OnGroundHit(groundSweepHit.collider, groundSweepHit.normal, groundSweepHit.point, true);
                        groundSweepingIsOver = true;
                    }
                    else
                    {
                        this.CharacterController.OnGroundHit(groundSweepHit.collider, groundSweepHit.normal, groundSweepHit.point, false);

                        // Calculate movement from this iteration and advance position
                        Vector3 sweepMovement = (groundSweepDirection * groundSweepHit.distance) + ((atRotation * Vector3.up) * Mathf.Clamp(CollisionOffset, 0f, groundSweepHit.distance));
                        groundSweepPosition = groundSweepPosition + sweepMovement;
                        groundProbeDistanceRemaining = Mathf.Clamp(groundProbeDistanceRemaining - sweepMovement.magnitude, 0f, Mathf.Infinity);

                        // Reorient direction
                        groundSweepDirection = Vector3.ProjectOnPlane(groundSweepDirection, groundSweepHit.normal).normalized;
                    }
                }
                else
                {
                    groundSweepingIsOver = true;
                }

                groundSweepsMade++;
            }
        }

        /// <summary>
        /// Forces the character to unground itself on its next grounding update
        /// </summary>
        public void ForceUnground()
        {
            _mustUnground = true;
            IsStableOnGround = false;
        }

        /// <summary>
        /// Returns the direction adjusted to be tangent to a specified surface normal relatively to the character's up direction.
        /// Useful for reorienting a direction on a slope without any lateral deviation in trajectory
        /// </summary>
        public Vector3 GetDirectionTangentToSurface(Vector3 direction, Vector3 surfaceNormal)
        {
            Vector3 directionRight = Vector3.Cross(direction, CharacterUp);
            return Vector3.Cross(surfaceNormal, directionRight).normalized;
        }

        /// <summary>
        /// Moves the character's position by given movement while taking into account all physics simulation and velocity projection rules that affect the character motor
        /// </summary>
        /// <returns> Returns false if movement could not be solved until the end </returns>
        private bool CharacterMove(ref Vector3 transientPosition, Quaternion transientRotation, Vector3 movement)
        {
            bool wasCompleted = true;
            Vector3 remainingFrameMovementDirection = movement.normalized;
            float remainingFrameMovementMagnitude = movement.magnitude;
            int successiveOverlaps = 0;
            int sweepsMade = 0;
            float overlapDistance;
            Vector3 overlapNormal;
            RaycastHit sweepHit;
            bool hitSomethingThisSweepIteration = true;

            while (remainingFrameMovementMagnitude > 0f &&
                (sweepsMade <= MaxMovementSweepIterations) &&
                hitSomethingThisSweepIteration &&
                successiveOverlaps < MaxSuccessiveMoveOverlaps)
            {
                if (CharacterCollisionsSweep(
                        transientPosition, // position
                        transientRotation, // rotation
                        remainingFrameMovementDirection, // direction
                        remainingFrameMovementMagnitude + CollisionOffset, // distance
                        out sweepHit, // closest hit
                        _internalCharacterHits) > 0) // all hits
                {
                    // Calculate movement from this iteration
                    Vector3 sweepMovement = (remainingFrameMovementDirection * sweepHit.distance) + (-remainingFrameMovementDirection * CollisionOffset);

                    Vector3 targetPositionAfterSweep = transientPosition + sweepMovement;
                    bool movementValid = false;
                    if (SafeMovement)
                    {
                        // Find out if we'd be overlapping with colliders at the target position, and project movement accordingly
                        int nbOverlaps = CharacterCollisionsOverlap(targetPositionAfterSweep, transientRotation, _internalProbedColliders);
                        if (nbOverlaps > 0)
                        {
                            successiveOverlaps++;

                            OnMovementHit(sweepHit.collider, sweepHit.normal, sweepHit.point, ref remainingFrameMovementDirection, ref remainingFrameMovementMagnitude);

                            // Process overlaps as additional hits
                            for (int i = 0; i < nbOverlaps; i++)
                            {
                                Transform probedColliderTransform = _internalProbedColliders[i].GetComponent<Transform>();
                                if (Physics.ComputePenetration(
                                    CharacterCapsule,
                                    targetPositionAfterSweep,
                                    transientRotation,
                                    _internalProbedColliders[i],
                                    probedColliderTransform.position,
                                    probedColliderTransform.rotation,
                                    out overlapNormal,
                                    out overlapDistance))
                                {
                                    OnMovementHit(_internalProbedColliders[i], overlapNormal, sweepHit.point, ref remainingFrameMovementDirection, ref remainingFrameMovementMagnitude);
                                }
                            }
                        }
                        else
                        {
                            successiveOverlaps = 0;
                            movementValid = true;
                        }
                    }
                    else
                    {
                        movementValid = true;
                    }

                    // Apply the actual movement
                    if (movementValid)
                    {
                        transientPosition = targetPositionAfterSweep;
                        remainingFrameMovementMagnitude = Mathf.Clamp(remainingFrameMovementMagnitude - sweepMovement.magnitude, 0f, Mathf.Infinity);
                        OnMovementHit(sweepHit.collider, sweepHit.normal, sweepHit.point, ref remainingFrameMovementDirection, ref remainingFrameMovementMagnitude);
                    }
                }
                // If we hit nothing...
                else
                {
                    hitSomethingThisSweepIteration = false;
                }

                // Safety for exceeding max sweeps allowed
                sweepsMade++;
                if (sweepsMade > MaxMovementSweepIterations)
                {
                    remainingFrameMovementMagnitude = 0;
                    wasCompleted = false;
                }
            }

            // Move position for the remainder of the movement
            Vector3 targetFinalPosition = transientPosition + (remainingFrameMovementDirection * remainingFrameMovementMagnitude);
            MovePositionIfNoOverlaps(ref transientPosition, targetFinalPosition, transientRotation, 1);

            return wasCompleted;
        }

        /// <summary>
        /// Processes velocity and movement project upon detecting a hit
        /// </summary>
        private void OnMovementHit(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint, ref Vector3 remainingMovementDirection, ref float remainingMovementMagnitude)
        {
            // Handle pushing rigidbodies in SimulatedDynamic mode
            if (InteractiveRigidbodyHandling && RigidbodyInteractionType == RigidbodyInteractionType.SimulatedDynamic)
            {
                Rigidbody hitRigidbody = hitCollider.attachedRigidbody;
                if (hitRigidbody && !hitRigidbody.isKinematic)
                {
                    bool alreadyPushedThisRigidbody = false;
                    for (int i = 0; i < _rigidbodiesPushedCount; i++)
                    {
                        if (_rigidbodiesPushedThisUpdate[i] == hitRigidbody)
                        {
                            alreadyPushedThisRigidbody = true;
                            break;
                        }
                    }

                    if (!alreadyPushedThisRigidbody)
                    {
                        _rigidbodiesPushedThisUpdate[_rigidbodiesPushedCount] = hitRigidbody;
                        _rigidbodiesPushedCount++;

                        float massMultiplier = DynamicPushForce / hitRigidbody.mass;
                        Vector3 effectiveHitRigidbodyVelocity = GetTotalVelocityFromInteractiveRigidbody(hitRigidbody, hitPoint, _deltaTime);
                        Vector3 relativeVelocity = Vector3.Project(Velocity, hitNormal) - effectiveHitRigidbodyVelocity;

                        hitRigidbody.AddForceAtPosition(massMultiplier * relativeVelocity, hitPoint, ForceMode.VelocityChange);
                    }
                }
            }

            Vector3 initialVelocity = _velocityForCurrentPhase;
            Vector3 remainingMovement = remainingMovementDirection * remainingMovementMagnitude;

            bool isStableOnHit = IsStableOnHit(hitCollider, hitNormal, hitPoint);

            // Find hit/obstruction/offset normal
            _sweepMovementFoundGround = isStableOnHit;
            Vector3 obstructionNormal = hitNormal;
            if (IsStableOnGround && !isStableOnHit)
            {
                Vector3 obstructionRightAlongGround = Vector3.Cross(GroundNormal, obstructionNormal);
                obstructionNormal = Vector3.Cross(obstructionRightAlongGround, CharacterUp).normalized;
            }

            // Blocking-corner handling
            bool isGoingTowardsBlockingCorner = false;
            if (_previousMovementHitNormal != _cachedZeroVector && !isStableOnHit)
            {
                Vector3 cornerVector = Vector3.Cross(_previousMovementHitNormal, obstructionNormal).normalized;
                Vector3 movementRight = Vector3.Cross(CharacterUp, Vector3.ProjectOnPlane(_velocityAtStartOfMove, cornerVector)).normalized;
                if (Vector3.Dot(movementRight, _previousMovementHitNormal) > 0f != Vector3.Dot(movementRight, obstructionNormal) > 0f)
                {
                    ProjectVelocityAndMovement(cornerVector, ref remainingMovement);
                    if (IsStableOnGround)
                    {
                        ProjectVelocityAndMovementOnPlane(CharacterUp, ref remainingMovement);
                    }
                    isGoingTowardsBlockingCorner = true;
                }
            }

            if (!isGoingTowardsBlockingCorner)
            {
                if (IsStableOnGround)
                {
                    // On stable slopes, simply reorient the velocity without any loss
                    if (isStableOnHit)
                    {
                        ReorientVelocityAndMovementOnSlope(obstructionNormal, ref remainingMovement);
                    }
                    // On blocking slopes, project the velocity on the obstruction while following the grounding plane
                    else
                    {
                        Vector3 obstructionUp = Vector3.ProjectOnPlane(GroundNormal, obstructionNormal).normalized;
                        ProjectVelocityAndMovementOnPlane(obstructionNormal, ref remainingMovement);
                        ProjectVelocityAndMovementOnPlane(obstructionUp, ref remainingMovement);
                    }
                }
                else
                {
                    // Handle stable landing
                    if (isStableOnHit)
                    {
                        // Cancels out velocity upon landing
                        ProjectVelocityAndMovementOnPlane(CharacterUp, ref remainingMovement);
                        ReorientVelocityAndMovementOnSlope(obstructionNormal, ref remainingMovement);
                    }
                    // Handle generic obstruction
                    else
                    {
                        ProjectVelocityAndMovementOnPlane(obstructionNormal, ref remainingMovement);
                    }
                }
            }

            // Interactive rigidbody detection and projection handling
            if (InteractiveRigidbodyHandling && !isStableOnHit)
            {
                Rigidbody interactiveRigidbody = GetInteractiveRigidbody(hitCollider);
                if (interactiveRigidbody && interactiveRigidbody != StableInteractiveRigidbody)
                {
                    if (Vector3.Dot(_velocityAtStartOfMove, interactiveRigidbody.velocity) > 0f)
                    {
                        // Take into account rigidbody's velocity
                        Vector3 collisionProjectionPlane = isStableOnHit ? CharacterUp : obstructionNormal;
                        Vector3 effectiveMoverVelocity = GetTotalVelocityFromInteractiveRigidbody(interactiveRigidbody, hitPoint, _deltaTime);
                        Vector3 additiveVelocity = Vector3.ClampMagnitude(Vector3.Project(_velocityAtStartOfMove, Vector3.Project(effectiveMoverVelocity, collisionProjectionPlane).normalized), effectiveMoverVelocity.magnitude);
                        _baseVelocity += additiveVelocity;
                    }
                }
            }

            // Prevent micro-stuttering in corners by clamping velocity
            if (_velocityForCurrentPhase.magnitude < MinVelocityMagnitude)
            {
                MultiplyVelocityAndMovementMagnitude(0f, ref remainingMovement);
            }

            remainingMovementDirection = remainingMovement.normalized;
            remainingMovementMagnitude = remainingMovement.magnitude;

            _previousMovementHitNormal = obstructionNormal;

            this.CharacterController.OnMovementHit(hitCollider, hitNormal, hitPoint, isStableOnHit);
        }

        /// <summary>
        /// Moves the input position only if we detect that the character would not be overlapping with anything at the target position
        /// </summary>
        /// <returns> Returns true if no overlaps were found </returns>
        private bool MovePositionIfNoOverlaps(ref Vector3 movedPosition, Vector3 targetPosition, Quaternion atRotation, int resolutionIterations)
        {
            if (SafeMovement)
            {
                float overlapDistance;
                Vector3 overlapNormal;

                // Try solving overlaps if any
                bool foundFinalOverlaps = false;
                int resolutionIterationsMade = 0;
                while (resolutionIterationsMade < resolutionIterations)
                {
                    int nbFinalOverlaps = CharacterCollisionsOverlap(targetPosition, atRotation, _internalProbedColliders);
                    foundFinalOverlaps = nbFinalOverlaps > 0;
                    if (foundFinalOverlaps)
                    {
                        for (int i = 0; i < nbFinalOverlaps; i++)
                        {
                            Transform probedColliderTransform = _internalProbedColliders[i].GetComponent<Transform>();
                            if (Physics.ComputePenetration(
                                CharacterCapsule,
                                targetPosition,
                                atRotation,
                                _internalProbedColliders[i],
                                probedColliderTransform.position,
                                probedColliderTransform.rotation,
                                out overlapNormal,
                                out overlapDistance))
                            {
                                targetPosition += overlapNormal * (overlapDistance + CollisionOffset);
                            }
                        }
                    }

                    resolutionIterationsMade++;
                }

                // Set final position if still not overlapping anything
                if (!foundFinalOverlaps || CharacterCollisionsOverlap(targetPosition, atRotation, _internalProbedColliders) <= 0)
                {
                    movedPosition = targetPosition;
                    return true;
                }
                return false;
            }
            else
            {
                movedPosition = targetPosition;
                return true;
            }
        }

        /// <summary>
        /// Determines if the input collider is valid for collision processing
        /// </summary>
        /// <returns> Returns true if the collider is valid </returns>
        private bool CheckIfColliderValidForCollisions(Collider coll)
        {
            if (coll == null ||
                coll == CharacterCapsule)
            {
                return false;
            }

            if (!IsColliderValidForCollisions(coll))
            {
                return false;
            }

            return true;
        }

        /// <summary>
        /// Determines if the input collider is valid for collision processing
        /// </summary>
        private bool IsColliderValidForCollisions(Collider coll)
        {
            if (RigidbodyInteractionType == RigidbodyInteractionType.Kinematic && coll.attachedRigidbody && !coll.attachedRigidbody.isKinematic)
            {
                return false;
            }

            if (_movementCalculationPhase == MovementCalculationPhase.StandingInteractiveRigidbodyVelocity && StableInteractiveRigidbody && coll.attachedRigidbody == StableInteractiveRigidbody)
            {
                return false;
            }

            if (!this.CharacterController.IsColliderValidForCollisions(coll))
            {
                return false;
            }

            return true;
        }

        /// <summary>
        /// Determines if the motor is considered stable on a given probing hit
        /// </summary>
        private bool IsStableOnHit(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint)
        {
            if (!CharacterController.CanBeStableOnCollider(hitCollider))
            {
                return false;
            }

            bool isValidStep = false;
            bool ledgeTestStable = true;
            bool stableOnNormal = IsStableOnNormal(hitNormal);

            if (StepAndLedgeHandling)
            {
                bool isValidStepForward = false;
                bool isValidStepBackward = false;
                bool ledgeDetected = false;
                Vector3 ledgeNormal = _baseVelocity.normalized;
                float distanceFromLedge = 0f;

                Vector3 characterBottom = TransientPosition + (TransientRotation * _characterTransformToCapsuleBottom);
                Vector3 characterBottomToHitVector = hitPoint - characterBottom;
                Vector3 flattenedCharacterToHitDirection = Vector3.ProjectOnPlane(characterBottomToHitVector, CharacterUp).normalized;
                
                Vector3 forwardStepRayEndPosition;
                Vector3 forwardStepRayEndNormal;
                Vector3 backwardStepRayEndPosition;
                Vector3 backwardStepRayEndNormal;
                isValidStepForward = SteppingDetectionSweep(hitPoint + (CharacterUp * SecondaryProbesVertical) + (flattenedCharacterToHitDirection * SecondaryProbesHorizontal), -CharacterUp, out forwardStepRayEndPosition, out forwardStepRayEndNormal, true);
                isValidStepBackward = SteppingDetectionSweep(hitPoint + (CharacterUp * SecondaryProbesVertical) + (-flattenedCharacterToHitDirection * SecondaryProbesHorizontal), -CharacterUp, out backwardStepRayEndPosition, out backwardStepRayEndNormal, false);

                // Should we consider stepp handling?
                float trueStepHeight = Vector3.Project(characterBottomToHitVector, CharacterUp).magnitude;
                if (trueStepHeight <= MaxStepHeight)
                {
                    isValidStep = isValidStepForward && isValidStepBackward;

                    // Anticipate extra step height resulting from slope angle
                    if (isValidStep && !stableOnNormal)
                    {
                        Vector3 bottomHemiCenter = TransientPosition + (TransientRotation * _characterTransformToCapsuleBottomHemi);
                        Vector3 characterBottomToVirtualStepContactPoint = (bottomHemiCenter + (-forwardStepRayEndNormal * CharacterCapsule.radius)) - characterBottom;

                        // Calc intersection of radius segment with step plane
                        float dotNumerator = Vector3.Dot((characterBottom - bottomHemiCenter), forwardStepRayEndNormal);
                        Vector3 characterBottomToVirtualStepContactPointOnPlane = bottomHemiCenter + (-forwardStepRayEndNormal * -dotNumerator);
                        characterBottomToVirtualStepContactPointOnPlane = characterBottomToVirtualStepContactPointOnPlane - characterBottom;
                        float extraHeight = (characterBottomToVirtualStepContactPointOnPlane - characterBottomToVirtualStepContactPoint).magnitude / Mathf.Cos(Mathf.Deg2Rad * Vector3.Angle(forwardStepRayEndNormal, CharacterUp));
                        trueStepHeight = trueStepHeight + extraHeight + ExtraStepHeightPadding;

                        isValidStep = isValidStep && trueStepHeight <= MaxStepHeight;
                    }
                }

                // Ledge handling
                ledgeDetected = isValidStepForward != isValidStepBackward;
                if (ledgeDetected)
                {
                    // Find ledge normal
                    RaycastHit ledgeHit;
                    if (isValidStepForward && !isValidStepBackward)
                    {
                        Vector3 ledgeRayEndToStableEnd = (forwardStepRayEndPosition - backwardStepRayEndPosition);
                        if (Physics.Raycast(backwardStepRayEndPosition, ledgeRayEndToStableEnd.normalized, out ledgeHit, ledgeRayEndToStableEnd.magnitude, CollidableLayers, QueryTriggerInteraction.Ignore))
                        {
                            ledgeNormal = ledgeHit.normal;
                        }
                        else
                        {
                            ledgeNormal = -ledgeRayEndToStableEnd.normalized;
                        }
                    }
                    else if (!isValidStepForward && isValidStepBackward)
                    {
                        Vector3 ledgeRayEndToStableEnd = (backwardStepRayEndPosition - forwardStepRayEndPosition);
                        if (Physics.Raycast(forwardStepRayEndPosition, ledgeRayEndToStableEnd.normalized, out ledgeHit, ledgeRayEndToStableEnd.magnitude, CollidableLayers, QueryTriggerInteraction.Ignore))
                        {
                            ledgeNormal = ledgeHit.normal;
                        }
                        else
                        {
                            ledgeNormal = -ledgeRayEndToStableEnd.normalized;
                        }
                    }
                    distanceFromLedge = Vector3.ProjectOnPlane((hitPoint - characterBottom), CharacterUp).magnitude;
                }

                // Final result
                ledgeTestStable = !ledgeDetected 
                    || (Vector3.Project(_baseVelocity, ledgeNormal).magnitude < MaxStableOnLedgeVelocity 
                        && distanceFromLedge < MaxStableDistanceFromLedge);
            }

            if ((stableOnNormal || isValidStep) && ledgeTestStable)
            {
                return true;
            }
            return false;
        }

        /// <summary>
        /// Method used by grounding evaluation to detect valid steps
        /// </summary>
        private bool SteppingDetectionSweep(Vector3 fromPoint, Vector3 initialDirection, out Vector3 endPosition, out Vector3 endNormal, bool mustBeValidOnFirstHit)
        {
            RaycastHit hit;
            bool sweepIsOver = false;
            int sweepsMade = 0;
            Vector3 sweepPosition = fromPoint;
            Vector3 sweepDirection = initialDirection;
            float sweepDistanceRemaining = SecondaryProbesVertical + Mathf.Max(SecondaryProbesVertical, MaxStepHeight);
            endPosition = sweepPosition;
            endNormal = _cachedZeroVector;
            while (sweepDistanceRemaining > 0f && (sweepsMade <= MaxSteppingSweepIterations) && !sweepIsOver)
            {
                if (Physics.Raycast(sweepPosition, sweepDirection, out hit, sweepDistanceRemaining, CollidableLayers, QueryTriggerInteraction.Ignore))
                {
                    if (IsStableOnNormal(hit.normal))
                    {
                        endPosition = hit.point;
                        endNormal = hit.normal;
                        return true;
                    }
                    else
                    {
                        if (mustBeValidOnFirstHit)
                        {
                            endPosition = sweepPosition + (sweepDirection * sweepDistanceRemaining);
                            sweepIsOver = true;
                        }
                        else
                        {
                            // Calculate movement from this iteration and advance position
                            Vector3 sweepMovement = (sweepDirection * hit.distance) + (-initialDirection * Mathf.Clamp(CollisionOffset, 0f, hit.distance));
                            sweepPosition = sweepPosition + sweepMovement;
                            endPosition = sweepPosition;
                            sweepDistanceRemaining = Mathf.Clamp(sweepDistanceRemaining - sweepMovement.magnitude, 0f, Mathf.Infinity);

                            // Reorient direction
                            sweepDirection = Vector3.ProjectOnPlane(sweepDirection, hit.normal).normalized;
                        }
                    }
                }
                else
                {
                    endPosition = sweepPosition + (sweepDirection * sweepDistanceRemaining);
                    sweepIsOver = true;
                }

                sweepsMade++;
            }

            return false;
        }

        /// <summary>
        /// Get true linear velocity (taking into account rotational velocity) on a given point of a rigidbody
        /// </summary>
        private Vector3 GetTotalVelocityFromInteractiveRigidbody(Rigidbody interactiveRigidbody, Vector3 atPoint, float deltaTime)
        {
            if (deltaTime > 0f)
            {
                Vector3 effectiveMoverVelocity = interactiveRigidbody.velocity;

                if (interactiveRigidbody.angularVelocity != Vector3.zero)
                {
                    Vector3 centerOfRotation = interactiveRigidbody.position;
                    Vector3 centerOfRotationToPoint = atPoint - centerOfRotation;
                    Quaternion rotationFromInteractiveRigidbody = Quaternion.Euler(Mathf.Rad2Deg * interactiveRigidbody.angularVelocity * deltaTime);
                    Vector3 finalPointPosition = centerOfRotation + (rotationFromInteractiveRigidbody * centerOfRotationToPoint);
                    effectiveMoverVelocity += (finalPointPosition - atPoint) / deltaTime;
                }
                return effectiveMoverVelocity;
            }
            else
            {
                return Vector3.zero;
            }
        }

        private void MultiplyVelocityAndMovementMagnitude(float multiplicator, ref Vector3 remainingMovement)
        {
            if (_velocityForCurrentPhase.sqrMagnitude > 0f)
            {
                _velocityForCurrentPhase = _velocityForCurrentPhase * multiplicator;
            }

            if (remainingMovement.sqrMagnitude > 0)
            {
                remainingMovement = remainingMovement * multiplicator;
            }
        }

        private void ProjectVelocityAndMovement(Vector3 onNormal, ref Vector3 remainingMovement)
        {
            if (_velocityForCurrentPhase.sqrMagnitude > 0f)
            {
                _velocityForCurrentPhase = Vector3.Project(_velocityForCurrentPhase, onNormal);
            }

            if (remainingMovement.sqrMagnitude > 0)
            {
                remainingMovement = Vector3.Project(remainingMovement, onNormal);
            }
        }

        private void ProjectVelocityAndMovementOnPlane(Vector3 onPlaneNormal, ref Vector3 remainingMovement)
        {
            if (_velocityForCurrentPhase.sqrMagnitude > 0f)
            {
                _velocityForCurrentPhase = Vector3.ProjectOnPlane(_velocityForCurrentPhase, onPlaneNormal);
            }

            if (remainingMovement.sqrMagnitude > 0)
            {
                remainingMovement = Vector3.ProjectOnPlane(remainingMovement, onPlaneNormal);
            }
        }

        private void ReorientVelocityAndMovementOnSlope(Vector3 slopeNormal, ref Vector3 remainingMovement)
        {
            if (_velocityForCurrentPhase.sqrMagnitude > 0f)
            {
                _velocityForCurrentPhase = GetDirectionTangentToSurface(_velocityForCurrentPhase, slopeNormal) * _velocityForCurrentPhase.magnitude;
            }

            if (remainingMovement.sqrMagnitude > 0)
            {
                remainingMovement = GetDirectionTangentToSurface(remainingMovement, slopeNormal) * remainingMovement.magnitude;
            }
        }

        /// <summary>
        /// Determines if a collider has an attached interactive rigidbody
        /// </summary>
        private Rigidbody GetInteractiveRigidbody(Collider onCollider)
        {
            if (onCollider.attachedRigidbody)
            {
                if (onCollider.attachedRigidbody.gameObject.GetComponent<PhysicsMover>())
                {
                    return onCollider.attachedRigidbody;
                }

                if (RigidbodyInteractionType == RigidbodyInteractionType.SimulatedDynamic && !onCollider.attachedRigidbody.isKinematic)
                {
                    return onCollider.attachedRigidbody;
                }
            }
            return null;
        }

        /// <summary>
        /// Calculates the velocity required to move the character to the target position over a specific deltaTime.
        /// Useful for when you wish to work with positions rather than velocities in the UpdateVelocity callback of BaseCharacterController
        /// </summary>
        public Vector3 GetVelocityForMovePosition(Vector3 toPosition, float deltaTime)
        {
            if (deltaTime > 0)
            {
                return (toPosition - TransientPosition) / deltaTime;
            }
            else
            {
                return Vector3.zero;
            }
        }

        /// <summary>
        /// Detect if the character capsule is overlapping with anything collidable
        /// </summary>
        /// <returns> Returns number of overlaps </returns>
        public int CharacterCollisionsOverlap(Vector3 atPosition, Quaternion atRotation, Collider[] overlappedColliders)
        {
            int nbHits = 0;
            int nbUnfilteredHits = Physics.OverlapCapsuleNonAlloc(
                        atPosition + (atRotation * _characterTransformToCapsuleBottomHemi),
                        atPosition + (atRotation * _characterTransformToCapsuleTopHemi),
                        CharacterCapsule.radius,
                        overlappedColliders,
                        CollidableLayers,
                        QueryTriggerInteraction.Ignore);

            // Filter out invalid colliders
            nbHits = nbUnfilteredHits;
            for (int i = 0; i < nbUnfilteredHits; i++)
            {
                if (!CheckIfColliderValidForCollisions(overlappedColliders[i]))
                {
                    nbHits--;
                    if (i < nbHits)
                    {
                        overlappedColliders[i] = overlappedColliders[nbHits];
                    }
                }
            }

            return nbHits;
        }

        /// <summary>
        /// Detect if the character capsule is overlapping with anything
        /// </summary>
        /// <returns> Returns number of overlaps </returns>
        public int CharacterOverlap(Vector3 atPosition, Quaternion atRotation, Collider[] overlappedColliders, LayerMask layers, QueryTriggerInteraction triggerInteraction)
        {
            int nbHits = 0;
            int nbUnfilteredHits = Physics.OverlapCapsuleNonAlloc(
                        atPosition + (atRotation * _characterTransformToCapsuleBottomHemi),
                        atPosition + (atRotation * _characterTransformToCapsuleTopHemi),
                        CharacterCapsule.radius,
                        overlappedColliders,
                        layers,
                        triggerInteraction);

            // Filter out the character capsule itself
            nbHits = nbUnfilteredHits;
            for (int i = 0; i < nbUnfilteredHits; i++)
            {
                if (overlappedColliders[i] == CharacterCapsule)
                {
                    nbHits--;
                    if (i < nbHits)
                    {
                        overlappedColliders[i] = overlappedColliders[nbHits];
                    }
                }
            }

            return nbHits;
        }

        /// <summary>
        /// Sweeps the capsule's volume to detect collision hits
        /// </summary>
        /// <returns> Returns the number of hits </returns>
        public int CharacterCollisionsSweep(Vector3 position, Quaternion rotation, Vector3 direction, float distance, out RaycastHit closestHit, RaycastHit[] hits)
        {
            direction.Normalize();
            closestHit = new RaycastHit();

            // Capsule cast
            int nbHits = 0;
            int nbUnfilteredHits = Physics.CapsuleCastNonAlloc(
                position + (rotation * _characterTransformToCapsuleBottomHemi) - (direction * SweepProbingBackstepDistance),
                position + (rotation * _characterTransformToCapsuleTopHemi) - (direction * SweepProbingBackstepDistance),
                CharacterCapsule.radius,
                direction,
                hits,
                distance + SweepProbingBackstepDistance,
                CollidableLayers,
                QueryTriggerInteraction.Ignore);

            // Hits filter
            float closestDistance = Mathf.Infinity;
            nbHits = nbUnfilteredHits;
            for (int i = 0; i < nbUnfilteredHits; i++)
            {
                // Filter out the invalid hits
                if (hits[i].distance <= 0f ||
                    !CheckIfColliderValidForCollisions(hits[i].collider))
                {
                    nbHits--;
                    if (i < nbHits)
                    {
                        hits[i] = hits[nbHits];
                    }
                }
                else
                {
                    // Remember closest valid hit
                    if (hits[i].distance < closestDistance)
                    {
                        closestHit = hits[i];
                        closestHit.distance -= SweepProbingBackstepDistance;
                        closestDistance = hits[i].distance;
                    }
                }
            }

            return nbHits;
        }

        /// <summary>
        /// Sweeps the capsule's volume to detect hits
        /// </summary>
        /// <returns> Returns the number of hits </returns>
        public int CharacterSweep(Vector3 position, Quaternion rotation, Vector3 direction, float distance, out RaycastHit closestHit, RaycastHit[] hits, LayerMask layers, QueryTriggerInteraction triggerInteraction)
        {
            direction.Normalize();
            closestHit = new RaycastHit();

            // Capsule cast
            int nbHits = 0;
            int nbUnfilteredHits = Physics.CapsuleCastNonAlloc(
                position + (rotation * _characterTransformToCapsuleBottomHemi),
                position + (rotation * _characterTransformToCapsuleTopHemi),
                CharacterCapsule.radius,
                direction,
                hits,
                distance,
                layers,
                triggerInteraction);

            // Hits filter
            float closestDistance = Mathf.Infinity;
            nbHits = nbUnfilteredHits;
            for (int i = 0; i < nbUnfilteredHits; i++)
            {
                // Filter out the character capsule
                if (hits[i].distance <= 0f || hits[i].collider == CharacterCapsule)
                {
                    nbHits--;
                    if (i < nbHits)
                    {
                        hits[i] = hits[nbHits];
                    }
                }
                else
                {
                    // Remember closest valid hit
                    if (hits[i].distance < closestDistance)
                    {
                        closestHit = hits[i];
                        closestDistance = hits[i].distance;
                    }
                }
            }

            return nbHits;
        }

        /// <summary>
        /// Casts a sphere in the character's downward direction to detect ground
        /// </summary>
        /// <returns> Returns the number of hits </returns>
        private bool CharacterGroundSweep(Vector3 position, Quaternion rotation, Vector3 direction, float distance, out RaycastHit closestHit)
        {
            direction.Normalize();
            closestHit = new RaycastHit();

            // Capsule cast
            int nbUnfilteredHits = Physics.CapsuleCastNonAlloc(
                position + (rotation * _characterTransformToCapsuleBottomHemi) - (direction * GroundProbingBackstepDistance),
                position + (rotation * _characterTransformToCapsuleTopHemi) - (direction * GroundProbingBackstepDistance),
                CharacterCapsule.radius,
                direction,
                _internalCharacterHits,
                distance + GroundProbingBackstepDistance,
                CollidableLayers,
                QueryTriggerInteraction.Ignore);

            // Hits filter
            bool foundValidHit = false;
            float closestDistance = Mathf.Infinity;
            for (int i = 0; i < nbUnfilteredHits; i++)
            {
                // Find the closest valid hit
                if (_internalCharacterHits[i].distance > 0f && CheckIfColliderValidForCollisions(_internalCharacterHits[i].collider))
                {
                    if (_internalCharacterHits[i].distance < closestDistance)
                    {
                        closestHit = _internalCharacterHits[i];
                        closestHit.distance -= GroundProbingBackstepDistance;
                        closestDistance = _internalCharacterHits[i].distance;

                        foundValidHit = true;
                    }
                }
            }

            return foundValidHit;
        }
    }
}