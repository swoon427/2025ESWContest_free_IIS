using UnityEngine;

public class FingerIK : MonoBehaviour
{
    [System.Serializable]
    public class Hinge
    {
        public Transform pivot;
        public Vector3 localAxis = Vector3.right; // pivot1=Vector3.up, pivot2/3=Vector3.right
        public float minDeg = -90f;
        public float maxDeg = 90f;

        [HideInInspector] public Quaternion restLocal; // 기준 로컬 회전
        [HideInInspector] public float angleDeg;       // 기준 대비 누적 힌지각(현재 상태)
    }

    public Transform targetObject;
    public Transform endEffector;

    // 피벗 정의 (pivot1=Y, pivot2/3=X)
    public Hinge hinge1 = new Hinge { localAxis = Vector3.up, minDeg = -45f, maxDeg = 45f };
    public Hinge hinge2 = new Hinge { localAxis = Vector3.right, minDeg = 0f, maxDeg = 90f };
    public Hinge hinge3 = new Hinge { localAxis = Vector3.right, minDeg = 0f, maxDeg = 90f };

    [Header("Solver")]
    [Range(1, 50)] public int iterations = 12;
    public float threshold = 0.003f;

    [Header("Stability")]
    [Tooltip("관절당 1회 루프에서 허용하는 최대 회전각(도)")]
    public float maxDegPerIter = 6f;
    [Tooltip("관절 속도 제한(도/초). 0 이하면 비활성")]
    public float maxDegPerSecond = 360f;

    [Header("EMA Smoothing")]
    [Tooltip("관절각 EMA 시간상수(초). 0 이하면 스무딩 끔")]
    public float angleTau = 0.12f;
    [Tooltip("타겟 위치 EMA(옵션) 시간상수(초). 0 이하면 스무딩 끔")]
    public float targetTau = 0.08f;

    [Header("Unreachable Target")]
    public bool projectUnreachable = true;
    [Range(0.90f, 1.00f)] public float reachSlack = 0.995f;

    // 내부 상태(타겟 위치 EMA)
    Vector3 _smoothedTarget;
    bool _targetInit = false;

    void Awake()
    {
        // 기준 포즈 저장
        if (!Validate()) return;
        hinge1.restLocal = hinge1.pivot.localRotation; hinge1.angleDeg = 0f;
        hinge2.restLocal = hinge2.pivot.localRotation; hinge2.angleDeg = 0f;
        hinge3.restLocal = hinge3.pivot.localRotation; hinge3.angleDeg = 0f;
    }

    bool Validate()
    {
        return targetObject && endEffector &&
               hinge1.pivot && hinge2.pivot && hinge3.pivot;
    }

    void LateUpdate()
    {
        if (!Validate()) return;
        SolveIK(Time.deltaTime);
    }

    void SolveIK(float dt)
    {
        // 0) 타겟 위치 EMA
        Vector3 targetRaw = targetObject.position;
        if (!_targetInit) { _smoothedTarget = targetRaw; _targetInit = true; }
        float aPos = AlphaFromTau(targetTau, dt);              // α = 1 - exp(-dt/τ)
        _smoothedTarget = EmaVec(_smoothedTarget, targetRaw, aPos);

        // 1) 도달 불가면 reach 경계로 투영
        Vector3 targetPos = projectUnreachable ? ProjectTargetToReach(_smoothedTarget)
                                               : _smoothedTarget;

        float thr2 = threshold * threshold;

        // 2) CCD 루프
        for (int it = 0; it < iterations; it++)
        {
            if ((endEffector.position - targetPos).sqrMagnitude < thr2) break;

            RotateHingeTowards(hinge3, targetPos, dt); // Distal
            RotateHingeTowards(hinge2, targetPos, dt); // Middle
            RotateHingeTowards(hinge1, targetPos, dt); // Proximal
        }
    }

    void RotateHingeTowards(Hinge h, Vector3 target, float dt)
    {
        Transform pivot = h.pivot;

        // 힌지 월드축
        Vector3 worldAxis = pivot.TransformDirection(h.localAxis);

        // pivot 기준 벡터
        Vector3 toEnd = endEffector.position - pivot.position;
        Vector3 toTarget = target - pivot.position;

        // 힌지축 직교 평면 투영
        Vector3 v1 = Vector3.ProjectOnPlane(toEnd, worldAxis);
        Vector3 v2 = Vector3.ProjectOnPlane(toTarget, worldAxis);
        if (v1.sqrMagnitude < 1e-12f || v2.sqrMagnitude < 1e-12f) return;

        // 목표 회전량(부호 포함)
        float deltaWanted = Vector3.SignedAngle(v1, v2, worldAxis);

        // per-iteration 스텝 캡
        deltaWanted = Mathf.Clamp(deltaWanted, -maxDegPerIter, maxDegPerIter);

        // 속도 제한(도/초)
        if (maxDegPerSecond > 0f)
        {
            float maxStep = maxDegPerSecond * dt;
            deltaWanted = Mathf.Clamp(deltaWanted, -maxStep, maxStep);
        }

        // 리밋 안으로 목표각 제안
        float proposed = Mathf.Clamp(h.angleDeg + deltaWanted, h.minDeg, h.maxDeg);

        // 관절각 EMA 적용 (래핑 안전: LerpAngle)
        float aAng = AlphaFromTau(angleTau, dt);              // α = 1 - exp(-dt/τ)
        float smoothed = EmaAngle(h.angleDeg, proposed, aAng);

        // (안전) 최종 리밋
        h.angleDeg = Mathf.Clamp(smoothed, h.minDeg, h.maxDeg);

        // 절대 로컬 회전 재구성: restLocal * AngleAxis(hinge angle)
        Quaternion q = Quaternion.AngleAxis(h.angleDeg, h.localAxis); // 로컬축 기준 회전
        h.pivot.localRotation = h.restLocal * q;
    }

    // ===== EMA / 유틸 =====
    static float AlphaFromTau(float tau, float dt)
    {
        if (tau <= 0f) return 1f;                    // 스무딩 끔
        // 1차 로우패스의 연속해: α = 1 - exp(-dt/τ)
        return 1f - Mathf.Exp(-dt / Mathf.Max(1e-6f, tau));
    }

    static Vector3 EmaVec(Vector3 prev, Vector3 x, float a)
    {
        return prev + a * (x - prev);               // (1-α)prev + αx
    }

    static float EmaAngle(float current, float target, float a)
    {
        // LerpAngle은 래핑/최단경로 보장
        return Mathf.LerpAngle(current, target, Mathf.Clamp01(a));
    }

    // ===== 도달 반경 투영/길이 =====
    Vector3 ProjectTargetToReach(Vector3 target)
    {
        Vector3 root = hinge1.pivot.position;
        float reach = GetReach();
        float d = Vector3.Distance(root, target);
        if (d > reach)
        {
            Vector3 dir = (target - root).normalized;
            return root + dir * (reach * reachSlack);
        }
        return target;
    }

    float GetReach()
    {
        float l1 = Vector3.Distance(hinge1.pivot.position, hinge2.pivot.position);
        float l2 = Vector3.Distance(hinge2.pivot.position, hinge3.pivot.position);
        float l3 = Vector3.Distance(hinge3.pivot.position, endEffector.position);
        return l1 + l2 + l3;
    }

#if UNITY_EDITOR
    void OnDrawGizmosSelected()
    {
        if (!hinge1.pivot || !hinge2.pivot || !hinge3.pivot || !endEffector) return;
        Gizmos.color = new Color(0.2f, 0.6f, 1f, 0.20f);
        Gizmos.DrawWireSphere(hinge1.pivot.position, GetReach());

        // EMA 타겟 디버그
        if (_targetInit)
        {
            Gizmos.color = new Color(1f, 0.4f, 0.2f, 0.6f);
            Gizmos.DrawWireSphere(_smoothedTarget, 0.005f);
        }
    }
#endif
}


//using UnityEngine;

//public class FingerIK : MonoBehaviour
//{
//    [System.Serializable]
//    public class Hinge
//    {
//        public Transform pivot;
//        public Vector3 localAxis = Vector3.right;   // pivot1은 Vector3.up, pivot2/3은 Vector3.right
//        public float minDeg = -90f;                 // 리밋(부호 기준은 로컬축 기준)
//        public float maxDeg = 90f;

//        [HideInInspector] public Quaternion restLocal; // 시작 시점의 로컬 회전(기준)
//        [HideInInspector] public float angleDeg;       // 기준 대비 누적 힌지각(현재 상태)
//    }

//    public Transform targetObject;
//    public Transform endEffector;

//    // 피벗 정의 (pivot1=Y, pivot2/3=X)
//    public Hinge hinge1 = new Hinge { localAxis = Vector3.up, minDeg = -45f, maxDeg = 45f };
//    public Hinge hinge2 = new Hinge { localAxis = Vector3.right, minDeg = -90f, maxDeg = 90f };
//    public Hinge hinge3 = new Hinge { localAxis = Vector3.right, minDeg = -90f, maxDeg = 90f };

//    [Header("Solver")]
//    [Range(1, 50)] public int iterations = 12;
//    public float threshold = 0.003f;
//    [Tooltip("관절당 1회 루프에서 허용하는 최대 회전각(도)")]
//    public float maxDegPerIter = 6f;
//    [Tooltip("회전 적용 감쇠(0~1), 1=즉시 적용")]
//    [Range(0.05f, 1f)] public float rotationLerp = 0.6f;

//    [Header("Unreachable Target")]
//    public bool projectUnreachable = true;
//    [Range(0.90f, 1.00f)] public float reachSlack = 0.995f;

//    void Awake()
//    {
//        // 바인드 포즈 저장(리밋 기준). 에디터에서 각 pivot을 할당해 주세요.
//        if (!Validate()) return;
//        hinge1.restLocal = hinge1.pivot.localRotation; hinge1.angleDeg = 0f;
//        hinge2.restLocal = hinge2.pivot.localRotation; hinge2.angleDeg = 0f;
//        hinge3.restLocal = hinge3.pivot.localRotation; hinge3.angleDeg = 0f;
//    }

//    bool Validate()
//    {
//        return targetObject && endEffector &&
//               hinge1.pivot && hinge2.pivot && hinge3.pivot;
//    }

//    void LateUpdate()
//    {
//        if (!Validate()) return;
//        SolveIK();
//    }

//    void SolveIK()
//    {
//        Vector3 targetPos = targetObject.position;
//        if (projectUnreachable) targetPos = ProjectTargetToReach(targetPos);

//        float thr2 = threshold * threshold;

//        for (int it = 0; it < iterations; it++)
//        {
//            if ((endEffector.position - targetPos).sqrMagnitude < thr2) break;

//            // 끝 → 시작 순서 (CCD)
//            RotateHingeTowards(hinge3, targetPos);
//            RotateHingeTowards(hinge2, targetPos);
//            RotateHingeTowards(hinge1, targetPos);
//        }
//    }

//    void RotateHingeTowards(Hinge h, Vector3 target)
//    {
//        Transform pivot = h.pivot;

//        // 힌지 월드축 (현 프레임 상태에서)
//        Vector3 worldAxis = pivot.TransformDirection(h.localAxis);

//        // pivot 기준 벡터
//        Vector3 toEnd = endEffector.position - pivot.position;
//        Vector3 toTarget = target - pivot.position;

//        // 힌지축 직교 평면으로 투영
//        Vector3 v1 = Vector3.ProjectOnPlane(toEnd, worldAxis);
//        Vector3 v2 = Vector3.ProjectOnPlane(toTarget, worldAxis);
//        if (v1.sqrMagnitude < 1e-12f || v2.sqrMagnitude < 1e-12f) return;

//        // 그 평면에서의 목표 각도 변화량
//        float delta = Vector3.SignedAngle(v1, v2, worldAxis);
//        delta = Mathf.Clamp(delta, -maxDegPerIter, maxDegPerIter);
//        delta *= rotationLerp;

//        // ★ 리밋은 "적용 전에" 누적 각에 반영
//        float next = Mathf.Clamp(h.angleDeg + delta, h.minDeg, h.maxDeg);
//        if (Mathf.Abs(next - h.angleDeg) < 1e-4f) return; // 변화 없음

//        h.angleDeg = next;

//        // ★ 항상 기준 회전(restLocal)에서 "한 축"만 반영해 절대 로컬 회전 재구성
//        Quaternion q = Quaternion.AngleAxis(h.angleDeg, h.localAxis); // 로컬축 기준
//        h.pivot.localRotation = h.restLocal * q;
//    }

//    Vector3 ProjectTargetToReach(Vector3 target)
//    {
//        Vector3 root = hinge1.pivot.position;
//        float d = Vector3.Distance(root, target);
//        float reach = GetReach();
//        if (d > reach)
//        {
//            Vector3 dir = (target - root).normalized;
//            return root + dir * (reach * reachSlack);
//        }
//        return target;
//    }

//    float GetReach()
//    {
//        float l1 = Vector3.Distance(hinge1.pivot.position, hinge2.pivot.position);
//        float l2 = Vector3.Distance(hinge2.pivot.position, hinge3.pivot.position);
//        float l3 = Vector3.Distance(hinge3.pivot.position, endEffector.position);
//        return l1 + l2 + l3;
//    }

//#if UNITY_EDITOR
//    void OnDrawGizmosSelected()
//    {
//        if (!hinge1.pivot || !hinge2.pivot || !hinge3.pivot || !endEffector) return;
//        Gizmos.color = new Color(0.2f, 0.6f, 1f, 0.2f);
//        Gizmos.DrawWireSphere(hinge1.pivot.position, GetReach());
//    }
//#endif
//}


//using UnityEngine;

//public class FingerIK : MonoBehaviour
//{
//    public Transform targetObject;

//    // 회전 기준 pivot (힌지축: pivot1=Y, pivot2/3=X)
//    public Transform pivot1;  // Index_Pivot1 (Y 축 회전)
//    public Transform pivot2;  // Index_Pivot2 (X 축 회전)
//    public Transform pivot3;  // Index_Pivot3 (X 축 회전)

//    public Transform endEffector;

//    [Range(1, 50)] public int iterations = 10;
//    [Tooltip("목표와 EE 사이가 이 값 미만이면 조기 종료")]
//    public float threshold = 0.005f;

//    [Header("Stability (optional)")]
//    [Tooltip("관절당 1회 루프에서 허용하는 최대 회전각(도)")]
//    public float maxDegPerIter = 5f;
//    [Tooltip("회전 감쇠(0~1). 1=즉시 적용, 0.2~0.6 권장")]
//    [Range(0.05f, 1f)] public float rotationLerp = 0.5f;
//    [Tooltip("도달 불가 타겟을 체인 reach 반경으로 투영")]
//    public bool projectUnreachable = true;
//    [Range(0.90f, 1.00f)] public float reachSlack = 0.995f; // reach*slack까지 허용

//    [Header("Joint Limits (deg) — optional")]
//    public bool useJointLimits = false;
//    public Vector2 pivot1YLimit = new Vector2(-45f, 45f);
//    public Vector2 pivot2XLimit = new Vector2(0f, 90f);
//    public Vector2 pivot3XLimit = new Vector2(0f, 90f);

//    void LateUpdate()
//    {
//        SolveIK();
//    }

//    void SolveIK()
//    {
//        if (!targetObject || !pivot1 || !pivot2 || !pivot3 || !endEffector) return;

//        // 1) 타겟 위치 준비 (도달 불가면 reach 경계로 투영)
//        Vector3 targetPos = targetObject.position;
//        if (projectUnreachable) targetPos = ProjectTargetToReach(targetPos);

//        float thr2 = threshold * threshold;

//        // 2) CCD: 끝→시작 순서 (pivot3, pivot2, pivot1)
//        for (int i = 0; i < iterations; i++)
//        {
//            if ((endEffector.position - targetPos).sqrMagnitude < thr2) break;

//            RotateHingeTowards(pivot3, targetPos, Vector3.right, maxDegPerIter, rotationLerp); // X
//            RotateHingeTowards(pivot2, targetPos, Vector3.right, maxDegPerIter, rotationLerp); // X
//            RotateHingeTowards(pivot1, targetPos, Vector3.up, maxDegPerIter, rotationLerp); // Y

//            if (useJointLimits) ApplyJointLimits();
//        }
//    }

//    // === 핵심: pivot 로컬축 → 월드축으로 변환 후, 그 축 "주변으로만" 회전 ===
//    void RotateHingeTowards(Transform pivot, Vector3 target, Vector3 localAxis, float maxDeg, float lerp)
//    {
//        // 1) 회전 축: pivot의 로컬축을 월드축으로
//        Vector3 worldAxis = pivot.TransformDirection(localAxis);

//        // 2) pivot 기준의 EE/타겟 방향 벡터
//        Vector3 toEnd = endEffector.position - pivot.position;
//        Vector3 toTarget = target - pivot.position;

//        // 3) 힌지 축에 수직인 평면으로 투영(회전 기준 축은 그대로 유지)
//        Vector3 v1 = Vector3.ProjectOnPlane(toEnd, worldAxis);
//        Vector3 v2 = Vector3.ProjectOnPlane(toTarget, worldAxis);
//        if (v1.sqrMagnitude < 1e-12f || v2.sqrMagnitude < 1e-12f) return;

//        // 4) 그 평면에서의 부호있는 각도
//        float angle = Vector3.SignedAngle(v1, v2, worldAxis);

//        // 5) per-iter 각도 클램프 + 감쇠
//        angle = Mathf.Clamp(angle, -maxDeg, maxDeg);
//        if (Mathf.Abs(angle) < 1e-4f) return;

//        // 6) 예시 코드와 동일한 기준: "월드축" 주변 회전
//        pivot.Rotate(worldAxis, angle * lerp, Space.World);  // ★ 회전 기준 일치
//    }

//    // 도달 불가 타겟 투영
//    Vector3 ProjectTargetToReach(Vector3 target)
//    {
//        Vector3 root = pivot1.position;
//        float d = Vector3.Distance(root, target);
//        float reach = GetReach();
//        if (d > reach)
//        {
//            Vector3 dir = (target - root).normalized;
//            return root + dir * (reach * reachSlack);
//        }
//        return target;
//    }

//    float GetReach()
//    {
//        float l1 = Vector3.Distance(pivot1.position, pivot2.position);
//        float l2 = Vector3.Distance(pivot2.position, pivot3.position);
//        float l3 = Vector3.Distance(pivot3.position, endEffector.position);
//        return l1 + l2 + l3;
//    }

//    void ApplyJointLimits()
//    {
//        // pivot1: Y축 힌지
//        Vector3 e1 = pivot1.localEulerAngles;
//        e1.y = ClampSigned(e1.y, pivot1YLimit.x, pivot1YLimit.y);
//        pivot1.localEulerAngles = new Vector3(e1.x, e1.y, e1.z);

//        // pivot2: X축 힌지
//        Vector3 e2 = pivot2.localEulerAngles;
//        e2.x = ClampSigned(e2.x, pivot2XLimit.x, pivot2XLimit.y);
//        pivot2.localEulerAngles = new Vector3(e2.x, e2.y, e2.z);

//        // pivot3: X축 힌지
//        Vector3 e3 = pivot3.localEulerAngles;
//        e3.x = ClampSigned(e3.x, pivot3XLimit.x, pivot3XLimit.y);
//        pivot3.localEulerAngles = new Vector3(e3.x, e3.y, e3.z);
//    }

//    static float ClampSigned(float angleDeg, float minDeg, float maxDeg)
//    {
//        float a = NormalizeSigned(angleDeg);
//        return Mathf.Clamp(a, minDeg, maxDeg);
//    }
//    static float NormalizeSigned(float a)
//    {
//        a %= 360f;
//        if (a > 180f) a -= 360f;
//        if (a < -180f) a += 360f;
//        return a;
//    }

//#if UNITY_EDITOR
//    void OnDrawGizmosSelected()
//    {
//        if (!pivot1 || !pivot2 || !pivot3 || !endEffector) return;
//        Gizmos.color = new Color(0.2f, 0.6f, 1f, 0.2f);
//        Gizmos.DrawWireSphere(pivot1.position, GetReach());
//    }
//#endif
//}


//using UnityEngine;

//public class FingerIK : MonoBehaviour
//{
//    public Transform targetObject;

//    // 회전 기준이 될 pivot (힌지 축: pivot1=Y, pivot2/3=X)
//    public Transform pivot1;  // Index_Pivot1 (Y축 회전)
//    public Transform pivot2;  // Index_Pivot2 (X축 회전)
//    public Transform pivot3;  // Index_Pivot3 (X축 회전)

//    public Transform endEffector;

//    [Range(1, 50)]
//    public int iterations = 10;

//    [Header("Stability")]
//    [Tooltip("목표와 EE 사이가 이 값 미만이면 조기 종료")]
//    public float threshold = 0.005f;
//    [Tooltip("관절당 1회전 루프에서 허용하는 최대 회전각(도)")]
//    public float maxDegPerIter = 5f;
//    [Tooltip("회전 적용 감쇠(0~1). 1=즉시 적용, 0.2~0.6 권장")]
//    [Range(0.05f, 1f)] public float rotationLerp = 0.5f;

//    [Header("Unreachable Target Handling")]
//    [Tooltip("도달 불가 타겟을 reach 반경으로 투영")]
//    public bool projectUnreachable = true;
//    [Tooltip("reach의 몇 %까지 허용할지(1보다 약간 작게)")]
//    [Range(0.90f, 1.00f)] public float reachSlack = 0.995f;

//    [Header("Joint Limits (deg) — Optional")]
//    public bool useJointLimits = false;
//    public Vector2 pivot1YLimit = new Vector2(-45f, 45f);
//    public Vector2 pivot2XLimit = new Vector2(0f, 90f);
//    public Vector2 pivot3XLimit = new Vector2(0f, 90f);

//    void LateUpdate()
//    {
//        SolveIK();
//    }

//    void SolveIK()
//    {
//        if (!targetObject || !pivot1 || !pivot2 || !pivot3 || !endEffector) return;

//        Vector3 targetPos = targetObject.position;

//        // 1) 도달 불가 타겟 투영 (지터 억제)
//        if (projectUnreachable)
//            targetPos = ProjectTargetToReach(targetPos);

//        float thr2 = threshold * threshold;

//        // 2) CCD 루프
//        for (int i = 0; i < iterations; i++)
//        {
//            if ((endEffector.position - targetPos).sqrMagnitude < thr2) break;

//            RotateHingeTowards(pivot3, targetPos, Vector3.right, maxDegPerIter, rotationLerp); // 끝 관절 - X축
//            RotateHingeTowards(pivot2, targetPos, Vector3.right, maxDegPerIter, rotationLerp); // 중간 관절 - X축
//            RotateHingeTowards(pivot1, targetPos, Vector3.up, maxDegPerIter, rotationLerp); // 첫 관절 - Y축

//            if (useJointLimits) ApplyJointLimits();
//        }
//    }

//    // 루트~타겟 거리가 reach보다 크면 reach*slack 지점으로 투영
//    Vector3 ProjectTargetToReach(Vector3 target)
//    {
//        Vector3 root = pivot1.position;
//        float reach = GetReach();
//        float d = Vector3.Distance(root, target);
//        if (d > reach)
//        {
//            Vector3 dir = (target - root).normalized;
//            return root + dir * (reach * reachSlack);
//        }
//        return target;
//    }

//    float GetReach()
//    {
//        float l1 = Vector3.Distance(pivot1.position, pivot2.position);
//        float l2 = Vector3.Distance(pivot2.position, pivot3.position);
//        float l3 = Vector3.Distance(pivot3.position, endEffector.position);
//        return l1 + l2 + l3;
//    }

//    // 힌지축 직교 평면으로 투영 후 각도 계산 → 힌지축으로만 회전(클램프+감쇠)
//    void RotateHingeTowards(Transform pivot, Vector3 target, Vector3 localAxis, float maxDeg, float lerp)
//    {
//        Vector3 worldAxis = pivot.TransformDirection(localAxis);

//        Vector3 toEnd = endEffector.position - pivot.position;
//        Vector3 toTarget = target - pivot.position;

//        // 힌지축 직교 평면으로 투영 (180° 부근 불연속 완화)
//        Vector3 v1 = Vector3.ProjectOnPlane(toEnd, worldAxis);
//        Vector3 v2 = Vector3.ProjectOnPlane(toTarget, worldAxis);
//        if (v1.sqrMagnitude < 1e-12f || v2.sqrMagnitude < 1e-12f) return;

//        float angle = Vector3.SignedAngle(v1, v2, worldAxis);
//        angle = Mathf.Clamp(angle, -maxDeg, maxDeg);
//        if (Mathf.Abs(angle) < 1e-4f) return;

//        // 감쇠 적용
//        Quaternion delta = Quaternion.AngleAxis(angle * lerp, worldAxis);
//        pivot.rotation = delta * pivot.rotation;
//    }

//    // (옵션) 조인트 리밋
//    void ApplyJointLimits()
//    {
//        Vector3 e1 = pivot1.localEulerAngles;
//        e1.y = ClampAngleSigned(e1.y, pivot1YLimit.x, pivot1YLimit.y);
//        pivot1.localEulerAngles = new Vector3(e1.x, e1.y, e1.z);

//        Vector3 e2 = pivot2.localEulerAngles;
//        e2.x = ClampAngleSigned(e2.x, pivot2XLimit.x, pivot2XLimit.y);
//        pivot2.localEulerAngles = new Vector3(e2.x, e2.y, e2.z);

//        Vector3 e3 = pivot3.localEulerAngles;
//        e3.x = ClampAngleSigned(e3.x, pivot3XLimit.x, pivot3XLimit.y);
//        pivot3.localEulerAngles = new Vector3(e3.x, e3.y, e3.z);
//    }

//    static float ClampAngleSigned(float angleDeg, float minDeg, float maxDeg)
//    {
//        float a = NormalizeSigned(angleDeg);
//        return Mathf.Clamp(a, minDeg, maxDeg);
//    }

//    static float NormalizeSigned(float a)
//    {
//        a %= 360f;
//        if (a > 180f) a -= 360f;
//        if (a < -180f) a += 360f;
//        return a;
//    }

//#if UNITY_EDITOR
//    void OnDrawGizmosSelected()
//    {
//        if (!pivot1 || !pivot2 || !pivot3 || !endEffector) return;
//        Gizmos.color = new Color(0.2f, 0.6f, 1f, 0.25f);
//        Gizmos.DrawWireSphere(pivot1.position, GetReach());
//    }
//#endif
//}



//using UnityEngine;

//public class FingerIK : MonoBehaviour
//{
//    public Transform targetObject;

//    // 회전 기준이 될 pivot
//    public Transform pivot1;  // Index_Pivot1 (Y축 회전)
//    public Transform pivot2;  // Index_pivot2 (X축 회전)
//    public Transform pivot3;  // Index_pivot3 (X축 회전)

//    public Transform endEffector;

//    [Range(1, 50)]
//    public int iterations = 10;

//    void LateUpdate()
//    {
//        SolveIK();
//    }

//    void SolveIK()
//    {
//        float threshold = 0.005f;

//        for (int i = 0; i < iterations; i++)
//        {
//            Vector3 endPos = endEffector.position;
//            Vector3 targetPos = targetObject.position;

//            if (Vector3.Distance(endPos, targetPos) < threshold)
//                break;

//            RotateJointTowards(pivot3, targetPos, Vector3.right); // 끝 관절 - X축
//            RotateJointTowards(pivot2, targetPos, Vector3.right); // 중간 관절 - X축
//            RotateJointTowards(pivot1, targetPos, Vector3.up);    // 첫 관절 - Y축
//        }
//    }

//    void RotateJointTowards(Transform pivot, Vector3 target, Vector3 localAxis)
//    {
//        Vector3 toEnd = endEffector.position - pivot.position;
//        Vector3 toTarget = target - pivot.position;

//        Quaternion deltaRotation = Quaternion.FromToRotation(toEnd, toTarget);
//        Vector3 worldAxis = pivot.TransformDirection(localAxis);

//        float signedAngle = Vector3.SignedAngle(toEnd, toTarget, worldAxis);
//        pivot.Rotate(worldAxis, signedAngle, Space.World);
//    }

//}


//using UnityEngine;

//public class FingerIKStable : MonoBehaviour
//{
//    [Header("Chain")]
//    public Transform targetObject;
//    public Transform pivot1;  // Y hinge
//    public Transform pivot2;  // X hinge
//    public Transform pivot3;  // X hinge
//    public Transform endEffector;

//    [Header("Solver")]
//    [Range(1, 50)] public int iterations = 12;
//    [Tooltip("목표와 EE 사이가 이 값 미만이면 조기 종료")]
//    [Range(0.0001f, 0.05f)] public float threshold = 0.003f;
//    [Tooltip("관절당 1회전 루프에서 허용하는 최대 회전각(도)")]
//    [Range(0.5f, 15f)] public float maxDegPerIter = 6f;
//    [Tooltip("회전 적용 감쇠(0~1). 1=즉시 적용, 0.2~0.6 권장")]
//    [Range(0.05f, 1f)] public float rotationLerp = 0.5f;

//    [Header("Unreachable Target Handling")]
//    [Tooltip("도달 불가 타겟을 reach 반경으로 투영")]
//    public bool projectUnreachable = true;
//    [Tooltip("reach의 몇 %까지 허용할지(1보다 약간 작게)")]
//    [Range(0.90f, 1.00f)] public float reachSlack = 0.995f;

//    [Header("Joint Limits (deg)")]
//    public Vector2 pivot1YLimit = new Vector2(-90f, 90f);
//    public Vector2 pivot2XLimit = new Vector2(-90f, 90f);
//    public Vector2 pivot3XLimit = new Vector2(-90f, 90f);

//    void LateUpdate()
//    {
//        SolveIK();
//    }

//    void SolveIK()
//    {
//        if (!targetObject || !pivot1 || !pivot2 || !pivot3 || !endEffector) return;

//        // 1) 타겟 투영(도달불가 시)
//        Vector3 targetPos = targetObject.position;
//        if (projectUnreachable)
//            targetPos = ProjectTargetToReach(targetPos);

//        // 2) CCD 루프
//        for (int i = 0; i < iterations; i++)
//        {
//            if ((endEffector.position - targetPos).sqrMagnitude < threshold * threshold)
//                break;

//            // 힌지 축 전용 회전: 축 직교평면으로 투영 후 각 산출
//            RotateHingeTowards(pivot3, targetPos, Vector3.right, maxDegPerIter, rotationLerp);
//            RotateHingeTowards(pivot2, targetPos, Vector3.right, maxDegPerIter, rotationLerp);
//            RotateHingeTowards(pivot1, targetPos, Vector3.up, maxDegPerIter, rotationLerp);

//            // 조인트 리밋 적용
//            ApplyJointLimits();
//        }
//    }

//    Vector3 ProjectTargetToReach(Vector3 target)
//    {
//        Vector3 root = pivot1.position;
//        float d = Vector3.Distance(root, target);
//        float reach = GetReach();
//        if (d > reach)
//        {
//            Vector3 dir = (target - root).normalized;
//            return root + dir * (reach * reachSlack);
//        }
//        return target;
//    }

//    float GetReach()
//    {
//        float l1 = Vector3.Distance(pivot1.position, pivot2.position);
//        float l2 = Vector3.Distance(pivot2.position, pivot3.position);
//        float l3 = Vector3.Distance(pivot3.position, endEffector.position);
//        return l1 + l2 + l3;
//    }

//    void RotateHingeTowards(Transform pivot, Vector3 target, Vector3 localAxis, float maxDeg, float lerp)
//    {
//        Vector3 worldAxis = pivot.TransformDirection(localAxis);

//        Vector3 toEnd = endEffector.position - pivot.position;
//        Vector3 toTarget = target - pivot.position;

//        // 축 직교 평면으로 투영 (180° 부근 불연속 완화)
//        Vector3 v1 = Vector3.ProjectOnPlane(toEnd, worldAxis);
//        Vector3 v2 = Vector3.ProjectOnPlane(toTarget, worldAxis);
//        if (v1.sqrMagnitude < 1e-12f || v2.sqrMagnitude < 1e-12f) return;

//        float angle = Vector3.SignedAngle(v1, v2, worldAxis);
//        angle = Mathf.Clamp(angle, -maxDeg, maxDeg);
//        if (Mathf.Abs(angle) < 1e-4f) return;

//        // 감쇠 적용
//        Quaternion delta = Quaternion.AngleAxis(angle * lerp, worldAxis);
//        pivot.rotation = delta * pivot.rotation;
//    }

//    void ApplyJointLimits()
//    {
//        // 로컬 오일러에서 해당 축만 클램프 (힌지 가정)
//        Vector3 e1 = pivot1.localEulerAngles;
//        e1.y = ClampAngleSigned(e1.y, pivot1YLimit.x, pivot1YLimit.y);
//        pivot1.localEulerAngles = new Vector3(e1.x, e1.y, e1.z);

//        Vector3 e2 = pivot2.localEulerAngles;
//        e2.x = ClampAngleSigned(e2.x, pivot2XLimit.x, pivot2XLimit.y);
//        pivot2.localEulerAngles = new Vector3(e2.x, e2.y, e2.z);

//        Vector3 e3 = pivot3.localEulerAngles;
//        e3.x = ClampAngleSigned(e3.x, pivot3XLimit.x, pivot3XLimit.y);
//        pivot3.localEulerAngles = new Vector3(e3.x, e3.y, e3.z);
//    }

//    static float ClampAngleSigned(float angleDeg, float minDeg, float maxDeg)
//    {
//        float a = NormalizeSigned(angleDeg);
//        float min = Mathf.Clamp(minDeg, -180f, 180f);
//        float max = Mathf.Clamp(maxDeg, -180f, 180f);
//        return Mathf.Clamp(a, min, max);
//    }

//    static float NormalizeSigned(float a)
//    {
//        a %= 360f;
//        if (a > 180f) a -= 360f;
//        if (a < -180f) a += 360f;
//        return a;
//    }

//#if UNITY_EDITOR
//    void OnDrawGizmosSelected()
//    {
//        if (!pivot1) return;
//        Gizmos.color = new Color(0.2f, 0.6f, 1f, 0.2f);
//        Gizmos.DrawWireSphere(pivot1.position, GetReach());
//    }
//#endif
//}