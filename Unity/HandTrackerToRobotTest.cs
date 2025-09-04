using UnityEngine;
using Oculus.Interaction.Input;
using System;

public class HandTrackerToRobotTest : MonoBehaviour
{
    [Header("OVR Hand Tracking")]
    public Hand ovrHand;

    //[Header("ovr hand")]
    //public Transform ovrthumb1;
    //public Transform ovrindex1;
    //public Transform ovrmiddle1;
    //public Transform ovrring1;

    [Header("Sphere Targets")]
    public Transform thumbSphere;
    public Transform indexSphere;
    public Transform middleSphere;
    public Transform ringSphere;

    [Header("Robot Hand Base")]
    public Transform robotBaseThumb;
    public Transform robotBaseIndex;
    public Transform robotBaseMiddle;
    public Transform robotBaseRing;

    [Header("Robot Hand 2")]
    public Transform robotThumb2;
    public Transform robotIndex2;
    public Transform robotMiddle2;
    public Transform robotRing2;

    [Header("Robot Hand 3")]
    public Transform robotThumb3;
    public Transform robotIndex3;
    public Transform robotMiddle3;
    public Transform robotRing3;

    [Header("Robot Hand Tip")]
    public Transform robotTipThumb;
    public Transform robotTipIndex;
    public Transform robotTipMiddle;
    public Transform robotTipRing;

    [Header("Scaling")]
    public float handScale = 110f;

    void Update()
    {
        if (ovrHand == null || !ovrHand.IsConnected)
        {
            return;
        }

        // Proximal -> Intermediate
        //SetFingerTargetPosition(HandJointId.HandThumb2, );
        Vector3 firstIndex = SetFingerFirstTargetPosition(HandJointId.HandIndex1, HandJointId.HandIndex2, robotBaseIndex, robotIndex2);
        Vector3 firstMiddle = SetFingerFirstTargetPosition(HandJointId.HandMiddle1, HandJointId.HandMiddle2, robotBaseMiddle, robotMiddle2);
        Vector3 firstRing = SetFingerFirstTargetPosition(HandJointId.HandRing1, HandJointId.HandRing2, robotBaseMiddle, robotMiddle2);

        // Intermediate -> Distal
        //SetFingerTargetPosition(HandJointId.HandThumb2, );
        //SetFingerTargetPosition(HandJointId.HandIndex2, HandJointId.HandIndex3, robotIndex2, robotIndex3); ///////////////////////////////////// 만약 안되면 여기 고치기
        //SetFingerTargetPosition(HandJointId.HandMiddle2, HandJointId.HandMiddle3, robotMiddle2, robotMiddle3);
        //SetFingerTargetPosition(HandJointId.HandRing2, HandJointId.HandRing3, robotRing2, robotRing3);
        Vector3 secondIndex = SetFingerSecondTargetPosition(HandJointId.HandIndex2, HandJointId.HandIndex3, robotIndex2, robotIndex3, firstIndex);
        Vector3 secondMiddle = SetFingerSecondTargetPosition(HandJointId.HandMiddle2, HandJointId.HandMiddle3, robotMiddle2, robotMiddle3, firstMiddle);
        Vector3 secondRing = SetFingerSecondTargetPosition(HandJointId.HandRing2, HandJointId.HandRing3, robotRing2, robotRing3, firstRing);

        // Distal -> Tip
        //SetFingerEndTargetPosition(HandJointId.HandThumb2);
        SetFingerEndTargetPosition(HandJointId.HandIndex3, HandJointId.HandIndexTip, indexSphere, robotIndex3, robotTipIndex, secondIndex);
        SetFingerEndTargetPosition(HandJointId.HandMiddle3, HandJointId.HandMiddleTip, middleSphere, robotMiddle3, robotTipMiddle, secondMiddle);
        SetFingerEndTargetPosition(HandJointId.HandRing3, HandJointId.HandRingTip, ringSphere, robotRing3, robotTipRing, secondRing);


        //SetFingerTargetPosition(HandJointId.HandThumb2, HandJointId.HandThumbTip, thumbSphere, robotBaseThumb, robotTipThumb);
        //SetFingerTargetPosition(HandJointId.HandIndex1, HandJointId.HandIndexTip, indexSphere, robotBaseIndex, robotTipIndex);
        //SetFingerTargetPosition(HandJointId.HandMiddle1, HandJointId.HandMiddleTip, middleSphere, robotBaseMiddle, robotTipMiddle);
        //SetFingerTargetPosition(HandJointId.HandRing1, HandJointId.HandRingTip, ringSphere, robotBaseRing, robotTipRing);
    }

    private Vector3 SetFingerFirstTargetPosition(HandJointId baseBoneId, HandJointId tipBoneId, Transform robotBase, Transform robotEnd)
    {
        if (!ovrHand.GetJointPose(baseBoneId, out Pose ovrBasePose))
        {
            return Vector3.zero;
        }
        if (!ovrHand.GetJointPose(tipBoneId, out Pose ovrTipPose))
        {
            return Vector3.zero;
        }
        if (!ovrHand.GetJointPose(HandJointId.HandPalm, out Pose ovrPalmPose))
        {
            return Vector3.zero;
        }

        CalculateHandSacale(ovrBasePose.position, ovrTipPose.position, robotBase.position, robotEnd.position);

        //1. OVR 손끝에서 기준점까지의 벡터
        Vector3 ovrLocalTipVector = ovrTipPose.position - ovrBasePose.position;

        Vector3 localVec = Quaternion.Inverse(ovrPalmPose.rotation) * ovrLocalTipVector;

        // 2. 현재 손 길이 계산
        float currentHandLength = localVec.magnitude;

        // 3. 방향 벡터로 정규화
        Vector3 direction = localVec.normalized;

        // 4. 길이 스케일 적용 (현재 손 길이 × handScale)
        Vector3 scaledVector = direction * currentHandLength * handScale;

        Vector3 robotVec = robotBase.rotation * scaledVector;
        robotVec.y = -robotVec.y; // 반전
        robotVec.x = -robotVec.x;

        // 5. 로봇 기준 위치 + 스케일된 벡터로 새 위치 계산
        Vector3 robotTargetPos = robotBase.position + robotVec;

        return robotTargetPos;
    }
    private Vector3 SetFingerSecondTargetPosition(HandJointId baseBoneId, HandJointId tipBoneId, Transform robotBase, Transform robotEnd, Vector3 previousPos)
    {
        if (!ovrHand.GetJointPose(baseBoneId, out Pose ovrBasePose))
        {
            return Vector3.zero;
        }
        if (!ovrHand.GetJointPose(tipBoneId, out Pose ovrTipPose))
        {
            return Vector3.zero;
        }
        if (!ovrHand.GetJointPose(HandJointId.HandPalm, out Pose ovrPalmPose))
        {
            return Vector3.zero;
        }

        CalculateHandSacale(ovrBasePose.position, ovrTipPose.position, robotBase.position, robotEnd.position);

        //1. OVR 손끝에서 기준점까지의 벡터
        Vector3 ovrLocalTipVector = ovrTipPose.position - ovrBasePose.position;

        Vector3 localVec = Quaternion.Inverse(ovrPalmPose.rotation) * ovrLocalTipVector;

        // 2. 현재 손 길이 계산
        float currentHandLength = localVec.magnitude;

        // 3. 방향 벡터로 정규화
        Vector3 direction = localVec.normalized;

        // 4. 길이 스케일 적용 (현재 손 길이 × handScale)
        Vector3 scaledVector = direction * currentHandLength * handScale;

        Vector3 robotVec = robotBase.rotation * scaledVector;
        robotVec.y = -robotVec.y; // 반전
        robotVec.x = -robotVec.x;

        // 5. 로봇 기준 위치 + 스케일된 벡터로 새 위치 계산
        Vector3 robotTargetPos = previousPos  + robotBase.position + robotVec;

        return robotTargetPos;
    }

    private void SetFingerEndTargetPosition(HandJointId baseBoneId, HandJointId tipBoneId, Transform targetSphere, Transform robotBase, Transform robotEnd, Vector3 robotTargetPos)
    {
        if (targetSphere == null) return;

        if (!ovrHand.GetJointPose(baseBoneId, out Pose ovrBasePose))
        {
            return;
        }
        if (!ovrHand.GetJointPose(tipBoneId, out Pose ovrTipPose))
        {
            return;
        }
        if (!ovrHand.GetJointPose(HandJointId.HandPalm, out Pose ovrPalmPose))
        {
            return;
        }

        CalculateHandSacale(ovrBasePose.position, ovrTipPose.position, robotBase.position, robotEnd.position);

        //1. OVR 손끝에서 기준점까지의 벡터
        Vector3 ovrLocalTipVector = ovrTipPose.position - ovrBasePose.position;

        Vector3 localVec = Quaternion.Inverse(ovrPalmPose.rotation) * ovrLocalTipVector;

        // 2. 현재 손 길이 계산
        float currentHandLength = localVec.magnitude;

        // 3. 방향 벡터로 정규화
        Vector3 direction = localVec.normalized;

        // 4. 길이 스케일 적용 (현재 손 길이 × handScale)
        Vector3 scaledVector = direction * currentHandLength * handScale;

        Vector3 robotVec = robotBase.rotation * scaledVector;
        robotVec.y = -robotVec.y; // 반전
        robotVec.x = -robotVec.x;

        // 5. 로봇 기준 위치 + 스케일된 벡터로 새 위치 계산
        Vector3 robotTargetEndPos = robotTargetPos + robotBase.position + robotVec;

        // 6. 가상 손 위치 적용
        targetSphere.position = robotTargetEndPos;
    }

    //CalculateHandSacale(ovrBasePose.position, ovrTipPose.position, robotBase.position, robotEnd.position);
    private void CalculateHandSacale(Vector3 ovrBasePose, Vector3 ovrTipPose, Vector3 robotBase, Vector3 robotEnd)
    {
        float ovrFingerDistance = Vector3.Distance(ovrBasePose, ovrTipPose);
        float robotFingerDistatnce = Vector3.Distance(robotBase, robotEnd);
        handScale = robotFingerDistatnce / ovrFingerDistance;
    }
}