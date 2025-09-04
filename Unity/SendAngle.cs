using System.Collections;
using System.Collections.Generic;
using System.Data;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

public class SendAngle : MonoBehaviour
{
    public List<Transform> targets;   // 12�� ���� �����Ϳ��� �Ҵ�
    
    [Header("�� ���� ���� ����")]
    public bool[] isOpposite = new bool[12];

    [Header("����Ƽ���� 0���϶� ���� ������ ����")]
    public float[] angle_list = new float[12];

    [Header("robot initial angle")]
    public float[] initialAngleList = new float[12];

    private float[] lastUnityAngle = new float[12];
    private bool initialCheck = false;

    UdpClient sender;
    UdpClient receiver;
    IPEndPoint remote;

    string ip = "127.0.0.1";
    int send_port = 9001; // ������(Unity -> python)
    int receive_port = 9002; // �ޱ�(Python -> Unity)
    
    float torque = 0.2f;
    float angle = 0f;

    void Start()
    {
        sender = new UdpClient();
        receiver = new UdpClient(receive_port);

        remote = new IPEndPoint(IPAddress.Any, 0);

    }

    private void FixedUpdate()
    {
        //if (!initialCheck && receiver.Available > 0)
        //{
        //    byte[] data = receiver.Receive(ref remote);
        //    string msg = Encoding.UTF8.GetString(data);

        //    string[] data_parts = msg.Split(',');
        //    for (int i = 0; i < initialAngleList.Length; i++)
        //    {
        //        float.TryParse(data_parts[i], out initialAngleList[i]);
        //    }
        //    Debug.Log(msg);
        //    initialCheck = true;
        //}
        // "1:45.00;2:30.50;...;12:90.00" ���·� ����ȭ
        var parts = new List<string>(targets.Count);
        for (int i = 0; i < targets.Count; i++)
        {
            switch (i)
            {
                case 0:
                case 3:
                case 6:
                case 9:  // axis : y
                    angle = Mathf.DeltaAngle(0f, targets[i].localEulerAngles.y);
                    break;

                case 1:
                case 2:  // axis : z
                    angle = Mathf.DeltaAngle(0f, targets[i].localEulerAngles.z);
                    break;
                case 4:
                case 5:
                    // axis : x
                    angle = Mathf.DeltaAngle(13f, targets[i].localEulerAngles.x);
                    //angle = targets[i].localEulerAngles.x;
                    break;
                case 7:
                case 8:
                    angle = Mathf.DeltaAngle(5f, targets[i].localEulerAngles.x);
                    break;
                case 10:
                    angle = Mathf.DeltaAngle(4f, targets[i].localEulerAngles.x);
                    break;
                case 11:
                    angle = Mathf.DeltaAngle(3f, targets[i].localEulerAngles.x);
                    break;
                default:
                    break;
            }
            if (isOpposite[i])
            {
                angle *= -1f;
            }

            parts.Add($"{i}:{angle:F2}");
            //angle = calDetlaAngle(angle, i);
            byte[] data = Encoding.UTF8.GetBytes($"p,{i},{angle:F2},{torque}");
            sender.Send(data, data.Length, ip, send_port);
        }
        string message = string.Join("; ", parts);
        //byte[] data = Encoding.UTF8.GetBytes(message);
        //client.Send(data, data.Length, ip, send_port);

        Debug.Log(message);

        //UpdateRealAngles();
    }

    void OnApplicationQuit()
    {
        sender.Close();
        receiver.Close();
    }
}
