using System.Collections;
using System.Collections.Generic;
using System.Data;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

public class SendAngle : MonoBehaviour
{
    public List<Transform> targets;   // 12개 모델을 에디터에서 할당
    
    [Header("각 모터 각도 반전")]
    public bool[] isOpposite = new bool[12];

    [Header("유니티에서 0도일때 실제 모터의 각도")]
    public float[] angle_list = new float[12];

    [Header("robot initial angle")]
    public float[] initialAngleList = new float[12];

    private float[] lastUnityAngle = new float[12];
    private bool initialCheck = false;

    UdpClient sender;
    UdpClient receiver;
    IPEndPoint remote;

    string ip = "127.0.0.1";
    int send_port = 9001; // 보내기(Unity -> python)
    int receive_port = 9002; // 받기(Python -> Unity)
    
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
        // "1:45.00;2:30.50;...;12:90.00" 형태로 직렬화
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
