using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using PBDLearn;

public class ClothSimulateTest : MonoBehaviour
{

    public ClothRenderer cloth;

    public List<AttachBind> attachBinds = new List<AttachBind>();

    void Start()
    {

        //初始化布料的绑定点
        foreach (var a in attachBinds)
        {
            cloth.Attach(a.vertexIndex, a.target);
        }
    }



    [System.Serializable]
    public class AttachBind
    {
        public int vertexIndex;
        public Transform target;
    }


}
