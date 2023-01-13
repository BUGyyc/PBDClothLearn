using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using Unity.Mathematics;

namespace PBDLearn
{
    //Cloth 对象脚本
    public class ClothRenderer : MonoBehaviour
    {
        [Range(0.005f, 0.02f)]
        public float dt = 0.01f;
        /// <summary>
        /// 场景内的风场
        /// </summary>
        public WindZone windZone;
        private ClothSimulator _simulator;

        private Mesh _mesh;
        private MeshFilter _meshFilter;

        //把碰撞体给到布料
        public UnityColliderProxy[] colliderProxies;

        /// <summary>
        /// 布料属性
        /// </summary>
        public ClothSetting setting;

        public bool drawGizmos;

        private ClothMeshModifier _meshModifier;

        private Dictionary<int, Transform> _attachments = new Dictionary<int, Transform>();

        void Awake()
        {
            _meshFilter = GetComponent<MeshFilter>();
            _meshFilter.sharedMesh = Object.Instantiate(_meshFilter.sharedMesh);
            _mesh = _meshFilter.sharedMesh;
            //创建一个布料网格修饰器
            _meshModifier = ClothMeshModifier.CreateFromMeshFilter(_meshFilter);
            _simulator = new ClothSimulator(_meshModifier, this.setting);
            //布料模拟器
            //模拟器写入存在的碰撞盒子
            foreach (var c in colliderProxies)
            {
                _simulator.AddCollider(c);
            }
        }

        void Start()
        {
            GetComponent<MeshRenderer>().sharedMaterial.EnableKeyword("_INPUT_WORLD_VERTEX");
            transform.localPosition = Vector3.zero;
        }

        public void Attach(int vertexIndex, Transform target)
        {
            if (!target)
            {
                throw new System.Exception("target can not be null");
            }
            _attachments.Add(vertexIndex, target);
            target.position = _simulator.positions[vertexIndex];
        }

        void OnDestroy()
        {
            if (_simulator != null)
            {
                _simulator.Dispose();
                _simulator = null;
            }
            GetComponent<MeshRenderer>().sharedMaterial.DisableKeyword("_INPUT_WORLD_VERTEX");
        }

        private float _elapsedTime = 0;
        void Update()
        {
            if (_simulator != null)
            {
                foreach (var pair in _attachments)
                {
                    _simulator.AddPinConstraint(pair.Key, pair.Value.position);
                }
                _simulator.SetFieldForce(windZone.transform.forward * windZone.windMain);
                _elapsedTime += Time.deltaTime;
                int cnt = Mathf.FloorToInt(_elapsedTime / dt);
                for (var i = 0; i < cnt; i++)
                {
                    _simulator.Step(dt);
                    if (i < cnt - 1)
                    {
                        _simulator.ComputeAsyncJobs();
                    }
                }
                _elapsedTime %= dt;
            }
        }

        void LateUpdate()
        {
            if (_simulator != null)
            {
                _simulator.ComputeAsyncJobs();
                _mesh.SetVertices(_simulator.positions);
                _mesh.SetNormals(_simulator.normals);
                _mesh.RecalculateBounds();
            }
        }

        void OnDrawGizmos()
        {
            if (drawGizmos)
            {
                if (_simulator != null)
                {
                    foreach (var p in _simulator.positions)
                    {
                        Gizmos.DrawSphere(p, 0.05f);
                    }
                    for (var i = 0; i < _simulator.distanceConstraintCount; i++)
                    {
                        var info = _simulator.GetDistanceConstraintInfo(i);
                        var fromPos = _simulator.positions[info.vIndex0];
                        var toPos = _simulator.positions[info.vIndex1];
                        Gizmos.DrawLine(fromPos, toPos);
                    }
                }
            }
        }
    }
}
