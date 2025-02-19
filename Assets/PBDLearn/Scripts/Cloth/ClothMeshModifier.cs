﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using Unity.Mathematics;

namespace PBDLearn
{
    /// <summary>
    /// 布料网格修饰器
    /// </summary>
    public class ClothMeshModifier
    {

        private NativeList<float3> _vertices;
        private NativeList<float3> _normals;
        private NativeList<float2> _uv;
        private NativeList<int> _indices;
        private List<VertexExtraInfo> _verticesInfo;
        private Dictionary<int, Edge> _edgeDict = new Dictionary<int, Edge>();

        private ClothMeshModifier(NativeList<float3> vertices, NativeList<float2> uvs, NativeList<int> indices, NativeList<float3> normals)
        {
            this._vertices = vertices;
            this._uv = uvs;
            this._indices = indices;
            this._normals = normals;
            this.BuildIndexes();
        }

        public void Dispose()
        {
            _vertices.Dispose();
            _uv.Dispose();
            _indices.Dispose();
            _normals.Dispose();
        }

        public int vertexCount
        {
            get
            {
                return _vertices.Length;
            }
        }

        private int NewVertex(float3 vertex, float2 uv)
        {
            var index = _vertices.Length;
            _vertices.Add(vertex);
            _uv.Add(uv);
            _verticesInfo.Add(new VertexExtraInfo());
            return index;
        }

        /// <summary>
        /// 给指定索引的顶点，塞入三角形编号
        /// </summary>
        /// <param name="vIndex0">顶点编号</param>
        /// <param name="vIndex1">顶点编号</param>
        /// <param name="triangleIndex">三角形编号</param>
        private void CacheEdge(int vIndex0, int vIndex1, int triangleIndex)
        {
            var edgeHash = Edge.GetEdgeHash(vIndex0, vIndex1);
            Edge edge;
            //字典缓存边信息
            if (_edgeDict.TryGetValue(edgeHash, out edge))
            {
                //新增边所属三角形信息
                edge.triangleIndexes.Add(triangleIndex);
            }
            else
            {
                //创建边
                edge = new Edge(vIndex0, vIndex1);
                //存储边所属三角形信息
                edge.triangleIndexes.Add(triangleIndex);
                //缓存起来
                _edgeDict.Add(edgeHash, edge);
                //顶点所属边信息写入
                _verticesInfo[vIndex0].edgeHashes.Add(edgeHash);
                _verticesInfo[vIndex1].edgeHashes.Add(edgeHash);
            }
        }

        /// <summary>
        /// 构建一个顶点信息集合
        /// </summary>
        private void BuildIndexes()
        {
            _verticesInfo = new List<VertexExtraInfo>(_vertices.Length);
            for (var i = 0; i < _vertices.Length; i++)
            {
                _verticesInfo.Add(new VertexExtraInfo());
            }

            //取得三角形数量
            var triangleCount = _indices.Length / 3;
            for (var i = 0; i < triangleCount; i++)
            {
                var offset = i * 3;
                var vIndex0 = _indices[offset];
                var vIndex1 = _indices[offset + 1];
                var vIndex2 = _indices[offset + 2];

                //按照读取顺序创建边
                var edge0 = new Edge(vIndex0, vIndex1);
                var edge1 = new Edge(vIndex1, vIndex2);
                var edge2 = new Edge(vIndex2, vIndex0);

                //写入边、点、三角形的信息
                CacheEdge(vIndex0, vIndex1, i);
                CacheEdge(vIndex1, vIndex2, i);
                CacheEdge(vIndex2, vIndex0, i);
            }
        }


        private void RemoveEdge(int edgeHash)
        {
            var edge = _edgeDict[edgeHash];
            _verticesInfo[edge.vIndex0].edgeHashes.Remove(edgeHash);
            _verticesInfo[edge.vIndex1].edgeHashes.Remove(edgeHash);
            _edgeDict.Remove(edgeHash);
        }

        private Edge GetEdge(int vIndex0, int vIndex1)
        {
            var hash = Edge.GetEdgeHash(vIndex0, vIndex1);
            if (_edgeDict.ContainsKey(hash))
            {
                return _edgeDict[hash];
            }
            return null;
        }

        /// <summary>
        /// 提取一个三角形中，与指定顶点相关的两条边
        /// </summary>
        private void GetEdge(int triangleIndex, int vertexIndex, out Edge edge0, out Edge edge1)
        {
            var offset = triangleIndex * 3;
            edge0 = null;
            edge1 = null;
            for (var i = 0; i < 3; i++)
            {
                var vIndex = _indices[offset + i];
                if (vIndex != vertexIndex)
                {
                    if (edge0 == null)
                    {
                        edge0 = GetEdge(vertexIndex, vIndex);
                    }
                    else
                    {
                        edge1 = GetEdge(vertexIndex, vIndex);
                    }
                }
            }
        }


        // /// <summary>
        // /// 根据边，撕开一个三角面
        // /// </summary>
        // public TearResult Tear(int vIndex0,int vIndex1){

        //     var result = new TearResult();

        //     var edgeHash = Edge.GetEdgeHash(vIndex0,vIndex1);
        //     var edge = _edgeDict[edgeHash];

        //     if(edge.triangleIndexes.Count < 2){
        //         return result;
        //     }

        //     result.success = true;

        //     var t0 = edge.triangleIndexes[0];
        //     var t1 = edge.triangleIndexes[1];

        //     //准备vIndex0进行复制拆分
        //     var splitVertexIndex = edge.vIndex0;

        //     result.splitVertexIndex = splitVertexIndex;
        //     result.oppVertexIndex = edge.vIndex1;

        //     var vertex = _vertices[splitVertexIndex];
        //     var uv = _uv[splitVertexIndex];

        //     this.RemoveEdge(edgeHash);


        //     vertex.y += 1;
        //     {


        //         Edge edge0,edge1;
        //         GetEdge(t0,splitVertexIndex,out edge0,out edge1);
        //         edge0.triangleIndexes.Remove(t0);

        //         //新增加一个顶点
        //         var newVertexIndex = NewVertex(vertex,uv);
        //         result.newVertexIndex0 = newVertexIndex;

        //         for(var i = 0; i < 3; i ++){
        //             var tIndex = t0 * 3 + i;
        //             if(_indices[tIndex] == splitVertexIndex){
        //                 _indices[tIndex] = newVertexIndex;
        //             }else{
        //                 this.CacheEdge(newVertexIndex,_indices[tIndex],t0);
        //             }
        //         }
        //     }

        //     {
        //         //新增加一个顶点

        //         Edge edge0,edge1;
        //         GetEdge(t1,splitVertexIndex,out edge0,out edge1);

        //         edge0.triangleIndexes.Remove(t1);

        //         var newVertexIndex = NewVertex(vertex,uv);
        //         result.newVertexIndex1 = newVertexIndex;

        //         for(var i = 0; i < 3; i ++){
        //             var tIndex = t1 * 3 + i;
        //             if(_indices[tIndex] == splitVertexIndex){
        //                 _indices[tIndex] = newVertexIndex;
        //             }else{
        //                 this.CacheEdge(newVertexIndex,_indices[tIndex],t1);
        //             }
        //         }
        //     }
        //     return result;
        // }

        /// <summary>
        /// 获取三角形的未被传参的第三点
        /// </summary>
        /// <param name="triangleIndex"></param>
        /// <param name="exceptVertex0"></param>
        /// <param name="exceptVertex1"></param>
        /// <returns></returns>
        public int GetTriangleVertexIndexExcept(int triangleIndex, int exceptVertex0, int exceptVertex1)
        {
            for (var i = 0; i < 3; i++)
            {
                var vIndex = _indices[triangleIndex * 3 + i];
                if (vIndex != exceptVertex0 && vIndex != exceptVertex1)
                {
                    return vIndex;
                }
            }
            return -1;
        }

        public IReadOnlyCollection<Edge> edges
        {
            get
            {
                return _edgeDict.Values;
            }
        }

        public NativeArray<int> indices
        {
            get
            {
                return _indices;
            }
        }

        public NativeArray<float2> uvs
        {
            get
            {
                return _uv;
            }
        }

        public NativeArray<float3> vertices
        {
            get
            {
                return _vertices;
            }
        }

        public NativeArray<float3> normals
        {
            get
            {
                return _normals;
            }
        }


        /// <summary>
        /// 通过网格创建布料数据
        /// </summary>
        /// <param name="meshFilter"></param>
        /// <returns></returns>
        public static ClothMeshModifier CreateFromMeshFilter(MeshFilter meshFilter)
        {
            var mesh = meshFilter.sharedMesh;
            //顶点数集合
            var vertices = mesh.vertices;
            //UV集合
            var uvs = mesh.uv;
            //三角形内是 顶点索引
            var indices = mesh.triangles;

            //申请了一个连续内存，这个很关键，为了缓存命中率
            var verticesList = new NativeList<float3>(vertices.Length, Allocator.Persistent);
            //https://docs.unity.cn/cn/2018.4/ScriptReference/Unity.Collections.NativeArrayOptions.html
            verticesList.Resize(vertices.Length, NativeArrayOptions.UninitializedMemory);
            //转换矩阵
            var localToWorld = meshFilter.transform.localToWorldMatrix;
            for (var i = 0; i < vertices.Length; i++)
            {
                //把坐标转换为世界空间
                verticesList[i] = localToWorld.MultiplyPoint3x4(vertices[i]);
            }
            //顶点上的法线
            var normals = new NativeList<float3>(vertices.Length, Allocator.Persistent);
            normals.Resize(vertices.Length, NativeArrayOptions.UninitializedMemory);

            //把顶点上的UV存下来
            var uvList = new NativeList<float2>(uvs.Length, Allocator.Persistent);
            uvList.Resize(uvs.Length, NativeArrayOptions.UninitializedMemory);
            uvList.AsArray().Reinterpret<Vector2>().CopyFrom(uvs);

            //三角形的索引
            var indicesList = new NativeList<int>(indices.Length, Allocator.Persistent);
            indicesList.Resize(indices.Length, NativeArrayOptions.UninitializedMemory);
            indicesList.AsArray().Reinterpret<int>().CopyFrom(indices);

            return new ClothMeshModifier(verticesList, uvList, indicesList, normals);
        }


        public struct TearResult
        {
            public bool success;

            public int splitVertexIndex;
            public int oppVertexIndex;

            public int newVertexIndex0;

            public int newVertexIndex1;

        }

        /// <summary>
        /// 一条边，连接两个顶点，两侧各有一个三角面
        /// </summary>
        public class Edge
        {
            public int vIndex0;
            public int vIndex1;
            /// <summary>
            /// 表示边所属哪些三角形
            /// </summary>
            /// <typeparam name="int"></typeparam>
            /// <returns></returns>
            public List<int> triangleIndexes = new List<int>(2);

            public Edge(int vIndex0, int vIndex1)
            {
                this.vIndex0 = math.min(vIndex0, vIndex1);
                this.vIndex1 = math.max(vIndex0, vIndex1);
            }

            public override int GetHashCode()
            {
                return (vIndex0 << 16 | vIndex1);
            }

            public override bool Equals(object obj)
            {
                if (!(obj is Edge))
                {
                    return false;
                }
                var edge2 = (Edge)obj;

                return vIndex0 == edge2.vIndex0 && vIndex1 == edge2.vIndex1;
            }

            public static bool operator ==(Edge e1, Edge e2)
            {
                var n1 = object.ReferenceEquals(e1, null);
                var n2 = object.ReferenceEquals(e2, null);
                if (n1 && n2)
                {
                    return true;
                }
                if (n1 != n2)
                {
                    return false;
                }
                return e1.vIndex0 == e2.vIndex0 && e1.vIndex1 == e2.vIndex1;
            }

            public static bool operator !=(Edge e1, Edge e2)
            {
                var n1 = object.ReferenceEquals(e1, null);
                var n2 = object.ReferenceEquals(e2, null);
                if (n1 && n2)
                {
                    return false;
                }
                if (n1 != n2)
                {
                    return true;
                }
                return e1.vIndex0 != e2.vIndex0 || e1.vIndex1 != e2.vIndex1;
            }

            /// <summary>
            /// 生成一个特定hash,把32位，拆成左16位和 右16位
            /// </summary>
            /// <param name="vIndex0"></param>
            /// <param name="vIndex1"></param>
            /// <returns></returns>
            public static int GetEdgeHash(int vIndex0, int vIndex1)
            {
                //根据索引大小，判断上下位
                if (vIndex0 < vIndex1)
                {
                    return (vIndex0 << 16 | vIndex1);
                }
                else
                {
                    return (vIndex1 << 16 | vIndex0);
                }
            }

        }

        private class VertexExtraInfo
        {
            //顶点相连的边
            public List<int> edgeHashes = new List<int>();
        }



    }
}
