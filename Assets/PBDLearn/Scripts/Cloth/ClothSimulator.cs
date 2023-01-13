using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Jobs;


namespace PBDLearn
{
    [System.Flags]
    public enum ConstraintType
    {
        /// <summary>
        /// 距离约束
        /// </summary>
        Distance,
        /// <summary>
        /// 弯曲约束
        /// </summary>
        Pin,
        /// <summary>
        /// 碰撞约束
        /// </summary>
        Collision,
    }

    public class ClothSimulator
    {
        public static readonly float3 G = new float3(0, -9.8f, 0);
        public const int MAX_DIST_CONSTRAINT_COUNT_PER_VERTEX = 8;
        private NativeList<float3> _velocities;
        private NativeList<float3> _predictPositions;

        private NativeList<float> _masses;

        /// <summary>
        /// 激活的约束类型
        /// </summary>
        private NativeList<ConstraintType> _constraintTypes;

        /// <summary>
        /// 距离约束
        /// </summary>
        private NativeList<DistanceConstraintInfo> _distanceConstraints;

        /// <summary>
        /// 弯曲约束
        /// </summary>
        private NativeList<BendConstraintInfo> _bendConstraints;

        /// <summary>
        /// 碰撞约束
        /// </summary>
        private NativeList<CollisionConstraintInfo> _collisionConstraints;

        /// <summary>
        /// 固定约束
        /// </summary>
        private NativeList<PinConstraintInfo> _pinConstraints;


        private NativeArray<float3> _positionCorrects;

        private CollidersGroup _colliderDescGroups;

        private Dictionary<int, ICollider> _colliderProxies = new Dictionary<int, ICollider>();

        private NativeList<RigidBodyForceApply> _rigidBodyForceApplys;

        private float3 _fieldForce;

        private ClothMeshModifier _meshModifier;

        private ClothSetting _setting;

        /// <summary>
        /// 创建碰撞模拟器
        /// </summary>
        /// <param name="meshModifier"></param>
        /// <param name="setting">布料设置信息</param>
        public ClothSimulator(ClothMeshModifier meshModifier, ClothSetting setting)
        {
            _meshModifier = meshModifier;
            _setting = setting;

            //为每个定点创建一个预测坐标
            var vertexCount = meshModifier.vertexCount;
            _predictPositions = new NativeList<float3>(vertexCount, Allocator.Persistent);
            //写入各自初始坐标
            _predictPositions.AddRange(meshModifier.vertices);

            //每个定点都会有的临时速度，单纯计算中使用
            _velocities = new NativeList<float3>(vertexCount, Allocator.Persistent);
            _velocities.Resize(vertexCount, NativeArrayOptions.ClearMemory);

            //每个定点的约束
            _pinConstraints = new NativeList<PinConstraintInfo>(vertexCount, Allocator.Persistent);
            _pinConstraints.Resize(vertexCount, NativeArrayOptions.ClearMemory);

            //碰撞约束
            _collisionConstraints = new NativeList<CollisionConstraintInfo>(vertexCount, Allocator.Persistent);
            _collisionConstraints.Resize(vertexCount, NativeArrayOptions.ClearMemory);

            //?约束类型？？
            _constraintTypes = new NativeList<ConstraintType>(vertexCount, Allocator.Persistent);
            _constraintTypes.Resize(vertexCount, NativeArrayOptions.ClearMemory);

            //TODO:？？
            _positionCorrects = new NativeArray<float3>(vertexCount, Allocator.Persistent);

            //TODO:碰撞盒子
            _colliderDescGroups = new CollidersGroup(Allocator.Persistent);

            //??
            _rigidBodyForceApplys = new NativeList<RigidBodyForceApply>(vertexCount * 8, Allocator.Persistent);

            //创建顶点的质量
            this.BuildMasses();
            //创建距离约束
            this.BuildDistConstraints();
            //创建弯曲约束
            this.BuildBendConstraints();
        }

        private void IncreaseVertex(float3 vertex)
        {
            _predictPositions.Add(vertex);
            _velocities.Add(float3.zero);
            for (var i = 0; i < MAX_DIST_CONSTRAINT_COUNT_PER_VERTEX; i++)
            {
                _distanceConstraints.Add(new DistanceConstraintInfo());
            }
            _pinConstraints.Add(new PinConstraintInfo());
            _collisionConstraints.Add(new CollisionConstraintInfo());
            _constraintTypes.Add(0);
            _masses.Add(0);
        }

        /// <summary>
        /// 计算每个质点的质量。 算法为三角面面积乘以密度，再平分到每个顶点
        /// </summary>
        private void BuildMasses()
        {
            var indices = _meshModifier.indices;
            var vertices = _meshModifier.vertices;
            //存储每个顶点的质量
            _masses = new NativeList<float>(this.vertexCount, Allocator.Persistent);
            _masses.Resize(this.vertexCount, NativeArrayOptions.ClearMemory);
            for (var i = 0; i < indices.Length / 3; i++)
            {
                var offset = i * 3;
                var i0 = indices[offset];
                var i1 = indices[offset + 1];
                var i2 = indices[offset + 2];
                var v0 = vertices[i0];
                var v1 = vertices[i1];
                var v2 = vertices[i2];
                //计算面积的方法
                var area = IntersectUtil.GetArea(v0, v1, v2);
                //通过密度，算出三角形的质量
                var m = area * _setting.density;
                //将三角形的质量平均分给顶点
                var m3 = m / 3;
                //因为顶点可能被共用，所以每个三角形各自叠加质量
                _masses[i0] += m3;
                _masses[i1] += m3;
                _masses[i2] += m3;
            }
        }

        /// <summary>
        /// 构建边与边的距离约束
        /// </summary>
        private void BuildDistConstraints()
        {
            var edges = _meshModifier.edges;
            _distanceConstraints = new NativeList<DistanceConstraintInfo>(edges.Count, Allocator.Persistent);
            foreach (var e in edges)
            {
                this.AddDistanceConstraint(e.vIndex0, e.vIndex1);
            }
        }

        /// <summary>
        /// 添加距离约束器
        /// </summary>
        /// <param name="index0"></param>
        /// <param name="index1"></param>
        private void AddDistanceConstraint(int index0, int index1)
        {
            var restLength = math.distance(this.positions[index0], this.positions[index1]);
            //TODO:??
            _constraintTypes[index0] |= ConstraintType.Distance;
            _constraintTypes[index1] |= ConstraintType.Distance;
            //??
            _distanceConstraints.Add(new DistanceConstraintInfo()
            {
                restLength = restLength,
                vIndex0 = index0,
                vIndex1 = index1
            });
        }

        /// <summary>
        /// 构建弯曲约束
        /// </summary>
        private void BuildBendConstraints()
        {
            var edges = _meshModifier.edges;
            _bendConstraints = new NativeList<BendConstraintInfo>(edges.Count, Allocator.Persistent);
            foreach (var e in edges)
            {
                //意思是边只被两个三角形共用时，其实就是说相邻边才具有弯曲限制，其他都是独享边，边被一个三角形独享
                if (e.triangleIndexes.Count == 2)
                {
                    //未传参的点
                    var v2 = _meshModifier.GetTriangleVertexIndexExcept(e.triangleIndexes[0], e.vIndex0, e.vIndex1);
                    //未传参的点
                    var v3 = _meshModifier.GetTriangleVertexIndexExcept(e.triangleIndexes[1], e.vIndex0, e.vIndex1);

                    //过滤错误情况
                    if (v2 < 0 || v3 < 0)
                    {
                        Debug.LogError("mesh data error");
                        continue;
                    }

                    //写入四个点，其实就是一个平行四边形，约束其中两个点的弯曲度
                    var bendConstraint = new BendConstraintInfo();
                    bendConstraint.vIndex0 = e.vIndex0;
                    bendConstraint.vIndex1 = e.vIndex1;
                    bendConstraint.vIndex2 = v2;
                    bendConstraint.vIndex3 = v3;

                    var p0 = this.positions[bendConstraint.vIndex0];
                    var p1 = this.positions[bendConstraint.vIndex1] - p0;
                    var p2 = this.positions[bendConstraint.vIndex2] - p0;
                    var p3 = this.positions[bendConstraint.vIndex3] - p0;
                    //两个法向量
                    var n1 = math.normalize(math.cross(p1, p2));
                    var n2 = math.normalize(math.cross(p1, p3));
                    //通过两个法向量去夹角
                    bendConstraint.rest = math.acos(math.dot(n1, n2));
                    _bendConstraints.Add(bendConstraint);
                }
            }
        }

        public void AddCollider(ICollider collider)
        {
            _colliderProxies.Add(collider.entityId, collider);
        }


        public int vertexCount
        {
            get
            {
                return _meshModifier.vertexCount;
            }
        }


        public NativeArray<float3> positions
        {
            get
            {
                return _meshModifier.vertices;
            }
        }

        public NativeArray<float3> normals
        {
            get
            {
                return _meshModifier.normals;
            }
        }

        public NativeArray<float3> predictPositions
        {
            get
            {
                return _predictPositions;
            }
        }

        public float compressStiffness
        {
            get
            {
                return _setting.compressStiffness;
            }
        }

        public float stretchStiffness
        {
            get
            {
                return _setting.stretchStiffness;
            }
        }

        public int distanceConstraintCount
        {
            get
            {
                return _distanceConstraints.Length;
            }
        }

        public DistanceConstraintInfo GetDistanceConstraintInfo(int constraintIndex)
        {
            return _distanceConstraints[constraintIndex];
        }

        /// <summary>
        /// 设置一个场力 - 例如风
        /// </summary>
        public void SetFieldForce(float3 fieldForce)
        {
            _fieldForce = fieldForce;
        }

        public void AddPinConstraint(int index)
        {
            this._pinConstraints[index] = new PinConstraintInfo()
            {
                position = this.positions[index]
            };
            _constraintTypes[index] |= ConstraintType.Pin;
        }

        public void AddPinConstraint(int index, float3 position)
        {
            this._pinConstraints[index] = new PinConstraintInfo()
            {
                position = position
            };
            _constraintTypes[index] |= ConstraintType.Pin;
        }

        public int constraintSolverIteratorCount
        {
            get
            {
                return _setting.constraintSolverIteratorCount;
            }
        }

        /// <summary>
        /// 计算法线Job
        /// </summary>
        /// <returns></returns>
        private JobHandle CalculateNormalsJob()
        {
            var job = new NormalCalulateJob();
            job.indices = _meshModifier.indices;
            job.normals = _meshModifier.normals;
            job.positions = _meshModifier.vertices;
            return job.Schedule();
        }

        /// <summary>
        /// 预估的Job
        /// </summary>
        /// <param name="depend"></param>
        /// <param name="dt"></param>
        /// <returns></returns>
        private JobHandle EstimatePositionsJob(JobHandle depend, float dt)
        {
            var job = new PositionEstimateJob();
            job.normals = this.normals;
            job.fieldForce = _fieldForce;
            job.positions = positions;
            job.velocities = _velocities;
            job.predictPositions = _predictPositions;
            job.masses = _masses;
            job.damper = _setting.damper;
            job.dt = dt;
            /// <summary>
            /// ?? 64 个一个Job??
            /// </summary>
            /// <returns></returns>
            return job.Schedule(positions.Length, 64, depend);
        }

        private float GetStiffnessForIteration(float stiffness, float iterationCount)
        {
            float di = 1.0f / iterationCount;
            return 1 - math.pow(1 - stiffness, di);
        }
        private JobHandle StartDistanceConstraintsJob(JobHandle depend, int iterIndex)
        {
            int iteratorCount = this.constraintSolverIteratorCount;
            float di = 1.0f / (iteratorCount - iterIndex);
            //??
            var compressStiffnessIt = GetStiffnessForIteration(this.compressStiffness, iteratorCount);
            //!
            var stretchStiffnessIt = GetStiffnessForIteration(this.stretchStiffness, iteratorCount);
            var job = new DistanceConstraintJob();
            job.di = di;
            job.distanceConstriants = _distanceConstraints;
            job.predictPositions = _predictPositions;
            job.masses = _masses;
            job.positionCorrects = _positionCorrects;
            job.stretchStiffness = compressStiffnessIt;
            job.compressStiffness = stretchStiffnessIt;
            return job.Schedule(_distanceConstraints.Length, depend);
        }


        private JobHandle StartBendConstraintsJob(JobHandle depend, int iterIndex)
        {
            float di = 1.0f / (this.constraintSolverIteratorCount - iterIndex);
            var job = new BendConstaintsGenerateJob();
            job.bendConstarints = _bendConstraints;
            job.masses = _masses;
            job.predictPositions = _predictPositions;
            job.verticesCorrectResult = _positionCorrects;
            job.di = di;
            job.bendStiffness = GetStiffnessForIteration(this._setting.bendStiffness, this.constraintSolverIteratorCount);
            return job.Schedule(_bendConstraints.Length, depend);
        }

        private JobHandle StartFinalConstraintsJob(JobHandle depend, int iteratorIndex)
        {
            float di = 1.0f / (this.constraintSolverIteratorCount - iteratorIndex);
            var job = new ConstraintsJob(positions, predictPositions, di);
            job.masses = _masses;
            job.pinConstriants = _pinConstraints;
            job.collisionConstraints = _collisionConstraints;
            job.activeConstraintTypes = _constraintTypes;
            job.iterateIndex = iteratorIndex;
            job.iteratorCount = this.constraintSolverIteratorCount;
            job.positionCorrect = _positionCorrects;
            return job.Schedule(predictPositions.Length, 64, depend);
        }

        /// <summary>
        /// 逐个解决约束器
        /// </summary>
        /// <param name="depend"></param>
        /// <returns></returns>
        private JobHandle StartConstraintsSolveJob(JobHandle depend)
        {
            JobHandle jobHandle = depend;
            var predictPositions = this._predictPositions;

            //TODO：这里的迭代次数很关键，很影响性能
            //! 重点 迭代次数
            //通过设置好的 迭代次数，去解决约束器上的问题
            for (var i = 0; i < this.constraintSolverIteratorCount; i++)
            {
                /// <summary>
                /// 距离约束
                /// </summary>
                /// <returns></returns>
                jobHandle = StartDistanceConstraintsJob(jobHandle, i);
                /// <summary>
                /// 弯曲约束
                /// </summary>
                /// <returns></returns>
                jobHandle = StartBendConstraintsJob(jobHandle, i);
                jobHandle = StartFinalConstraintsJob(jobHandle, i);
            }
            return jobHandle;
        }

        private JobHandle StartUpdateVelocitiesAndPositions(JobHandle depend, float dt)
        {
            var job = new UpdateVelocitiesAndPositionsJob();
            job.collisionConstraintInfos = _collisionConstraints;
            job.constraintTypes = _constraintTypes;
            job.positions = this.positions;
            job.predictPositions = this._predictPositions;
            job.velocities = this._velocities;
            job.dt = dt;
            return job.Schedule(this._velocities.Length, 64, depend);
        }


        private void UpdateColliderDescs()
        {
            //清理上一次的信息
            _colliderDescGroups.Clear();
            foreach (var pair in _colliderProxies)
            {
                //更新最新的 碰撞盒信息
                pair.Value.FillColliderDesc(_colliderDescGroups);
            }
        }

        /// <summary>
        /// 碰撞检测 Job
        /// </summary>
        /// <param name="depend"></param>
        /// <param name="dt"></param>
        /// <returns></returns>
        private JobHandle StartCollisionDetectionJob(JobHandle depend, float dt)
        {
            this.UpdateColliderDescs();
            //清理残留信息
            _rigidBodyForceApplys.Clear();
            var job = new CollisionJob();
            job.dt = dt;
            job.constraintTypes = _constraintTypes;
            job.collidersGroup = _colliderDescGroups;
            job.collisionConstraints = _collisionConstraints;
            job.masses = _masses;
            job.positions = positions;
            job.predictPositions = _predictPositions;
            //?? 并行写入？？
            job.rigidBodyForceApplies = _rigidBodyForceApplys.AsParallelWriter();
            return job.Schedule(vertexCount, 64, depend);
        }


        private void ApplyPhysicalSimulationToRigidbodies()
        {
            for (var i = 0; i < _rigidBodyForceApplys.Length; i++)
            {
                var forceInfo = _rigidBodyForceApplys[i];
                _colliderProxies[forceInfo.entityId].attachedRigidbody.AddForce(forceInfo.velocity, ForceMode.VelocityChange);
            }
        }

        private JobHandle _jobHandle;
        /// <summary>
        /// Tick Job System
        /// </summary>
        /// <param name="dt"></param>
        public void Step(float dt)
        {
            //? 为何指针覆盖？  传递 handle 是为了表示依赖关心，这里最后是串行 Job ，下一个Job 等待上一个Job结果
            var handle = this.CalculateNormalsJob();
            //预测位置
            handle = this.EstimatePositionsJob(handle, dt);
            handle = this.StartCollisionDetectionJob(handle, dt);
            handle = StartConstraintsSolveJob(handle);
            handle = StartUpdateVelocitiesAndPositions(handle, dt);
            _jobHandle = handle;
        }

        public void ComputeAsyncJobs()
        {
            _jobHandle.Complete();
            this.ApplyPhysicalSimulationToRigidbodies();
        }

        public void Dispose()
        {
            _constraintTypes.Dispose();
            _velocities.Dispose();
            _predictPositions.Dispose();
            _distanceConstraints.Dispose();
            _bendConstraints.Dispose();
            _pinConstraints.Dispose();
            _collisionConstraints.Dispose();
            _masses.Dispose();
            _meshModifier.Dispose();
            _colliderDescGroups.Dispose();
            _rigidBodyForceApplys.Dispose();
            _positionCorrects.Dispose();
        }
    }

    public struct RigidbodyColliderDesc<T> where T : IColliderDesc
    {
        public int entityId;
        public T collider;
        public RigidbodyDesc rigidbody;
    }

    public interface ICollider
    {

        int entityId
        {
            get;
        }

        Rigidbody attachedRigidbody
        {
            get;
        }

        bool GetConcatInfo(float3 fromPosition, float3 toPosition, out ConcatInfo concatInfo);

        void FillColliderDesc(CollidersGroup collidersGroup);

    }

    /// <summary>
    /// 存储场景中的碰撞器
    /// </summary>
    public struct CollidersGroup
    {

        private NativeList<RigidbodyColliderDesc<SphereDesc>> _spheres;
        private NativeList<RigidbodyColliderDesc<BoxDesc>> _boxes;
        private NativeList<RigidbodyColliderDesc<CapsuleDesc>> _capsules;

        public CollidersGroup(Allocator allocator)
        {
            this._spheres = new NativeList<RigidbodyColliderDesc<SphereDesc>>(2, allocator);
            this._boxes = new NativeList<RigidbodyColliderDesc<BoxDesc>>(2, allocator);
            this._capsules = new NativeList<RigidbodyColliderDesc<CapsuleDesc>>(2, allocator);
        }


        public void AddSphere(SphereDesc sphere, RigidbodyDesc rigidbodyDesc, int entityId)
        {
            _spheres.Add(new RigidbodyColliderDesc<SphereDesc>()
            {
                entityId = entityId,
                collider = sphere,
                rigidbody = rigidbodyDesc
            });
        }

        public void AddBox(BoxDesc box, RigidbodyDesc rigidbodyDesc, int entityId)
        {
            _boxes.Add(new RigidbodyColliderDesc<BoxDesc>()
            {
                entityId = entityId,
                collider = box,
                rigidbody = rigidbodyDesc,
            });
        }

        public void AddCapsule(CapsuleDesc capsule, RigidbodyDesc rigidbodyDesc, int entityId)
        {
            _capsules.Add(new RigidbodyColliderDesc<CapsuleDesc>()
            {
                entityId = entityId,
                collider = capsule,
                rigidbody = rigidbodyDesc,
            });
        }

        public NativeArray<RigidbodyColliderDesc<SphereDesc>> spheres
        {
            get
            {
                return _spheres;
            }
        }

        public NativeArray<RigidbodyColliderDesc<BoxDesc>> boxes
        {
            get
            {
                return _boxes;
            }
        }

        public NativeArray<RigidbodyColliderDesc<CapsuleDesc>> capsules
        {
            get
            {
                return _capsules;
            }
        }
        public void Clear()
        {
            this._spheres.Clear();
            this._boxes.Clear();
            this._capsules.Clear();
        }

        public void Dispose()
        {
            this._spheres.Dispose();
            this._boxes.Dispose();
            this._capsules.Dispose();
        }
    }

    /// <summary>
    /// TODO:??  
    /// </summary>
    public struct RigidBodyForceApply
    {
        public int entityId;
        public float3 velocity;
    }
}






