using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Collections;

namespace PBDLearn
{

    /// <summary>
    /// 顶点约束信息
    /// </summary>
    public struct DistanceConstraintInfo
    {
        public float restLength;
        public int vIndex0;
        public int vIndex1;
    }

    /// <summary>
    /// 弯曲约束信息
    /// </summary>
    public struct BendConstraintInfo
    {
        public int vIndex0;
        public int vIndex1;
        public int vIndex2;
        public int vIndex3;
        public float rest; // angle
    }

    public struct PinConstraintInfo
    {
        public float3 position;
    }

    /// <summary>
    /// 碰撞约束
    /// </summary>
    public struct CollisionConstraintInfo
    {
        public float3 concatPosition;
        public float3 normal;
        public float3 velocity;
    }

    [Unity.Burst.BurstCompile]
    struct PositionEstimateJob : IJobParallelFor
    {
        [ReadOnly]
        public NativeArray<float3> positions;
        [ReadOnly]
        public NativeArray<float3> normals;

        [ReadOnly]
        public NativeArray<float3> velocities;
        [ReadOnly]
        public NativeArray<float> masses;
        [WriteOnly]
        public NativeArray<float3> predictPositions;
        /// <summary>
        /// 场力
        /// </summary>
        public float3 fieldForce;
        /// <summary>
        /// 阻尼
        /// </summary>
        public float damper;
        public float dt;
        public void Execute(int index)
        {
            var p = positions[index];
            var v = velocities[index];
            var m = masses[index];
            if (m > 0)
            {
                var normal = normals[index];
                //?? 为啥要和法线运算  , 这里单独的是风的场力，因为布料的均匀介质，和受力角度关系很大，所以要点乘，再乘以法线，表示一个矢量力度（长度就是力的大小）
                var fieldForceAtNormal = math.dot(fieldForce, normal) * normal;
                //?? 为啥要 除以 mass ??
                // v =  v + gt + a*t/mass
                var v1 = v + ClothSimulator.G * dt + fieldForceAtNormal * dt / m;
                //等同于 v = v - at,  a 和阻尼系数有关
                v1 *= math.max(0, (1 - damper * dt / m)); //阻尼
                //得到预测位置
                var p1 = p + v1 * dt;
                //写入预测位置
                predictPositions[index] = p1;
            }
            else
            {
                //没有质量的点，保留在原地，例如布料栓住的点
                predictPositions[index] = p;
            }

        }
    }

    [Unity.Burst.BurstCompile]
    public struct CollisionJob : IJobParallelFor
    {
        [ReadOnly]
        public float dt;
        [ReadOnly]
        public NativeArray<float3> positions;
        [ReadOnly]
        public NativeArray<float3> predictPositions;

        [ReadOnly]
        public NativeArray<float> masses;

        [ReadOnly]
        public CollidersGroup collidersGroup;

        [WriteOnly]
        public NativeArray<CollisionConstraintInfo> collisionConstraints;
        public NativeArray<ConstraintType> constraintTypes;
        public NativeList<RigidBodyForceApply>.ParallelWriter rigidBodyForceApplies;


        /// <summary>
        /// 执行碰撞约束
        /// </summary>
        /// <param name="index"></param>
        /// <param name="concatInfo"></param>
        /// <param name="rigidbody"></param>
        /// <param name="entityId"></param>
        private void EnableCollisionConstraint(int index, ref ConcatInfo concatInfo, ref RigidbodyDesc rigidbody, int entityId)
        {
            //标记一下这个顶点 有 碰撞约束
            constraintTypes[index] |= ConstraintType.Collision;

            var collisionConstraintInfo = new CollisionConstraintInfo()
            {
                //?? 为啥是 0.05f 精度问题？？
                concatPosition = concatInfo.position + concatInfo.normal * 0.05f,
                normal = concatInfo.normal
            };
            var m1 = rigidbody.mass;
            if (m1 > 0)
            {
                //如果碰撞盒有质量，才让他受力

                //反弹力
                var bounciness = rigidbody.bounciness;
                var m0 = masses[index];
                //计算历史位置到预测位置的 平均速度
                var v0 = (predictPositions[index] - positions[index]) / dt;
                //表示 v0 在 normal 上的投影
                var v0Normal = math.dot(v0, concatInfo.normal) * concatInfo.normal;
                //碰撞盒速度在 normal 上的投影
                var v1Normal = math.dot(rigidbody.velocity, concatInfo.normal) * concatInfo.normal;


                //?? 冲量计算
                var v0NormalNew = (bounciness + 1) * m1 * v1Normal + v0Normal * (m0 - bounciness * m1);
                var v1NormalNew = (bounciness + 1) * m0 * v0Normal + v1Normal * (m1 - bounciness * m0);

                v0NormalNew /= (m0 + m1);
                v1NormalNew /= (m0 + m1);

                rigidBodyForceApplies.AddNoResize(new RigidBodyForceApply()
                {
                    entityId = entityId,
                    velocity = v1NormalNew - v1Normal
                });
                collisionConstraintInfo.velocity = (v0 - v0Normal + v0NormalNew);
            }

            collisionConstraints[index] = collisionConstraintInfo;
        }

        private void DisableCollisionConstraint(int index)
        {
            constraintTypes[index] &= (~ConstraintType.Collision);
        }


        public void Execute(int index)
        {
            //上一次的坐标，相当于历史坐标
            var position = this.positions[index];
            //拿到计算好的预测坐标
            var predictPosition = this.positions[index];
            var spheres = collidersGroup.spheres;
            var boxes = collidersGroup.boxes;
            var capsules = collidersGroup.capsules;

            //分别对不同的碰撞盒检测
            for (var i = 0; i < spheres.Length; i++)
            {
                var s = spheres[i];
                ConcatInfo concatInfo;
                //判断是否碰撞
                if (IntersectUtil.GetClosestSurfacePoint(position, s.collider, out concatInfo))
                {
                    EnableCollisionConstraint(index, ref concatInfo, ref s.rigidbody, s.entityId);
                    //这里认为，只要有碰撞，后面就不需要判断，但是这样容易遗漏
                    return;
                }
            }

            for (var i = 0; i < boxes.Length; i++)
            {
                var s = boxes[i];
                ConcatInfo concatInfo;
                if (IntersectUtil.GetClosestSurfacePoint(position, s.collider, out concatInfo))
                {
                    EnableCollisionConstraint(index, ref concatInfo, ref s.rigidbody, s.entityId);
                    //这里认为，只要有碰撞，后面就不需要判断，但是这样容易遗漏
                    return;
                }
            }

            for (var i = 0; i < capsules.Length; i++)
            {
                var s = capsules[i];
                ConcatInfo concatInfo;
                if (IntersectUtil.GetClosestSurfacePoint(position, s.collider, out concatInfo))
                {
                    EnableCollisionConstraint(index, ref concatInfo, ref s.rigidbody, s.entityId);
                    //这里认为，只要有碰撞，后面就不需要判断，但是这样容易遗漏
                    return;
                }
            }
            DisableCollisionConstraint(index);
        }
    }

    /// <summary>
    /// 距离约束任务
    /// </summary>
    [Unity.Burst.BurstCompile]
    public struct DistanceConstraintJob : IJobFor
    {
        [ReadOnly]
        public float compressStiffness;

        [ReadOnly]
        public float stretchStiffness;

        [ReadOnly]
        public NativeArray<float3> predictPositions;

        [ReadOnly]
        public NativeArray<float> masses;

        [ReadOnly]
        public NativeArray<DistanceConstraintInfo> distanceConstriants;

        public NativeArray<float3> positionCorrects;

        public float di;

        public void Execute(int index)
        {
            var constraint = distanceConstriants[index];

            //取得 约束器内的 预测点 坐标
            var p0 = predictPositions[constraint.vIndex0];
            var p1 = predictPositions[constraint.vIndex1];
            //约束其 的 定点质量
            var m0 = masses[constraint.vIndex0];
            var m1 = masses[constraint.vIndex1];
            //预测后的距离
            var distV = p1 - p0;
            var normal = math.normalize(distV);
            var length = math.length(distV);
            //和提前算好的约束器中的值比较
            var err = length - constraint.restLength;
            float3 correct;
            if (err < 0)
            {
                //小于约束值 ， 表现压力
                correct = compressStiffness * normal * err;
            }
            else
            {
                //大于约束值  ， 表现拉力
                correct = stretchStiffness * normal * err;
            }
            var totalM = m0 + m1;
            positionCorrects[constraint.vIndex0] += correct * di * m1 / totalM;
            positionCorrects[constraint.vIndex1] -= correct * di * m0 / totalM;
        }
    }

    [Unity.Burst.BurstCompile]
    /// <summary>
    /// 为每一条边(即一对三角面片)生成关于4顶点的约束信息
    /// </summary>
    public struct BendConstaintsGenerateJob : IJobFor
    {

        [ReadOnly]
        public NativeArray<BendConstraintInfo> bendConstarints;

        [ReadOnly]
        public NativeArray<float3> predictPositions;

        [ReadOnly]
        public NativeArray<float> masses;

        public NativeArray<float3> verticesCorrectResult;

        [ReadOnly]
        public float di;

        [ReadOnly]
        public float bendStiffness;

        public void Execute(int index)
        {
            var cons = bendConstarints[index];

            var p1 = predictPositions[cons.vIndex0];
            var p2 = predictPositions[cons.vIndex1] - p1;
            var p3 = predictPositions[cons.vIndex2] - p1;
            var p4 = predictPositions[cons.vIndex3] - p1;
            p1 = 0;
            var n1 = math.normalize(math.cross(p2, p3));
            var n2 = math.normalize(math.cross(p2, p4));

            var d = math.dot(n1, n2);

            var p23Len = math.length(math.cross(p2, p3));
            var p24Len = math.length(math.cross(p2, p4));

            var q3 = (math.cross(p2, n2) + math.cross(n1, p2) * d) / p23Len;
            var q4 = (math.cross(p2, n1) + math.cross(n2, p2) * d) / p24Len;
            var q2 = -(math.cross(p3, n2) + math.cross(n1, p3) * d) / p23Len
            - (math.cross(p4, n1) + math.cross(n2, p4) * d) / p24Len;
            var q1 = -q2 - q3 - q4;

            var w1 = 1 / masses[cons.vIndex0];
            var w2 = 1 / masses[cons.vIndex1];
            var w3 = 1 / masses[cons.vIndex2];
            var w4 = 1 / masses[cons.vIndex3];

            var sum = w1 * math.lengthsq(q1)
            + w2 * math.lengthsq(q2)
            + w3 * math.lengthsq(q3)
            + w4 * math.lengthsq(q4);

            sum = math.max(0.01f, sum);

            var s = -(math.acos(d) - cons.rest) * math.sqrt(1 - d * d) / sum;

            if (math.isfinite(s))
            {
                var dp1 = s * w1 * q1 * di * bendStiffness;
                var dp2 = s * w2 * q2 * di * bendStiffness;
                var dp3 = s * w3 * q3 * di * bendStiffness;
                var dp4 = s * w4 * q4 * di * bendStiffness;
                verticesCorrectResult[cons.vIndex0] += dp1;
                verticesCorrectResult[cons.vIndex1] += dp2;
                verticesCorrectResult[cons.vIndex2] += dp3;
                verticesCorrectResult[cons.vIndex3] += dp4;
            }

        }
    }




    /// <summary>
    /// 约束合计
    /// </summary>
    [Unity.Burst.BurstCompile]
    public struct ConstraintsJob : IJobParallelFor
    {

        /// <summary>
        /// 总共迭代次数
        /// </summary>
        public int iteratorCount;

        /// <summary>
        /// 当前第几次迭代
        /// </summary>
        public int iterateIndex;


        [ReadOnly]
        public NativeArray<float3> positions;

        public NativeArray<float3> predictPositions;


        [ReadOnly]
        public NativeArray<float> masses;

        [ReadOnly]
        public NativeArray<PinConstraintInfo> pinConstriants;


        [ReadOnly]
        public NativeArray<CollisionConstraintInfo> collisionConstraints;

        [ReadOnly]
        public NativeArray<ConstraintType> activeConstraintTypes;

        public NativeArray<float3> positionCorrect;

        public float di;

        public ConstraintsJob(NativeArray<float3> positions,
        NativeArray<float3> predictPositions, float di)
        {
            this.positions = positions;
            this.predictPositions = predictPositions;
            this.di = di;
            this.pinConstriants = default;
            this.collisionConstraints = default;
            this.activeConstraintTypes = default;
            this.iteratorCount = 1;
            this.iterateIndex = 0;
            this.masses = default;
            this.positionCorrect = default;
        }

        public void Execute(int index)
        {

            var constraintType = activeConstraintTypes[index];
            var positionCorrect = this.positionCorrect[index];
            this.positionCorrect[index] = 0;
            if ((constraintType & ConstraintType.Pin) == ConstraintType.Pin)
            {
                //固定约束
                this.predictPositions[index] = this.pinConstriants[index].position;
                return;
            }
            if ((constraintType & ConstraintType.Collision) == ConstraintType.Collision)
            {
                //碰撞约束
                var position = this.positions[index];
                var collisionInfo = collisionConstraints[index];
                var deltaP = collisionInfo.concatPosition - position;
                position += deltaP * (iterateIndex * 1f / iteratorCount);
                predictPositions[index] = position;
                return;
            }
            //其他之前计算好的约束修正
            this.predictPositions[index] += positionCorrect;
        }

    }

    [Unity.Burst.BurstCompile]
    public struct UpdateVelocitiesAndPositionsJob : IJobParallelFor
    {
        [ReadOnly]
        public NativeArray<ConstraintType> constraintTypes;

        [ReadOnly]
        public NativeArray<float3> predictPositions;

        [ReadOnly]
        public NativeArray<CollisionConstraintInfo> collisionConstraintInfos;

        public NativeArray<float3> positions;

        [WriteOnly]
        public NativeArray<float3> velocities;

        public float dt;

        public void Execute(int index)
        {
            if ((constraintTypes[index] & ConstraintType.Collision) == ConstraintType.Collision)
            {
                velocities[index] = this.collisionConstraintInfos[index].velocity;
            }
            else
            {
                velocities[index] = (predictPositions[index] - positions[index]) / dt;
            }
            positions[index] = predictPositions[index];
        }
    }


    [Unity.Burst.BurstCompile]
    public struct NormalCalulateJob : IJob
    {
        [ReadOnly]
        public NativeArray<int> indices;
        [ReadOnly]
        public NativeArray<float3> positions;


        /// <summary>
        /// 只有法线才需要读写
        /// </summary>
        public NativeArray<float3> normals;

        public void Execute()
        {
            for (var i = 0; i < normals.Length; i++)
            {
                normals[i] = 0;
            }

            for (var i = 0; i < indices.Length / 3; i++)
            {
                var offset = i * 3;
                var vIndex0 = indices[offset];
                var vIndex1 = indices[offset + 1];
                var vIndex2 = indices[offset + 2];
                var p0 = positions[vIndex0];
                var p1 = positions[vIndex1];
                var p2 = positions[vIndex2];
                //通过边来计算法向量
                var n = math.normalize(math.cross(p1 - p0, p2 - p0));

                //叠加起来，这样可以混合共用点的法线
                normals[vIndex0] += n;
                normals[vIndex1] += n;
                normals[vIndex2] += n;
            }


            //最后归一化
            for (var i = 0; i < normals.Length; i++)
            {
                normals[i] = math.normalizesafe(normals[i]);
            }
        }
    }
}
