#region

using UnityEngine;
using UnityEngine.UI;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using Demo.Behaviours;
using NavJob.Components;
using NavJob.Systems;

#endregion

namespace Demo
{
    public class SpawnSystem : ComponentSystem
    {
        public int pendingSpawn;
        private EntityManager _manager;

        private PopulationSpawner _spawner;
        private int _lastSpawned;
        private float _nextUpdate;

        private Vector3 one = Vector3.one;
        private EntityArchetype agent;

        private int spawned;

        private Text spawnedText;

        private Text SpawnedText
        {
            get
            {
                if (spawnedText == null)
                {
                    spawnedText = GameObject.Find("SpawnedText").GetComponent<Text>();
                }

                return spawnedText;
            }
        }

        private PopulationSpawner Getspawner()
        {
            if (_spawner == null)
            {
                _spawner = Object.FindObjectOfType<PopulationSpawner>();
            }

            return _spawner;
        }

        private EntityManager Getmanager()
        {
            if (_manager == null)
            {
                _manager = EntityManager;
            }

            return _manager;
        }

        protected override void OnCreate()
        {
            base.OnCreate();
            // create the system
            World.DefaultGameObjectInjectionWorld.GetOrCreateSystem<NavAgentSystem>();
            World.DefaultGameObjectInjectionWorld.GetOrCreateSystem<NavAgentToPositionSyncSystem>();
            agent = Getmanager().CreateArchetype(
                typeof(NavAgent),
                // optional avoidance
                // typeof(NavAgentAvoidance),
                // optional
                // typeof (Position),
                // typeof (Rotation),
                // typeof (SyncPositionToNavAgent),
                // typeof (SyncRotationToNavAgent),
                // typeof (SyncPositionFromNavAgent),
                // typeof (SyncRotationFromNavAgent),
                typeof(LocalToWorld)
            );
        }

        private BuildingCacheSystem buildings;
        
        
        protected override void OnUpdate()
        {
            
            if (UnityEngine.Time.time > _nextUpdate && _lastSpawned != spawned)
            {
                _nextUpdate = UnityEngine.Time.time + 0.5f;
                _lastSpawned = spawned;
                SpawnedText.text = $"Spawned: {spawned} people";
            }

            if (Getspawner().Renderers.Length == 0)
            {
                return;
            }

            if (buildings.ResidentialBuildings.Length == 0)
            {
                return;
            }
            
            var spawnData = GetSingleton<PendingSpawn>();// data.Spawn[0];
            pendingSpawn = spawnData.Quantity;
            spawnData.Quantity = 0;
            
            var manager = Getmanager();
            var query = Entities.WithAll<PendingSpawn>().ToEntityQuery().ToComponentDataArray<PendingSpawn>(Allocator.Temp);
            for (var i = 0; i < pendingSpawn; i++)
            {
                spawned++;
                var position = buildings.GetResidentialBuilding();
                var entity = manager.CreateEntity(agent);
                var navAgent = new NavAgent(
                    position,
                    Quaternion.identity,
                    spawnData.AgentStoppingDistance,
                    spawnData.AgentMoveSpeed,
                    spawnData.AgentAcceleration,
                    spawnData.AgentRotationSpeed,
                    spawnData.AgentAreaMask
                );
                // optional if set on the archetype
                // manager.SetComponentData (entity, new Position { Value = position });
                manager.SetComponentData(entity, navAgent);
                // optional for avoidance
                // var navAvoidance = new NavAgentAvoidance(2f);
                // manager.SetComponentData(entity, navAvoidance);
                manager.AddSharedComponentData(entity, Getspawner().Renderers[UnityEngine.Random.Range(0, Getspawner().Renderers.Length)].Value);
            }
            return;
        }

        
    }

    public class DetectIdleAgentSystem : ComponentSystem
    {
        public struct AgentData
        {
            public int index;
            public Entity entity;
            public NavAgent agent;
        }

        private Text awaitingNavmeshText;

        private Text AwaitingNavmeshText
        {
            get
            {
                if (awaitingNavmeshText == null)
                {
                    awaitingNavmeshText = GameObject.Find("AwaitingNavmeshText").GetComponent<Text>();
                }

                return awaitingNavmeshText;
            }
        }

        private Text cachedPathText;

        private Text CachedPathText
        {
            get
            {
                if (cachedPathText == null)
                {
                    cachedPathText = GameObject.Find("CachedPathText").GetComponent<Text>();
                }

                return cachedPathText;
            }
        }

        private float _nextUpdate;

        private NativeQueue<AgentData> needsPath = new NativeQueue<AgentData>(Allocator.Persistent);

        [BurstCompile]
        private struct DetectIdleAgentJob : IJobParallelFor
        {
            public InjectData data;
            [NativeDisableParallelForRestriction]
            public NativeQueue<AgentData> needsPath;

            public void Execute(int index)
            {
                var agent = data.Agents[index];
                if (data.Agents[index].status == AgentStatus.Idle)
                {
                    needsPath.Enqueue(new AgentData { index = index, agent = agent, entity = data.Entities[index] });
                    agent.status = AgentStatus.PathQueued;
                    data.Agents[index] = agent;
                }
            }
        }

        private struct SetNextPathJob : IJob
        {
            public InjectData data;
            public NativeQueue<AgentData> needsPath;
            public void Execute()
            {
                while (needsPath.TryDequeue(out AgentData item))
                {
                    var destination = BuildingCacheSystem.GetCommercialBuilding();
                    NavAgentSystem.SetDestinationStatic(item.entity, item.agent, destination, item.agent.areaMask);
                }
            }
        }

        //private struct InjectData
        //{
        //    public readonly int Length;
        //    [ReadOnly] public EntityArray Entities;
        //    public ComponentDataArray<NavAgent> Agents;
        //}

        //[Inject] InjectData data;
        //[Inject] NavMeshQuerySystem navQuery;

        protected override void OnUpdate()
        {
            var query = Entities.WithAll<NavAgent>().ToEntityQuery().ToComponentDataArray<NavAgent>(Allocator.TempJob);
            var navQuery = World.GetOrCreateSystem<NavMeshQuerySystem>();
            if (UnityEngine.Time.time > _nextUpdate)
            {
                AwaitingNavmeshText.text = $"Awaiting Path: {navQuery.PendingCount} people";
                CachedPathText.text = $"Cached Paths: {navQuery.CachedCount}";
                _nextUpdate = UnityEngine.Time.time + 0.5f;
            }
            var inputDeps = new DetectIdleAgentJob { data = query, needsPath = needsPath }.Schedule(query.Length, 64);
            inputDeps = new SetNextPathJob { data = query, needsPath = needsPath }.Schedule(inputDeps);
            inputDeps.Complete();
        }

        protected override void OnDestroy()
        {
            needsPath.Dispose();
        }
    }

    public class BuildingCacheSystem : ComponentSystem
    {
        public NativeList<Vector3> CommercialBuildings = new NativeList<Vector3>(Allocator.Persistent);
        public NativeList<Vector3> ResidentialBuildings = new NativeList<Vector3>(Allocator.Persistent);
        private PopulationSpawner spawner;
        private int nextCommercial = 0;
        private int nextResidential = 0;
        private static BuildingCacheSystem instance;

        protected override void OnCreate()
        {
            instance = this;
        }

        //[Inject] private InjectData data;

        //private struct InjectData
        //{
        //    public readonly int Length;
        //    [ReadOnly] public ComponentDataArray<BuildingData> Buildings;
        //}

        private PopulationSpawner Spawner
        {
            get
            {
                if (spawner == null)
                {
                    spawner = Object.FindObjectOfType<PopulationSpawner>();
                }

                return spawner;
            }
        }

        public Vector3 GetResidentialBuilding()
        {
            nextResidential++;
            if (nextResidential >= ResidentialBuildings.Length)
            {
                nextResidential = 0;
            }

            return ResidentialBuildings[nextResidential];
        }

        public static Vector3 GetCommercialBuilding()
        {
            var building = instance.CommercialBuildings[0];
            try
            {
                if (instance.nextCommercial < instance.CommercialBuildings.Length)
                {
                    building = instance.CommercialBuildings[instance.nextCommercial];
                    instance.nextCommercial++;
                }
                else
                {
                    instance.nextCommercial = 0;
                }
                return building;
            }
            catch
            {
                return building;
            }
        }

        protected override void OnUpdate()
        {
            var query = Entities.WithAll<BuildingData>().ToEntityQuery().ToComponentDataArray<BuildingData>(Allocator.Temp);
            for (var i = 0; i < query.Length; i++)
            {
                var building = query[i];
                if (building.Type == BuildingType.Residential)
                {
                    ResidentialBuildings.Add(building.Position);
                }
                else
                {
                    CommercialBuildings.Add(building.Position);
                }

                PostUpdateCommands.RemoveComponent<BuildingData>(building.Entity);
            }
        }

        protected override void OnDestroy()
        {
            ResidentialBuildings.Dispose();
            CommercialBuildings.Dispose();
        }
    }
}