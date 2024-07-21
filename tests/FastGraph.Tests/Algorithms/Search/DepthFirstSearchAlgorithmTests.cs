using System;
using System.Collections.Generic;
using System.Linq;
using JetBrains.Annotations;
using NUnit.Framework;
using FastGraph.Algorithms.Search;
using static FastGraph.Tests.Algorithms.AlgorithmTestHelpers;
using static FastGraph.Tests.GraphTestHelpers;
using FastGraph.Algorithms.Observers;
using System.Diagnostics;

namespace FastGraph.Tests.Algorithms.Search
{
    /// <summary>
    /// Tests for <see cref="DepthFirstSearchAlgorithm{TVertex,TEdge}"/>.
    /// </summary>
    [TestFixture]
    internal sealed class DepthFirstAlgorithmSearchTests : SearchAlgorithmTestsBase
    {
        #region Test helpers

        private static void RunDFSAndCheck<TVertex, TEdge>(
            [NotNull] IVertexListGraph<TVertex, TEdge> graph,
            int maxDepth = int.MaxValue)
            where TEdge : IEdge<TVertex>
        {
            var parents = new Dictionary<TVertex, TVertex>();
            var discoverTimes = new Dictionary<TVertex, int>();
            var finishTimes = new Dictionary<TVertex, int>();
            int time = 0;
            var dfs = new DepthFirstSearchAlgorithm<TVertex, TEdge>(graph)
            {
                MaxDepth = maxDepth
            };

            dfs.InitializeVertex += vertex =>
            {
                Assert.AreEqual(GraphColor.White, dfs.VerticesColors[vertex]);
            };

            dfs.StartVertex += vertex =>
            {
                Assert.AreEqual(GraphColor.White, dfs.VerticesColors[vertex]);
                Assert.IsFalse(parents.ContainsKey(vertex));
                parents[vertex] = vertex;
            };

            dfs.DiscoverVertex += vertex =>
            {
                Assert.AreEqual(GraphColor.Gray, dfs.VerticesColors[vertex]);
                Assert.AreEqual(GraphColor.Gray, dfs.VerticesColors[parents[vertex]]);

                discoverTimes[vertex] = time++;
            };

            dfs.ExamineEdge += edge =>
            {
                Assert.AreEqual(GraphColor.Gray, dfs.VerticesColors[edge.Source]);
            };

            dfs.TreeEdge += edge =>
            {
                Assert.AreEqual(GraphColor.White, dfs.VerticesColors[edge.Target]);
                parents[edge.Target] = edge.Source;
            };

            dfs.BackEdge += edge =>
            {
                Assert.AreEqual(GraphColor.Gray, dfs.VerticesColors[edge.Target]);
            };

            dfs.ForwardOrCrossEdge += edge =>
            {
                Assert.AreEqual(GraphColor.Black, dfs.VerticesColors[edge.Target]);
            };

            dfs.FinishVertex += vertex =>
            {
                Assert.AreEqual(GraphColor.Black, dfs.VerticesColors[vertex]);
                finishTimes[vertex] = time++;
            };

            dfs.Compute();

            // Check
            // All vertices should be black
            foreach (TVertex vertex in graph.Vertices)
            {
                Assert.IsTrue(dfs.VerticesColors.ContainsKey(vertex));
                Assert.AreEqual(dfs.VerticesColors[vertex], GraphColor.Black);
            }

            foreach (TVertex u in graph.Vertices)
            {
                foreach (TVertex v in graph.Vertices)
                {
                    if (!u.Equals(v))
                    {
                        Assert.IsTrue(
                            finishTimes[u] < discoverTimes[v]
                            || finishTimes[v] < discoverTimes[u]
                            || (discoverTimes[v] < discoverTimes[u] && finishTimes[u] < finishTimes[v] && IsDescendant(parents, u, v))
                            || (discoverTimes[u] < discoverTimes[v] && finishTimes[v] < finishTimes[u] && IsDescendant(parents, v, u)));
                    }
                }
            }
        }

        #endregion Test helpers

        [Test]
        public void Constructor()
        {
            var graph = new AdjacencyGraph<int, Edge<int>>();
            var algorithm = new DepthFirstSearchAlgorithm<int, Edge<int>>(graph);
            AssertAlgorithmProperties(algorithm, graph);

            var verticesColors = new Dictionary<int, GraphColor>();
            algorithm = new DepthFirstSearchAlgorithm<int, Edge<int>>(graph, verticesColors);
            AssertAlgorithmProperties(algorithm, graph, verticesColors);

            algorithm = new DepthFirstSearchAlgorithm<int, Edge<int>>(null, graph);
            AssertAlgorithmProperties(algorithm, graph);

            algorithm = new DepthFirstSearchAlgorithm<int, Edge<int>>(null, graph, verticesColors);
            AssertAlgorithmProperties(algorithm, graph, verticesColors);

            algorithm = new DepthFirstSearchAlgorithm<int, Edge<int>>(null, graph, verticesColors, edges => edges.Where(e => e != null));
            AssertAlgorithmProperties(algorithm, graph, verticesColors);

            algorithm.MaxDepth = 12;
            AssertAlgorithmProperties(algorithm, graph, verticesColors, 12);

            algorithm.ProcessAllComponents = true;
            AssertAlgorithmProperties(algorithm, graph, verticesColors, 12, true);

            #region Local function

            void AssertAlgorithmProperties<TVertex, TEdge>(
                DepthFirstSearchAlgorithm<TVertex, TEdge> algo,
                IVertexListGraph<TVertex, TEdge> g,
                IDictionary<TVertex, GraphColor> vColors = null,
                int maxDepth = int.MaxValue,
                bool processAllComponents = false)
                where TEdge : IEdge<TVertex>
            {
                AssertAlgorithmState(algo, g);
                if (vColors is null)
                    CollectionAssert.IsEmpty(algo.VerticesColors);
                else
                    Assert.AreSame(vColors, algo.VerticesColors);
                Assert.AreEqual(maxDepth, algo.MaxDepth);
                Assert.AreEqual(processAllComponents, algo.ProcessAllComponents);
                Assert.IsNotNull(algo.OutEdgesFilter);
            }

            #endregion Local function
        }

        [Test]
        public void Constructor_Throws()
        {
            // ReSharper disable ObjectCreationAsStatement
            // ReSharper disable AssignNullToNotNullAttribute
            var graph = new AdjacencyGraph<int, Edge<int>>();
            var verticesColors = new Dictionary<int, GraphColor>();
            IEnumerable<Edge<int>> Filter(IEnumerable<Edge<int>> edges) => edges.Where(e => e != null);

            Assert.Throws<ArgumentNullException>(
                () => new DepthFirstSearchAlgorithm<int, Edge<int>>(null));

            Assert.Throws<ArgumentNullException>(
                () => new DepthFirstSearchAlgorithm<int, Edge<int>>(graph, null));
            Assert.Throws<ArgumentNullException>(
                () => new DepthFirstSearchAlgorithm<int, Edge<int>>(null, verticesColors));
            Assert.Throws<ArgumentNullException>(
                () => new DepthFirstSearchAlgorithm<int, Edge<int>>((IVertexListGraph<int, Edge<int>>)null, null));

            Assert.Throws<ArgumentNullException>(
                () => new DepthFirstSearchAlgorithm<int, Edge<int>>(null, (IVertexListGraph<int, Edge<int>>)null));

            Assert.Throws<ArgumentNullException>(
                () => new DepthFirstSearchAlgorithm<int, Edge<int>>(null, null, verticesColors));
            Assert.Throws<ArgumentNullException>(
                () => new DepthFirstSearchAlgorithm<int, Edge<int>>(null, graph, null));
            Assert.Throws<ArgumentNullException>(
                () => new DepthFirstSearchAlgorithm<int, Edge<int>>(null, null, null));

            Assert.Throws<ArgumentNullException>(
                () => new DepthFirstSearchAlgorithm<int, Edge<int>>(null, null, verticesColors, Filter));
            Assert.Throws<ArgumentNullException>(
                () => new DepthFirstSearchAlgorithm<int, Edge<int>>(null, graph, null, Filter));
            Assert.Throws<ArgumentNullException>(
                () => new DepthFirstSearchAlgorithm<int, Edge<int>>(null, graph, verticesColors, null));
            Assert.Throws<ArgumentNullException>(
                () => new DepthFirstSearchAlgorithm<int, Edge<int>>(null, graph, null, null));
            Assert.Throws<ArgumentNullException>(
                () => new DepthFirstSearchAlgorithm<int, Edge<int>>(null, null, null, Filter));
            Assert.Throws<ArgumentNullException>(
                () => new DepthFirstSearchAlgorithm<int, Edge<int>>(null, null, verticesColors, null));
            Assert.Throws<ArgumentNullException>(
                () => new DepthFirstSearchAlgorithm<int, Edge<int>>(null, null, null, null));
            // ReSharper restore AssignNullToNotNullAttribute
            // ReSharper restore ObjectCreationAsStatement

            Assert.Throws<ArgumentOutOfRangeException>(() => new DepthFirstSearchAlgorithm<int, Edge<int>>(graph).MaxDepth = -1);
        }

        #region Rooted algorithm

        [Test]
        public void TryGetRootVertex()
        {
            var graph = new AdjacencyGraph<int, Edge<int>>();
            var algorithm = new DepthFirstSearchAlgorithm<int, Edge<int>>(graph);
            TryGetRootVertex_Test(algorithm);
        }

        [Test]
        public void SetRootVertex()
        {
            var graph = new AdjacencyGraph<int, Edge<int>>();
            var algorithm = new DepthFirstSearchAlgorithm<int, Edge<int>>(graph);
            SetRootVertex_Test(algorithm);
        }

        [Test]
        public void SetRootVertex_Throws()
        {
            var graph = new AdjacencyGraph<TestVertex, Edge<TestVertex>>();
            var algorithm = new DepthFirstSearchAlgorithm<TestVertex, Edge<TestVertex>>(graph);
            SetRootVertex_Throws_Test(algorithm);
        }

        [Test]
        public void ClearRootVertex()
        {
            var graph = new AdjacencyGraph<int, Edge<int>>();
            var algorithm = new DepthFirstSearchAlgorithm<int, Edge<int>>(graph);
            ClearRootVertex_Test(algorithm);
        }

        [Test]
        public void ComputeWithoutRoot_Throws()
        {
            var graph = new AdjacencyGraph<int, Edge<int>>();
            ComputeWithoutRoot_NoThrows_Test(
                graph,
                () => new DepthFirstSearchAlgorithm<int, Edge<int>>(graph));
        }

        [Test]
        public void ComputeWithRoot()
        {
            var graph = new AdjacencyGraph<int, Edge<int>>();
            graph.AddVertex(0);
            var algorithm = new DepthFirstSearchAlgorithm<int, Edge<int>>(graph);
            ComputeWithRoot_Test(algorithm);
        }

        [Test]
        public void ComputeWithRoot_Throws()
        {
            var graph = new AdjacencyGraph<TestVertex, Edge<TestVertex>>();
            ComputeWithRoot_Throws_Test(
                () => new DepthFirstSearchAlgorithm<TestVertex, Edge<TestVertex>>(graph));
        }

        #endregion Rooted algorithm

        [Test]
        public void GetVertexColor()
        {
            var graph = new AdjacencyGraph<int, Edge<int>>();
            graph.AddVerticesAndEdge(new Edge<int>(1, 2));

            var algorithm = new DepthFirstSearchAlgorithm<int, Edge<int>>(graph);
            // Algorithm not run
            // ReSharper disable once ReturnValueOfPureMethodIsNotUsed
            Assert.Throws<VertexNotFoundException>(() => algorithm.GetVertexColor(1));

            algorithm.Compute();

            Assert.AreEqual(GraphColor.Black, algorithm.GetVertexColor(1));
            Assert.AreEqual(GraphColor.Black, algorithm.GetVertexColor(2));
        }

        [Test]
        public void DepthFirstSearch()
        {
            foreach (AdjacencyGraph<string, Edge<string>> graph in TestGraphFactory.GetAdjacencyGraphs_SlowTests())
            {
                RunDFSAndCheck(graph);
                RunDFSAndCheck(graph, 12);
            }
        }

        // <image url="$(ProjectDir)DocumentImages\ShreveStreamOrder.png"/>
        /// <summary>
        /// 测试通过Graph获取 Shreve Stream Order
        /// </summary>
        [TestCase(0)]
        [TestCase(1)]
        [TestCase(2)]
        [TestCase(3)]
        [TestCase(4)]
        [TestCase(5)]
        public void GraphShreveStreamOrder(int root)
        {
            // <image url="$(ProjectDir)DocumentImages\Graph_ShreveStreamOrder_Original.png"/>
            var graph = new UndirectedGraph<int, Edge<int>>(true);
            graph.AddVerticesAndEdgeRange(new[]
            {
                new Edge<int>(0, 1),
                new Edge<int>(1, 2),
                new Edge<int>(1, 3),
                new Edge<int>(2, 4),
                new Edge<int>(2, 5),
            });
            //Note需要添加反向的边
            AddReversedEdge(graph);

            // <image url="$(ProjectDir)DocumentImages\Graph_ShreveStreamOrder_0.png"/>
            // <image url="$(ProjectDir)DocumentImages\Graph_ShreveStreamOrder_1.png"/>
            // <image url="$(ProjectDir)DocumentImages\Graph_ShreveStreamOrder_2.png"/>
            // <image url="$(ProjectDir)DocumentImages\Graph_ShreveStreamOrder_3.png"/>
            // <image url="$(ProjectDir)DocumentImages\Graph_ShreveStreamOrder_4.png"/>
            // <image url="$(ProjectDir)DocumentImages\Graph_ShreveStreamOrder_5.png"/>
            var dic = GetGraphEdgeShreveOrderFromRoot(graph.ToBidirectionalGraph(), root);
        }

        private void AddReversedEdge(UndirectedGraph<int, Edge<int>> graph)
        {
            var edges = graph.Edges.ToList();
            foreach (var edge in edges)
            {
                var reversedEdge = new Edge<int>(edge.Target, edge.Source);
                if (reversedEdge == null || graph.ContainsEdge(reversedEdge)) continue;
                graph.AddEdge(reversedEdge);
            }
        }

        /// <summary>
        /// 从指定根节点获取图的Shreve Order
        /// </summary>
        /// <param name="graph">图</param>
        /// <param name="root">根节点</param>
        /// <returns></returns>
        private BidirectionalGraph<int, TaggedEdge<int, int>> GetGraphEdgeShreveOrderFromRoot(BidirectionalGraph<int, Edge<int>> graph, int root)
        {
            //var dic = new Dictionary<Edge<int>, int>();

            var algorithm = new DepthFirstSearchAlgorithm<int, Edge<int>>(graph)
            {
                ProcessAllComponents = false,
            };
            var recorder = new EdgeRecorderObserver<int, Edge<int>>();
            recorder.Attach(algorithm);
            algorithm.Compute(root);

            if (!recorder.Edges.Any()) return null;

            //dfs从根节点遍历得到的有向图
            var rootDirectedGraph = recorder.Edges.ToBidirectionalGraph<int, Edge<int>>(false);
            return GetGraphEdgeShreveOrder(rootDirectedGraph);
        }

        private BidirectionalGraph<int, TaggedEdge<int, int>> GetGraphEdgeShreveOrder(BidirectionalGraph<int, Edge<int>> rootDirectedGraph)
        {
            //Dictionary<Edge<int>, int> dic = InitializeDictionary(rootDirectedGraph);
            var tagedGraph = InitializeTagedGraph(rootDirectedGraph);

            var clonedRootGraph = rootDirectedGraph.Clone();

            clonedRootGraph.EdgeRemoved += e =>
            {
                Assert.IsNotNull(e);

                Trace.WriteLine($"Edge[{e}]被移除");

                int target = e.Target;

                int sum = 0;
                foreach (var itemVkp in tagedGraph.Edges)
                {
                    int itemEdgeSource = itemVkp.Source;
                    if (!target.Equals(itemEdgeSource)) continue;

                    sum += itemVkp.Tag;
                }
                TaggedEdge<int, int> matchTaggedEdge = tagedGraph.Edges.FirstOrDefault(edge => Equals(edge.Source, e.Source) && Equals(edge.Target, e.Target));
                if (matchTaggedEdge == null) return;

                if (clonedRootGraph.IsOutEdgesEmpty(target) && sum < 1) /*dic[e] = 1*/matchTaggedEdge.Tag = 1;
                else /*dic[e] = sum*/matchTaggedEdge.Tag = sum;

                //设置边的值
                Trace.WriteLine($"设置Edge[{e}]的Shreve Stream Order值[{/*dic[e]*/matchTaggedEdge.Tag}]");
            };

            while (clonedRootGraph.Edges.ToList().Count > 0)
            {
                int removedEdgeCount = clonedRootGraph.RemoveEdgeIf(edge => clonedRootGraph.IsOutEdgesEmpty(edge.Target));
            }
            return /*dic*/ tagedGraph;
        }

        private static BidirectionalGraph<int, TaggedEdge<int, int>> InitializeTagedGraph(BidirectionalGraph<int, Edge<int>> graph)
        {
            var tagedGraph = new BidirectionalGraph<int, TaggedEdge<int, int>>();

            tagedGraph.AddVertexRange(graph.Vertices);
            graph.Edges.ForEach(e => tagedGraph.AddEdge(new TaggedEdge<int, int>(e.Source, e.Target, 0)));

            return tagedGraph;
        }

        private Dictionary<Edge<int>, int> InitializeDictionary(BidirectionalGraph<int, Edge<int>> rootDirectedGraph)
        {
            var dic = new Dictionary<Edge<int>, int>();
            foreach (var itemEdge in rootDirectedGraph.Edges)
            {
                dic[itemEdge] = 0;
            }
            return dic;
        }

        [TestCase(false)]
        [TestCase(true)]
        public void ProcessAllComponents(bool processAll)
        {
            var graph = new AdjacencyGraph<int, Edge<int>>();
            graph.AddVerticesAndEdgeRange(new[]
            {
                new Edge<int>(1, 2),
                new Edge<int>(1, 3),
                new Edge<int>(2, 1),
                new Edge<int>(2, 4),
                new Edge<int>(2, 5),

                new Edge<int>(6, 7),
                new Edge<int>(6, 8),
                new Edge<int>(8, 6)
            });

            var algorithm = new DepthFirstSearchAlgorithm<int, Edge<int>>(graph)
            {
                ProcessAllComponents = processAll
            };
            int root = 1;

            var recorder = new EdgeRecorderObserver<int, Edge<int>>();
            recorder.Attach(algorithm);
            // <image url="$(ProjectDir)DocumentImages\graph_DepthFirstSearch.png"/>
            algorithm.Compute(root);

            //dfs从根节点遍历得到的有向图
            var rootDirectedGraph = new BidirectionalGraph<int, Edge<int>>();
            if (recorder.Edges.Any())
            {
                rootDirectedGraph = recorder.Edges.ToBidirectionalGraph<int, Edge<int>>(false);
                DFS(rootDirectedGraph, root);

                var tagedGraph = new BidirectionalGraph<int, TaggedEdge<int, int>>();
                tagedGraph.AddVertexRange(rootDirectedGraph.Vertices);
                rootDirectedGraph.Edges.ForEach(e => tagedGraph.AddEdge(new TaggedEdge<int, int>(e.Source, e.Target, 0)));
                // <image url="$(ProjectDir)DocumentImages\tagedGraph.png"/>
                UpdateEdgeTags(tagedGraph);
            }

            if (processAll)
            {
                FastGraphAssert.TrueForAll(algorithm.VerticesColors, pair => pair.Value == GraphColor.Black);
            }
            else
            {
                FastGraphAssert.TrueForAll(
                    new[] { 1, 2, 3, 4, 5 },
                    vertex => algorithm.VerticesColors[vertex] == GraphColor.Black);
                FastGraphAssert.TrueForAll(
                    new[] { 6, 7, 8 },
                    vertex => algorithm.VerticesColors[vertex] == GraphColor.White);
            }
        }

        private void DFS(BidirectionalGraph<int, Edge<int>> graph, int start)
        {
            // Is source
            if (graph.IsInEdgesEmpty(start))
            {
                Trace.WriteLine($"起点[{start}]为Source");
            }
            else
            {
                Trace.WriteLine($"起点[{start}]不是一个Source");
                return;
            }
            if (!graph.Vertices.Any(x => x == start)) return;

            var visited = new List<int>();
            var stack = new Stack<int>();
            graph.OutEdges(start).ForEach(edge => stack.Push(edge.Target));
            while (stack.Count > 0)
            {
                int current = stack.Pop();

                // Is sink
                if (graph.IsOutEdgesEmpty(current))
                {
                    Trace.WriteLine($"终点[{current}]为Sink");
                }
                if (visited.Any(t => t.Equals(current))) continue;

                visited.Add(current);
                Visit(current);
                foreach (var n in graph.OutEdges(current))
                {
                    if (visited.Contains(n.Target)) continue;
                    stack.Push(n.Target);
                }
            }
        }

        #region 计算Edge的ShreveOrder 性能不高有多重循环

        /// <summary>
        /// 遍历图并更新边的Tag值
        /// </summary>
        /// <param name="graph"></param>
        public void UpdateEdgeTags(BidirectionalGraph<int, TaggedEdge<int, int>> graph)
        {
            for (int i = 0; i < graph.Edges.ToList().Count; i++)
            {
                foreach (var edge in graph.Edges.ToList())
                {
                    edge.Tag = CalculateEdgeTag(graph, edge.Target);
                }
            }
        }

        private int CalculateEdgeTag(BidirectionalGraph<int, TaggedEdge<int, int>> graph, int node)
        {
            if (graph.IsOutEdgesEmpty(node))
            {
                return 1; // 如果节点是末端，返回1
            }

            int tagSum = 0;
            foreach (var edge in graph.OutEdges(node))
            {
                tagSum += CalculateEdgeTag(graph, edge.Target);
            }

            return tagSum;
        }

        #endregion 计算Edge的ShreveOrder 性能不高有多重循环

        public void Visit(int vertex)
        {
            Trace.WriteLine("遍历节点Vertex " + vertex.ToString());
        }

        [Pure]
        [NotNull]
        public static DepthFirstSearchAlgorithm<T, Edge<T>> CreateAlgorithmAndMaybeDoComputation<T>(
            [NotNull] ContractScenario<T> scenario)
        {
            var graph = new AdjacencyGraph<T, Edge<T>>();
            graph.AddVerticesAndEdgeRange(scenario.EdgesInGraph.Select(e => new Edge<T>(e.Source, e.Target)));
            graph.AddVertexRange(scenario.SingleVerticesInGraph);

            var algorithm = new DepthFirstSearchAlgorithm<T, Edge<T>>(graph);

            if (scenario.DoComputation)
                algorithm.Compute(scenario.Root);
            return algorithm;
        }
    }
}