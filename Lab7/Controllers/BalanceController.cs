using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using Accord.Math;
using Balance;
using Lab7.Models;
using Microsoft.AspNetCore.Mvc;
using Newtonsoft.Json;
using TreeCollections;

namespace Lab7.Controllers
{
    [Route("api/[controller]")]
    [ApiController]
    public class BalanceController : ControllerBase
    {
        [HttpPost("solve")]
        public async Task<Responce> PostGraphAsync([FromBody] InputGraph input)
        {
            var converted = await GraphToMatrixAsync(input);

            return await Task.Run(() =>
            {
                try
                {
                    // Проверка аргумента на null
                    _ = input ?? throw new ArgumentNullException(nameof(input));

                    // Решение задачи
                    IBalanceSolver solver = new AccordBalanceSolver();

                    var output = new OutputData
                    {
                        X = solver.Solve(converted.X0, converted.A, converted.B, converted.Measurability,
                            converted.Tolerance,
                            converted.UseTechnologic
                                ? converted.LowerTechnologic
                                : converted.LowerMetrologic,
                            converted.UseTechnologic
                                ? converted.UpperTechnologic
                                : converted.UpperMetrologic),
                        DisbalanceOriginal = solver.DisbalanceOriginal,
                        Disbalance = solver.Disbalance,
                        Time = solver.TimeAll.TotalSeconds,
                        TimeMatrix = solver.Time.TotalSeconds
                    };

                    return new Responce
                    {
                        Type = "result",
                        Data = output
                    };
                }
                catch (Exception e)
                {
                    return new Responce
                    {
                        Type = "error",
                        Data = e.Message
                    };
                }
            });
        }

        [HttpPost("gt")]
        public async Task<Responce> GlobalTestGraphAsync([FromBody] InputGraph input)
        {
            var converted = await GraphToMatrixAsync(input);

            return await Task.Run(() =>
            {
                try
                {
                    // Проверка аргумента на null
                    _ = input ?? throw new ArgumentNullException(nameof(input));

                    // Значение глобального теста
                    IBalanceSolver solver = new AccordBalanceSolver();
                    var output = solver.GlobalTest(converted.X0, converted.A, converted.Measurability,
                        converted.Tolerance);

                    return new Responce
                    {
                        Type = "result",
                        Data = output
                    };
                }
                catch (Exception e)
                {
                    return new Responce
                    {
                        Type = "error",
                        Data = e.Message
                    };
                }
            });
        }

        [HttpPost("glr")]
        public async Task<Responce> GlrTestGraphAsync([FromBody] InputGraph input)
        {
            var converted = await GraphToMatrixAsync(input);

            return await Task.Run(() =>
            {
                try
                {
                    _ = converted ?? throw new ArgumentNullException(nameof(converted));

                    IBalanceSolver solver = new AccordBalanceSolver();

                    const int maxSubNodesCount = 3;
                    const int maxTreeDepth = 5;

                    var flows = solver.GetFlows(converted.A).ToList();
                    var nodesCount = converted.A.GetLength(0);

                    var root = new MutableEntityTreeNode<Guid, TreeElement>(x => x.Id, new TreeElement());
                    var currentNode = root;

                    while (currentNode != null)
                    {
                        var newA = converted.A;
                        var newX0 = converted.X0;
                        var newMeasurability = converted.Measurability;
                        var newTolerance = converted.Tolerance;

                        foreach (var (fi, fj) in currentNode.Item.Flows)
                        {
                            var aColumn = new double[nodesCount];
                            aColumn[fi] = 1;
                            aColumn[fj] = -1;

                            newA = newA.InsertColumn(aColumn);
                            newX0 = newX0.Append(0).ToArray();
                            newMeasurability = newMeasurability.Append(0).ToArray();
                            newTolerance = newTolerance.Append(0).ToArray();
                        }

                        var globalTest = solver.GlobalTest(newX0, newA, newMeasurability,
                            newTolerance);

                        var glr = solver.GlrTest(newX0, newA, newMeasurability,
                            newTolerance, flows, globalTest);

                        // Поиск следующего максимума
                        var (i, j) = (0, 0);
                        for (var k = 0; k < currentNode.Children.Count + 1; k++)
                        {
                            (i, j) = glr.ArgMax();

                            if (glr[i, j] <= 0)
                            {
                                break;
                            }

                            // Если итерация не последняя, сбрасываем значение
                            if (k != currentNode.Children.Count)
                            {
                                glr[i, j] = 0.0;
                            }
                        }

                        // Проверяем можно ли добавить дочерний узел в дерево
                        if (currentNode.Children.Count < maxSubNodesCount &&
                            currentNode.Level < maxTreeDepth && glr[i, j] > 0 && globalTest >= 1)
                        {
                            var node = new TreeElement(new List<(int, int)>(currentNode.Item.Flows), globalTest - glr[i, j]);
                            node.Flows.Add((i, j));

                            currentNode = currentNode.AddChild(node);
                        }
                        else
                        {
                            currentNode = currentNode.Parent;
                        }
                    }

                    //Находим все листья и выводим их
                    var leafs = root.Where(x => x.IsLeaf);
                    var results = new List<GlrOutput>();

                    foreach (var leaf in leafs)
                    {
                        var result = new List<GlrOutputFlow>();
                        var flowsToAdd = new List<Variable>();

                        foreach (var flow in leaf.Item.Flows)
                        {
                            var (i, j) = flow;

                            var newFlow = new GlrOutputFlow
                            {
                                Id = Guid.NewGuid().ToString(),
                                Name = "New flow",
                                Number = -1,
                                Info = $"{i} -> {j}"
                            };

                            // Если у нас есть существующий поток, то выводим информацию о нем
                            var existingFlowIdx = flows.FindIndex(x => x.Item1 == i && x.Item2 == j);
                            if (existingFlowIdx != -1)
                            {
                                var (_, _, existingFlow) = flows[existingFlowIdx];

                                newFlow.Id = converted.Guids[existingFlow];
                                newFlow.Name = converted.Names[existingFlow];
                                newFlow.Number = existingFlow;

                                // Формируем информацию о добавляемом потоке
                                var variable = new Variable
                                {
                                    Id = Guid.NewGuid().ToString(),
                                    SourceId = converted.NodesGuids[i],
                                    DestinationId = converted.NodesGuids[j],
                                    Name = converted.Names[existingFlow] + " (additional)",
                                    MetrologicRange = new Models.Range
                                    {
                                        Min = converted.LowerMetrologic[existingFlow] - converted.X0[existingFlow],
                                        Max = converted.UpperMetrologic[existingFlow] - converted.X0[existingFlow]
                                    },
                                    TechnologicRange = new Models.Range
                                    {
                                        Min = converted.LowerTechnologic[existingFlow] - converted.X0[existingFlow],
                                        Max = converted.UpperTechnologic[existingFlow] - converted.X0[existingFlow]
                                    },
                                    Tolerance = converted.Tolerance[existingFlow],
                                    IsMeasured = true,
                                    VarType = "FLOW"
                                };

                                flowsToAdd.Add(variable);
                            }

                            result.Add(newFlow);
                        }

                        results.Add(new GlrOutput
                        {
                            FlowsInfo = result,
                            FlowsToAdd = flowsToAdd,
                            TestValue = leaf.Item.TestValue
                        });
                    }

                    return new Responce
                    {
                        Type = "result",
                        Data = results.OrderBy(x => x.TestValue)
                    };
                }
                catch (Exception e)
                {
                    return new Responce
                    {
                        Type = "error",
                        Data = e.Message
                    };
                }
            });
        }

        [HttpPost("glrbest")]
        public async Task<Responce> GlrTestBestGraphAsync([FromBody] InputGraph input)
        {
            var converted = await GraphToMatrixAsync(input);

            return await Task.Run(() =>
            {
                try
                {
                    // Проверка аргумента на null
                    _ = converted ?? throw new ArgumentNullException(nameof(converted));

                    IBalanceSolver solver = new AccordBalanceSolver();

                    var flows = solver.GetFlows(converted.A).ToList();

                    var globalTest = solver.GlobalTest(converted.X0, converted.A, converted.Measurability,
                        converted.Tolerance);

                    var nodesCount = converted.A.GetLength(0);
                    var glr = solver.GlrTest(converted.X0, converted.A, converted.Measurability,
                        converted.Tolerance, flows, globalTest);

                    var results = new List<GlrOutputFlow>();

                    while (globalTest >= 1)
                    {
                        // Находим максимальное значение GLR теста в массиве
                        var (i, j) = glr.ArgMax();

                        // Если у нас не осталось значений больше нуля, то выходим
                        if (glr[i, j] <= 0)
                        {
                            break;
                        }

                        // Добавляем новый поток
                        var aColumn = new double[nodesCount];
                        aColumn[i] = 1;
                        aColumn[j] = -1;

                        converted.A = converted.A.InsertColumn(aColumn);
                        converted.X0 = converted.X0.Append(0).ToArray();
                        converted.Measurability = converted.Measurability.Append(0).ToArray();
                        converted.Tolerance = converted.Tolerance.Append(0).ToArray();

                        var newFlow = new GlrOutputFlow
                        {
                            Id = Guid.NewGuid().ToString(),
                            Name = "New flow",
                            Number = converted.A.Length - 1,
                            Info = $"{i} -> {j}"
                        };

                        // Если у нас есть существующий поток, то выводим информацию о нем
                        var existingFlowIdx = flows.FindIndex(x => x.Item1 == i && x.Item2 == j);
                        if (existingFlowIdx != -1)
                        {
                            var (_, _, existingFlow) = flows[existingFlowIdx];

                            newFlow.Id = converted.Guids[existingFlow];
                            newFlow.Name = converted.Names[existingFlow];
                            newFlow.Number = existingFlow;
                        }

                        results.Add(newFlow);

                        // Считаем новое значение глобального теста
                        globalTest = solver.GlobalTest(converted.X0, converted.A, converted.Measurability,
                            converted.Tolerance);

                        // Считаем новое значение GLR теста
                        glr = solver.GlrTest(converted.X0, converted.A, converted.Measurability,
                            converted.Tolerance, flows, globalTest);
                    }

                    return new Responce
                    {
                        Type = "result",
                        Data = results
                    };
                }
                catch (Exception e)
                {
                    return new Responce
                    {
                        Type = "error",
                        Data = e.Message
                    };
                }
            });
        }

        public static async Task<InputData> GraphToMatrixAsync(InputGraph graph)
        {
            return await Task.Run(() =>
            {
                //Заполняем список узлов
                var nodes = new List<string>();
                foreach (var variable in graph.Variables)
                {
                    if ((variable.DestinationId != null) && (!nodes.Contains(variable.DestinationId)))
                    {
                        nodes.Add(variable.DestinationId);
                    }

                    if ((variable.SourceId != null) && (!nodes.Contains(variable.SourceId)))
                    {
                        nodes.Add(variable.SourceId);
                    }
                }

                var inputData = new InputData
                {
                    X0 = new double[graph.Variables.Count],
                    A = new double[nodes.Count, graph.Variables.Count],
                    B = new double[nodes.Count],
                    Measurability = new double[graph.Variables.Count],
                    Tolerance = new double[graph.Variables.Count],
                    LowerMetrologic = new double[graph.Variables.Count],
                    UpperMetrologic = new double[graph.Variables.Count],
                    LowerTechnologic = new double[graph.Variables.Count],
                    UpperTechnologic = new double[graph.Variables.Count],
                    Names = new string[graph.Variables.Count],
                    Guids = new string[graph.Variables.Count],
                    NodesGuids = nodes.ToArray(),
                    UseTechnologic = graph.BalanceSettings.BoundsType != "METROLOGY_ONLY"
                };

                for (var i = 0; i < graph.Variables.Count; i++)
                {
                    //X0
                    inputData.X0[i] = graph.Variables[i].Measured;

                    //Names
                    inputData.Names[i] = graph.Variables[i].Name;

                    //Guids
                    inputData.Guids[i] = graph.Variables[i].Id;

                    //A
                    if (graph.Variables[i].DestinationId != null)
                    {
                        inputData.A[nodes.IndexOf(graph.Variables[i].DestinationId), i] = 1;
                    }

                    if (graph.Variables[i].SourceId != null)
                    {
                        inputData.A[nodes.IndexOf(graph.Variables[i].SourceId), i] = -1;
                    }

                    //Measurability
                    inputData.Measurability[i] = graph.Variables[i].IsMeasured ? 1 : 0;

                    //Tolerance
                    inputData.Tolerance[i] = Math.Abs(graph.Variables[i].Tolerance) > 0.000000001 ? graph.Variables[i].Tolerance : 0.000000001;

                    //LowerMetrologic
                    inputData.LowerMetrologic[i] = graph.Variables[i].MetrologicRange.Min;

                    //UpperMetrologic
                    inputData.UpperMetrologic[i] = graph.Variables[i].MetrologicRange.Max;

                    //LowerTechnologic
                    inputData.LowerTechnologic[i] = graph.Variables[i].TechnologicRange.Min;

                    //UpperTechnologic
                    inputData.UpperTechnologic[i] = graph.Variables[i].TechnologicRange.Max;
                }

                return inputData;
            });
        }

        [HttpPost("generate")]
        public async Task<Responce> PostGenerateAsync([FromQuery] int nodesCount, [FromQuery] int flowsCount,
            [FromQuery] double min = -100, [FromQuery] double max = 100)
        {
            return await Task.Run(() =>
            {
                var rand = new Random();

                var nodes = new List<string>();
                for (var i = 0; i < nodesCount; i++)
                {
                    nodes.Add(Guid.NewGuid().ToString());
                }

                var result = new InputGraph
                {
                    BalanceSettings = new BalanceSettings(),
                    Dependencies = null,
                    Variables = new List<Variable>()
                };

                for (var i = 0; i < flowsCount; i++)
                {
                    var destination = rand.Next(-1, nodesCount);
                    var source = rand.Next(-1, nodesCount);

                    var variable = new Variable
                    {
                        Id = Guid.NewGuid().ToString(),
                        Name = $"Variable {i}",
                        DestinationId = destination != -1 ? nodes[destination] : null,
                        SourceId = source != -1 ? nodes[source] : null,
                        Measured = min + rand.NextDouble() * (max - min),
                        Tolerance = (min + rand.NextDouble() * (max - min)) / 10.0,
                        MetrologicRange = new Models.Range
                        {
                            Min = min + rand.NextDouble() * (max - min) / 2.0,
                            Max = max - rand.NextDouble() * (max - min) / 2.0,
                        },
                        TechnologicRange = new Models.Range
                        {
                            Min = min + rand.NextDouble() * (max - min) / 2.0,
                            Max = max - rand.NextDouble() * (max - min) / 2.0,
                        },
                        IsMeasured = true,
                        InService = true,
                        VarType = "FLOW"
                    };

                    result.Variables.Add(variable);
                }

                return new Responce
                {
                    Type = "result",
                    Data = result
                };
            });
        }
    }
}
