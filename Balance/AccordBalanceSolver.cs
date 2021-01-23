using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Accord.Math;
using Accord.Math.Optimization;
using MathNet.Numerics.Distributions;
using MathNet.Numerics.LinearAlgebra.Double;
using Matrix = Accord.Math.Matrix;

namespace Balance
{
    public class AccordBalanceSolver : IBalanceSolver
    {
        public struct Connection
        {
            public int input;
            public int output;
        }
        public double DisbalanceOriginal { get; private set; }
        public double Disbalance { get; private set; }

        public TimeSpan Time { get; private set; }
        public TimeSpan TimeAll { get; private set; }

        public double[] Solve(double[] x0, double[,] a, double[] b, double[] measurability, double[] tolerance,
            double[] lower, double[] upper)
        {
            // Проверка аргументов на null
            _ = x0 ?? throw new ArgumentNullException(nameof(x0));
            _ = a ?? throw new ArgumentNullException(nameof(a));
            _ = b ?? throw new ArgumentNullException(nameof(b));
            _ = measurability ?? throw new ArgumentNullException(nameof(measurability));
            _ = tolerance ?? throw new ArgumentNullException(nameof(tolerance));
            _ = lower ?? throw new ArgumentNullException(nameof(lower));
            _ = upper ?? throw new ArgumentNullException(nameof(upper));

            //Проверка аргументов на размерности
            if (x0.Length == 0) throw new ArgumentException(nameof(x0));
            if (a.GetLength(1) != x0.Length)
                throw new ArgumentException("Array length by dimension 1 is not equal to X0 length.", nameof(a));
            if (b.Length != a.GetLength(0))
                throw new ArgumentException("Array length is not equal to A length by 0 dimension.", nameof(b));
            if (measurability.Length != x0.Length)
                throw new ArgumentException("Array length is not equal to X0 length.", nameof(measurability));
            if (tolerance.Length != x0.Length)
                throw new ArgumentException("Array length is not equal to X0 length.", nameof(tolerance));
            if (lower.Length != x0.Length)
                throw new ArgumentException("Array length is not equal to X0 length.", nameof(lower));
            if (upper.Length != x0.Length)
                throw new ArgumentException("Array length is not equal to X0 length.", nameof(upper));

            var stopWatch = new Stopwatch();
            stopWatch.Start();

            var i = SparseMatrix.OfDiagonalArray(measurability);
            var w = SparseMatrix.OfDiagonalVector(1 / SparseVector.OfEnumerable(tolerance).PointwisePower(2));

            var h = i * w;
            var d = -(h * SparseVector.OfEnumerable(x0));

            var func = new QuadraticObjectiveFunction(h.ToArray(), d.ToArray());
            var constraints = new List<LinearConstraint>();

            Time = stopWatch.Elapsed;

            //Нижние и верхние границы
            for (var j = 0; j < x0.Length; j++)
            {
                constraints.Add(new LinearConstraint(1)
                {
                    VariablesAtIndices = new[] { j },
                    ShouldBe = ConstraintType.GreaterThanOrEqualTo,
                    Value = lower[j]
                });

                constraints.Add(new LinearConstraint(1)
                {
                    VariablesAtIndices = new[] { j },
                    ShouldBe = ConstraintType.LesserThanOrEqualTo,
                    Value = upper[j]
                });
            }

            //Ограничения для решения задачи баланса
            for (var j = 0; j < b.Length; j++)
            {
                var notNullElements = Array.FindAll(a.GetRow(j), x => Math.Abs(x) > 0.0000001);
                var notNullElementsIndexes = new List<int>();
                for (var k = 0; k < x0.Length; k++)
                {
                    if (Math.Abs(a[j, k]) > 0.0000001)
                    {
                        notNullElementsIndexes.Add(k);
                    }
                }

                constraints.Add(new LinearConstraint(notNullElements.Length)
                {
                    VariablesAtIndices = notNullElementsIndexes.ToArray(),
                    CombinedAs = notNullElements,
                    ShouldBe = ConstraintType.EqualTo,
                    Value = b[j]
                });
            }

            var solver = new GoldfarbIdnani(func, constraints);
            if (!solver.Minimize()) throw new ApplicationException("Failed to solve balance task.");

            stopWatch.Stop();
            TimeAll = stopWatch.Elapsed;

            DisbalanceOriginal = a.Dot(x0).Subtract(b).Euclidean();
            Disbalance = a.Dot(solver.Solution).Subtract(b).Euclidean();

            return solver.Solution;
        }

        public double GlobalTest(double[] x0, double[,] a, double[] measurability, double[] tolerance)
        {
            var aMatrix = SparseMatrix.OfArray(a);
            var aTransposedMatrix = SparseMatrix.OfMatrix(aMatrix.Transpose());
            var x0Vector = SparseVector.OfEnumerable(x0);

            // Введение погрешностей по неизмеряемым потокам
            var xStd = SparseVector.OfEnumerable(tolerance) / 1.96;

            for (var i = 0; i < xStd.Count; i++)
            {
                if (Math.Abs(measurability[i]) < 0.0000001)
                {
                    xStd[i] = Math.Pow(10, 2) * x0Vector.Maximum();
                }
            }

            var sigma = SparseMatrix.OfDiagonalVector(xStd.PointwisePower(2));

            var r = aMatrix * x0Vector;
            var v = aMatrix * sigma * aTransposedMatrix;

            var result = r * v.PseudoInverse() * r.ToColumnMatrix();
            var chi = ChiSquared.InvCDF(aMatrix.RowCount, 1 - 0.05);

            return result[0] / chi;
        }

        public double[,] GlrTest(double[] x0, double[,] a, double[] measurability, double[] tolerance,
            IEnumerable<(int, int, int)> flows, double globalTest)
        {
            var nodesCount = a.GetLength(0);

            var glrTable = new double[nodesCount, nodesCount];

            if (flows != null)
            {
                foreach (var flow in flows)
                {
                    var (i, j, _) = flow;

                    // Добавляем новый поток
                    var aColumn = new double[nodesCount];
                    aColumn[i] = 1;
                    aColumn[j] = -1;

                    var aNew = a.InsertColumn(aColumn);
                    var x0New = x0.Append(0).ToArray();
                    var measurabilityNew = measurability.Append(0).ToArray();
                    var toleranceNew = tolerance.Append(0).ToArray();

                    // Считаем тест и находим разницу
                    glrTable[i, j] = globalTest - GlobalTest(x0New, aNew, measurabilityNew, toleranceNew);
                }
            }
            else
            {
                for (var i = 0; i < nodesCount; i++)
                {
                    for (var j = i + 1; j < nodesCount; j++)
                    {
                        // Добавляем новый поток
                        var aColumn = new double[nodesCount];
                        aColumn[i] = 1;
                        aColumn[j] = -1;

                        var aNew = a.InsertColumn(aColumn);
                        var x0New = x0.Append(0).ToArray();
                        var measurabilityNew = measurability.Append(0).ToArray();
                        var toleranceNew = tolerance.Append(0).ToArray();

                        // Считаем тест и находим разницу
                        glrTable[i, j] = globalTest - GlobalTest(x0New, aNew, measurabilityNew, toleranceNew);
                    }
                }
            }

            return glrTable;
        }

        public IEnumerable<(int, int, int)> GetFlows(double[,] a)
        {
            var flows = new List<(int, int, int)>();
            for (var k = 0; k < a.GetLength(1); k++)
            {
                var column = a.GetColumn(k);

                var i = column.IndexOf(-1);
                var j = column.IndexOf(1);

                if (i == -1 || j == -1)
                {
                    continue;
                }

                flows.Add((i, j, k));
            }

            return flows;
        }
    }
}
