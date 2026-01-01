using System;
using System.Collections.Generic;
using System.Linq;
using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;

namespace User.PluginSdkDemo
{
    public static class GeneralKinematics
    {
        private const int SampleCount = 200;
        private const int MaxIterations = 100;
        private const double Tolerance = 1e-6;
        private const double Damping = 1e-6;
        private const double LengthEpsilon = 1e-9;
        private const double CollinearTolerance = 1e-4;
        private const double DampingMin = 1e-8;
        private const double DampingMax = 1e3;
        private const int LmMaxTries = 8;
        private const double MaxStep = 5.0;
        private const double MaxThetaStep = 0.2;
        private const double ConditionLimit = 1e8;

        private class PinInfo
        {
            public uint Id;
            public double X;
            public double Y;
            public bool Grounded;
            public bool IsContactPoint;
            public bool IsRailInterface;
        }

        private class BarLine
        {
            public int RefPin;
            public int AxisPin;
            public int[] PinIndices;
            public double DirX;
            public double DirY;
        }

        private enum ConstraintType
        {
            Distance,
            Fix
        }

        private struct Constraint
        {
            public ConstraintType Type;
            public int PinA;
            public int PinB;
            public double Length;
            public int Pin;
            public int Axis;
        }

        public static KinematicParameters CalcKinematicParameters(GeneralKinematicConfig config)
        {
            if (config == null) throw new ArgumentNullException(nameof(config));
            if (config.Pins == null || config.Pins.Count == 0) throw new ArgumentException("GeneralKinematicConfig requires pins.");
            if (config.Bars == null || config.Bars.Count == 0) throw new ArgumentException("GeneralKinematicConfig requires bars.");
            if (config.RailTravelNegative < 0.0 || config.RailTravelPositive < 0.0)
            {
                throw new ArgumentException("GeneralKinematicConfig.rail_travel_negative/rail_travel_positive must be >= 0.");
            }

            int contactIndex;
            int railIndex;
            List<PinInfo> pins = BuildPins(config, out contactIndex, out railIndex);

            int meteringConstraintIndex;
            List<BarLine> barLines;
            int[] pinBarIndex;
            double[] pinBarOffset;
            List<Constraint> constraints = BuildConstraints(
                config,
                pins,
                out meteringConstraintIndex,
                out barLines,
                out pinBarIndex,
                out pinBarOffset);
            if (meteringConstraintIndex < 0) {
                throw new ArgumentException("GeneralKinematicConfig requires exactly one 2-pin metering bar.");
            }

            int[] varIndexX;
            int[] varIndexY;
            double[] variables;
            int[] barVarBase;
            BuildVariableMap(pins, railIndex, barLines, pinBarIndex, out varIndexX, out varIndexY, out variables, out barVarBase);

            double travelNegative = config.RailTravelNegative;
            double travelPositive = config.RailTravelPositive;
            double travelTotal = travelNegative + travelPositive;
            if (travelTotal <= 0.0)
            {
                throw new ArgumentException("GeneralKinematicConfig.rail_travel_negative/rail_travel_positive must sum to > 0.");
            }
            double railOffsetMin = -travelNegative;
            double railStep = travelTotal / (SampleCount - 1);

            double[] contactX = new double[SampleCount];
            double[] contactY = new double[SampleCount];
            double[] sledPos = new double[SampleCount];
            double[][] positionsX = new double[SampleCount][];
            double[][] positionsY = new double[SampleCount][];
            double[] railOffsets = new double[SampleCount];

            for (int i = 0; i < SampleCount; i++)
            {
                double railOffset = railOffsetMin + railStep * i;
                if (!SolvePositions(
                    pins,
                    constraints,
                    railIndex,
                    railOffset,
                    varIndexX,
                    varIndexY,
                    barVarBase,
                    variables,
                    pinBarIndex,
                    pinBarOffset))
                {
                    throw new InvalidOperationException("General kinematics solver failed to converge.");
                }

                double[] posX = new double[pins.Count];
                double[] posY = new double[pins.Count];
                FillPositions(
                    pins,
                    railIndex,
                    railOffset,
                    varIndexX,
                    varIndexY,
                    barVarBase,
                    variables,
                    pinBarIndex,
                    pinBarOffset,
                    posX,
                    posY);

                positionsX[i] = posX;
                positionsY[i] = posY;
                railOffsets[i] = railOffset;

                contactX[i] = posX[contactIndex];
                contactY[i] = posY[contactIndex];
                sledPos[i] = railOffset + travelNegative;
            }

            double[] contactPos = BuildContactPath(contactX, contactY);
            double centerIndex = (-railOffsetMin) / railStep;
            double centerOffset = InterpolateAtIndex(contactPos, centerIndex);
            for (int i = 0; i < contactPos.Length; i++)
            {
                contactPos[i] -= centerOffset;
            }
            double[] forceFactors = new double[SampleCount];

            int varCount = variables.Length;
            for (int i = 0; i < SampleCount; i++)
            {
                GetContactTangent(contactX, contactY, i, out double tx, out double ty);
                Vector<double> fExt = Vector<double>.Build.Dense(varCount);
                BuildBarTrigFromPositions(positionsX[i], positionsY[i], barLines, out double[] barCos, out double[] barSin);
                ApplyForceOnPin(
                    fExt,
                    contactIndex,
                    tx,
                    ty,
                    varIndexX,
                    varIndexY,
                    pinBarIndex,
                    pinBarOffset,
                    barVarBase,
                    barCos,
                    barSin);

                Matrix<double> jacobian = BuildJacobian(
                    constraints,
                    positionsX[i],
                    positionsY[i],
                    railOffsets[i],
                    pins,
                    varIndexX,
                    varIndexY,
                    pinBarIndex,
                    pinBarOffset,
                    barVarBase,
                    barCos,
                    barSin,
                    varCount);
                Matrix<double> A = jacobian * jacobian.Transpose();
                for (int d = 0; d < A.RowCount; d++) A[d, d] += Damping;
                Vector<double> b = -(jacobian * fExt);
                Vector<double> lambda;
                try
                {
                    lambda = A.Solve(b);
                }
                catch (Exception)
                {
                    lambda = A.PseudoInverse() * b;
                }

                double meter = lambda[meteringConstraintIndex];
                if (Math.Abs(meter) < LengthEpsilon)
                {
                    meter = (meter < 0.0) ? -LengthEpsilon : LengthEpsilon;
                }
                forceFactors[i] = 1.0 / meter;
            }

            double[] coeffsSled = Fit.Polynomial(contactPos, sledPos, 4);
            double[] coeffsForce = Fit.Polynomial(contactPos, forceFactors, 4);

            KinematicParameters parameters = new KinematicParameters();
            parameters.CoeffsSledPosOverContactPointPos.AddRange(coeffsSled);
            parameters.CoeffsForceFactorOverContactPointPos.AddRange(coeffsForce);
            parameters.ContactPointPosMinAbs = (int)(contactPos[0] * 10.0);
            parameters.ContactPointPosMaxAbs = (int)(contactPos[contactPos.Length - 1] * 10.0);
            return parameters;
        }

        private static List<PinInfo> BuildPins(GeneralKinematicConfig config, out int contactIndex, out int railIndex)
        {
            List<PinInfo> pins = new List<PinInfo>(config.Pins.Count);
            Dictionary<uint, int> idToIndex = new Dictionary<uint, int>();
            contactIndex = -1;
            railIndex = -1;

            foreach (GeneralKinematicPin pin in config.Pins)
            {
                if (idToIndex.ContainsKey(pin.PinId))
                {
                    throw new ArgumentException("Duplicate pin_id in GeneralKinematicConfig.");
                }
                PinInfo info = new PinInfo
                {
                    Id = pin.PinId,
                    X = pin.X,
                    Y = pin.Y,
                    Grounded = pin.Grounded,
                    IsContactPoint = pin.IsContactPoint,
                    IsRailInterface = pin.IsRailInterface
                };
                if (info.IsContactPoint)
                {
                    if (contactIndex >= 0) throw new ArgumentException("Only one contact point pin is allowed.");
                    contactIndex = pins.Count;
                }
                if (info.IsRailInterface)
                {
                    if (railIndex >= 0) throw new ArgumentException("Only one rail interface pin is allowed.");
                    railIndex = pins.Count;
                }
                if (info.Grounded && info.IsRailInterface)
                {
                    throw new ArgumentException("Rail interface pin cannot be grounded.");
                }
                if (info.Grounded && info.IsContactPoint)
                {
                    throw new ArgumentException("Contact point pin cannot be grounded.");
                }
                if (info.IsContactPoint && info.IsRailInterface)
                {
                    throw new ArgumentException("Contact point pin cannot be the rail interface pin.");
                }

                idToIndex[info.Id] = pins.Count;
                pins.Add(info);
            }

            if (contactIndex < 0) throw new ArgumentException("GeneralKinematicConfig requires exactly one contact point pin.");
            if (railIndex < 0) throw new ArgumentException("GeneralKinematicConfig requires exactly one rail interface pin.");
            return pins;
        }

        private static List<Constraint> BuildConstraints(
            GeneralKinematicConfig config,
            List<PinInfo> pins,
            out int meteringConstraintIndex,
            out List<BarLine> barLines,
            out int[] pinBarIndex,
            out double[] pinBarOffset)
        {
            Dictionary<uint, int> idToIndex = new Dictionary<uint, int>();
            for (int i = 0; i < pins.Count; i++) idToIndex[pins[i].Id] = i;

            List<Constraint> constraints = new List<Constraint>();
            barLines = new List<BarLine>();
            pinBarIndex = Enumerable.Repeat(-1, pins.Count).ToArray();
            pinBarOffset = new double[pins.Count];
            meteringConstraintIndex = -1;
            bool meteringFound = false;

            foreach (GeneralKinematicBar bar in config.Bars)
            {
                List<uint> uniqueIds = bar.PinIds.Distinct().ToList();
                if (uniqueIds.Count < 2) throw new ArgumentException("Each bar must reference at least two pins.");
                if (bar.IsMetering)
                {
                    if (meteringFound) throw new ArgumentException("Only one metering bar is allowed.");
                    if (uniqueIds.Count != 2) throw new ArgumentException("Metering bar must have exactly two pins.");
                    meteringFound = true;
                }

                if (uniqueIds.Count == 2)
                {
                    uint pinA = uniqueIds[0];
                    uint pinB = uniqueIds[1];
                    if (!idToIndex.TryGetValue(pinA, out int idxA) || !idToIndex.TryGetValue(pinB, out int idxB))
                    {
                        throw new ArgumentException("Bar references unknown pin id.");
                    }

                    double dx = pins[idxA].X - pins[idxB].X;
                    double dy = pins[idxA].Y - pins[idxB].Y;
                    double length = Math.Sqrt(dx * dx + dy * dy);
                    if (length < LengthEpsilon)
                    {
                        throw new ArgumentException("Bar length must be > 0.");
                    }

                    constraints.Add(new Constraint
                    {
                        Type = ConstraintType.Distance,
                        PinA = idxA,
                        PinB = idxB,
                        Length = length
                    });
                    if (bar.IsMetering) meteringConstraintIndex = constraints.Count - 1;
                    continue;
                }

                if (bar.IsMetering)
                {
                    throw new ArgumentException("Metering bar must have exactly two pins.");
                }

                List<int> pinIndices = new List<int>(uniqueIds.Count);
                foreach (uint pinId in uniqueIds)
                {
                    if (!idToIndex.TryGetValue(pinId, out int idx))
                    {
                        throw new ArgumentException("Bar references unknown pin id.");
                    }
                    if (pinBarIndex[idx] >= 0)
                    {
                        throw new ArgumentException("Pin participates in multiple collinear bars.");
                    }
                    pinIndices.Add(idx);
                }

                int refIdx = pinIndices[0];
                int axisIdx = pinIndices[1];
                double axisDx = pins[axisIdx].X - pins[refIdx].X;
                double axisDy = pins[axisIdx].Y - pins[refIdx].Y;
                double axisLen = Math.Sqrt(axisDx * axisDx + axisDy * axisDy);
                if (axisLen < LengthEpsilon)
                {
                    throw new ArgumentException("Bar length must be > 0.");
                }
                double dirX = axisDx / axisLen;
                double dirY = axisDy / axisLen;

                foreach (int idx in pinIndices)
                {
                    double px = pins[idx].X - pins[refIdx].X;
                    double py = pins[idx].Y - pins[refIdx].Y;
                    double s = px * dirX + py * dirY;
                    double perp = px * dirY - py * dirX;
                    if (Math.Abs(perp) > CollinearTolerance * axisLen)
                    {
                        throw new ArgumentException("Collinear bar pins must lie on the same line.");
                    }
                    pinBarIndex[idx] = barLines.Count;
                    pinBarOffset[idx] = s;
                }

                barLines.Add(new BarLine
                {
                    RefPin = refIdx,
                    AxisPin = axisIdx,
                    PinIndices = pinIndices.ToArray(),
                    DirX = dirX,
                    DirY = dirY
                });
            }

            for (int i = 0; i < pins.Count; i++)
            {
                if (pinBarIndex[i] < 0) continue;
                if (pins[i].Grounded || pins[i].IsRailInterface)
                {
                    constraints.Add(new Constraint { Type = ConstraintType.Fix, Pin = i, Axis = 0 });
                    constraints.Add(new Constraint { Type = ConstraintType.Fix, Pin = i, Axis = 1 });
                }
            }

            return constraints;
        }

        private static void BuildVariableMap(
            List<PinInfo> pins,
            int railIndex,
            List<BarLine> barLines,
            int[] pinBarIndex,
            out int[] varIndexX,
            out int[] varIndexY,
            out double[] variables,
            out int[] barVarBase)
        {
            int pinCount = pins.Count;
            varIndexX = Enumerable.Repeat(-1, pinCount).ToArray();
            varIndexY = Enumerable.Repeat(-1, pinCount).ToArray();
            List<double> variableList = new List<double>();

            for (int i = 0; i < pinCount; i++)
            {
                if (pinBarIndex[i] >= 0)
                {
                    continue;
                }
                if (i == railIndex || pins[i].Grounded)
                {
                    continue;
                }
                varIndexX[i] = variableList.Count;
                varIndexY[i] = variableList.Count + 1;
                variableList.Add(pins[i].X);
                variableList.Add(pins[i].Y);
            }

            List<int> barVarBaseList = new List<int>(barLines.Count);
            foreach (BarLine bar in barLines)
            {
                int baseIndex = variableList.Count;
                barVarBaseList.Add(baseIndex);
                double theta = Math.Atan2(bar.DirY, bar.DirX);
                variableList.Add(pins[bar.RefPin].X);
                variableList.Add(pins[bar.RefPin].Y);
                variableList.Add(theta);
            }

            variables = variableList.ToArray();
            barVarBase = barVarBaseList.ToArray();
        }

        private static void BuildBarPose(
            double[] variables,
            int[] barVarBase,
            out double[] barX0,
            out double[] barY0,
            out double[] barCos,
            out double[] barSin)
        {
            int count = barVarBase.Length;
            barX0 = new double[count];
            barY0 = new double[count];
            barCos = new double[count];
            barSin = new double[count];
            for (int i = 0; i < count; i++)
            {
                int baseIndex = barVarBase[i];
                barX0[i] = variables[baseIndex];
                barY0[i] = variables[baseIndex + 1];
                double theta = variables[baseIndex + 2];
                barCos[i] = Math.Cos(theta);
                barSin[i] = Math.Sin(theta);
            }
        }

        private static void BuildBarTrigFromPositions(
            double[] posX,
            double[] posY,
            List<BarLine> barLines,
            out double[] barCos,
            out double[] barSin)
        {
            int count = barLines.Count;
            barCos = new double[count];
            barSin = new double[count];
            for (int i = 0; i < count; i++)
            {
                BarLine bar = barLines[i];
                double dx = posX[bar.AxisPin] - posX[bar.RefPin];
                double dy = posY[bar.AxisPin] - posY[bar.RefPin];
                double dist = Math.Sqrt(dx * dx + dy * dy);
                if (dist < LengthEpsilon) dist = LengthEpsilon;
                barCos[i] = dx / dist;
                barSin[i] = dy / dist;
            }
        }

        private static void FillPositionsCached(
            List<PinInfo> pins,
            int railIndex,
            double railOffset,
            int[] varIndexX,
            int[] varIndexY,
            double[] variables,
            int[] pinBarIndex,
            double[] pinBarOffset,
            double[] barX0,
            double[] barY0,
            double[] barCos,
            double[] barSin,
            double[] posX,
            double[] posY)
        {
            for (int i = 0; i < pins.Count; i++)
            {
                int barIdx = pinBarIndex[i];
                if (barIdx >= 0)
                {
                    double s = pinBarOffset[i];
                    posX[i] = barX0[barIdx] + s * barCos[barIdx];
                    posY[i] = barY0[barIdx] + s * barSin[barIdx];
                }
                else if (i == railIndex)
                {
                    posX[i] = pins[i].X + railOffset;
                    posY[i] = pins[i].Y;
                }
                else if (pins[i].Grounded)
                {
                    posX[i] = pins[i].X;
                    posY[i] = pins[i].Y;
                }
                else
                {
                    posX[i] = variables[varIndexX[i]];
                    posY[i] = variables[varIndexY[i]];
                }
            }
        }

        private static void FillPositions(
            List<PinInfo> pins,
            int railIndex,
            double railOffset,
            int[] varIndexX,
            int[] varIndexY,
            int[] barVarBase,
            double[] variables,
            int[] pinBarIndex,
            double[] pinBarOffset,
            double[] posX,
            double[] posY)
        {
            BuildBarPose(variables, barVarBase, out double[] barX0, out double[] barY0, out double[] barCos, out double[] barSin);
            FillPositionsCached(
                pins,
                railIndex,
                railOffset,
                varIndexX,
                varIndexY,
                variables,
                pinBarIndex,
                pinBarOffset,
                barX0,
                barY0,
                barCos,
                barSin,
                posX,
                posY);
        }

        private static void AccumulatePinJacobian(
            Matrix<double> jacobian,
            int row,
            int pinIndex,
            double weightX,
            double weightY,
            int[] varIndexX,
            int[] varIndexY,
            int[] pinBarIndex,
            double[] pinBarOffset,
            int[] barVarBase,
            double[] barCos,
            double[] barSin)
        {
            int barIdx = pinBarIndex[pinIndex];
            if (barIdx >= 0)
            {
                int baseIndex = barVarBase[barIdx];
                jacobian[row, baseIndex] += weightX;
                jacobian[row, baseIndex + 1] += weightY;
                double s = pinBarOffset[pinIndex];
                jacobian[row, baseIndex + 2] += weightX * (-s * barSin[barIdx]) + weightY * (s * barCos[barIdx]);
                return;
            }

            int ix = varIndexX[pinIndex];
            int iy = varIndexY[pinIndex];
            if (ix >= 0) jacobian[row, ix] += weightX;
            if (iy >= 0) jacobian[row, iy] += weightY;
        }

        private static void ApplyForceOnPin(
            Vector<double> force,
            int pinIndex,
            double fx,
            double fy,
            int[] varIndexX,
            int[] varIndexY,
            int[] pinBarIndex,
            double[] pinBarOffset,
            int[] barVarBase,
            double[] barCos,
            double[] barSin)
        {
            int barIdx = pinBarIndex[pinIndex];
            if (barIdx >= 0)
            {
                int baseIndex = barVarBase[barIdx];
                force[baseIndex] += fx;
                force[baseIndex + 1] += fy;
                double s = pinBarOffset[pinIndex];
                force[baseIndex + 2] += fx * (-s * barSin[barIdx]) + fy * (s * barCos[barIdx]);
                return;
            }

            int ix = varIndexX[pinIndex];
            int iy = varIndexY[pinIndex];
            if (ix >= 0) force[ix] += fx;
            if (iy >= 0) force[iy] += fy;
        }

        private static void BuildResidualsAndJacobian(
            List<Constraint> constraints,
            List<PinInfo> pins,
            double[] posX,
            double[] posY,
            double railOffset,
            int[] varIndexX,
            int[] varIndexY,
            int[] pinBarIndex,
            double[] pinBarOffset,
            int[] barVarBase,
            double[] barCos,
            double[] barSin,
            Vector<double> residuals,
            Matrix<double> jacobian)
        {
            for (int i = 0; i < constraints.Count; i++)
            {
                Constraint c = constraints[i];
                if (c.Type == ConstraintType.Distance)
                {
                    double dx = posX[c.PinA] - posX[c.PinB];
                    double dy = posY[c.PinA] - posY[c.PinB];
                    double len = Math.Sqrt(dx * dx + dy * dy);
                    if (len < LengthEpsilon) len = LengthEpsilon;
                    residuals[i] = len - c.Length;

                    double ux = dx / len;
                    double uy = dy / len;

                    AccumulatePinJacobian(
                        jacobian,
                        i,
                        c.PinA,
                        ux,
                        uy,
                        varIndexX,
                        varIndexY,
                        pinBarIndex,
                        pinBarOffset,
                        barVarBase,
                        barCos,
                        barSin);
                    AccumulatePinJacobian(
                        jacobian,
                        i,
                        c.PinB,
                        -ux,
                        -uy,
                        varIndexX,
                        varIndexY,
                        pinBarIndex,
                        pinBarOffset,
                        barVarBase,
                        barCos,
                        barSin);
                    continue;
                }

                int pinIdx = c.Pin;
                if (c.Axis == 0)
                {
                    residuals[i] = posX[pinIdx] - (pins[pinIdx].X + (pins[pinIdx].IsRailInterface ? railOffset : 0.0));
                    AccumulatePinJacobian(
                        jacobian,
                        i,
                        pinIdx,
                        1.0,
                        0.0,
                        varIndexX,
                        varIndexY,
                        pinBarIndex,
                        pinBarOffset,
                        barVarBase,
                        barCos,
                        barSin);
                }
                else
                {
                    residuals[i] = posY[pinIdx] - pins[pinIdx].Y;
                    AccumulatePinJacobian(
                        jacobian,
                        i,
                        pinIdx,
                        0.0,
                        1.0,
                        varIndexX,
                        varIndexY,
                        pinBarIndex,
                        pinBarOffset,
                        barVarBase,
                        barCos,
                        barSin);
                }
            }
        }

        private static bool SolvePositions(
            List<PinInfo> pins,
            List<Constraint> constraints,
            int railIndex,
            double railOffset,
            int[] varIndexX,
            int[] varIndexY,
            int[] barVarBase,
            double[] variables,
            int[] pinBarIndex,
            double[] pinBarOffset)
        {
            int varCount = variables.Length;
            int constraintCount = constraints.Count;
            double[] posX = new double[pins.Count];
            double[] posY = new double[pins.Count];

            if (varCount == 0)
            {
                BuildBarPose(variables, barVarBase, out double[] barX0, out double[] barY0, out double[] barCos, out double[] barSin);
                FillPositionsCached(
                    pins,
                    railIndex,
                    railOffset,
                    varIndexX,
                    varIndexY,
                    variables,
                    pinBarIndex,
                    pinBarOffset,
                    barX0,
                    barY0,
                    barCos,
                    barSin,
                    posX,
                    posY);
                Vector<double> residuals = Vector<double>.Build.Dense(constraintCount);
                Matrix<double> jacobian = Matrix<double>.Build.Dense(constraintCount, varCount);
                BuildResidualsAndJacobian(
                    constraints,
                    pins,
                    posX,
                    posY,
                    railOffset,
                    varIndexX,
                    varIndexY,
                    pinBarIndex,
                    pinBarOffset,
                    barVarBase,
                    barCos,
                    barSin,
                    residuals,
                    jacobian);
                return residuals.L2Norm() < Tolerance;
            }

            double damping = Damping;
            double[] stepLimits = BuildStepLimits(varCount, barVarBase);

            for (int iter = 0; iter < MaxIterations; iter++)
            {
                BuildBarPose(variables, barVarBase, out double[] barX0, out double[] barY0, out double[] barCos, out double[] barSin);
                FillPositionsCached(
                    pins,
                    railIndex,
                    railOffset,
                    varIndexX,
                    varIndexY,
                    variables,
                    pinBarIndex,
                    pinBarOffset,
                    barX0,
                    barY0,
                    barCos,
                    barSin,
                    posX,
                    posY);
                Vector<double> residuals = Vector<double>.Build.Dense(constraintCount);
                Matrix<double> jacobian = Matrix<double>.Build.Dense(constraintCount, varCount);
                BuildResidualsAndJacobian(
                    constraints,
                    pins,
                    posX,
                    posY,
                    railOffset,
                    varIndexX,
                    varIndexY,
                    pinBarIndex,
                    pinBarOffset,
                    barVarBase,
                    barCos,
                    barSin,
                    residuals,
                    jacobian);

                double norm = residuals.L2Norm();
                if (norm < Tolerance) return true;

                double cond = EstimateCondition(jacobian);
                double localDamping = damping;
                if (cond > ConditionLimit)
                {
                    double scale = Math.Min(cond / ConditionLimit, 1e4);
                    localDamping = Math.Max(localDamping, Damping * scale);
                }

                bool accepted = false;
                Vector<double> delta = Vector<double>.Build.Dense(varCount);
                for (int attempt = 0; attempt < LmMaxTries; attempt++)
                {
                    Matrix<double> A = jacobian.TransposeThisAndMultiply(jacobian);
                    for (int d = 0; d < varCount; d++) A[d, d] += localDamping;
                    Vector<double> b = -jacobian.TransposeThisAndMultiply(residuals);
                    try
                    {
                        delta = A.Solve(b);
                    }
                    catch (Exception)
                    {
                        delta = A.PseudoInverse() * b;
                    }
                    ApplyStepLimits(delta, stepLimits);

                    double[] trialVars = new double[varCount];
                    for (int i = 0; i < varCount; i++) trialVars[i] = variables[i] + delta[i];

                    BuildBarPose(trialVars, barVarBase, out double[] tBarX0, out double[] tBarY0, out double[] tBarCos, out double[] tBarSin);
                    double[] tPosX = new double[pins.Count];
                    double[] tPosY = new double[pins.Count];
                    FillPositionsCached(
                        pins,
                        railIndex,
                        railOffset,
                        varIndexX,
                        varIndexY,
                        trialVars,
                        pinBarIndex,
                        pinBarOffset,
                        tBarX0,
                        tBarY0,
                        tBarCos,
                        tBarSin,
                        tPosX,
                        tPosY);
                    Vector<double> trialResiduals = Vector<double>.Build.Dense(constraintCount);
                    BuildResiduals(constraints, pins, tPosX, tPosY, railOffset, trialResiduals);

                    if (trialResiduals.L2Norm() <= norm)
                    {
                        Array.Copy(trialVars, variables, varCount);
                        damping = Math.Max(DampingMin, localDamping / 10.0);
                        accepted = true;
                        break;
                    }

                    localDamping = Math.Min(DampingMax, localDamping * 10.0);
                    if (delta.L2Norm() < Tolerance)
                    {
                        break;
                    }
                }

                if (!accepted) return false;
                if (delta.L2Norm() < Tolerance) return true;
            }
            return false;
        }

        private static Matrix<double> BuildJacobian(
            List<Constraint> constraints,
            double[] posX,
            double[] posY,
            double railOffset,
            List<PinInfo> pins,
            int[] varIndexX,
            int[] varIndexY,
            int[] pinBarIndex,
            double[] pinBarOffset,
            int[] barVarBase,
            double[] barCos,
            double[] barSin,
            int varCount)
        {
            Matrix<double> jacobian = Matrix<double>.Build.Dense(constraints.Count, varCount);
            Vector<double> residuals = Vector<double>.Build.Dense(constraints.Count);
            BuildResidualsAndJacobian(
                constraints,
                pins,
                posX,
                posY,
                railOffset,
                varIndexX,
                varIndexY,
                pinBarIndex,
                pinBarOffset,
                barVarBase,
                barCos,
                barSin,
                residuals,
                jacobian);
            return jacobian;
        }

        private static void BuildResiduals(
            List<Constraint> constraints,
            List<PinInfo> pins,
            double[] posX,
            double[] posY,
            double railOffset,
            Vector<double> residuals)
        {
            for (int i = 0; i < constraints.Count; i++)
            {
                Constraint c = constraints[i];
                if (c.Type == ConstraintType.Distance)
                {
                    double dx = posX[c.PinA] - posX[c.PinB];
                    double dy = posY[c.PinA] - posY[c.PinB];
                    double len = Math.Sqrt(dx * dx + dy * dy);
                    if (len < LengthEpsilon) len = LengthEpsilon;
                    residuals[i] = len - c.Length;
                }
                else
                {
                    int pinIdx = c.Pin;
                    if (c.Axis == 0)
                    {
                        residuals[i] = posX[pinIdx] - (pins[pinIdx].X + (pins[pinIdx].IsRailInterface ? railOffset : 0.0));
                    }
                    else
                    {
                        residuals[i] = posY[pinIdx] - pins[pinIdx].Y;
                    }
                }
            }
        }

        private static double[] BuildStepLimits(int varCount, int[] barVarBase)
        {
            double[] limits = Enumerable.Repeat(MaxStep, varCount).ToArray();
            foreach (int baseIndex in barVarBase)
            {
                if (baseIndex + 2 < varCount)
                {
                    limits[baseIndex + 2] = MaxThetaStep;
                }
            }
            return limits;
        }

        private static void ApplyStepLimits(Vector<double> delta, double[] limits)
        {
            for (int i = 0; i < delta.Count; i++)
            {
                double limit = limits[i];
                if (delta[i] > limit) delta[i] = limit;
                else if (delta[i] < -limit) delta[i] = -limit;
            }
        }

        private static double EstimateCondition(Matrix<double> jacobian)
        {
            if (jacobian.RowCount == 0 || jacobian.ColumnCount == 0)
            {
                return 0.0;
            }
            try
            {
                var svd = jacobian.Svd(true);
                var s = svd.S;
                if (s.Count == 0) return double.PositiveInfinity;
                double sMax = s[0];
                double sMin = s[s.Count - 1];
                if (sMin <= LengthEpsilon) return double.PositiveInfinity;
                return sMax / sMin;
            }
            catch (Exception)
            {
                return double.PositiveInfinity;
            }
        }

        private static double[] BuildContactPath(double[] contactX, double[] contactY)
        {
            double[] path = new double[contactX.Length];
            for (int i = 1; i < contactX.Length; i++)
            {
                double dx = contactX[i] - contactX[i - 1];
                double dy = contactY[i] - contactY[i - 1];
                path[i] = path[i - 1] + Math.Sqrt(dx * dx + dy * dy);
            }
            return path;
        }

        private static void GetContactTangent(double[] contactX, double[] contactY, int index, out double tx, out double ty)
        {
            int last = contactX.Length - 1;
            if (index <= 0)
            {
                tx = contactX[1] - contactX[0];
                ty = contactY[1] - contactY[0];
            }
            else if (index >= last)
            {
                tx = contactX[last] - contactX[last - 1];
                ty = contactY[last] - contactY[last - 1];
            }
            else
            {
                tx = contactX[index + 1] - contactX[index - 1];
                ty = contactY[index + 1] - contactY[index - 1];
            }

            double len = Math.Sqrt(tx * tx + ty * ty);
            if (len < LengthEpsilon)
            {
                tx = 1.0;
                ty = 0.0;
            }
            else
            {
                tx /= len;
                ty /= len;
            }
        }

        private static double InterpolateAtIndex(double[] values, double index)
        {
            if (values == null || values.Length == 0) return 0.0;
            if (index <= 0.0) return values[0];
            int last = values.Length - 1;
            if (index >= last) return values[last];

            int low = (int)Math.Floor(index);
            int high = low + 1;
            double t = index - low;
            return values[low] + (values[high] - values[low]) * t;
        }
    }
}
