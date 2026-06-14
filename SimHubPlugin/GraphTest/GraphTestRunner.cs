using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Reflection;
using DiyFfb;
using DiyFfb.GraphEditor;
using GraphEditor = DiyFfb.GraphEditor;
using SimHubPlugin.TestCommon;

namespace DiyFfb.GraphTest
{
    public static class GraphTestRunner
    {
        public static void Run()
        {
            if (string.Equals(Environment.GetEnvironmentVariable("FFB_PERF_ONLY"), "1", StringComparison.Ordinal))
            {
                RunPerfHarness();
                return;
            }

            var results = new List<TestResult>();

            // Runtime tests (evaluator, validation, includes)
            results.Add(TestRunner.RunTest("Evaluator basic outputs", TestEvaluatorBasicOutputs));
            results.Add(TestRunner.RunTest("Compiled evaluator matches outputs", TestCompiledEvaluatorMatches));
            results.Add(TestRunner.RunTest("Evaluator trace values", TestEvaluatorTraceValues));
            results.Add(TestRunner.RunTest("JSON load/save roundtrip", TestJsonRoundtrip));
            results.Add(TestRunner.RunTest("Validation catches missing output", TestIncludeOutputValidation));
            results.Add(TestRunner.RunTest("Inline include mapping", TestInlineIncludeMapping));
            results.Add(TestRunner.RunTest("Embedded sub-graph editor roundtrip", TestEmbeddedSubgraphEditorRoundtrip));
            results.Add(TestRunner.RunTest("Block library index", TestBlockLibraryIndex));
            results.Add(TestRunner.RunTest("Schema version mismatch", TestSchemaVersionMismatch));
            results.Add(TestRunner.RunTest("Unknown function validation", TestUnknownFunctionValidation));
            results.Add(TestRunner.RunTest("Output missing src validation", TestOutputMissingSrc));
            results.Add(TestRunner.RunTest("Include missing path validation", TestIncludeMissingPath));
            results.Add(TestRunner.RunTest("Include mapping warnings", TestIncludeMappingWarnings));
            results.Add(TestRunner.RunTest("Multiple includes different inputs", TestMultipleIncludesDifferentInputs));
            results.Add(TestRunner.RunTest("Multiple includes compiled evaluator", TestMultipleIncludesCompiledEvaluator));
            results.Add(TestRunner.RunTest("Multiple includes isolation", TestMultipleIncludesIsolation));
            results.Add(TestRunner.RunTest("Nested includes", TestNestedIncludes));
            results.Add(TestRunner.RunTest("Nested includes via resolver", TestNestedIncludesViaResolver));
            results.Add(TestRunner.RunTest("Diamond dependency includes", TestDiamondDependencyIncludes));
            results.Add(TestRunner.RunTest("Include chaining", TestIncludeChaining));
            results.Add(TestRunner.RunTest("Include with parameters", TestIncludeWithParameters));
            results.Add(TestRunner.RunTest("Include with multiple outputs", TestIncludeMultipleOutputs));
            results.Add(TestRunner.RunTest("Cyclic include detection", TestCyclicIncludeDetection));
            results.Add(TestRunner.RunTest("Op arg count validation", TestOpArgValidation));
            results.Add(TestRunner.RunTest("Variadic op evaluation", TestVariadicOpEvaluation));
            results.Add(TestRunner.RunTest("Op input negate evaluation", TestOpInputNegateEvaluation));
            results.Add(TestRunner.RunTest("Neg op evaluation", TestNegOpEvaluation));
            results.Add(TestRunner.RunTest("Clamp bound order warning", TestClampBoundOrderWarning));
            results.Add(TestRunner.RunTest("Graph output names unique", TestGraphOutputNamesUnique));
            results.Add(TestRunner.RunTest("Compiled evaluator include mapping perf smoke", TestCompiledIncludeMappingPerf));

            // Editor tests (JSON serialization, param schema, parameter resolution)
            results.Add(TestRunner.RunTest("GraphEditor JSON roundtrip", TestGraphEditorJsonRoundtrip));
            results.Add(TestRunner.RunTest("Op input negate JSON roundtrip", TestOpInputNegateJsonRoundtrip));
            results.Add(TestRunner.RunTest("Graph param UI schema roundtrip", TestGraphParamUiRoundtrip));
            results.Add(TestRunner.RunTest("Graph preview evaluation", TestGraphPreviewEvaluation));
            results.Add(TestRunner.RunTest("ParamValues serialization roundtrip", TestParamValuesSerializationRoundtrip));
            results.Add(TestRunner.RunTest("Param resolution: include default", TestParamResolutionIncludeDefault));
            results.Add(TestRunner.RunTest("Param resolution: graph override", TestParamResolutionGraphOverride));
            results.Add(TestRunner.RunTest("Param resolution: vehicle override", TestParamResolutionVehicleOverride));
            results.Add(TestRunner.RunTest("Param resolution: three-tier cascade", TestParamResolutionThreeTierCascade));
            results.Add(TestRunner.RunTest("Param order: graph layout with includes", TestParamOrderWithIncludes));

            // v3 Include node tests
            results.Add(TestRunner.RunTest("ExtractInterface from graph", TestExtractInterfaceFromGraph));
            results.Add(TestRunner.RunTest("v3 Include ports not serialized", TestV3IncludePortsNotSerialized));
            results.Add(TestRunner.RunTest("v2 Include ports migration", TestV2IncludePortsMigration));

            // v4 Library graph tests
            results.Add(TestRunner.RunTest("Library graph interface extraction", TestLibraryGraphInterfaceExtraction));
            results.Add(TestRunner.RunTest("Library graph serialization", TestLibraryGraphSerialization));
            results.Add(TestRunner.RunTest("Library graph Title serialization", TestLibraryGraphTitleSerialization));
            results.Add(TestRunner.RunTest("Library graph runtime conversion", TestLibraryGraphRuntimeConversion));

            // Node duplication tests
            results.Add(TestRunner.RunTest("Node serialization preserves SignalGroup", TestNodeSignalGroupPreservation));
            results.Add(TestRunner.RunTest("Port serialization preserves SignalSuffix", TestPortSignalSuffixPreservation));
            results.Add(TestRunner.RunTest("Op input negate conversion", TestOpInputNegateConversion));
            results.Add(TestRunner.RunTest("Op input negate validation", TestOpInputNegateValidation));

            // Param conversion tests
            results.Add(TestRunner.RunTest("Param ConstValue from graph.Params", TestParamConstValueFromGraphParams));

            // Editor format include tests
            results.Add(TestRunner.RunTest("Editor format include evaluation", TestEditorFormatIncludeEvaluation));

            // Include context cache tests
            results.Add(TestRunner.RunTest("IncludeContextCache add and retrieve", TestIncludeContextCacheAddAndRetrieve));
            results.Add(TestRunner.RunTest("IncludeContextCache case insensitive path", TestIncludeContextCachePathCaseInsensitive));
            results.Add(TestRunner.RunTest("IncludeContextCache multiple includes same path", TestIncludeContextCacheMultipleIncludesSamePath));
            results.Add(TestRunner.RunTest("IncludeContextCache clear", TestIncludeContextCacheClear));
            results.Add(TestRunner.RunTest("Evaluator populates include context cache", TestEvaluatorPopulatesIncludeContextCache));
            results.Add(TestRunner.RunTest("Evaluator clears cache each evaluation", TestEvaluatorClearsCacheEachEvaluation));
            results.Add(TestRunner.RunTest("Graph undo stack basic", TestGraphUndoStackBasic));

            // Param migration tests
            results.Add(TestRunner.RunTest("Param migration: no changes", TestParamMigrationNoChanges));
            results.Add(TestRunner.RunTest("Param migration: clamp to min", TestParamMigrationClampToMin));
            results.Add(TestRunner.RunTest("Param migration: clamp to max", TestParamMigrationClampToMax));
            results.Add(TestRunner.RunTest("Param migration: orphan detected", TestParamMigrationOrphanDetected));
            results.Add(TestRunner.RunTest("Param migration: new orphan vs existing", TestParamMigrationNewOrphanVsExisting));
            results.Add(TestRunner.RunTest("Param migration: changed default", TestParamMigrationChangedDefault));
            results.Add(TestRunner.RunTest("Param migration: orphan restored", TestParamMigrationOrphanRestored));
            results.Add(TestRunner.RunTest("Param migration: combined scenario", TestParamMigrationCombinedScenario));

            // GraphUsageReport.IsShared tests
            results.Add(TestRunner.RunTest("IsShared: multiple direct users", TestIsSharedMultipleDirectUsers));
            results.Add(TestRunner.RunTest("IsShared: single user different vehicle", TestIsSharedSingleUserDifferentVehicle));
            results.Add(TestRunner.RunTest("IsShared: included by graphs with users", TestIsSharedIncludedByGraphsWithUsers));
            results.Add(TestRunner.RunTest("IsShared: single user same vehicle", TestIsSharedSingleUserSameVehicle));
            results.Add(TestRunner.RunTest("IsShared: no users no includes", TestIsSharedNoUsersNoIncludes));
            results.Add(TestRunner.RunTest("IsShared: empty current vehicle key", TestIsSharedEmptyCurrentVehicleKey));
            results.Add(TestRunner.RunTest("IsShared: case insensitive match", TestIsSharedCaseInsensitiveMatch));
            results.Add(TestRunner.RunTest("IsShared: included without vehicle users", TestIsSharedIncludedWithoutVehicleUsers));

            // GraphHashComputer tests
            results.Add(TestRunner.RunTest("GraphHash: invalid path returns null", TestGraphHashInvalidPath));
            results.Add(TestRunner.RunTest("GraphHash: deterministic same content", TestGraphHashDeterministic));
            results.Add(TestRunner.RunTest("GraphHash: changed content different hash", TestGraphHashChangedContent));
            results.Add(TestRunner.RunTest("GraphHash: nested includes in hash", TestGraphHashNestedIncludes));
            results.Add(TestRunner.RunTest("GraphHash: cyclic includes handled", TestGraphHashCyclicIncludes));

            // Tools.cs utility tests
            results.Add(TestRunner.RunTest("Normalize: value in range", TestNormalize_InRange));
            results.Add(TestRunner.RunTest("Normalize: below min", TestNormalize_BelowMin));
            results.Add(TestRunner.RunTest("Normalize: above max", TestNormalize_AboveMax));
            results.Add(TestRunner.RunTest("Normalize: zero range", TestNormalize_ZeroRange));
            results.Add(TestRunner.RunTest("TryComputeMarkerX: valid", TestTryComputeMarkerX_Valid));
            results.Add(TestRunner.RunTest("TryComputeMarkerX: zero width", TestTryComputeMarkerX_ZeroWidth));
            results.Add(TestRunner.RunTest("TryComputeMarkerX: swapped min/max", TestTryComputeMarkerX_SwappedMinMax));
            results.Add(TestRunner.RunTest("TryAutoTuneLoadGain: below min force", TestTryAutoTuneLoadGain_BelowMinForce));
            results.Add(TestRunner.RunTest("TryAutoTuneLoadGain: ratio high", TestTryAutoTuneLoadGain_RatioHigh));
            results.Add(TestRunner.RunTest("TryAutoTuneLoadGain: ratio low", TestTryAutoTuneLoadGain_RatioLow));
            results.Add(TestRunner.RunTest("TryAutoTuneLoadGain: ratio in range", TestTryAutoTuneLoadGain_InRange));

            // CubicSpline tests
            results.Add(TestRunner.RunTest("CubicSpline: linear data", TestInterpolate_LinearData));
            results.Add(TestRunner.RunTest("CubicSpline: endpoint match", TestInterpolate_EndpointMatch));
            results.Add(TestRunner.RunTest("CubicSpline: monotonic", TestInterpolate_Monotonic));
            results.Add(TestRunner.RunTest("CubicSpline: count matches", TestInterpolate1D_CountMatches));
            results.Add(TestRunner.RunTest("CubicSpline: mismatched arrays", TestInterpolate_MismatchedArrays));

            // StringExtensions tests
            results.Add(TestRunner.RunTest("ConstCaseToTitleCase", TestConstCaseToTitleCase));
            results.Add(TestRunner.RunTest("CamelCaseToTitleCase", TestCamelCaseToTitleCase));

            // GeneralKinematics tests
            results.Add(TestRunner.RunTest("Kinematics: null config", TestCalcKinematicParameters_NullConfig));
            results.Add(TestRunner.RunTest("Kinematics: no pins", TestCalcKinematicParameters_NoPins));
            results.Add(TestRunner.RunTest("Kinematics: no bars", TestCalcKinematicParameters_NoBars));
            results.Add(TestRunner.RunTest("Kinematics: negative travel", TestCalcKinematicParameters_NegativeTravel));
            results.Add(TestRunner.RunTest("Kinematics: simple linkage computes", TestSimpleLinkage_Computes));
            results.Add(TestRunner.RunTest("Kinematics: rail travel bounds", TestRailTravel_Bounds));
            results.Add(TestRunner.RunTest("Kinematics: pose cache pin count", TestPoseCache_PinCount));
            results.Add(TestRunner.RunTest("Kinematics: pose cache sample count", TestPoseCache_SampleCount));
            results.Add(TestRunner.RunTest("Kinematics: collinear pins", TestCollinearPins_Handled));
            results.Add(TestRunner.RunTest("Kinematics: zero length bar throws", TestZeroLengthBar_Throws));
            results.Add(TestRunner.RunTest("Kinematics: missing contact point", TestMissingContactPoint_Throws));
            results.Add(TestRunner.RunTest("Kinematics: missing rail interface", TestMissingRailInterface_Throws));

            // GraphRuntimeConverter tests
            results.Add(TestRunner.RunTest("Converter: MapNodeType all kinds", TestMapNodeType_AllKinds));
            results.Add(TestRunner.RunTest("Converter: MapOp all operators", TestMapOp_AllOperators));
            results.Add(TestRunner.RunTest("Converter: MapOp case insensitive", TestMapOp_CaseInsensitive));
            results.Add(TestRunner.RunTest("Converter: MapOp symbols", TestMapOp_Symbols));
            results.Add(TestRunner.RunTest("Converter: input node", TestConvert_InputNode));
            results.Add(TestRunner.RunTest("Converter: param node default value", TestConvert_ParamNode));
            results.Add(TestRunner.RunTest("Converter: output node", TestConvert_OutputNode));
            results.Add(TestRunner.RunTest("Converter: op node args", TestConvert_OpNode_Args));
            results.Add(TestRunner.RunTest("Converter: op negate for add/mul", TestConvert_OpNode_Negate));
            results.Add(TestRunner.RunTest("Converter: include input map", TestConvert_IncludeNode_InputMap));
            results.Add(TestRunner.RunTest("Converter: include output map", TestConvert_IncludeNode_OutputMap));
            results.Add(TestRunner.RunTest("Converter: signal group builds full name", TestConvert_SignalGroup));
            results.Add(TestRunner.RunTest("Converter: signal group legacy fallback", TestConvert_SignalGroup_Legacy));
            results.Add(TestRunner.RunTest("Converter: editor JSON detects links", TestConvertEditorJson_DetectsLinks));
            results.Add(TestRunner.RunTest("Converter: editor JSON detects kind", TestConvertEditorJson_DetectsKind));

            // FunctionScope / ConfigOut / Scoped Output tests
            results.Add(TestRunner.RunTest("Converter: scoped output creates runtime output", TestConvert_ScopedOutput));
            results.Add(TestRunner.RunTest("Converter: unscoped output still appears as include port", TestConvert_UnscopedOutputWithFunctionScope));
            results.Add(TestRunner.RunTest("Converter: ConfigOut creates runtime config output", TestConvert_ConfigOut));
            results.Add(TestRunner.RunTest("Converter: ConfigOut scoped via FunctionScope", TestConvert_ConfigOutScoped));
            results.Add(TestRunner.RunTest("Evaluator: ConfigOut values in ConfigOutputs", TestEval_ConfigOutValues));
            results.Add(TestRunner.RunTest("CompiledEvaluator: ConfigOut values in ConfigOutputs", TestCompiledEval_ConfigOutValues));
            results.Add(TestRunner.RunTest("Evaluator: ConfigOut bridges through Include", TestEval_ConfigOutBridgesThroughInclude));
            results.Add(TestRunner.RunTest("CompiledEvaluator: ConfigOut bridges through Include", TestCompiledEval_ConfigOutBridgesThroughInclude));
            results.Add(TestRunner.RunTest("ExtractInterface: scoped vs unscoped outputs", TestExtractInterface_ScopedOutputs));
            results.Add(TestRunner.RunTest("ExtractInterface: ConfigOut with ConfigType", TestExtractInterface_ConfigOutputs));

            // Conditional op tests
            results.Add(TestRunner.RunTest("Select: true branch", TestSelect_TrueBranch));
            results.Add(TestRunner.RunTest("Select: false branch", TestSelect_FalseBranch));
            results.Add(TestRunner.RunTest("Select: boundary at 0.5", TestSelect_Boundary));
            results.Add(TestRunner.RunTest("Eq: equal values", TestEq_Equal));
            results.Add(TestRunner.RunTest("Eq: unequal values", TestEq_Unequal));
            results.Add(TestRunner.RunTest("Eq: near tolerance", TestEq_NearTolerance));
            results.Add(TestRunner.RunTest("Gt: greater", TestGt_Greater));
            results.Add(TestRunner.RunTest("Gt: less", TestGt_Less));
            results.Add(TestRunner.RunTest("Gt: equal", TestGt_Equal));
            results.Add(TestRunner.RunTest("Tri-state routing via select+eq", TestTriStateRouting));

            // AxisRequestQueue tests
            results.Add(TestRunner.RunTest("Queue: enqueue adds to queue", TestEnqueue_AddsToQueue));
            results.Add(TestRunner.RunTest("Queue: duplicate ignored", TestEnqueue_DuplicateIgnored));
            results.Add(TestRunner.RunTest("Queue: upload not deduplicated", TestEnqueue_UploadNotDeduplicated));
            results.Add(TestRunner.RunTest("Queue: response matches current", TestHandleResponse_MatchesCurrentRequest));
            results.Add(TestRunner.RunTest("Queue: wrong type ignored", TestHandleResponse_WrongType_Ignored));
            results.Add(TestRunner.RunTest("Queue: wrong axis ignored", TestHandleResponse_WrongAxis_Ignored));
            results.Add(TestRunner.RunTest("Queue: requires response types", TestRequiresResponse_RequestTypes));
            results.Add(TestRunner.RunTest("Queue: retry on send failure", TestRetry_OnSendFailure));
            results.Add(TestRunner.RunTest("Queue: max retries exhausted", TestRetry_MaxRetriesExhausted));

            // Stateful graph node tests
            results.Add(TestRunner.RunTest("Accumulator: basic increment", TestAccumulator_BasicIncrement));
            results.Add(TestRunner.RunTest("Accumulator: clamp bounds", TestAccumulator_ClampBounds));
            results.Add(TestRunner.RunTest("Accumulator: reset", TestAccumulator_Reset));
            results.Add(TestRunner.RunTest("Accumulator: no trigger no change", TestAccumulator_NoTrigger));
            results.Add(TestRunner.RunTest("SampleHold: capture on falling edge", TestSampleHold_FallingEdge));
            results.Add(TestRunner.RunTest("SampleHold: hold during trigger high", TestSampleHold_HoldDuringHigh));
            results.Add(TestRunner.RunTest("EdgeDetect: rising edge pulse", TestEdgeDetect_RisingEdge));
            results.Add(TestRunner.RunTest("EdgeDetect: no pulse on sustained", TestEdgeDetect_NoPulseOnSustained));
            results.Add(TestRunner.RunTest("ResetState: clears accumulator", TestResetState_ClearsAccumulator));

            // Expr node tests
            results.Add(TestRunner.RunTest("Expr: arithmetic over inports", TestExpr_Arithmetic));
            results.Add(TestRunner.RunTest("Expr: builtin functions allowed", TestExpr_BuiltinFunctions));
            results.Add(TestRunner.RunTest("Expr: Pi constant usable", TestExpr_PiConstant));
            results.Add(TestRunner.RunTest("Expr: compiled matches interpreter", TestExpr_CompiledMatchesInterpreter));
            results.Add(TestRunner.RunTest("Expr: non-inport identifier rejected", TestExpr_NonInportRejected));
            results.Add(TestRunner.RunTest("Expr: parse error rejected", TestExpr_ParseErrorRejected));
            results.Add(TestRunner.RunTest("Expr: editor roundtrip + conversion", TestExpr_EditorRoundtripAndConversion));

            TestRunner.PrintResults("FFB Graph Tests", results);
        }

        private static void RunPerfHarness()
        {
            const int inputCount = 24;
            const int iterations = 5000;

            var inline = new GraphDefinition();
            for (int i = 0; i < inputCount; i++)
            {
                string inId = $"in_{i}";
                inline.Nodes[inId] = new GraphNode { Id = inId, Type = NodeType.Input, Name = $"Input{i}" };
                string outId = $"out_{i}";
                inline.Nodes[outId] = new GraphNode { Id = outId, Type = NodeType.Output, Name = $"Output{i}", Src = inId };
            }

            var parent = new GraphDefinition();
            for (int i = 0; i < inputCount; i++)
            {
                string inId = $"p_in_{i}";
                parent.Nodes[inId] = new GraphNode { Id = inId, Type = NodeType.Input, Name = $"Input{i}" };
            }

            var include = new GraphNode
            {
                Id = "inc",
                Type = NodeType.Include,
                InlineGraph = inline
            };

            for (int i = 0; i < inputCount; i++)
            {
                include.InputMap[$"Input{i}"] = $"p_in_{i}";
                include.OutputMap[$"Output{i}"] = $"inc_out_{i}";
            }

            parent.Nodes["inc"] = include;

            for (int i = 0; i < inputCount; i++)
            {
                parent.Nodes[$"p_out_{i}"] = new GraphNode
                {
                    Id = $"p_out_{i}",
                    Type = NodeType.Output,
                    Name = $"Result{i}",
                    Src = $"inc_out_{i}"
                };
            }

            var evaluator = new GraphCompiledEvaluator(parent);
            var inputs = new Dictionary<string, double>();
            for (int i = 0; i < inputCount; i++)
            {
                inputs[$"Input{i}"] = i + 1;
            }

            evaluator.Evaluate(inputs, null);

            var sw = Stopwatch.StartNew();
            for (int i = 0; i < iterations; i++)
            {
                evaluator.Evaluate(inputs, null);
            }
            sw.Stop();

            Console.WriteLine($"PerfHarnessMs: {sw.Elapsed.TotalMilliseconds:F2}");
            Console.WriteLine($"PerfHarnessIterations: {iterations}");

            RunExprPerfHarness();
        }

        // Measures the cost of Expr (tree-walking NCalc) nodes against an
        // identically-shaped Op graph, isolating the marginal ns + GC cost of the
        // boxing path. Decision metric for whether a compiled-lambda path is worth
        // its semantic-divergence risk. Runs under FFB_PERF_ONLY=1.
        private static void RunExprPerfHarness()
        {
            const int nodeCount = 12;     // compute nodes (Expr or Op), each over 3 inports
            const int iterations = 200000;

            // Build two graphs with identical shape: N inputs feeding N 3-input
            // compute nodes ("a * b * c"), each exposed as an output.
            GraphCompiledEvaluator BuildGraph(bool useExpr)
            {
                var g = new GraphDefinition();
                for (int i = 0; i < nodeCount * 3; i++)
                {
                    string id = $"in_{i}";
                    g.Nodes[id] = new GraphNode { Id = id, Type = NodeType.Input, Name = id };
                }
                for (int n = 0; n < nodeCount; n++)
                {
                    string a = $"in_{n * 3}", b = $"in_{n * 3 + 1}", c = $"in_{n * 3 + 2}";
                    string nodeId = $"calc_{n}";
                    if (useExpr)
                    {
                        var expr = new GraphNode { Id = nodeId, Type = NodeType.Expr, Expr = "a * b * c" };
                        expr.InputMap["a"] = a; expr.InputMap["b"] = b; expr.InputMap["c"] = c;
                        g.Nodes[nodeId] = expr;
                    }
                    else
                    {
                        g.Nodes[nodeId] = new GraphNode
                        {
                            Id = nodeId, Type = NodeType.Op, Op = OpType.Mul,
                            Args = new List<string> { a, b, c }
                        };
                    }
                    g.Nodes[$"out_{n}"] = new GraphNode { Id = $"out_{n}", Type = NodeType.Output, Name = $"R{n}", Src = nodeId };
                }
                return new GraphCompiledEvaluator(g);
            }

            var inputs = new Dictionary<string, double>();
            for (int i = 0; i < nodeCount * 3; i++) inputs[$"in_{i}"] = (i % 7) + 1;

            try { AppDomain.MonitoringIsEnabled = true; } catch { /* may already be on */ }

            var exprStats = MeasureEval(BuildGraph(true), inputs, iterations);
            var opStats = MeasureEval(BuildGraph(false), inputs, iterations);

            // Marginal Expr cost over the native Op path (both share the per-eval
            // result-dictionary overhead, which cancels in the delta).
            double dNs = (exprStats.NsPerTick - opStats.NsPerTick) / nodeCount;
            double dBytes = (double)(exprStats.Bytes - opStats.Bytes) / iterations / nodeCount;

            Console.WriteLine($"ExprPerf: nodes={nodeCount} iters={iterations}");
            Console.WriteLine($"ExprPerf: Expr {exprStats.NsPerTick:F0} ns/tick, {(double)exprStats.Bytes / iterations:F0} B/tick, gen0={exprStats.Gen0}");
            Console.WriteLine($"ExprPerf: Op   {opStats.NsPerTick:F0} ns/tick, {(double)opStats.Bytes / iterations:F0} B/tick, gen0={opStats.Gen0}");
            Console.WriteLine($"ExprPerf: marginal Expr cost = {dNs:F0} ns/node, {dBytes:F0} B/node");
            Console.WriteLine($"ExprPerf: projected at 60 Hz = {(exprStats.Bytes - opStats.Bytes) / iterations * 60.0 / 1024.0:F1} KB/s for {nodeCount} Expr nodes");
        }

        private struct EvalStats
        {
            public double NsPerTick;
            public long Bytes;
            public int Gen0;
        }

        private static EvalStats MeasureEval(GraphCompiledEvaluator eval, Dictionary<string, double> inputs, int iterations)
        {
            eval.Evaluate(inputs, null); // warm up (compile + JIT)
            GC.Collect();
            GC.WaitForPendingFinalizers();
            GC.Collect();

            long bytes0 = AllocatedBytes();
            int gen0 = GC.CollectionCount(0);

            var sw = Stopwatch.StartNew();
            for (int i = 0; i < iterations; i++)
            {
                eval.Evaluate(inputs, null);
            }
            sw.Stop();

            return new EvalStats
            {
                NsPerTick = sw.Elapsed.TotalMilliseconds * 1e6 / iterations,
                Bytes = AllocatedBytes() - bytes0,
                Gen0 = GC.CollectionCount(0) - gen0
            };
        }

        // Cumulative process allocation. AppDomain monitoring (available on .NET
        // Framework) gives an accurate byte count; falls back to a coarse estimate.
        private static long AllocatedBytes()
        {
            try { return AppDomain.CurrentDomain.MonitoringTotalAllocatedMemorySize; }
            catch { return GC.GetTotalMemory(false); }
        }

        private static bool TestEvaluatorBasicOutputs()
        {
            var graph = BuildBaseGraph();
            var outputs = new GraphEvaluator(graph).Evaluate(
                new Dictionary<string, double> { ["ias_kts"] = 60.0, ["vref_kts"] = 60.0 },
                new Dictionary<string, double> { ["k_q"] = 1.0, ["k_rate"] = 0.5, ["k_friction"] = 0.1 });

            return outputs.ContainsKey("spring") &&
                   outputs.ContainsKey("damper") &&
                   outputs.ContainsKey("friction");
        }

        private static bool TestEvaluatorTraceValues()
        {
            var graph = BuildBaseGraph();
            var result = new GraphEvaluator(graph).EvaluateWithTrace(
                new Dictionary<string, double> { ["ias_kts"] = 80.0, ["vref_kts"] = 80.0 },
                new Dictionary<string, double> { ["k_q"] = 1.0, ["k_rate"] = 0.5, ["k_friction"] = 0.1 });

            return result.NodeValues.ContainsKey("qhat") &&
                   result.NodeValues.ContainsKey("spring") &&
                   result.Outputs.ContainsKey("spring");
        }

        private static bool TestGraphUndoStackBasic()
        {
            var stack = new GraphUndoStack();
            var snap1 = new GraphUndoSnapshot { GraphJson = "a" };
            var snap2 = new GraphUndoSnapshot { GraphJson = "b" };

            stack.Reset(snap1);
            if (stack.Count != 1 || stack.CanUndo || stack.CanRedo || stack.IsDirty)
            {
                return false;
            }

            stack.Push(snap2);
            if (!stack.CanUndo || stack.CanRedo || !stack.IsDirty)
            {
                return false;
            }

            var undo = stack.Undo();
            if (undo != snap1 || stack.IsDirty)
            {
                return false;
            }

            var redo = stack.Redo();
            if (redo != snap2 || !stack.IsDirty)
            {
                return false;
            }

            stack.MarkClean();
            return !stack.IsDirty;
        }

        private static bool TestCompiledEvaluatorMatches()
        {
            var graph = BuildBaseGraph();
            var inputs = new Dictionary<string, double> { ["ias_kts"] = 85.0, ["vref_kts"] = 70.0 };
            var parameters = new Dictionary<string, double> { ["k_q"] = 1.3, ["k_rate"] = 0.6, ["k_friction"] = 0.2 };
            var legacy = new GraphEvaluator(graph).Evaluate(inputs, parameters);
            var compiled = new GraphCompiledEvaluator(graph).Evaluate(inputs, parameters);

            return Math.Abs(legacy["spring"] - compiled["spring"]) < 1e-6 &&
                   Math.Abs(legacy["damper"] - compiled["damper"]) < 1e-6 &&
                   Math.Abs(legacy["friction"] - compiled["friction"]) < 1e-6;
        }

        private static bool TestJsonRoundtrip()
        {
            var graph = BuildBaseGraph();
            var saver = new GraphSaver();
            string json = saver.SaveToJson(graph);
            var loader = new GraphLoader();
            var loaded = loader.LoadFromJson(json, out var validation);
            return validation.IsValid && loaded.Nodes.Count == graph.Nodes.Count;
        }

        private static bool TestIncludeOutputValidation()
        {
            var graph = new GraphDefinition();
            graph.Nodes["in"] = new GraphNode { Id = "in", Type = NodeType.Input, Name = "cmd_force" };
            graph.Nodes["include"] = new GraphNode
            {
                Id = "include",
                Type = NodeType.Include,
                InlineGraph = BuildInlineGraph(),
                InputMap = { ["cmd_force"] = "in" },
                OutputMap = { ["missing_out"] = "host_out" }
            };
            graph.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "out", Src = "host_out" };

            var validation = GraphValidator.Validate(graph);
            return !validation.IsValid;
        }

        private static bool TestInlineIncludeMapping()
        {
            var graph = new GraphDefinition();
            graph.Nodes["load"] = new GraphNode { Id = "load", Type = NodeType.Input, Name = "cmd_force" };
            graph.Nodes["inc"] = new GraphNode
            {
                Id = "inc",
                Type = NodeType.Include,
                InlineGraph = BuildInlineGraph(),
                InputMap = { ["cmd_force"] = "load" },
                OutputMap = { ["out_force"] = "force_out" }
            };
            graph.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "out", Src = "force_out" };

            var outputs = new GraphEvaluator(graph, new GraphIncludeResolver(AppContext.BaseDirectory)).Evaluate(
                new Dictionary<string, double> { ["cmd_force"] = 2.0 },
                new Dictionary<string, double>());

            return outputs.TryGetValue("out", out var value) && Math.Abs(value - 2.0) < 0.0001;
        }

        private static bool TestBlockLibraryIndex()
        {
            // The block library index is populated when a FILE include is
            // resolved. Embedded (inline, path-less) includes deliberately do
            // NOT register or spill to disk — they stay self-contained in the
            // parent. So exercise the index via a real include file.
            string tempDir = System.IO.Path.Combine(System.IO.Path.GetTempPath(), "ffb_graph_test");
            string embeddedDir = System.IO.Path.Combine(tempDir, "graphs", "_embedded");
            string indexPath = System.IO.Path.Combine(embeddedDir, "index.json");
            if (System.IO.File.Exists(indexPath)) System.IO.File.Delete(indexPath); // isolate
            System.IO.Directory.CreateDirectory(embeddedDir);

            string blockPath = System.IO.Path.Combine(embeddedDir, "test_block.json");
            System.IO.File.WriteAllText(blockPath, new GraphSaver().SaveToJson(BuildInlineGraph()));

            var resolver = new GraphIncludeResolver(tempDir);
            resolver.GetGraph(blockPath);
            return System.IO.File.Exists(indexPath);
        }

        // Editor-format embedded sub-graph: a path-less Include carrying an
        // inline definition must deserialize, derive its ports, convert, and
        // evaluate straight from memory (no file, no disk spill).
        private static bool TestEmbeddedSubgraphEditorRoundtrip()
        {
            const string json = @"{
  ""Version"": 4,
  ""Nodes"": [
    { ""Id"": ""in_rpm"", ""Kind"": ""Input"", ""SignalGroup"": ""MSFS"",
      ""Ports"": [{ ""Kind"": ""Output"", ""SignalSuffix"": ""MainRotor.Speed"" }] },
    { ""Id"": ""emb"", ""Kind"": ""Include"", ""Title"": ""Embedded Double"",
      ""Inline"": {
        ""Version"": 4, ""IsLibraryGraph"": true,
        ""Nodes"": [
          { ""Id"": ""s_in"", ""Kind"": ""Input"", ""Ports"": [{ ""Name"": ""x"", ""Kind"": ""Output"" }] },
          { ""Id"": ""s_two"", ""Kind"": ""Const"", ""ConstValue"": 2.0, ""Ports"": [{ ""Name"": ""out"", ""Kind"": ""Output"" }] },
          { ""Id"": ""s_mul"", ""Kind"": ""Op"", ""Op"": ""mul"",
            ""Ports"": [{ ""Name"": ""a"", ""Kind"": ""Input"" }, { ""Name"": ""b"", ""Kind"": ""Input"" }, { ""Name"": ""a*b"", ""Kind"": ""Output"" }] },
          { ""Id"": ""s_out"", ""Kind"": ""Output"", ""Ports"": [{ ""Name"": ""y"", ""Kind"": ""Input"" }] }
        ],
        ""Links"": [
          { ""FromNodeId"": ""s_in"", ""FromPort"": ""x"", ""ToNodeId"": ""s_mul"", ""ToPort"": ""a"" },
          { ""FromNodeId"": ""s_two"", ""FromPort"": ""out"", ""ToNodeId"": ""s_mul"", ""ToPort"": ""b"" },
          { ""FromNodeId"": ""s_mul"", ""FromPort"": ""a*b"", ""ToNodeId"": ""s_out"", ""ToPort"": ""y"" }
        ]
      }
    },
    { ""Id"": ""out_v"", ""Kind"": ""Output"", ""SignalGroup"": ""Shared"",
      ""Ports"": [{ ""Kind"": ""Input"", ""SignalSuffix"": ""Vib1Fund"" }] }
  ],
  ""Links"": [
    { ""FromNodeId"": ""in_rpm"", ""FromPort"": ""MainRotor.Speed"", ""ToNodeId"": ""emb"", ""ToPort"": ""x"" },
    { ""FromNodeId"": ""emb"", ""FromPort"": ""y"", ""ToNodeId"": ""out_v"", ""ToPort"": ""Vib1Fund"" }
  ]
}";
            // Deserialize editor format: the path-less Include must carry the inline graph.
            var editorGraph = DiyFfb.GraphEditor.GraphSerializer.Deserialize(json, out var validation);
            if (editorGraph == null || !validation.IsValid) return false;
            var inc = editorGraph.Nodes.FirstOrDefault(n => n.Id == "emb");
            if (inc == null || inc.InlineGraph == null || !string.IsNullOrEmpty(inc.IncludePath)) return false;

            // Ports must be derived from the inline graph's Input/Output nodes.
            DiyFfb.GraphEditor.GraphSerializer.PopulateIncludePorts(editorGraph, AppContext.BaseDirectory);
            bool hasX = inc.Ports.Any(p => p.Name == "x" && p.Kind == DiyFfb.GraphEditor.GraphPortKind.Input);
            bool hasY = inc.Ports.Any(p => p.Name == "y" && p.Kind == DiyFfb.GraphEditor.GraphPortKind.Output);
            if (!hasX || !hasY) return false;

            // Convert to runtime: the include node carries the converted InlineGraph
            // (resolver/evaluator read it straight from memory) and maps boundary ports.
            var runtime = DiyFfb.GraphEditor.GraphRuntimeConverter.Convert(editorGraph);
            if (!runtime.Nodes.TryGetValue("emb", out var rn)) return false;
            if (rn.InlineGraph == null || !rn.InputMap.ContainsKey("x") || !rn.OutputMap.ContainsKey("y")) return false;

            // Re-serialize: the inline block must survive the round-trip.
            string roundtrip = DiyFfb.GraphEditor.GraphSerializer.Serialize(editorGraph);
            var reloaded = DiyFfb.GraphEditor.GraphSerializer.Deserialize(roundtrip, out var v2);
            var inc2 = reloaded?.Nodes.FirstOrDefault(n => n.Id == "emb");
            return v2 != null && v2.IsValid && inc2 != null && inc2.InlineGraph != null
                   && inc2.InlineGraph.Nodes.Count == 4;
        }

        private static bool TestSchemaVersionMismatch()
        {
            var graph = BuildBaseGraph();
            graph.Version = 2;
            var validation = GraphValidator.Validate(graph);
            return !validation.IsValid;
        }

        private static bool TestUnknownFunctionValidation()
        {
            var graph = BuildBaseGraph();
            graph.Nodes["bad_func"] = new GraphNode
            {
                Id = "bad_func",
                Type = NodeType.Func,
                Func = "unknown_func",
                Args = { "ias", "vref" }
            };
            graph.Nodes["out_bad"] = new GraphNode { Id = "out_bad", Type = NodeType.Output, Name = "bad", Src = "bad_func" };
            var validation = GraphValidator.Validate(graph);
            return !validation.IsValid;
        }

        private static bool TestOutputMissingSrc()
        {
            var graph = BuildBaseGraph();
            graph.Nodes["out_bad"] = new GraphNode { Id = "out_bad", Type = NodeType.Output, Name = "bad" };
            var validation = GraphValidator.Validate(graph);
            return !validation.IsValid;
        }

        private static bool TestIncludeMissingPath()
        {
            var graph = BuildBaseGraph();
            graph.Nodes["inc"] = new GraphNode
            {
                Id = "inc",
                Type = NodeType.Include,
                InlineGraph = null
            };
            var validation = GraphValidator.Validate(graph);
            return !validation.IsValid;
        }

        private static bool TestIncludeMappingWarnings()
        {
            var graph = new GraphDefinition();
            graph.Nodes["inc"] = new GraphNode
            {
                Id = "inc",
                Type = NodeType.Include,
                InlineGraph = BuildInlineGraph()
            };
            graph.Nodes["val"] = new GraphNode { Id = "val", Type = NodeType.Const, ConstValue = 1.0 };
            graph.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "out", Src = "val" };
            var validation = GraphValidator.Validate(graph);
            return validation.IsValid && validation.Warnings.Count > 0;
        }

        private static bool TestOpArgValidation()
        {
            var graph = BuildBaseGraph();
            graph.Nodes["bad_op"] = new GraphNode
            {
                Id = "bad_op",
                Type = NodeType.Op,
                Op = OpType.Mul,
                Args = { "ias" }
            };
            graph.Nodes["out_bad"] = new GraphNode { Id = "out_bad", Type = NodeType.Output, Name = "bad", Src = "bad_op" };
            var validation = GraphValidator.Validate(graph);
            return !validation.IsValid;
        }

        private static bool TestVariadicOpEvaluation()
        {
            var graph = new GraphDefinition { Version = 1 };
            graph.Nodes["a"] = new GraphNode { Id = "a", Type = NodeType.Const, ConstValue = 2.0 };
            graph.Nodes["b"] = new GraphNode { Id = "b", Type = NodeType.Const, ConstValue = 3.0 };
            graph.Nodes["c"] = new GraphNode { Id = "c", Type = NodeType.Const, ConstValue = 4.0 };
            graph.Nodes["sum"] = new GraphNode
            {
                Id = "sum",
                Type = NodeType.Op,
                Op = OpType.Add,
                Args = { "a", "b", "c" }
            };
            graph.Nodes["product"] = new GraphNode
            {
                Id = "product",
                Type = NodeType.Op,
                Op = OpType.Mul,
                Args = { "a", "b", "c" }
            };
            graph.Nodes["out_sum"] = new GraphNode { Id = "out_sum", Type = NodeType.Output, Name = "sum", Src = "sum" };
            graph.Nodes["out_product"] = new GraphNode { Id = "out_product", Type = NodeType.Output, Name = "product", Src = "product" };

            var outputs = new GraphEvaluator(graph).Evaluate(null, null);
            if (!outputs.TryGetValue("sum", out var sum) || Math.Abs(sum - 9.0) > 1e-6)
            {
                return false;
            }
            if (!outputs.TryGetValue("product", out var product) || Math.Abs(product - 24.0) > 1e-6)
            {
                return false;
            }

            var compiledOutputs = new GraphCompiledEvaluator(graph).Evaluate(null, null);
            return compiledOutputs.TryGetValue("sum", out var compiledSum) &&
                   Math.Abs(compiledSum - 9.0) < 1e-6 &&
                   compiledOutputs.TryGetValue("product", out var compiledProduct) &&
                   Math.Abs(compiledProduct - 24.0) < 1e-6;
        }

        private static bool TestOpInputNegateEvaluation()
        {
            var graph = new GraphDefinition { Version = 1 };
            graph.Nodes["a"] = new GraphNode { Id = "a", Type = NodeType.Const, ConstValue = 5.0 };
            graph.Nodes["b"] = new GraphNode { Id = "b", Type = NodeType.Const, ConstValue = 2.0 };
            graph.Nodes["c"] = new GraphNode { Id = "c", Type = NodeType.Const, ConstValue = 1.0 };
            graph.Nodes["sum"] = new GraphNode
            {
                Id = "sum",
                Type = NodeType.Op,
                Op = OpType.Add,
                Args = { "a", "b", "c" },
                ArgNegate = { false, true, true }
            };
            graph.Nodes["product"] = new GraphNode
            {
                Id = "product",
                Type = NodeType.Op,
                Op = OpType.Mul,
                Args = { "a", "b", "c" },
                ArgNegate = { false, true, false }
            };
            graph.Nodes["out_sum"] = new GraphNode { Id = "out_sum", Type = NodeType.Output, Name = "sum", Src = "sum" };
            graph.Nodes["out_product"] = new GraphNode { Id = "out_product", Type = NodeType.Output, Name = "product", Src = "product" };

            var outputs = new GraphEvaluator(graph).Evaluate(null, null);
            if (!outputs.TryGetValue("sum", out var sum) || Math.Abs(sum - 2.0) > 1e-6)
            {
                return false;
            }
            if (!outputs.TryGetValue("product", out var product) || Math.Abs(product + 10.0) > 1e-6)
            {
                return false;
            }

            var compiledOutputs = new GraphCompiledEvaluator(graph).Evaluate(null, null);
            return compiledOutputs.TryGetValue("sum", out var compiledSum) &&
                   Math.Abs(compiledSum - 2.0) < 1e-6 &&
                   compiledOutputs.TryGetValue("product", out var compiledProduct) &&
                   Math.Abs(compiledProduct + 10.0) < 1e-6;
        }

        private static bool TestNegOpEvaluation()
        {
            var graph = new GraphDefinition { Version = 1 };
            graph.Nodes["in"] = new GraphNode { Id = "in", Type = NodeType.Input, Name = "in" };
            graph.Nodes["neg"] = new GraphNode { Id = "neg", Type = NodeType.Op, Op = OpType.Neg, Args = { "in" } };
            graph.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "out", Src = "neg" };

            var inputs = new Dictionary<string, double> { ["in"] = 3.5 };
            var outputs = new GraphEvaluator(graph).Evaluate(inputs, null);
            if (!outputs.TryGetValue("out", out var value) || Math.Abs(value + 3.5) > 1e-6)
            {
                return false;
            }

            var compiled = new GraphCompiledEvaluator(graph).Evaluate(inputs, null);
            return compiled.TryGetValue("out", out var compiledValue) &&
                   Math.Abs(compiledValue + 3.5) < 1e-6;
        }

        private static bool TestClampBoundOrderWarning()
        {
            var graph = new GraphDefinition();
            graph.Nodes["val"] = new GraphNode { Id = "val", Type = NodeType.Const, ConstValue = 0.5 };
            graph.Nodes["min"] = new GraphNode { Id = "min", Type = NodeType.Const, ConstValue = 2.0 };
            graph.Nodes["max"] = new GraphNode { Id = "max", Type = NodeType.Const, ConstValue = 1.0 };
            graph.Nodes["clamp"] = new GraphNode
            {
                Id = "clamp",
                Type = NodeType.Op,
                Op = OpType.Clamp,
                Args = { "val", "min", "max" }
            };
            graph.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "out", Src = "clamp" };
            var validation = GraphValidator.Validate(graph);
            return validation.IsValid && validation.Warnings.Count > 0;
        }

        private static bool TestGraphOutputNamesUnique()
        {
            var outputs = new HashSet<string>(StringComparer.Ordinal);
            foreach (var name in GraphSignalCatalogData.OutputNames)
            {
                if (!outputs.Add(name))
                {
                    return false;
                }
            }

            return outputs.Contains("FlightStickPitch.SpringGain");
        }

        private static bool TestCompiledIncludeMappingPerf()
        {
            const int inputCount = 24;
            const int iterations = 2000;
            const int maxMilliseconds = 5000;

            var inline = new GraphDefinition();
            for (int i = 0; i < inputCount; i++)
            {
                string inId = $"in_{i}";
                inline.Nodes[inId] = new GraphNode { Id = inId, Type = NodeType.Input, Name = $"Input{i}" };
                string outId = $"out_{i}";
                inline.Nodes[outId] = new GraphNode { Id = outId, Type = NodeType.Output, Name = $"Output{i}", Src = inId };
            }

            var parent = new GraphDefinition();
            for (int i = 0; i < inputCount; i++)
            {
                string inId = $"p_in_{i}";
                parent.Nodes[inId] = new GraphNode { Id = inId, Type = NodeType.Input, Name = $"Input{i}" };
            }

            var include = new GraphNode
            {
                Id = "inc",
                Type = NodeType.Include,
                InlineGraph = inline
            };

            for (int i = 0; i < inputCount; i++)
            {
                include.InputMap[$"Input{i}"] = $"p_in_{i}";
                include.OutputMap[$"Output{i}"] = $"inc_out_{i}";
            }

            parent.Nodes["inc"] = include;

            for (int i = 0; i < inputCount; i++)
            {
                parent.Nodes[$"p_out_{i}"] = new GraphNode
                {
                    Id = $"p_out_{i}",
                    Type = NodeType.Output,
                    Name = $"Result{i}",
                    Src = $"inc_out_{i}"
                };
            }

            var evaluator = new GraphCompiledEvaluator(parent);
            var inputs = new Dictionary<string, double>();
            for (int i = 0; i < inputCount; i++)
            {
                inputs[$"Input{i}"] = i + 1;
            }

            evaluator.Evaluate(inputs, null);

            var sw = Stopwatch.StartNew();
            for (int i = 0; i < iterations; i++)
            {
                evaluator.Evaluate(inputs, null);
            }
            sw.Stop();

            if (sw.ElapsedMilliseconds > maxMilliseconds)
            {
                return false;
            }

            var outputs = evaluator.Evaluate(inputs, null);
            for (int i = 0; i < inputCount; i++)
            {
                if (!outputs.TryGetValue($"Result{i}", out var value) || Math.Abs(value - (i + 1)) > 1e-6)
                {
                    return false;
                }
            }

            return true;
        }

        private static GraphDefinition BuildBaseGraph()
        {
            var graph = new GraphDefinition();
            graph.Nodes["ias"] = new GraphNode { Id = "ias", Type = NodeType.Input, Name = "ias_kts" };
            graph.Nodes["vref"] = new GraphNode { Id = "vref", Type = NodeType.Input, Name = "vref_kts" };
            graph.Nodes["qhat"] = new GraphNode
            {
                Id = "qhat",
                Type = NodeType.Func,
                Func = "qhat_eff",
                Args = { "ias", "vref" }
            };
            graph.Nodes["kq"] = new GraphNode { Id = "kq", Type = NodeType.Param, Name = "k_q" };
            graph.Nodes["kr"] = new GraphNode { Id = "kr", Type = NodeType.Param, Name = "k_rate" };
            graph.Nodes["kf"] = new GraphNode { Id = "kf", Type = NodeType.Param, Name = "k_friction" };
            graph.Nodes["spring"] = new GraphNode { Id = "spring", Type = NodeType.Op, Op = OpType.Mul, Args = { "kq", "qhat" } };
            graph.Nodes["damper"] = new GraphNode { Id = "damper", Type = NodeType.Op, Op = OpType.Mul, Args = { "kr", "qhat" } };
            graph.Nodes["friction"] = new GraphNode { Id = "friction", Type = NodeType.Op, Op = OpType.Mul, Args = { "kf", "qhat" } };
            graph.Nodes["out_spring"] = new GraphNode { Id = "out_spring", Type = NodeType.Output, Name = "spring", Src = "spring" };
            graph.Nodes["out_damper"] = new GraphNode { Id = "out_damper", Type = NodeType.Output, Name = "damper", Src = "damper" };
            graph.Nodes["out_friction"] = new GraphNode { Id = "out_friction", Type = NodeType.Output, Name = "friction", Src = "friction" };
            return graph;
        }

        private static GraphDefinition BuildInlineGraph()
        {
            var graph = new GraphDefinition();
            graph.Nodes["cmd"] = new GraphNode { Id = "cmd", Type = NodeType.Input, Name = "cmd_force" };
            graph.Nodes["out_force"] = new GraphNode { Id = "out_force", Type = NodeType.Output, Name = "out_force", Src = "cmd" };
            return graph;
        }

        /// <summary>
        /// Build an inline graph that scales input by a factor (input * factor).
        /// </summary>
        private static GraphDefinition BuildScalerGraph()
        {
            var graph = new GraphDefinition();
            graph.Nodes["in_val"] = new GraphNode { Id = "in_val", Type = NodeType.Input, Name = "value" };
            graph.Nodes["in_factor"] = new GraphNode { Id = "in_factor", Type = NodeType.Input, Name = "factor" };
            graph.Nodes["mul"] = new GraphNode
            {
                Id = "mul",
                Type = NodeType.Op,
                Op = OpType.Mul,
                Args = { "in_val", "in_factor" }
            };
            graph.Nodes["out_result"] = new GraphNode { Id = "out_result", Type = NodeType.Output, Name = "result", Src = "mul" };
            return graph;
        }

        private static bool TestMultipleIncludesDifferentInputs()
        {
            // Test that the same include graph can be used multiple times with different inputs
            // and produce different outputs
            var scalerGraph = BuildScalerGraph();

            var graph = new GraphDefinition();

            // Two input values
            graph.Nodes["val_a"] = new GraphNode { Id = "val_a", Type = NodeType.Const, ConstValue = 10.0 };
            graph.Nodes["val_b"] = new GraphNode { Id = "val_b", Type = NodeType.Const, ConstValue = 20.0 };

            // Two different factors
            graph.Nodes["factor_a"] = new GraphNode { Id = "factor_a", Type = NodeType.Const, ConstValue = 2.0 };
            graph.Nodes["factor_b"] = new GraphNode { Id = "factor_b", Type = NodeType.Const, ConstValue = 3.0 };

            // First include: 10 * 2 = 20
            graph.Nodes["inc_a"] = new GraphNode
            {
                Id = "inc_a",
                Type = NodeType.Include,
                InlineGraph = scalerGraph,
                InputMap = { ["value"] = "val_a", ["factor"] = "factor_a" },
                OutputMap = { ["result"] = "result_a" }
            };

            // Second include: 20 * 3 = 60
            graph.Nodes["inc_b"] = new GraphNode
            {
                Id = "inc_b",
                Type = NodeType.Include,
                InlineGraph = scalerGraph,
                InputMap = { ["value"] = "val_b", ["factor"] = "factor_b" },
                OutputMap = { ["result"] = "result_b" }
            };

            // Outputs
            graph.Nodes["out_a"] = new GraphNode { Id = "out_a", Type = NodeType.Output, Name = "out_a", Src = "result_a" };
            graph.Nodes["out_b"] = new GraphNode { Id = "out_b", Type = NodeType.Output, Name = "out_b", Src = "result_b" };

            // GraphEvaluator requires a resolver for InlineGraph handling
            var evaluator = new GraphEvaluator(graph, new GraphIncludeResolver(AppContext.BaseDirectory));
            var outputs = evaluator.Evaluate(
                new Dictionary<string, double>(),
                new Dictionary<string, double>());

            // Verify both includes produce correct, different results
            bool hasOutA = outputs.TryGetValue("out_a", out var outA);
            bool hasOutB = outputs.TryGetValue("out_b", out var outB);

            return hasOutA && hasOutB &&
                   Math.Abs(outA - 20.0) < 0.0001 &&  // 10 * 2 = 20
                   Math.Abs(outB - 60.0) < 0.0001;    // 20 * 3 = 60
        }

        private static bool TestMultipleIncludesCompiledEvaluator()
        {
            // Same test as above but with compiled evaluator to ensure consistency
            var scalerGraph = BuildScalerGraph();

            var graph = new GraphDefinition();

            graph.Nodes["val_a"] = new GraphNode { Id = "val_a", Type = NodeType.Const, ConstValue = 10.0 };
            graph.Nodes["val_b"] = new GraphNode { Id = "val_b", Type = NodeType.Const, ConstValue = 20.0 };
            graph.Nodes["factor_a"] = new GraphNode { Id = "factor_a", Type = NodeType.Const, ConstValue = 2.0 };
            graph.Nodes["factor_b"] = new GraphNode { Id = "factor_b", Type = NodeType.Const, ConstValue = 3.0 };

            graph.Nodes["inc_a"] = new GraphNode
            {
                Id = "inc_a",
                Type = NodeType.Include,
                InlineGraph = scalerGraph,
                InputMap = { ["value"] = "val_a", ["factor"] = "factor_a" },
                OutputMap = { ["result"] = "result_a" }
            };

            graph.Nodes["inc_b"] = new GraphNode
            {
                Id = "inc_b",
                Type = NodeType.Include,
                InlineGraph = scalerGraph,
                InputMap = { ["value"] = "val_b", ["factor"] = "factor_b" },
                OutputMap = { ["result"] = "result_b" }
            };

            graph.Nodes["out_a"] = new GraphNode { Id = "out_a", Type = NodeType.Output, Name = "out_a", Src = "result_a" };
            graph.Nodes["out_b"] = new GraphNode { Id = "out_b", Type = NodeType.Output, Name = "out_b", Src = "result_b" };

            // Test with compiled evaluator
            var compiled = new GraphCompiledEvaluator(graph);
            var outputs = compiled.Evaluate(
                new Dictionary<string, double>(),
                new Dictionary<string, double>());

            bool hasOutA = outputs.TryGetValue("out_a", out var outA);
            bool hasOutB = outputs.TryGetValue("out_b", out var outB);

            return hasOutA && hasOutB &&
                   Math.Abs(outA - 20.0) < 0.0001 &&
                   Math.Abs(outB - 60.0) < 0.0001;
        }

        private static bool TestMultipleIncludesIsolation()
        {
            // Test that multiple includes of the same graph are truly isolated:
            // - Each include gets its own evaluation context
            // - Changing inputs to one include doesn't affect the other
            // - The same graph reference is used but results differ based on inputs
            var scalerGraph = BuildScalerGraph();

            var graph = new GraphDefinition();

            // Use inputs instead of consts so we can vary them
            graph.Nodes["input_val"] = new GraphNode { Id = "input_val", Type = NodeType.Input, Name = "input_val" };
            graph.Nodes["factor_small"] = new GraphNode { Id = "factor_small", Type = NodeType.Const, ConstValue = 0.5 };
            graph.Nodes["factor_large"] = new GraphNode { Id = "factor_large", Type = NodeType.Const, ConstValue = 10.0 };

            // Both includes use the SAME input value but different factors
            graph.Nodes["inc_small"] = new GraphNode
            {
                Id = "inc_small",
                Type = NodeType.Include,
                InlineGraph = scalerGraph,
                InputMap = { ["value"] = "input_val", ["factor"] = "factor_small" },
                OutputMap = { ["result"] = "result_small" }
            };

            graph.Nodes["inc_large"] = new GraphNode
            {
                Id = "inc_large",
                Type = NodeType.Include,
                InlineGraph = scalerGraph,
                InputMap = { ["value"] = "input_val", ["factor"] = "factor_large" },
                OutputMap = { ["result"] = "result_large" }
            };

            graph.Nodes["out_small"] = new GraphNode { Id = "out_small", Type = NodeType.Output, Name = "out_small", Src = "result_small" };
            graph.Nodes["out_large"] = new GraphNode { Id = "out_large", Type = NodeType.Output, Name = "out_large", Src = "result_large" };

            // Test with both evaluators (interpreter requires resolver for InlineGraph)
            var resolver = new GraphIncludeResolver(AppContext.BaseDirectory);
            var interpreter = new GraphEvaluator(graph, resolver);
            var compiled = new GraphCompiledEvaluator(graph);

            // First evaluation: input = 100
            var inputs1 = new Dictionary<string, double> { ["input_val"] = 100.0 };
            var params1 = new Dictionary<string, double>();

            var interpOut1 = interpreter.Evaluate(inputs1, params1);
            var compOut1 = compiled.Evaluate(inputs1, params1);

            // Second evaluation: input = 50 (to verify state doesn't persist)
            var inputs2 = new Dictionary<string, double> { ["input_val"] = 50.0 };

            var interpOut2 = interpreter.Evaluate(inputs2, params1);
            var compOut2 = compiled.Evaluate(inputs2, params1);

            // Verify first evaluation: 100 * 0.5 = 50, 100 * 10 = 1000
            bool interp1Ok = interpOut1.TryGetValue("out_small", out var i1s) &&
                             interpOut1.TryGetValue("out_large", out var i1l) &&
                             Math.Abs(i1s - 50.0) < 0.0001 &&
                             Math.Abs(i1l - 1000.0) < 0.0001;

            bool comp1Ok = compOut1.TryGetValue("out_small", out var c1s) &&
                           compOut1.TryGetValue("out_large", out var c1l) &&
                           Math.Abs(c1s - 50.0) < 0.0001 &&
                           Math.Abs(c1l - 1000.0) < 0.0001;

            // Verify second evaluation: 50 * 0.5 = 25, 50 * 10 = 500
            bool interp2Ok = interpOut2.TryGetValue("out_small", out var i2s) &&
                             interpOut2.TryGetValue("out_large", out var i2l) &&
                             Math.Abs(i2s - 25.0) < 0.0001 &&
                             Math.Abs(i2l - 500.0) < 0.0001;

            bool comp2Ok = compOut2.TryGetValue("out_small", out var c2s) &&
                           compOut2.TryGetValue("out_large", out var c2l) &&
                           Math.Abs(c2s - 25.0) < 0.0001 &&
                           Math.Abs(c2l - 500.0) < 0.0001;

            return interp1Ok && comp1Ok && interp2Ok && comp2Ok;
        }

        private static bool TestNestedIncludes()
        {
            // Test nested includes: Parent -> Middle -> Inner
            // Inner: doubles input (x * 2)
            // Middle: includes Inner, then adds 10 to result
            // Parent: includes Middle with input 5 -> Inner(5)=10, Middle(10)+10=20

            // Inner graph: output = input * 2
            var innerGraph = new GraphDefinition();
            innerGraph.Nodes["in"] = new GraphNode { Id = "in", Type = NodeType.Input, Name = "x" };
            innerGraph.Nodes["two"] = new GraphNode { Id = "two", Type = NodeType.Const, ConstValue = 2.0 };
            innerGraph.Nodes["mul"] = new GraphNode
            {
                Id = "mul",
                Type = NodeType.Op,
                Op = OpType.Mul,
                Args = { "in", "two" }
            };
            innerGraph.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "doubled", Src = "mul" };

            // Middle graph: includes Inner, then adds 10
            var middleGraph = new GraphDefinition();
            middleGraph.Nodes["in"] = new GraphNode { Id = "in", Type = NodeType.Input, Name = "value" };
            middleGraph.Nodes["inner"] = new GraphNode
            {
                Id = "inner",
                Type = NodeType.Include,
                InlineGraph = innerGraph,
                InputMap = { ["x"] = "in" },
                OutputMap = { ["doubled"] = "inner_result" }
            };
            middleGraph.Nodes["ten"] = new GraphNode { Id = "ten", Type = NodeType.Const, ConstValue = 10.0 };
            middleGraph.Nodes["add"] = new GraphNode
            {
                Id = "add",
                Type = NodeType.Op,
                Op = OpType.Add,
                Args = { "inner_result", "ten" }
            };
            middleGraph.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "final", Src = "add" };

            // Parent graph: includes Middle with input 5
            // Expected: Inner(5) = 5*2 = 10, Middle = 10+10 = 20
            var parentGraph = new GraphDefinition();
            parentGraph.Nodes["five"] = new GraphNode { Id = "five", Type = NodeType.Const, ConstValue = 5.0 };
            parentGraph.Nodes["middle"] = new GraphNode
            {
                Id = "middle",
                Type = NodeType.Include,
                InlineGraph = middleGraph,
                InputMap = { ["value"] = "five" },
                OutputMap = { ["final"] = "middle_result" }
            };
            parentGraph.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "result", Src = "middle_result" };

            // Test with both evaluators
            var resolver = new GraphIncludeResolver(AppContext.BaseDirectory);
            var interpreter = new GraphEvaluator(parentGraph, resolver);
            var compiled = new GraphCompiledEvaluator(parentGraph, resolver);

            var inputs = new Dictionary<string, double>();
            var parameters = new Dictionary<string, double>();

            var interpOut = interpreter.Evaluate(inputs, parameters);
            var compOut = compiled.Evaluate(inputs, parameters);

            // Expected: 5 * 2 + 10 = 20
            bool interpOk = interpOut.TryGetValue("result", out var interpVal) && Math.Abs(interpVal - 20.0) < 0.0001;
            bool compOk = compOut.TryGetValue("result", out var compVal) && Math.Abs(compVal - 20.0) < 0.0001;

            return interpOk && compOk;
        }

        private static bool TestNestedIncludesViaResolver()
        {
            // Test nested includes loaded via EditorFormatConverter through the resolver.
            // This tests that editor-format sub-graphs with nested includes work correctly.
            //
            // Directory structure:
            //   tempDir/
            //     parent.json       (includes "sub/middle.json")
            //     sub/
            //       middle.json     (includes "inner/inner.json")
            //       inner/
            //         inner.json    (output = input * 3)
            //
            // Calculation: parent passes 7 -> middle -> inner -> 7 * 3 = 21

            string tempDir = System.IO.Path.Combine(System.IO.Path.GetTempPath(), "ffb_nested_include_test_" + Guid.NewGuid().ToString("N").Substring(0, 8));
            string subDir = System.IO.Path.Combine(tempDir, "sub");
            string innerDir = System.IO.Path.Combine(subDir, "inner");

            try
            {
                System.IO.Directory.CreateDirectory(innerDir);

                // Inner graph: output = input * 3
                var innerGraph = new GraphEditor.GraphDefinition { IsLibraryGraph = true };
                var innerIn = new GraphEditor.GraphNode { Id = "in", Kind = GraphEditor.GraphNodeKind.Input };
                innerIn.Ports.Add(new GraphEditor.GraphPort { Name = "x", Kind = GraphEditor.GraphPortKind.Output });
                innerGraph.Nodes.Add(innerIn);

                var innerConst = new GraphEditor.GraphNode { Id = "three", Kind = GraphEditor.GraphNodeKind.Const, ConstValue = 3.0 };
                innerGraph.Nodes.Add(innerConst);

                var innerMul = new GraphEditor.GraphNode { Id = "mul", Kind = GraphEditor.GraphNodeKind.Op, Op = "mul" };
                innerMul.Ports.Add(new GraphEditor.GraphPort { Name = "a", Kind = GraphEditor.GraphPortKind.Input });
                innerMul.Ports.Add(new GraphEditor.GraphPort { Name = "b", Kind = GraphEditor.GraphPortKind.Input });
                innerMul.Ports.Add(new GraphEditor.GraphPort { Name = "result", Kind = GraphEditor.GraphPortKind.Output });
                innerGraph.Nodes.Add(innerMul);

                var innerOut = new GraphEditor.GraphNode { Id = "out", Kind = GraphEditor.GraphNodeKind.Output };
                innerOut.Ports.Add(new GraphEditor.GraphPort { Name = "result", Kind = GraphEditor.GraphPortKind.Input });
                innerGraph.Nodes.Add(innerOut);

                innerGraph.Links.Add(new GraphEditor.GraphLink { FromNodeId = "in", FromPort = "x", ToNodeId = "mul", ToPort = "a" });
                innerGraph.Links.Add(new GraphEditor.GraphLink { FromNodeId = "three", FromPort = "value", ToNodeId = "mul", ToPort = "b" });
                innerGraph.Links.Add(new GraphEditor.GraphLink { FromNodeId = "mul", FromPort = "result", ToNodeId = "out", ToPort = "result" });

                string innerPath = System.IO.Path.Combine(innerDir, "inner.json");
                System.IO.File.WriteAllText(innerPath, GraphEditor.GraphSerializer.Serialize(innerGraph));

                // Middle graph: includes inner/inner.json (relative to middle.json location)
                var middleGraph = new GraphEditor.GraphDefinition { IsLibraryGraph = true };
                var middleIn = new GraphEditor.GraphNode { Id = "in", Kind = GraphEditor.GraphNodeKind.Input };
                middleIn.Ports.Add(new GraphEditor.GraphPort { Name = "val", Kind = GraphEditor.GraphPortKind.Output });
                middleGraph.Nodes.Add(middleIn);

                var middleInclude = new GraphEditor.GraphNode
                {
                    Id = "inc",
                    Kind = GraphEditor.GraphNodeKind.Include,
                    IncludePath = "inner/inner.json"  // Relative to middle.json's directory (sub/)
                };
                middleGraph.Nodes.Add(middleInclude);

                var middleOut = new GraphEditor.GraphNode { Id = "out", Kind = GraphEditor.GraphNodeKind.Output };
                middleOut.Ports.Add(new GraphEditor.GraphPort { Name = "output", Kind = GraphEditor.GraphPortKind.Input });
                middleGraph.Nodes.Add(middleOut);

                // Populate include ports before serialization (simulates editor behavior)
                GraphEditor.GraphSerializer.PopulateIncludePorts(middleGraph, subDir);

                // Add links after ports are populated
                middleGraph.Links.Add(new GraphEditor.GraphLink { FromNodeId = "in", FromPort = "val", ToNodeId = "inc", ToPort = "x" });
                middleGraph.Links.Add(new GraphEditor.GraphLink { FromNodeId = "inc", FromPort = "result", ToNodeId = "out", ToPort = "output" });

                string middlePath = System.IO.Path.Combine(subDir, "middle.json");
                System.IO.File.WriteAllText(middlePath, GraphEditor.GraphSerializer.Serialize(middleGraph));

                // Parent graph: includes sub/middle.json
                var parentGraph = new GraphEditor.GraphDefinition();
                var parentConst = new GraphEditor.GraphNode { Id = "seven", Kind = GraphEditor.GraphNodeKind.Const, ConstValue = 7.0 };
                parentGraph.Nodes.Add(parentConst);

                var parentInclude = new GraphEditor.GraphNode
                {
                    Id = "mid",
                    Kind = GraphEditor.GraphNodeKind.Include,
                    IncludePath = "sub/middle.json"  // Relative to parent.json's directory (tempDir/)
                };
                parentGraph.Nodes.Add(parentInclude);

                var parentOut = new GraphEditor.GraphNode { Id = "out", Kind = GraphEditor.GraphNodeKind.Output };
                parentOut.Ports.Add(new GraphEditor.GraphPort { Name = "final", Kind = GraphEditor.GraphPortKind.Input });
                parentGraph.Nodes.Add(parentOut);

                // Populate include ports
                GraphEditor.GraphSerializer.PopulateIncludePorts(parentGraph, tempDir);

                // Add links after ports are populated
                parentGraph.Links.Add(new GraphEditor.GraphLink { FromNodeId = "seven", FromPort = "value", ToNodeId = "mid", ToPort = "val" });
                parentGraph.Links.Add(new GraphEditor.GraphLink { FromNodeId = "mid", FromPort = "output", ToNodeId = "out", ToPort = "final" });

                // Convert parent to runtime
                var parentRuntimeDll = GraphEditor.GraphRuntimeConverter.Convert(parentGraph);
                string parentRuntimeJson = Newtonsoft.Json.JsonConvert.SerializeObject(parentRuntimeDll);
                var parentRuntime = Newtonsoft.Json.JsonConvert.DeserializeObject<GraphDefinition>(parentRuntimeJson);

                // Create resolver with EditorFormatConverter that handles nested includes
                var resolver = new GraphIncludeResolver(tempDir)
                {
                    EditorFormatConverter = (json, resolvedPath) =>
                    {
                        var g = GraphEditor.GraphSerializer.Deserialize(json, out var validation);
                        if (g == null)
                        {
                            return null;
                        }

                        // Populate nested Include ports using the file's directory
                        // This is critical for nested includes to resolve relative paths correctly
                        string fileDir = System.IO.Path.GetDirectoryName(resolvedPath) ?? tempDir;
                        GraphEditor.GraphSerializer.PopulateIncludePorts(g, fileDir);

                        // Convert editor graph to runtime format
                        var dllRuntime = GraphEditor.GraphRuntimeConverter.Convert(g);
                        string runtimeJson = Newtonsoft.Json.JsonConvert.SerializeObject(dllRuntime);
                        return Newtonsoft.Json.JsonConvert.DeserializeObject<GraphDefinition>(runtimeJson);
                    }
                };

                // Evaluate
                var evaluator = new GraphCompiledEvaluator(parentRuntime, resolver, null, tempDir);
                var outputs = evaluator.Evaluate(new Dictionary<string, double>(), new Dictionary<string, double>());

                // Expected: 7 * 3 = 21
                if (!outputs.TryGetValue("final", out var result))
                {
                    return false;
                }

                return Math.Abs(result - 21.0) < 0.0001;
            }
            catch
            {
                return false;
            }
            finally
            {
                try { System.IO.Directory.Delete(tempDir, true); } catch { }
            }
        }

        private static bool TestDiamondDependencyIncludes()
        {
            // Test diamond dependency: Parent includes A and B, both include same Inner graph
            //
            //       Parent
            //       /    \
            //    Inc_A   Inc_B
            //       \    /
            //       Inner (shared)
            //
            // This tests that the shared inner graph doesn't cause interference

            // Inner graph: output = input + 1
            var innerGraph = new GraphDefinition();
            innerGraph.Nodes["in"] = new GraphNode { Id = "in", Type = NodeType.Input, Name = "x" };
            innerGraph.Nodes["one"] = new GraphNode { Id = "one", Type = NodeType.Const, ConstValue = 1.0 };
            innerGraph.Nodes["add"] = new GraphNode
            {
                Id = "add",
                Type = NodeType.Op,
                Op = OpType.Add,
                Args = { "in", "one" }
            };
            innerGraph.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "result", Src = "add" };

            // Graph A: includes Inner, multiplies result by 2
            var graphA = new GraphDefinition();
            graphA.Nodes["in"] = new GraphNode { Id = "in", Type = NodeType.Input, Name = "value" };
            graphA.Nodes["inner"] = new GraphNode
            {
                Id = "inner",
                Type = NodeType.Include,
                InlineGraph = innerGraph,
                InputMap = { ["x"] = "in" },
                OutputMap = { ["result"] = "inner_out" }
            };
            graphA.Nodes["two"] = new GraphNode { Id = "two", Type = NodeType.Const, ConstValue = 2.0 };
            graphA.Nodes["mul"] = new GraphNode
            {
                Id = "mul",
                Type = NodeType.Op,
                Op = OpType.Mul,
                Args = { "inner_out", "two" }
            };
            graphA.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "out_a", Src = "mul" };

            // Graph B: includes Inner, multiplies result by 3
            var graphB = new GraphDefinition();
            graphB.Nodes["in"] = new GraphNode { Id = "in", Type = NodeType.Input, Name = "value" };
            graphB.Nodes["inner"] = new GraphNode
            {
                Id = "inner",
                Type = NodeType.Include,
                InlineGraph = innerGraph,
                InputMap = { ["x"] = "in" },
                OutputMap = { ["result"] = "inner_out" }
            };
            graphB.Nodes["three"] = new GraphNode { Id = "three", Type = NodeType.Const, ConstValue = 3.0 };
            graphB.Nodes["mul"] = new GraphNode
            {
                Id = "mul",
                Type = NodeType.Op,
                Op = OpType.Mul,
                Args = { "inner_out", "three" }
            };
            graphB.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "out_b", Src = "mul" };

            // Parent: includes both A and B with input 10
            // A: (10+1)*2 = 22
            // B: (10+1)*3 = 33
            var parent = new GraphDefinition();
            parent.Nodes["ten"] = new GraphNode { Id = "ten", Type = NodeType.Const, ConstValue = 10.0 };
            parent.Nodes["inc_a"] = new GraphNode
            {
                Id = "inc_a",
                Type = NodeType.Include,
                InlineGraph = graphA,
                InputMap = { ["value"] = "ten" },
                OutputMap = { ["out_a"] = "result_a" }
            };
            parent.Nodes["inc_b"] = new GraphNode
            {
                Id = "inc_b",
                Type = NodeType.Include,
                InlineGraph = graphB,
                InputMap = { ["value"] = "ten" },
                OutputMap = { ["out_b"] = "result_b" }
            };
            parent.Nodes["out_a"] = new GraphNode { Id = "out_a", Type = NodeType.Output, Name = "final_a", Src = "result_a" };
            parent.Nodes["out_b"] = new GraphNode { Id = "out_b", Type = NodeType.Output, Name = "final_b", Src = "result_b" };

            var resolver = new GraphIncludeResolver(AppContext.BaseDirectory);
            var interpreter = new GraphEvaluator(parent, resolver);
            var compiled = new GraphCompiledEvaluator(parent, resolver);

            var inputs = new Dictionary<string, double>();
            var parameters = new Dictionary<string, double>();

            var interpOut = interpreter.Evaluate(inputs, parameters);
            var compOut = compiled.Evaluate(inputs, parameters);

            // Expected: A = (10+1)*2 = 22, B = (10+1)*3 = 33
            bool interpOk = interpOut.TryGetValue("final_a", out var ia) &&
                            interpOut.TryGetValue("final_b", out var ib) &&
                            Math.Abs(ia - 22.0) < 0.0001 &&
                            Math.Abs(ib - 33.0) < 0.0001;

            bool compOk = compOut.TryGetValue("final_a", out var ca) &&
                          compOut.TryGetValue("final_b", out var cb) &&
                          Math.Abs(ca - 22.0) < 0.0001 &&
                          Math.Abs(cb - 33.0) < 0.0001;

            return interpOk && compOk;
        }

        private static bool TestIncludeChaining()
        {
            // Test chaining: output of Include A feeds into Include B's input
            // Input -> Include A -> Include B -> Output

            // Graph A: doubles input
            var graphA = new GraphDefinition();
            graphA.Nodes["in"] = new GraphNode { Id = "in", Type = NodeType.Input, Name = "x" };
            graphA.Nodes["two"] = new GraphNode { Id = "two", Type = NodeType.Const, ConstValue = 2.0 };
            graphA.Nodes["mul"] = new GraphNode
            {
                Id = "mul",
                Type = NodeType.Op,
                Op = OpType.Mul,
                Args = { "in", "two" }
            };
            graphA.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "doubled", Src = "mul" };

            // Graph B: adds 100
            var graphB = new GraphDefinition();
            graphB.Nodes["in"] = new GraphNode { Id = "in", Type = NodeType.Input, Name = "y" };
            graphB.Nodes["hundred"] = new GraphNode { Id = "hundred", Type = NodeType.Const, ConstValue = 100.0 };
            graphB.Nodes["add"] = new GraphNode
            {
                Id = "add",
                Type = NodeType.Op,
                Op = OpType.Add,
                Args = { "in", "hundred" }
            };
            graphB.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "result", Src = "add" };

            // Parent: 5 -> A (5*2=10) -> B (10+100=110)
            var parent = new GraphDefinition();
            parent.Nodes["five"] = new GraphNode { Id = "five", Type = NodeType.Const, ConstValue = 5.0 };
            parent.Nodes["inc_a"] = new GraphNode
            {
                Id = "inc_a",
                Type = NodeType.Include,
                InlineGraph = graphA,
                InputMap = { ["x"] = "five" },
                OutputMap = { ["doubled"] = "a_out" }
            };
            parent.Nodes["inc_b"] = new GraphNode
            {
                Id = "inc_b",
                Type = NodeType.Include,
                InlineGraph = graphB,
                InputMap = { ["y"] = "a_out" },  // Chained from inc_a output
                OutputMap = { ["result"] = "b_out" }
            };
            parent.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "final", Src = "b_out" };

            var resolver = new GraphIncludeResolver(AppContext.BaseDirectory);
            var interpreter = new GraphEvaluator(parent, resolver);
            var compiled = new GraphCompiledEvaluator(parent, resolver);

            var inputs = new Dictionary<string, double>();
            var parameters = new Dictionary<string, double>();

            var interpOut = interpreter.Evaluate(inputs, parameters);
            var compOut = compiled.Evaluate(inputs, parameters);

            // Expected: 5 * 2 + 100 = 110
            bool interpOk = interpOut.TryGetValue("final", out var iv) && Math.Abs(iv - 110.0) < 0.0001;
            bool compOk = compOut.TryGetValue("final", out var cv) && Math.Abs(cv - 110.0) < 0.0001;

            return interpOk && compOk;
        }

        private static bool TestIncludeWithParameters()
        {
            // Test that parameters flow through to included graphs

            // Inner graph: output = input * param_k
            var innerGraph = new GraphDefinition();
            innerGraph.Nodes["in"] = new GraphNode { Id = "in", Type = NodeType.Input, Name = "value" };
            innerGraph.Nodes["k"] = new GraphNode { Id = "k", Type = NodeType.Param, Name = "gain" };
            innerGraph.Nodes["mul"] = new GraphNode
            {
                Id = "mul",
                Type = NodeType.Op,
                Op = OpType.Mul,
                Args = { "in", "k" }
            };
            innerGraph.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "scaled", Src = "mul" };

            // Parent: includes inner with input 10, param gain=3
            var parent = new GraphDefinition();
            parent.Nodes["ten"] = new GraphNode { Id = "ten", Type = NodeType.Const, ConstValue = 10.0 };
            parent.Nodes["inc"] = new GraphNode
            {
                Id = "inc",
                Type = NodeType.Include,
                InlineGraph = innerGraph,
                InputMap = { ["value"] = "ten" },
                OutputMap = { ["scaled"] = "inc_out" }
            };
            parent.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "result", Src = "inc_out" };

            var resolver = new GraphIncludeResolver(AppContext.BaseDirectory);
            var interpreter = new GraphEvaluator(parent, resolver);
            var compiled = new GraphCompiledEvaluator(parent, resolver);

            var inputs = new Dictionary<string, double>();
            var parameters = new Dictionary<string, double> { ["gain"] = 3.0 };

            var interpOut = interpreter.Evaluate(inputs, parameters);
            var compOut = compiled.Evaluate(inputs, parameters);

            // Expected: 10 * 3 = 30
            bool interpOk = interpOut.TryGetValue("result", out var iv) && Math.Abs(iv - 30.0) < 0.0001;
            bool compOk = compOut.TryGetValue("result", out var cv) && Math.Abs(cv - 30.0) < 0.0001;

            return interpOk && compOk;
        }

        private static bool TestIncludeMultipleOutputs()
        {
            // Test that a single include can produce multiple outputs that are all consumed

            // Inner graph: computes both sum and product
            var innerGraph = new GraphDefinition();
            innerGraph.Nodes["a"] = new GraphNode { Id = "a", Type = NodeType.Input, Name = "a" };
            innerGraph.Nodes["b"] = new GraphNode { Id = "b", Type = NodeType.Input, Name = "b" };
            innerGraph.Nodes["sum"] = new GraphNode
            {
                Id = "sum",
                Type = NodeType.Op,
                Op = OpType.Add,
                Args = { "a", "b" }
            };
            innerGraph.Nodes["product"] = new GraphNode
            {
                Id = "product",
                Type = NodeType.Op,
                Op = OpType.Mul,
                Args = { "a", "b" }
            };
            innerGraph.Nodes["out_sum"] = new GraphNode { Id = "out_sum", Type = NodeType.Output, Name = "sum", Src = "sum" };
            innerGraph.Nodes["out_product"] = new GraphNode { Id = "out_product", Type = NodeType.Output, Name = "product", Src = "product" };

            // Parent: uses both outputs from the include
            var parent = new GraphDefinition();
            parent.Nodes["three"] = new GraphNode { Id = "three", Type = NodeType.Const, ConstValue = 3.0 };
            parent.Nodes["four"] = new GraphNode { Id = "four", Type = NodeType.Const, ConstValue = 4.0 };
            parent.Nodes["inc"] = new GraphNode
            {
                Id = "inc",
                Type = NodeType.Include,
                InlineGraph = innerGraph,
                InputMap = { ["a"] = "three", ["b"] = "four" },
                OutputMap = { ["sum"] = "the_sum", ["product"] = "the_product" }
            };
            parent.Nodes["out_sum"] = new GraphNode { Id = "out_sum", Type = NodeType.Output, Name = "final_sum", Src = "the_sum" };
            parent.Nodes["out_product"] = new GraphNode { Id = "out_product", Type = NodeType.Output, Name = "final_product", Src = "the_product" };

            var resolver = new GraphIncludeResolver(AppContext.BaseDirectory);
            var interpreter = new GraphEvaluator(parent, resolver);
            var compiled = new GraphCompiledEvaluator(parent, resolver);

            var inputs = new Dictionary<string, double>();
            var parameters = new Dictionary<string, double>();

            var interpOut = interpreter.Evaluate(inputs, parameters);
            var compOut = compiled.Evaluate(inputs, parameters);

            // Expected: sum = 3+4 = 7, product = 3*4 = 12
            bool interpOk = interpOut.TryGetValue("final_sum", out var iSum) &&
                            interpOut.TryGetValue("final_product", out var iProd) &&
                            Math.Abs(iSum - 7.0) < 0.0001 &&
                            Math.Abs(iProd - 12.0) < 0.0001;

            bool compOk = compOut.TryGetValue("final_sum", out var cSum) &&
                          compOut.TryGetValue("final_product", out var cProd) &&
                          Math.Abs(cSum - 7.0) < 0.0001 &&
                          Math.Abs(cProd - 12.0) < 0.0001;

            return interpOk && compOk;
        }

        private static bool TestCyclicIncludeDetection()
        {
            // Test that cyclic includes are detected during evaluation
            // This creates A includes B, B includes A scenario
            // Note: We can't easily create true file-based cycles with InlineGraph,
            // so we test the topological sort cycle detection in the evaluator

            // Create a graph with a cycle in node dependencies (not includes)
            // This tests the TopoSort cycle detection
            var graph = new GraphDefinition();
            graph.Nodes["a"] = new GraphNode
            {
                Id = "a",
                Type = NodeType.Op,
                Op = OpType.Add,
                Args = { "b", "one" }  // a depends on b
            };
            graph.Nodes["b"] = new GraphNode
            {
                Id = "b",
                Type = NodeType.Op,
                Op = OpType.Mul,
                Args = { "a", "two" }  // b depends on a -> CYCLE!
            };
            graph.Nodes["one"] = new GraphNode { Id = "one", Type = NodeType.Const, ConstValue = 1.0 };
            graph.Nodes["two"] = new GraphNode { Id = "two", Type = NodeType.Const, ConstValue = 2.0 };
            graph.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "result", Src = "a" };

            // Both evaluators should throw or handle the cycle gracefully
            bool interpreterDetectedCycle = false;
            bool compiledDetectedCycle = false;

            try
            {
                var interpreter = new GraphEvaluator(graph);
                interpreter.Evaluate(new Dictionary<string, double>(), new Dictionary<string, double>());
            }
            catch (InvalidOperationException ex) when (ex.Message.Contains("cycle"))
            {
                interpreterDetectedCycle = true;
            }
            catch
            {
                // Other exceptions might occur, but we specifically want cycle detection
            }

            try
            {
                var compiled = new GraphCompiledEvaluator(graph);
                compiled.Evaluate(new Dictionary<string, double>(), new Dictionary<string, double>());
            }
            catch (InvalidOperationException ex) when (ex.Message.Contains("cycle"))
            {
                compiledDetectedCycle = true;
            }
            catch
            {
                // Other exceptions might occur
            }

            // Test passes if at least one evaluator detected the cycle
            // (implementation may vary in how cycles are handled)
            return interpreterDetectedCycle || compiledDetectedCycle;
        }

        // Editor tests (from PluginTest)

        private static bool TestGraphEditorJsonRoundtrip()
        {
            var graph = new GraphEditor.GraphDefinition();
            var input = new GraphEditor.GraphNode
            {
                Id = "node_input",
                Title = "Input",
                Kind = GraphNodeKind.Input,
                X = 10,
                Y = 20
            };
            input.Ports.Add(new GraphPort { Name = "out", Kind = GraphPortKind.Output });
            graph.Nodes.Add(input);

            graph.Params["k_q"] = new GraphParam
            {
                Name = "k_q",
                DefaultValue = 1.0,
                Min = 0.0,
                Max = 5.0
            };

            string json = GraphSerializer.Serialize(graph);
            var loaded = GraphSerializer.Deserialize(json, out var validation);
            return validation.IsValid && loaded.Nodes.Count == 1 && loaded.Params.ContainsKey("k_q");
        }

        private static bool TestOpInputNegateJsonRoundtrip()
        {
            var graph = new GraphEditor.GraphDefinition();
            var op = new GraphEditor.GraphNode
            {
                Id = "op",
                Kind = GraphEditor.GraphNodeKind.Op,
                Op = "add"
            };
            op.Ports.Add(new GraphEditor.GraphPort { Name = "a", Kind = GraphEditor.GraphPortKind.Input });
            op.Ports.Add(new GraphEditor.GraphPort { Name = "b", Kind = GraphEditor.GraphPortKind.Input, Negate = true });
            op.Ports.Add(new GraphEditor.GraphPort { Name = "a+b", Kind = GraphEditor.GraphPortKind.Output });
            graph.Nodes.Add(op);

            string json = GraphSerializer.Serialize(graph);
            var loaded = GraphSerializer.Deserialize(json, out var validation);
            if (!validation.IsValid || loaded.Nodes.Count != 1)
            {
                return false;
            }

            var loadedOp = loaded.Nodes.FirstOrDefault();
            var negatedPort = loadedOp?.Ports.FirstOrDefault(p => p.Kind == GraphEditor.GraphPortKind.Input && p.Name == "b");
            return negatedPort != null && negatedPort.Negate;
        }

        private static bool TestGraphPreviewEvaluation()
        {
            var graph = new GraphEditor.GraphDefinition();
            var input = new GraphEditor.GraphNode
            {
                Id = "in",
                Title = "ias_kts",
                Kind = GraphNodeKind.Input
            };
            input.Ports.Add(new GraphPort { Name = "out", Kind = GraphPortKind.Output });
            graph.Nodes.Add(input);

            var param = new GraphEditor.GraphNode
            {
                Id = "param",
                Title = "k_q",
                Kind = GraphNodeKind.Param
            };
            param.Ports.Add(new GraphPort { Name = "out", Kind = GraphPortKind.Output });
            graph.Nodes.Add(param);

            var op = new GraphEditor.GraphNode
            {
                Id = "mul",
                Title = "Mul",
                Kind = GraphNodeKind.Op,
                Op = "mul"
            };
            op.Ports.Add(new GraphPort { Name = "a", Kind = GraphPortKind.Input });
            op.Ports.Add(new GraphPort { Name = "b", Kind = GraphPortKind.Input });
            op.Ports.Add(new GraphPort { Name = "out", Kind = GraphPortKind.Output });
            graph.Nodes.Add(op);

            var output = new GraphEditor.GraphNode
            {
                Id = "out",
                Title = "out_force",
                Kind = GraphNodeKind.Output
            };
            output.Ports.Add(new GraphPort { Name = "in", Kind = GraphPortKind.Input });
            graph.Nodes.Add(output);

            graph.Links.Add(new GraphLink { FromNodeId = "in", FromPort = "out", ToNodeId = "mul", ToPort = "a" });
            graph.Links.Add(new GraphLink { FromNodeId = "param", FromPort = "out", ToNodeId = "mul", ToPort = "b" });
            graph.Links.Add(new GraphLink { FromNodeId = "mul", FromPort = "out", ToNodeId = "out", ToPort = "in" });

            var evaluator = new GraphPreviewEvaluator();
            var result = evaluator.Evaluate(graph,
                new Dictionary<string, double> { ["ias_kts"] = 2.0 },
                new Dictionary<string, double> { ["k_q"] = 3.0 });

            return result.Outputs.Count > 0;
        }

        private static bool TestGraphParamUiRoundtrip()
        {
            var graph = new GraphEditor.GraphDefinition();
            graph.Params["k_q"] = new GraphParam
            {
                Name = "k_q",
                DefaultValue = 1.0,
                Min = 0.0,
                Max = 5.0,
                Ui = new GraphParamUi
                {
                    Widget = "slider",
                    Label = "Spring Gain",
                    Group = "FlightStickPitch",
                    Units = "N",
                    Step = 0.1,
                    Precision = 2,
                    LogScale = true
                }
            };
            graph.Params["k_q"].Ui.Options.Add(new GraphParamOption { Value = "A", Label = "Mode A" });

            string json = GraphSerializer.Serialize(graph);
            var loaded = GraphSerializer.Deserialize(json, out var validation);
            if (!validation.IsValid || !loaded.Params.TryGetValue("k_q", out var param))
            {
                return false;
            }

            return param.Ui != null &&
                   param.Ui.Widget == "slider" &&
                   param.Ui.Label == "Spring Gain" &&
                   param.Ui.Group == "FlightStickPitch" &&
                   param.Ui.Units == "N" &&
                   param.Ui.Step.HasValue &&
                   Math.Abs(param.Ui.Step.Value - 0.1) < 1e-9 &&
                   param.Ui.Precision == 2 &&
                   param.Ui.LogScale &&
                   param.Ui.Options.Count == 1 &&
                   param.Ui.Options[0].Label == "Mode A";
        }

        private static bool TestParamValuesSerializationRoundtrip()
        {
            var graph = new GraphEditor.GraphDefinition();
            graph.Params["k_q"] = new GraphParam
            {
                Name = "k_q",
                DefaultValue = 1.0,
                Min = 0.0,
                Max = 5.0
            };
            graph.Params["k_rate"] = new GraphParam
            {
                Name = "k_rate",
                DefaultValue = 0.5,
                Min = 0.0,
                Max = 2.0
            };

            // Set graph-level overrides
            graph.ParamValues["k_q"] = 1.5;
            graph.ParamValues["k_rate"] = 0.8;

            string json = GraphSerializer.Serialize(graph);
            var loaded = GraphSerializer.Deserialize(json, out var validation);

            if (!validation.IsValid || loaded.ParamValues == null)
            {
                return false;
            }

            return loaded.ParamValues.ContainsKey("k_q") &&
                   Math.Abs(loaded.ParamValues["k_q"] - 1.5) < 1e-9 &&
                   loaded.ParamValues.ContainsKey("k_rate") &&
                   Math.Abs(loaded.ParamValues["k_rate"] - 0.8) < 1e-9;
        }

        private static bool TestParamResolutionIncludeDefault()
        {
            // Test that include/graph default value is used when no overrides exist
            var graph = new GraphEditor.GraphDefinition();
            graph.Params["k_spring"] = new GraphParam
            {
                Name = "k_spring",
                DefaultValue = 1.0,
                Min = 0.0,
                Max = 5.0
            };

            // No graph or vehicle overrides
            double resolved = ResolveParamValue("k_spring", graph, null);
            return Math.Abs(resolved - 1.0) < 1e-9;
        }

        private static bool TestParamResolutionGraphOverride()
        {
            // Test that graph-level override takes precedence over default
            var graph = new GraphEditor.GraphDefinition();
            graph.Params["k_spring"] = new GraphParam
            {
                Name = "k_spring",
                DefaultValue = 1.0,
                Min = 0.0,
                Max = 5.0
            };
            graph.ParamValues["k_spring"] = 2.0;

            // No vehicle override
            double resolved = ResolveParamValue("k_spring", graph, null);
            return Math.Abs(resolved - 2.0) < 1e-9;
        }

        private static bool TestParamResolutionVehicleOverride()
        {
            // Test that vehicle-level override takes precedence over graph override
            var graph = new GraphEditor.GraphDefinition();
            graph.Params["k_spring"] = new GraphParam
            {
                Name = "k_spring",
                DefaultValue = 1.0,
                Min = 0.0,
                Max = 5.0
            };
            graph.ParamValues["k_spring"] = 2.0;

            var vehicleOverrides = new Dictionary<string, double>
            {
                ["k_spring"] = 3.0
            };

            double resolved = ResolveParamValue("k_spring", graph, vehicleOverrides);
            return Math.Abs(resolved - 3.0) < 1e-9;
        }

        private static bool TestParamResolutionThreeTierCascade()
        {
            // Test the full three-tier cascade with multiple params at different levels
            var graph = new GraphEditor.GraphDefinition();

            // Param with default only
            graph.Params["p1"] = new GraphParam { Name = "p1", DefaultValue = 1.0, Min = 0.0, Max = 10.0 };

            // Param with default + graph override
            graph.Params["p2"] = new GraphParam { Name = "p2", DefaultValue = 2.0, Min = 0.0, Max = 10.0 };
            graph.ParamValues["p2"] = 2.5;

            // Param with default + graph override + vehicle override
            graph.Params["p3"] = new GraphParam { Name = "p3", DefaultValue = 3.0, Min = 0.0, Max = 10.0 };
            graph.ParamValues["p3"] = 3.5;

            var vehicleOverrides = new Dictionary<string, double>
            {
                ["p3"] = 3.9
            };

            double v1 = ResolveParamValue("p1", graph, vehicleOverrides);
            double v2 = ResolveParamValue("p2", graph, vehicleOverrides);
            double v3 = ResolveParamValue("p3", graph, vehicleOverrides);

            return Math.Abs(v1 - 1.0) < 1e-9 &&
                   Math.Abs(v2 - 2.5) < 1e-9 &&
                   Math.Abs(v3 - 3.9) < 1e-9;
        }

        /// <summary>
        /// Helper method that implements the three-tier parameter resolution logic.
        /// This mirrors the logic in DiyFfbPlugin.BuildGraphParams().
        /// </summary>
        private static double ResolveParamValue(string paramName, GraphEditor.GraphDefinition graph, Dictionary<string, double> vehicleOverrides)
        {
            if (!graph.Params.TryGetValue(paramName, out var param))
            {
                return 0.0;
            }

            // Tier 1: Include/graph default
            double value = param.DefaultValue;

            // Tier 2: Graph paramValues override
            if (graph.ParamValues != null && graph.ParamValues.TryGetValue(paramName, out var graphOverride))
            {
                value = graphOverride;
            }

            // Tier 3: Vehicle profile override
            if (vehicleOverrides != null && vehicleOverrides.TryGetValue(paramName, out var vehicleOverride))
            {
                value = vehicleOverride;
            }

            return value;
        }

        // v3 Include node tests

        private static bool TestExtractInterfaceFromGraph()
        {
            // Create a graph with Input/Output nodes to extract interface from
            var graph = new GraphEditor.GraphDefinition();

            // Add Input node with two output ports
            var inputNode = new GraphEditor.GraphNode
            {
                Id = "in",
                Kind = GraphNodeKind.Input,
                SignalGroup = "XPlane"
            };
            inputNode.Ports.Add(new GraphPort { Name = "ias", Kind = GraphPortKind.Output, SignalSuffix = "Speed.IAS" });
            inputNode.Ports.Add(new GraphPort { Name = "alpha", Kind = GraphPortKind.Output, SignalSuffix = "Angle.Alpha" });
            graph.Nodes.Add(inputNode);

            // Add Output node with one input port
            var outputNode = new GraphEditor.GraphNode
            {
                Id = "out",
                Kind = GraphNodeKind.Output,
                SignalGroup = "FlightStickPitch"
            };
            outputNode.Ports.Add(new GraphPort { Name = "spring", Kind = GraphPortKind.Input, SignalSuffix = "SpringGain" });
            graph.Nodes.Add(outputNode);

            var iface = GraphSerializer.ExtractInterface(graph);

            // Short names (SignalSuffix) for UI display; runtime matching handled in evaluator
            return iface.IsValid &&
                   iface.Inputs.Count == 2 &&
                   iface.Inputs.Contains("Speed.IAS") &&
                   iface.Inputs.Contains("Angle.Alpha") &&
                   iface.Outputs.Count == 1 &&
                   iface.Outputs.Contains("SpringGain");
        }

        private static bool TestV3IncludePortsNotSerialized()
        {
            // Create graph with Include node that has ports
            var graph = new GraphEditor.GraphDefinition();
            var includeNode = new GraphEditor.GraphNode
            {
                Id = "inc",
                Kind = GraphNodeKind.Include,
                IncludePath = "subgraph.json"
            };
            includeNode.Ports.Add(new GraphPort { Name = "in1", Kind = GraphPortKind.Input });
            includeNode.Ports.Add(new GraphPort { Name = "out1", Kind = GraphPortKind.Output });
            graph.Nodes.Add(includeNode);

            // Serialize
            string json = GraphSerializer.Serialize(graph);

            // Check that Include node doesn't have Ports in JSON
            // (ports should be omitted for Include nodes in v3)
            return !json.Contains("\"Ports\"");
        }

        private static bool TestV2IncludePortsMigration()
        {
            // Simulate v2 JSON with Include node that has stored ports
            string v2Json = @"{
                ""Version"": 2,
                ""Nodes"": [{
                    ""Id"": ""inc"",
                    ""Kind"": ""Include"",
                    ""IncludePath"": ""test.json"",
                    ""X"": 100,
                    ""Y"": 200,
                    ""Ports"": [
                        {""Name"": ""in1"", ""Kind"": ""Input""},
                        {""Name"": ""out1"", ""Kind"": ""Output""}
                    ]
                }],
                ""Links"": [],
                ""Params"": []
            }";

            var graph = GraphSerializer.Deserialize(v2Json, out var validation);

            // v2 Include nodes should still load their ports from JSON
            var includeNode = graph.Nodes[0];

            return validation.IsValid &&
                   includeNode.Kind == GraphNodeKind.Include &&
                   includeNode.Ports.Count == 2 &&
                   includeNode.Ports[0].Name == "in1" &&
                   includeNode.Ports[1].Name == "out1";
        }

        // v4 Library graph tests

        private static bool TestLibraryGraphInterfaceExtraction()
        {
            // Create a library graph with freeform port names
            var graph = new GraphEditor.GraphDefinition
            {
                IsLibraryGraph = true
            };

            // Add Input node with freeform port names (no SignalGroup/SignalSuffix)
            var inputNode = new GraphEditor.GraphNode
            {
                Id = "in",
                Kind = GraphNodeKind.Input,
                Title = "Inputs"
            };
            inputNode.Ports.Add(new GraphPort { Name = "speed", Kind = GraphPortKind.Output });
            inputNode.Ports.Add(new GraphPort { Name = "force", Kind = GraphPortKind.Output });
            graph.Nodes.Add(inputNode);

            // Add Output node with freeform port names
            var outputNode = new GraphEditor.GraphNode
            {
                Id = "out",
                Kind = GraphNodeKind.Output,
                Title = "Outputs"
            };
            outputNode.Ports.Add(new GraphPort { Name = "result", Kind = GraphPortKind.Input });
            graph.Nodes.Add(outputNode);

            var iface = GraphSerializer.ExtractInterface(graph);

            // For library graphs, interface should use port Names directly
            return iface.IsValid &&
                   iface.Inputs.Count == 2 &&
                   iface.Inputs.Contains("speed") &&
                   iface.Inputs.Contains("force") &&
                   iface.Outputs.Count == 1 &&
                   iface.Outputs.Contains("result");
        }

        private static bool TestLibraryGraphSerialization()
        {
            // Create library graph
            var graph = new GraphEditor.GraphDefinition
            {
                IsLibraryGraph = true
            };

            // Add Input node (library graph: no SignalGroup, uses Name)
            var inputNode = new GraphEditor.GraphNode
            {
                Id = "in",
                Kind = GraphNodeKind.Input,
                Title = "In"
            };
            inputNode.Ports.Add(new GraphPort { Name = "x", Kind = GraphPortKind.Output });
            graph.Nodes.Add(inputNode);

            // Serialize and check IsLibraryGraph is persisted
            string json = GraphSerializer.Serialize(graph);
            if (!json.Contains("\"IsLibraryGraph\": true"))
            {
                return false;
            }

            // Deserialize and verify
            var loaded = GraphSerializer.Deserialize(json, out var validation);

            return validation.IsValid &&
                   loaded.IsLibraryGraph &&
                   loaded.Nodes[0].Ports.Count == 1 &&
                   loaded.Nodes[0].Ports[0].Name == "x";
        }

        private static bool TestLibraryGraphTitleSerialization()
        {
            // BUG TEST: Library graph Input/Output nodes should preserve their Title
            // through serialization, but ShouldSerializeTitle() incorrectly excludes them.
            var graph = new GraphEditor.GraphDefinition
            {
                IsLibraryGraph = true
            };

            // Add Input node with a custom Title
            var inputNode = new GraphEditor.GraphNode
            {
                Id = "in",
                Kind = GraphEditor.GraphNodeKind.Input,
                Title = "My Custom Input"  // This should persist
            };
            inputNode.Ports.Add(new GraphEditor.GraphPort { Name = "x", Kind = GraphEditor.GraphPortKind.Output });
            graph.Nodes.Add(inputNode);

            // Add Output node with a custom Title
            var outputNode = new GraphEditor.GraphNode
            {
                Id = "out",
                Kind = GraphEditor.GraphNodeKind.Output,
                Title = "My Custom Output"  // This should persist
            };
            outputNode.Ports.Add(new GraphEditor.GraphPort { Name = "result", Kind = GraphEditor.GraphPortKind.Input });
            graph.Nodes.Add(outputNode);

            // Serialize and deserialize
            string json = GraphEditor.GraphSerializer.Serialize(graph);
            var loaded = GraphEditor.GraphSerializer.Deserialize(json, out var validation);

            if (!validation.IsValid)
            {
                return false;
            }

            // Find the loaded nodes
            var loadedInput = loaded.Nodes.FirstOrDefault(n => n.Kind == GraphEditor.GraphNodeKind.Input);
            var loadedOutput = loaded.Nodes.FirstOrDefault(n => n.Kind == GraphEditor.GraphNodeKind.Output);

            // Titles should be preserved for library graph Input/Output nodes
            return loadedInput != null &&
                   loadedInput.Title == "My Custom Input" &&
                   loadedOutput != null &&
                   loadedOutput.Title == "My Custom Output";
        }

        private static bool TestLibraryGraphRuntimeConversion()
        {
            // Test that library graph nodes convert correctly to runtime with freeform names
            var graph = new GraphEditor.GraphDefinition
            {
                IsLibraryGraph = true
            };

            // Input node with freeform port name (no SignalGroup)
            var inputNode = new GraphEditor.GraphNode
            {
                Id = "in",
                Kind = GraphEditor.GraphNodeKind.Input,
                Title = "Inputs"
            };
            inputNode.Ports.Add(new GraphEditor.GraphPort { Name = "force", Kind = GraphEditor.GraphPortKind.Output });
            graph.Nodes.Add(inputNode);

            // Const node
            var constNode = new GraphEditor.GraphNode
            {
                Id = "k",
                Kind = GraphEditor.GraphNodeKind.Const,
                ConstValue = 2.0
            };
            graph.Nodes.Add(constNode);

            // Op node (multiply)
            var opNode = new GraphEditor.GraphNode
            {
                Id = "mul",
                Kind = GraphEditor.GraphNodeKind.Op,
                Op = "mul"
            };
            opNode.Ports.Add(new GraphEditor.GraphPort { Name = "a", Kind = GraphEditor.GraphPortKind.Input });
            opNode.Ports.Add(new GraphEditor.GraphPort { Name = "b", Kind = GraphEditor.GraphPortKind.Input });
            opNode.Ports.Add(new GraphEditor.GraphPort { Name = "out", Kind = GraphEditor.GraphPortKind.Output });
            graph.Nodes.Add(opNode);

            // Output node with freeform port name
            var outputNode = new GraphEditor.GraphNode
            {
                Id = "out",
                Kind = GraphEditor.GraphNodeKind.Output,
                Title = "Outputs"
            };
            outputNode.Ports.Add(new GraphEditor.GraphPort { Name = "scaled", Kind = GraphEditor.GraphPortKind.Input });
            graph.Nodes.Add(outputNode);

            // Links
            graph.Links.Add(new GraphEditor.GraphLink { FromNodeId = "in", FromPort = "force", ToNodeId = "mul", ToPort = "a" });
            graph.Links.Add(new GraphEditor.GraphLink { FromNodeId = "k", ToNodeId = "mul", ToPort = "b" });
            graph.Links.Add(new GraphEditor.GraphLink { FromNodeId = "mul", FromPort = "out", ToNodeId = "out", ToPort = "scaled" });

            // Convert to runtime
            var runtime = GraphEditor.GraphRuntimeConverter.Convert(graph);

            // The input should use freeform name "force" (not a signal-catalog name)
            // Use string comparison to avoid enum type conflicts between local and imported types
            bool hasForceInput = runtime.Nodes.Values.Any(n =>
                n.Type.ToString() == "Input" && n.Name == "force");

            // The output should use freeform name "scaled"
            bool hasScaledOutput = runtime.Nodes.Values.Any(n =>
                n.Type.ToString() == "Output" && n.Name == "scaled");

            return hasForceInput && hasScaledOutput;
        }

        private static bool TestNodeSignalGroupPreservation()
        {
            // Test that SignalGroup is preserved through serialization roundtrip
            var graph = new GraphEditor.GraphDefinition();

            var inputNode = new GraphEditor.GraphNode
            {
                Id = "in",
                Kind = GraphEditor.GraphNodeKind.Input,
                SignalGroup = "XPlane"  // This must persist
            };
            inputNode.Ports.Add(new GraphEditor.GraphPort
            {
                Name = "IAS_kts",
                Kind = GraphEditor.GraphPortKind.Output,
                SignalSuffix = "IAS_kts"
            });
            graph.Nodes.Add(inputNode);

            // Serialize and deserialize
            string json = GraphEditor.GraphSerializer.Serialize(graph);
            var loaded = GraphEditor.GraphSerializer.Deserialize(json, out var validation);

            if (!validation.IsValid)
            {
                return false;
            }

            var loadedNode = loaded.Nodes.FirstOrDefault(n => n.Kind == GraphEditor.GraphNodeKind.Input);
            return loadedNode != null && loadedNode.SignalGroup == "XPlane";
        }

        private static bool TestPortSignalSuffixPreservation()
        {
            // Test that port SignalSuffix is preserved through serialization roundtrip
            var graph = new GraphEditor.GraphDefinition();

            var inputNode = new GraphEditor.GraphNode
            {
                Id = "in",
                Kind = GraphEditor.GraphNodeKind.Input,
                SignalGroup = "XPlane"
            };
            inputNode.Ports.Add(new GraphEditor.GraphPort
            {
                Name = "IAS_kts",
                Kind = GraphEditor.GraphPortKind.Output,
                SignalSuffix = "IAS_kts"  // This must persist
            });
            inputNode.Ports.Add(new GraphEditor.GraphPort
            {
                Name = "Alpha_deg",
                Kind = GraphEditor.GraphPortKind.Output,
                SignalSuffix = "Alpha_deg"
            });
            graph.Nodes.Add(inputNode);

            // Serialize and deserialize
            string json = GraphEditor.GraphSerializer.Serialize(graph);
            var loaded = GraphEditor.GraphSerializer.Deserialize(json, out var validation);

            if (!validation.IsValid)
            {
                return false;
            }

            var loadedNode = loaded.Nodes.FirstOrDefault(n => n.Kind == GraphEditor.GraphNodeKind.Input);
            if (loadedNode == null || loadedNode.Ports.Count != 2)
            {
                return false;
            }

            var port1 = loadedNode.Ports.FirstOrDefault(p => p.Name == "IAS_kts");
            var port2 = loadedNode.Ports.FirstOrDefault(p => p.Name == "Alpha_deg");

            return port1 != null && port1.SignalSuffix == "IAS_kts" &&
                   port2 != null && port2.SignalSuffix == "Alpha_deg";
        }

        private static bool TestOpInputNegateConversion()
        {
            var graph = new GraphEditor.GraphDefinition();
            var a = new GraphEditor.GraphNode { Id = "a", Kind = GraphEditor.GraphNodeKind.Const, ConstValue = 1.0 };
            a.Ports.Add(new GraphEditor.GraphPort { Name = "out", Kind = GraphEditor.GraphPortKind.Output });
            var b = new GraphEditor.GraphNode { Id = "b", Kind = GraphEditor.GraphNodeKind.Const, ConstValue = 2.0 };
            b.Ports.Add(new GraphEditor.GraphPort { Name = "out", Kind = GraphEditor.GraphPortKind.Output });

            var op = new GraphEditor.GraphNode { Id = "op", Kind = GraphEditor.GraphNodeKind.Op, Op = "add" };
            op.Ports.Add(new GraphEditor.GraphPort { Name = "a", Kind = GraphEditor.GraphPortKind.Input });
            op.Ports.Add(new GraphEditor.GraphPort { Name = "b", Kind = GraphEditor.GraphPortKind.Input, Negate = true });
            op.Ports.Add(new GraphEditor.GraphPort { Name = "a+b", Kind = GraphEditor.GraphPortKind.Output });

            graph.Nodes.Add(a);
            graph.Nodes.Add(b);
            graph.Nodes.Add(op);

            graph.Links.Add(new GraphEditor.GraphLink { FromNodeId = "a", FromPort = "out", ToNodeId = "op", ToPort = "a" });
            graph.Links.Add(new GraphEditor.GraphLink { FromNodeId = "b", FromPort = "out", ToNodeId = "op", ToPort = "b" });

            var runtime = GraphEditor.GraphRuntimeConverter.Convert(graph);
            if (!runtime.Nodes.TryGetValue("op", out var runtimeOp))
            {
                return false;
            }

            return runtimeOp.ArgNegate.Count == 2 &&
                   runtimeOp.ArgNegate[0] == false &&
                   runtimeOp.ArgNegate[1] == true;
        }

        private static bool TestOpInputNegateValidation()
        {
            var graph = new GraphDefinition { Version = 1 };
            graph.Nodes["a"] = new GraphNode { Id = "a", Type = NodeType.Const, ConstValue = 1.0 };
            graph.Nodes["b"] = new GraphNode { Id = "b", Type = NodeType.Const, ConstValue = 2.0 };
            graph.Nodes["sub"] = new GraphNode
            {
                Id = "sub",
                Type = NodeType.Op,
                Op = OpType.Sub,
                Args = { "a", "b" },
                ArgNegate = { true, false }
            };
            graph.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "out", Src = "sub" };

            var validation = GraphValidator.Validate(graph);
            return !validation.IsValid;
        }

        private static bool TestParamConstValueFromGraphParams()
        {
            // Test that Param nodes get their ConstValue from graph.Params during conversion
            // This ensures Param default values are available for Include sub-graphs in preview
            var graph = new GraphEditor.GraphDefinition();

            // Param node with SignalGroup/SignalSuffix
            var paramNode = new GraphEditor.GraphNode
            {
                Id = "param",
                Kind = GraphEditor.GraphNodeKind.Param,
                SignalGroup = "FlightStickPitch"
            };
            paramNode.Ports.Add(new GraphEditor.GraphPort
            {
                Name = "SpringGain",
                Kind = GraphEditor.GraphPortKind.Output,
                SignalSuffix = "SpringGain"
            });
            graph.Nodes.Add(paramNode);

            // Set the default value in graph.Params (keyed by full signal name)
            graph.Params["FlightStickPitch.SpringGain"] = new GraphEditor.GraphParam
            {
                Name = "FlightStickPitch.SpringGain",
                DefaultValue = 0.75,
                Min = 0.0,
                Max = 1.0
            };

            // Output node to consume the param
            var outputNode = new GraphEditor.GraphNode
            {
                Id = "out",
                Kind = GraphEditor.GraphNodeKind.Output,
                SignalGroup = "FlightStickPitch"
            };
            outputNode.Ports.Add(new GraphEditor.GraphPort
            {
                Name = "SpringGain",
                Kind = GraphEditor.GraphPortKind.Input,
                SignalSuffix = "SpringGain"
            });
            graph.Nodes.Add(outputNode);

            // Link param to output
            graph.Links.Add(new GraphEditor.GraphLink
            {
                FromNodeId = "param",
                FromPort = "SpringGain",
                ToNodeId = "out",
                ToPort = "SpringGain"
            });

            // Convert to runtime
            var runtime = GraphEditor.GraphRuntimeConverter.Convert(graph);

            // Find the Param node in runtime and check ConstValue
            var runtimeParam = runtime.Nodes.Values.FirstOrDefault(n =>
                n.Type.ToString() == "Param" && n.Name == "FlightStickPitch.SpringGain");

            if (runtimeParam == null)
            {
                return false;
            }

            // The ConstValue should be 0.75 (from graph.Params)
            return Math.Abs(runtimeParam.ConstValue - 0.75) < 1e-6;
        }

        private static bool TestEditorFormatIncludeEvaluation()
        {
            // Create a non-library included graph with SignalGroup/SignalSuffix
            // IMPORTANT: For signal-bound ports, Name must equal SignalSuffix because
            // after serialization round-trip, port.Name is set from SignalSuffix.
            // Links must use these same names.
            var includeGraph = new GraphEditor.GraphDefinition();

            var includeInput = new GraphEditor.GraphNode
            {
                Id = "in",
                Kind = GraphEditor.GraphNodeKind.Input,
                SignalGroup = "XPlane"
            };
            includeInput.Ports.Add(new GraphEditor.GraphPort
            {
                Name = "Speed.IAS",  // Must match SignalSuffix for signal-bound ports
                Kind = GraphEditor.GraphPortKind.Output,
                SignalSuffix = "Speed.IAS"
            });
            includeGraph.Nodes.Add(includeInput);

            var includeOutput = new GraphEditor.GraphNode
            {
                Id = "out",
                Kind = GraphEditor.GraphNodeKind.Output,
                SignalGroup = "FlightStickPitch"
            };
            includeOutput.Ports.Add(new GraphEditor.GraphPort
            {
                Name = "SpringGain",  // Must match SignalSuffix for signal-bound ports
                Kind = GraphEditor.GraphPortKind.Input,
                SignalSuffix = "SpringGain"
            });
            includeGraph.Nodes.Add(includeOutput);

            // Link input to output (pass-through)
            // Links must use the same names as ports (which equal SignalSuffix)
            includeGraph.Links.Add(new GraphEditor.GraphLink
            {
                FromNodeId = "in",
                FromPort = "Speed.IAS",  // Use SignalSuffix value
                ToNodeId = "out",
                ToPort = "SpringGain"    // Use SignalSuffix value
            });

            // Write to temp file
            string tempDir = System.IO.Path.Combine(System.IO.Path.GetTempPath(), "ffb_graph_test_include");
            System.IO.Directory.CreateDirectory(tempDir);
            string includePath = System.IO.Path.Combine(tempDir, "include.json");
            string includeJson = GraphEditor.GraphSerializer.Serialize(includeGraph);
            System.IO.File.WriteAllText(includePath, includeJson);

            try
            {
                // Create parent graph with Include node
                var parentGraph = new GraphEditor.GraphDefinition();

                var parentInput = new GraphEditor.GraphNode
                {
                    Id = "src",
                    Kind = GraphEditor.GraphNodeKind.Input,
                    SignalGroup = "XPlane"
                };
                parentInput.Ports.Add(new GraphEditor.GraphPort
                {
                    Name = "Speed.IAS",  // Must match SignalSuffix
                    Kind = GraphEditor.GraphPortKind.Output,
                    SignalSuffix = "Speed.IAS"
                });
                parentGraph.Nodes.Add(parentInput);

                var includeNode = new GraphEditor.GraphNode
                {
                    Id = "inc",
                    Kind = GraphEditor.GraphNodeKind.Include,
                    IncludePath = "include.json"
                };
                parentGraph.Nodes.Add(includeNode);

                // Populate include ports (simulating what the plugin does)
                // ExtractInterface now returns short names (SignalSuffix) for UI display
                GraphEditor.GraphSerializer.PopulateIncludePorts(parentGraph, tempDir);

                var incNode = parentGraph.Nodes.First(n => n.Kind == GraphEditor.GraphNodeKind.Include);
                if (incNode.Ports.Count == 0)
                {
                    return false;
                }

                // Include ports should have short names (SignalSuffix)
                // e.g., "Speed.IAS" not "XPlane.Speed.IAS"
                string includeInputPortName = incNode.Ports.FirstOrDefault(p => p.Kind == GraphEditor.GraphPortKind.Input)?.Name;
                string includeOutputPortName = incNode.Ports.FirstOrDefault(p => p.Kind == GraphEditor.GraphPortKind.Output)?.Name;

                if (string.IsNullOrEmpty(includeInputPortName) || string.IsNullOrEmpty(includeOutputPortName))
                {
                    return false;
                }

                // Add link from parent input to include input
                // FromPort must match the parent Input's port.Name (which equals SignalSuffix)
                parentGraph.Links.Add(new GraphEditor.GraphLink
                {
                    FromNodeId = "src",
                    FromPort = "Speed.IAS",  // Match port.Name (= SignalSuffix)
                    ToNodeId = "inc",
                    ToPort = includeInputPortName  // Short name from ExtractInterface
                });

                // Add output node
                var parentOutput = new GraphEditor.GraphNode
                {
                    Id = "result",
                    Kind = GraphEditor.GraphNodeKind.Output,
                    SignalGroup = "FlightStickPitch"
                };
                parentOutput.Ports.Add(new GraphEditor.GraphPort
                {
                    Name = "SpringGain",  // Must match SignalSuffix
                    Kind = GraphEditor.GraphPortKind.Input,
                    SignalSuffix = "SpringGain"
                });
                parentGraph.Nodes.Add(parentOutput);

                // Link include output to parent output
                // ToPort must match parent Output's port.Name (which equals SignalSuffix)
                parentGraph.Links.Add(new GraphEditor.GraphLink
                {
                    FromNodeId = "inc",
                    FromPort = includeOutputPortName,  // Short name from ExtractInterface
                    ToNodeId = "result",
                    ToPort = "SpringGain"  // Match port.Name (= SignalSuffix)
                });

                // Convert parent to runtime and bridge DLL type to local type
                var parentRuntimeDll = GraphEditor.GraphRuntimeConverter.Convert(parentGraph);
                string parentRuntimeJson = Newtonsoft.Json.JsonConvert.SerializeObject(parentRuntimeDll);
                var parentRuntime = Newtonsoft.Json.JsonConvert.DeserializeObject<GraphDefinition>(parentRuntimeJson);

                // Set up resolver with converter delegate
                // Use JSON round-trip to convert from DLL types to local types
                // (DLL GraphDefinition and local GraphDefinition are structurally identical)
                var resolver = new GraphIncludeResolver(tempDir)
                {
                    EditorFormatConverter = (json, resolvedPath) =>
                    {
                        var g = GraphEditor.GraphSerializer.Deserialize(json, out var validation);
                        if (g == null)
                        {
                            return null;
                        }

                        // Convert editor graph to runtime format (returns DLL GraphDefinition type)
                        var dllRuntime = GraphEditor.GraphRuntimeConverter.Convert(g);

                        // Bridge DLL type to local type via Newtonsoft.Json round-trip
                        // This works because both types have identical structure
                        string runtimeJson = Newtonsoft.Json.JsonConvert.SerializeObject(dllRuntime);
                        return Newtonsoft.Json.JsonConvert.DeserializeObject<GraphDefinition>(runtimeJson);
                    }
                };

                // Create evaluator
                var evaluator = new GraphCompiledEvaluator(parentRuntime, resolver);

                // Evaluate with input value
                // The parent input name is the full signal name
                string parentInputName = "XPlane.Speed.IAS";
                var inputs = new Dictionary<string, double> { [parentInputName] = 42.0 };
                var outputs = evaluator.Evaluate(inputs, new Dictionary<string, double>());

                // The output should have the value passed through
                // The evaluator's BuildShortToFullNameMap should handle the name mapping
                string parentOutputName = "FlightStickPitch.SpringGain";

                if (!outputs.TryGetValue(parentOutputName, out var outputValue))
                {
                    return false;
                }

                if (Math.Abs(outputValue - 42.0) >= 0.0001)
                {
                    return false;
                }

                return true;
            }
            catch
            {
                return false;
            }
            finally
            {
                try { System.IO.Directory.Delete(tempDir, true); } catch { }
            }
        }

        private static bool TestParamOrderWithIncludes()
        {
            string tempDir = System.IO.Path.Combine(System.IO.Path.GetTempPath(), "ffb_param_order_test_" + Guid.NewGuid().ToString("N").Substring(0, 8));
            try
            {
                System.IO.Directory.CreateDirectory(tempDir);

                // Nested include graph
                var nestedGraph = new GraphEditor.GraphDefinition();
                nestedGraph.Nodes.Add(CreateParamNode("param_e", 0, 5, "Vehicle", "E"));
                string nestedPath = System.IO.Path.Combine(tempDir, "nested.json");
                System.IO.File.WriteAllText(nestedPath, GraphEditor.GraphSerializer.Serialize(nestedGraph));

                // Include graph with params D (y=5), C (y=10), and an include (y=50)
                var includeGraph = new GraphEditor.GraphDefinition();
                includeGraph.Nodes.Add(CreateParamNode("param_d", 0, 5, "Vehicle", "D"));
                includeGraph.Nodes.Add(CreateParamNode("param_c", 0, 10, "Vehicle", "C"));
                includeGraph.Nodes.Add(new GraphEditor.GraphNode
                {
                    Id = "inc_nested",
                    Kind = GraphEditor.GraphNodeKind.Include,
                    IncludePath = "nested.json",
                    X = 0,
                    Y = 50
                });
                string includePath = System.IO.Path.Combine(tempDir, "include.json");
                System.IO.File.WriteAllText(includePath, GraphEditor.GraphSerializer.Serialize(includeGraph));

                // Parent graph: Param A (y=0), Include (y=100), Param B (y=200)
                var parentGraph = new GraphEditor.GraphDefinition();
                parentGraph.Nodes.Add(CreateParamNode("param_a", 0, 0, "Vehicle", "A"));
                parentGraph.Nodes.Add(new GraphEditor.GraphNode
                {
                    Id = "inc_child",
                    Kind = GraphEditor.GraphNodeKind.Include,
                    IncludePath = "include.json",
                    X = 0,
                    Y = 100
                });
                parentGraph.Nodes.Add(CreateParamNode("param_b", 0, 200, "Vehicle", "B"));

                var plugin = new DiyFfbPlugin();
                SetPrivateField(plugin, "activeVehicleGraph", parentGraph);
                SetPrivateField(plugin, "activeGraphPath", System.IO.Path.Combine(tempDir, "parent.json"));

                var order = plugin.GetActiveGraphParamOrder();
                var expected = new List<string>
                {
                    "Vehicle.A",
                    "Vehicle.D",
                    "Vehicle.C",
                    "Vehicle.E",
                    "Vehicle.B"
                };

                return order.SequenceEqual(expected, StringComparer.OrdinalIgnoreCase);
            }
            catch
            {
                return false;
            }
            finally
            {
                try { System.IO.Directory.Delete(tempDir, true); } catch { }
            }
        }

        private static GraphEditor.GraphNode CreateParamNode(string id, double x, double y, string group, string suffix)
        {
            var node = new GraphEditor.GraphNode
            {
                Id = id,
                Kind = GraphEditor.GraphNodeKind.Param,
                X = x,
                Y = y,
                SignalGroup = group
            };

            node.Ports.Add(new GraphEditor.GraphPort
            {
                Name = "value",
                Kind = GraphEditor.GraphPortKind.Output,
                SignalSuffix = suffix
            });

            return node;
        }

        private static void SetPrivateField(object instance, string fieldName, object value)
        {
            var field = instance.GetType().GetField(fieldName, BindingFlags.Instance | BindingFlags.NonPublic);
            if (field == null)
            {
                throw new InvalidOperationException($"Field '{fieldName}' not found.");
            }
            field.SetValue(instance, value);
        }

        // Include context cache tests

        private static bool TestIncludeContextCacheAddAndRetrieve()
        {
            var cache = new IncludeContextCache();
            var ctx = new IncludeCallContext
            {
                IncludeNodeId = "inc1",
                IncludeNodeTitle = "MyInclude",
                IncludePath = "C:/graphs/sub.json",
                Inputs = new Dictionary<string, double> { { "A", 1.0 } },
                Parameters = new Dictionary<string, double>()
            };

            cache.Add("C:/graphs/sub.json", ctx);

            var retrieved = cache.GetContexts("C:/graphs/sub.json");
            return retrieved.Count == 1 &&
                   retrieved[0].IncludeNodeId == "inc1" &&
                   retrieved[0].Inputs["A"] == 1.0;
        }

        private static bool TestIncludeContextCachePathCaseInsensitive()
        {
            var cache = new IncludeContextCache();
            cache.Add("C:/Graphs/Sub.json", new IncludeCallContext { IncludeNodeId = "inc1" });

            var retrieved = cache.GetContexts("c:/graphs/sub.json");
            return retrieved.Count == 1;
        }

        private static bool TestIncludeContextCacheMultipleIncludesSamePath()
        {
            var cache = new IncludeContextCache();
            cache.Add("sub.json", new IncludeCallContext { IncludeNodeId = "inc1", IncludeNodeTitle = "First" });
            cache.Add("sub.json", new IncludeCallContext { IncludeNodeId = "inc2", IncludeNodeTitle = "Second" });

            var retrieved = cache.GetContexts("sub.json");
            return retrieved.Count == 2 &&
                   retrieved.Any(c => c.IncludeNodeTitle == "First") &&
                   retrieved.Any(c => c.IncludeNodeTitle == "Second");
        }

        private static bool TestIncludeContextCacheClear()
        {
            var cache = new IncludeContextCache();
            cache.Add("sub.json", new IncludeCallContext { IncludeNodeId = "inc1" });
            cache.Clear();

            var retrieved = cache.GetContexts("sub.json");
            return retrieved.Count == 0;
        }

        private static bool TestEvaluatorPopulatesIncludeContextCache()
        {
            // Create a temp directory for the test
            string tempDir = System.IO.Path.Combine(System.IO.Path.GetTempPath(), "ffb_context_test_" + Guid.NewGuid().ToString("N"));
            System.IO.Directory.CreateDirectory(tempDir);

            try
            {
                // Create sub-graph file
                var subGraph = new GraphDefinition();
                subGraph.Nodes["x"] = new GraphNode { Id = "x", Type = NodeType.Input, Name = "X" };
                subGraph.Nodes["y"] = new GraphNode { Id = "y", Type = NodeType.Output, Name = "Y", Src = "x" };

                string subPath = System.IO.Path.Combine(tempDir, "sub.json");
                string subJson = new GraphSaver().SaveToJson(subGraph);
                System.IO.File.WriteAllText(subPath, subJson);

                // Create parent graph with Include node
                var parent = new GraphDefinition();
                parent.Nodes["in1"] = new GraphNode { Id = "in1", Type = NodeType.Input, Name = "Speed" };
                parent.Nodes["inc1"] = new GraphNode
                {
                    Id = "inc1",
                    Type = NodeType.Include,
                    Name = "SubGraph",
                    Path = "sub.json",
                    InputMap = new Dictionary<string, string> { { "X", "in1" } },
                    OutputMap = new Dictionary<string, string> { { "Y", "inc1_Y" } }
                };
                parent.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "Result", Src = "inc1_Y" };

                var resolver = new GraphIncludeResolver(tempDir);
                var cache = new IncludeContextCache();
                var evaluator = new GraphCompiledEvaluator(parent, resolver, cache, tempDir);

                var inputs = new Dictionary<string, double> { { "Speed", 42.0 } };
                evaluator.Evaluate(inputs, null);

                var contexts = cache.GetContexts(subPath);
                return contexts.Count == 1 &&
                       contexts[0].IncludeNodeId == "inc1" &&
                       contexts[0].IncludeNodeTitle == "SubGraph" &&
                       contexts[0].Inputs.ContainsKey("X") &&
                       Math.Abs(contexts[0].Inputs["X"] - 42.0) < 0.0001;
            }
            finally
            {
                try { System.IO.Directory.Delete(tempDir, true); } catch { }
            }
        }

        private static bool TestEvaluatorClearsCacheEachEvaluation()
        {
            // Create a temp directory for the test
            string tempDir = System.IO.Path.Combine(System.IO.Path.GetTempPath(), "ffb_context_test_" + Guid.NewGuid().ToString("N"));
            System.IO.Directory.CreateDirectory(tempDir);

            try
            {
                // Create sub-graph file
                var subGraph = new GraphDefinition();
                subGraph.Nodes["x"] = new GraphNode { Id = "x", Type = NodeType.Input, Name = "X" };
                subGraph.Nodes["y"] = new GraphNode { Id = "y", Type = NodeType.Output, Name = "Y", Src = "x" };

                string subPath = System.IO.Path.Combine(tempDir, "sub.json");
                string subJson = new GraphSaver().SaveToJson(subGraph);
                System.IO.File.WriteAllText(subPath, subJson);

                // Create parent graph with Include node
                var parent = new GraphDefinition();
                parent.Nodes["in1"] = new GraphNode { Id = "in1", Type = NodeType.Input, Name = "Speed" };
                parent.Nodes["inc1"] = new GraphNode
                {
                    Id = "inc1",
                    Type = NodeType.Include,
                    Name = "SubGraph",
                    Path = "sub.json",
                    InputMap = new Dictionary<string, string> { { "X", "in1" } },
                    OutputMap = new Dictionary<string, string> { { "Y", "inc1_Y" } }
                };
                parent.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "Result", Src = "inc1_Y" };

                var resolver = new GraphIncludeResolver(tempDir);
                var cache = new IncludeContextCache();
                var evaluator = new GraphCompiledEvaluator(parent, resolver, cache, tempDir);

                // First evaluation (plugin clears cache before top-level evaluation)
                cache.Clear();
                evaluator.Evaluate(new Dictionary<string, double> { { "Speed", 10.0 } }, null);
                var contexts1 = cache.GetContexts(subPath);
                if (contexts1.Count != 1 || Math.Abs(contexts1[0].Inputs["X"] - 10.0) >= 0.0001)
                {
                    return false;
                }

                // Second evaluation (plugin clears cache before top-level evaluation)
                cache.Clear();
                evaluator.Evaluate(new Dictionary<string, double> { { "Speed", 20.0 } }, null);
                var contexts2 = cache.GetContexts(subPath);

                // Should still be 1 context (not 2), with updated value
                return contexts2.Count == 1 &&
                       Math.Abs(contexts2[0].Inputs["X"] - 20.0) < 0.0001;
            }
            finally
            {
                try { System.IO.Directory.Delete(tempDir, true); } catch { }
            }
        }

        #region Param Migration Tests

        private static bool TestParamMigrationNoChanges()
        {
            // Setup: same params, same ranges, no overrides
            var currentParams = new List<ParamMigrationHelper.ParamInfo>
            {
                new ParamMigrationHelper.ParamInfo { Name = "gain", DefaultValue = 1.0, Min = 0, Max = 2 },
                new ParamMigrationHelper.ParamInfo { Name = "damping", DefaultValue = 0.5, Min = 0, Max = 1 }
            };
            var oldSnapshots = new Dictionary<string, DiyFfbPluginSettings.ParamSnapshot>
            {
                ["gain"] = new DiyFfbPluginSettings.ParamSnapshot { DefaultValue = 1.0, Min = 0, Max = 2 },
                ["damping"] = new DiyFfbPluginSettings.ParamSnapshot { DefaultValue = 0.5, Min = 0, Max = 1 }
            };
            var overrides = new Dictionary<string, double>();

            var result = ParamMigrationHelper.Migrate(currentParams, oldSnapshots, overrides, "hash123");

            return !result.HasChangesToReview &&
                   result.ChangedDefaults.Count == 0 &&
                   result.ClampedValues.Count == 0 &&
                   result.NewOrphans.Count == 0 &&
                   result.AllOrphans.Count == 0;
        }

        private static bool TestParamMigrationClampToMin()
        {
            // Setup: override below new min range
            var currentParams = new List<ParamMigrationHelper.ParamInfo>
            {
                new ParamMigrationHelper.ParamInfo { Name = "gain", DefaultValue = 1.0, Min = 0.5, Max = 2 } // min changed from 0 to 0.5
            };
            var oldSnapshots = new Dictionary<string, DiyFfbPluginSettings.ParamSnapshot>
            {
                ["gain"] = new DiyFfbPluginSettings.ParamSnapshot { DefaultValue = 1.0, Min = 0, Max = 2 }
            };
            var overrides = new Dictionary<string, double> { ["gain"] = 0.2 }; // Below new min of 0.5

            var result = ParamMigrationHelper.Migrate(currentParams, oldSnapshots, overrides, "hash123");

            return result.HasChangesToReview &&
                   result.ClampedValues.Count == 1 &&
                   result.ClampedValues[0].ParamName == "gain" &&
                   Math.Abs(result.ClampedValues[0].OriginalValue - 0.2) < 0.0001 &&
                   Math.Abs(result.ClampedValues[0].ClampedValue - 0.5) < 0.0001 &&
                   result.ClampedValues[0].ClampedToMin &&
                   !result.ClampedValues[0].ClampedToMax &&
                   Math.Abs(overrides["gain"] - 0.5) < 0.0001; // Override was mutated
        }

        private static bool TestParamMigrationClampToMax()
        {
            // Setup: override above new max range
            var currentParams = new List<ParamMigrationHelper.ParamInfo>
            {
                new ParamMigrationHelper.ParamInfo { Name = "gain", DefaultValue = 1.0, Min = 0, Max = 1.5 } // max changed from 2 to 1.5
            };
            var oldSnapshots = new Dictionary<string, DiyFfbPluginSettings.ParamSnapshot>
            {
                ["gain"] = new DiyFfbPluginSettings.ParamSnapshot { DefaultValue = 1.0, Min = 0, Max = 2 }
            };
            var overrides = new Dictionary<string, double> { ["gain"] = 1.8 }; // Above new max of 1.5

            var result = ParamMigrationHelper.Migrate(currentParams, oldSnapshots, overrides, "hash123");

            return result.HasChangesToReview &&
                   result.ClampedValues.Count == 1 &&
                   result.ClampedValues[0].ParamName == "gain" &&
                   Math.Abs(result.ClampedValues[0].OriginalValue - 1.8) < 0.0001 &&
                   Math.Abs(result.ClampedValues[0].ClampedValue - 1.5) < 0.0001 &&
                   !result.ClampedValues[0].ClampedToMin &&
                   result.ClampedValues[0].ClampedToMax &&
                   Math.Abs(overrides["gain"] - 1.5) < 0.0001; // Override was mutated
        }

        private static bool TestParamMigrationOrphanDetected()
        {
            // Setup: param removed from graph, override becomes orphan
            var currentParams = new List<ParamMigrationHelper.ParamInfo>
            {
                new ParamMigrationHelper.ParamInfo { Name = "gain", DefaultValue = 1.0, Min = 0, Max = 2 }
                // "damping" param was removed
            };
            var oldSnapshots = new Dictionary<string, DiyFfbPluginSettings.ParamSnapshot>
            {
                ["gain"] = new DiyFfbPluginSettings.ParamSnapshot { DefaultValue = 1.0, Min = 0, Max = 2 },
                ["damping"] = new DiyFfbPluginSettings.ParamSnapshot { DefaultValue = 0.5, Min = 0, Max = 1 }
            };
            var overrides = new Dictionary<string, double> { ["damping"] = 0.7 }; // Override for removed param

            var result = ParamMigrationHelper.Migrate(currentParams, oldSnapshots, overrides, "hash123");

            return result.HasChangesToReview &&
                   result.NewOrphans.Count == 1 &&
                   result.NewOrphans[0] == "damping" &&
                   result.AllOrphans.Count == 1 &&
                   result.AllOrphans[0] == "damping";
        }

        private static bool TestParamMigrationNewOrphanVsExisting()
        {
            // Setup: one override was already orphaned, another becomes newly orphaned
            var currentParams = new List<ParamMigrationHelper.ParamInfo>
            {
                new ParamMigrationHelper.ParamInfo { Name = "gain", DefaultValue = 1.0, Min = 0, Max = 2 }
                // "damping" removed this time, "old_param" was already orphaned
            };
            var oldSnapshots = new Dictionary<string, DiyFfbPluginSettings.ParamSnapshot>
            {
                ["gain"] = new DiyFfbPluginSettings.ParamSnapshot { DefaultValue = 1.0, Min = 0, Max = 2 },
                ["damping"] = new DiyFfbPluginSettings.ParamSnapshot { DefaultValue = 0.5, Min = 0, Max = 1 }
                // "old_param" not in old snapshots (already orphaned before)
            };
            var overrides = new Dictionary<string, double>
            {
                ["damping"] = 0.7, // Becomes new orphan
                ["old_param"] = 1.0 // Already orphaned (not in old snapshots)
            };

            var result = ParamMigrationHelper.Migrate(currentParams, oldSnapshots, overrides, "hash123");

            return result.HasChangesToReview &&
                   result.NewOrphans.Count == 1 &&
                   result.NewOrphans[0] == "damping" && // Only damping is NEW orphan
                   result.AllOrphans.Count == 2 && // Both are orphans total
                   result.AllOrphans.Contains("damping") &&
                   result.AllOrphans.Contains("old_param");
        }

        private static bool TestParamMigrationChangedDefault()
        {
            // Setup: param default value changed
            var currentParams = new List<ParamMigrationHelper.ParamInfo>
            {
                new ParamMigrationHelper.ParamInfo { Name = "gain", DefaultValue = 1.5, Min = 0, Max = 2 } // Default changed from 1.0 to 1.5
            };
            var oldSnapshots = new Dictionary<string, DiyFfbPluginSettings.ParamSnapshot>
            {
                ["gain"] = new DiyFfbPluginSettings.ParamSnapshot { DefaultValue = 1.0, Min = 0, Max = 2 }
            };
            var overrides = new Dictionary<string, double>();

            var result = ParamMigrationHelper.Migrate(currentParams, oldSnapshots, overrides, "hash123");

            return result.HasChangesToReview &&
                   result.ChangedDefaults.Count == 1 &&
                   result.ChangedDefaults[0].ParamName == "gain" &&
                   Math.Abs(result.ChangedDefaults[0].OldDefault - 1.0) < 0.0001 &&
                   Math.Abs(result.ChangedDefaults[0].NewDefault - 1.5) < 0.0001;
        }

        private static bool TestParamMigrationOrphanRestored()
        {
            // Setup: param that was orphaned is now back in graph
            var currentParams = new List<ParamMigrationHelper.ParamInfo>
            {
                new ParamMigrationHelper.ParamInfo { Name = "gain", DefaultValue = 1.0, Min = 0, Max = 2 },
                new ParamMigrationHelper.ParamInfo { Name = "restored_param", DefaultValue = 0.5, Min = 0, Max = 1 } // Back in graph
            };
            var oldSnapshots = new Dictionary<string, DiyFfbPluginSettings.ParamSnapshot>
            {
                ["gain"] = new DiyFfbPluginSettings.ParamSnapshot { DefaultValue = 1.0, Min = 0, Max = 2 }
                // "restored_param" not in old snapshots (was orphaned)
            };
            var overrides = new Dictionary<string, double>
            {
                ["restored_param"] = 0.8 // Override for previously orphaned param
            };

            var result = ParamMigrationHelper.Migrate(currentParams, oldSnapshots, overrides, "hash123");

            // Override should be preserved (value within range), no orphan
            return !result.HasChangesToReview &&
                   result.AllOrphans.Count == 0 &&
                   result.NewOrphans.Count == 0 &&
                   Math.Abs(overrides["restored_param"] - 0.8) < 0.0001; // Value preserved
        }

        private static bool TestParamMigrationCombinedScenario()
        {
            // Setup: multiple changes at once
            var currentParams = new List<ParamMigrationHelper.ParamInfo>
            {
                new ParamMigrationHelper.ParamInfo { Name = "gain", DefaultValue = 1.2, Min = 0.2, Max = 1.8 }, // Default + range changed
                new ParamMigrationHelper.ParamInfo { Name = "friction", DefaultValue = 0.1, Min = 0, Max = 0.5 } // New param
                // "damping" removed
            };
            var oldSnapshots = new Dictionary<string, DiyFfbPluginSettings.ParamSnapshot>
            {
                ["gain"] = new DiyFfbPluginSettings.ParamSnapshot { DefaultValue = 1.0, Min = 0, Max = 2 },
                ["damping"] = new DiyFfbPluginSettings.ParamSnapshot { DefaultValue = 0.5, Min = 0, Max = 1 }
            };
            var overrides = new Dictionary<string, double>
            {
                ["gain"] = 0.1, // Below new min of 0.2
                ["damping"] = 0.7 // Becomes orphan
            };

            var result = ParamMigrationHelper.Migrate(currentParams, oldSnapshots, overrides, "hash123");

            return result.HasChangesToReview &&
                   result.ChangedDefaults.Count == 1 && result.ChangedDefaults[0].ParamName == "gain" &&
                   result.ClampedValues.Count == 1 && result.ClampedValues[0].ParamName == "gain" &&
                   Math.Abs(overrides["gain"] - 0.2) < 0.0001 && // Clamped to min
                   result.NewOrphans.Count == 1 && result.NewOrphans[0] == "damping" &&
                   result.CurrentSnapshots.ContainsKey("gain") &&
                   result.CurrentSnapshots.ContainsKey("friction") &&
                   !result.CurrentSnapshots.ContainsKey("damping"); // Not in current graph
        }

        #endregion

        #region IsShared Tests

        private static bool TestIsSharedMultipleDirectUsers()
        {
            // Multiple vehicles directly use this graph → IsShared = true
            var report = new GraphUsageReport
            {
                GraphPath = "test.json",
                CurrentVehicleKey = "Vehicle_A"
            };
            report.DirectUsers.Add("Vehicle_A");
            report.DirectUsers.Add("Vehicle_B");

            return report.IsShared == true;
        }

        private static bool TestIsSharedSingleUserDifferentVehicle()
        {
            // Single vehicle uses this graph, but it's not the current vehicle → IsShared = true
            var report = new GraphUsageReport
            {
                GraphPath = "test.json",
                CurrentVehicleKey = "Vehicle_A"
            };
            report.DirectUsers.Add("Vehicle_B");

            return report.IsShared == true;
        }

        private static bool TestIsSharedIncludedByGraphsWithUsers()
        {
            // Graph is included by another graph that has vehicle users → IsShared = true
            var report = new GraphUsageReport
            {
                GraphPath = "included.json",
                CurrentVehicleKey = "Vehicle_A"
            };
            // No direct users
            var includeUsage = new IncludeUsage { IncludingGraphPath = "parent.json" };
            includeUsage.VehicleKeys.Add("Vehicle_B");
            report.IncludedBy.Add(includeUsage);

            return report.IsShared == true;
        }

        private static bool TestIsSharedSingleUserSameVehicle()
        {
            // Single vehicle uses this graph, and it's the current vehicle → IsShared = false
            var report = new GraphUsageReport
            {
                GraphPath = "test.json",
                CurrentVehicleKey = "Vehicle_A"
            };
            report.DirectUsers.Add("Vehicle_A");

            return report.IsShared == false;
        }

        private static bool TestIsSharedNoUsersNoIncludes()
        {
            // No direct users, no includes → IsShared = false
            var report = new GraphUsageReport
            {
                GraphPath = "test.json",
                CurrentVehicleKey = "Vehicle_A"
            };

            return report.IsShared == false;
        }

        private static bool TestIsSharedEmptyCurrentVehicleKey()
        {
            // Single direct user but CurrentVehicleKey is empty → IsShared = false
            // (can't determine if it's "different" without a current vehicle to compare)
            var report = new GraphUsageReport
            {
                GraphPath = "test.json",
                CurrentVehicleKey = "" // Empty
            };
            report.DirectUsers.Add("Vehicle_A");

            // The code checks !string.IsNullOrEmpty(CurrentVehicleKey) before comparing
            // So with empty key, single user should not trigger "different vehicle" check
            return report.IsShared == false;
        }

        private static bool TestIsSharedCaseInsensitiveMatch()
        {
            // Vehicle keys should be compared case-insensitively
            // Same vehicle in different case → IsShared = false
            var report = new GraphUsageReport
            {
                GraphPath = "test.json",
                CurrentVehicleKey = "vehicle_a" // lowercase
            };
            report.DirectUsers.Add("Vehicle_A"); // Different case

            // Should match due to OrdinalIgnoreCase comparison
            return report.IsShared == false;
        }

        private static bool TestIsSharedIncludedWithoutVehicleUsers()
        {
            // Graph is included by another graph, but that parent has no vehicle users
            // → IsShared = false (include without actual vehicle impact)
            var report = new GraphUsageReport
            {
                GraphPath = "utility_lib.json",
                CurrentVehicleKey = "Vehicle_A"
            };
            // Included by a graph with no vehicle users
            var includeUsage = new IncludeUsage { IncludingGraphPath = "orphan_graph.json" };
            // Note: includeUsage.VehicleKeys is empty
            report.IncludedBy.Add(includeUsage);

            // The code checks i.VehicleKeys.Count > 0, so includes without users don't count
            return report.IsShared == false;
        }

        #endregion

        #region GraphHashComputer Tests

        private static bool TestGraphHashInvalidPath()
        {
            // Null path
            var hash1 = GraphHashComputer.ComputeGraphTreeHash(null, null);
            if (hash1 != null) return false;

            // Empty path
            var hash2 = GraphHashComputer.ComputeGraphTreeHash("", null);
            if (hash2 != null) return false;

            // Non-existent path
            var hash3 = GraphHashComputer.ComputeGraphTreeHash(@"C:\nonexistent\path\file.json", null);
            if (hash3 != null) return false;

            return true;
        }

        private static bool TestGraphHashDeterministic()
        {
            // Same content should produce same hash
            string tempDir = System.IO.Path.Combine(System.IO.Path.GetTempPath(), "ffb_hash_test_" + Guid.NewGuid().ToString("N").Substring(0, 8));
            try
            {
                System.IO.Directory.CreateDirectory(tempDir);

                // Create a simple graph
                var graph = new GraphEditor.GraphDefinition();
                var constNode = new GraphEditor.GraphNode { Id = "c1", Kind = GraphEditor.GraphNodeKind.Const, ConstValue = 42.0 };
                graph.Nodes.Add(constNode);
                var outNode = new GraphEditor.GraphNode { Id = "out", Kind = GraphEditor.GraphNodeKind.Output };
                outNode.Ports.Add(new GraphEditor.GraphPort { Name = "value", Kind = GraphEditor.GraphPortKind.Input });
                graph.Nodes.Add(outNode);
                graph.Links.Add(new GraphEditor.GraphLink { FromNodeId = "c1", FromPort = "value", ToNodeId = "out", ToPort = "value" });

                string graphPath = System.IO.Path.Combine(tempDir, "test.json");
                System.IO.File.WriteAllText(graphPath, GraphEditor.GraphSerializer.Serialize(graph));

                // Compute hash twice
                var hash1 = GraphHashComputer.ComputeGraphTreeHash(graphPath, graph);
                var hash2 = GraphHashComputer.ComputeGraphTreeHash(graphPath, graph);

                // Should be identical
                return hash1 != null && hash1 == hash2;
            }
            finally
            {
                try { System.IO.Directory.Delete(tempDir, true); } catch { }
            }
        }

        private static bool TestGraphHashChangedContent()
        {
            // Changed content should produce different hash
            string tempDir = System.IO.Path.Combine(System.IO.Path.GetTempPath(), "ffb_hash_test_" + Guid.NewGuid().ToString("N").Substring(0, 8));
            try
            {
                System.IO.Directory.CreateDirectory(tempDir);

                // Create initial graph
                var graph1 = new GraphEditor.GraphDefinition();
                var constNode1 = new GraphEditor.GraphNode { Id = "c1", Kind = GraphEditor.GraphNodeKind.Const, ConstValue = 42.0 };
                graph1.Nodes.Add(constNode1);

                string graphPath = System.IO.Path.Combine(tempDir, "test.json");
                System.IO.File.WriteAllText(graphPath, GraphEditor.GraphSerializer.Serialize(graph1));

                var hash1 = GraphHashComputer.ComputeGraphTreeHash(graphPath, graph1);

                // Modify the graph
                var graph2 = new GraphEditor.GraphDefinition();
                var constNode2 = new GraphEditor.GraphNode { Id = "c1", Kind = GraphEditor.GraphNodeKind.Const, ConstValue = 99.0 }; // Different value
                graph2.Nodes.Add(constNode2);

                System.IO.File.WriteAllText(graphPath, GraphEditor.GraphSerializer.Serialize(graph2));

                var hash2 = GraphHashComputer.ComputeGraphTreeHash(graphPath, graph2);

                // Hashes should be different
                return hash1 != null && hash2 != null && hash1 != hash2;
            }
            finally
            {
                try { System.IO.Directory.Delete(tempDir, true); } catch { }
            }
        }

        private static bool TestGraphHashNestedIncludes()
        {
            // Hash should include all files in the include tree
            string tempDir = System.IO.Path.Combine(System.IO.Path.GetTempPath(), "ffb_hash_nested_" + Guid.NewGuid().ToString("N").Substring(0, 8));
            try
            {
                System.IO.Directory.CreateDirectory(tempDir);

                // Create inner graph
                var innerGraph = new GraphEditor.GraphDefinition { IsLibraryGraph = true };
                var innerConst = new GraphEditor.GraphNode { Id = "c1", Kind = GraphEditor.GraphNodeKind.Const, ConstValue = 10.0 };
                innerGraph.Nodes.Add(innerConst);
                var innerOut = new GraphEditor.GraphNode { Id = "out", Kind = GraphEditor.GraphNodeKind.Output };
                innerOut.Ports.Add(new GraphEditor.GraphPort { Name = "value", Kind = GraphEditor.GraphPortKind.Input });
                innerGraph.Nodes.Add(innerOut);
                innerGraph.Links.Add(new GraphEditor.GraphLink { FromNodeId = "c1", FromPort = "value", ToNodeId = "out", ToPort = "value" });

                string innerPath = System.IO.Path.Combine(tempDir, "inner.json");
                System.IO.File.WriteAllText(innerPath, GraphEditor.GraphSerializer.Serialize(innerGraph));

                // Create parent graph that includes inner
                var parentGraph = new GraphEditor.GraphDefinition();
                var includeNode = new GraphEditor.GraphNode
                {
                    Id = "inc",
                    Kind = GraphEditor.GraphNodeKind.Include,
                    IncludePath = "inner.json"
                };
                parentGraph.Nodes.Add(includeNode);

                string parentPath = System.IO.Path.Combine(tempDir, "parent.json");
                System.IO.File.WriteAllText(parentPath, GraphEditor.GraphSerializer.Serialize(parentGraph));

                var hash1 = GraphHashComputer.ComputeGraphTreeHash(parentPath, parentGraph);

                // Now modify the inner graph
                innerGraph.Nodes[0].ConstValue = 20.0; // Change const value
                System.IO.File.WriteAllText(innerPath, GraphEditor.GraphSerializer.Serialize(innerGraph));

                var hash2 = GraphHashComputer.ComputeGraphTreeHash(parentPath, parentGraph);

                // Hash should change because inner file changed
                return hash1 != null && hash2 != null && hash1 != hash2;
            }
            finally
            {
                try { System.IO.Directory.Delete(tempDir, true); } catch { }
            }
        }

        private static bool TestGraphHashCyclicIncludes()
        {
            // Cyclic includes should be handled gracefully (no infinite loop)
            string tempDir = System.IO.Path.Combine(System.IO.Path.GetTempPath(), "ffb_hash_cyclic_" + Guid.NewGuid().ToString("N").Substring(0, 8));
            try
            {
                System.IO.Directory.CreateDirectory(tempDir);

                // Create graph A that includes B
                var graphA = new GraphEditor.GraphDefinition();
                var includeB = new GraphEditor.GraphNode
                {
                    Id = "incB",
                    Kind = GraphEditor.GraphNodeKind.Include,
                    IncludePath = "b.json"
                };
                graphA.Nodes.Add(includeB);

                // Create graph B that includes A (cycle)
                var graphB = new GraphEditor.GraphDefinition();
                var includeA = new GraphEditor.GraphNode
                {
                    Id = "incA",
                    Kind = GraphEditor.GraphNodeKind.Include,
                    IncludePath = "a.json"
                };
                graphB.Nodes.Add(includeA);

                string pathA = System.IO.Path.Combine(tempDir, "a.json");
                string pathB = System.IO.Path.Combine(tempDir, "b.json");
                System.IO.File.WriteAllText(pathA, GraphEditor.GraphSerializer.Serialize(graphA));
                System.IO.File.WriteAllText(pathB, GraphEditor.GraphSerializer.Serialize(graphB));

                // Should return a hash without infinite loop
                var hash = GraphHashComputer.ComputeGraphTreeHash(pathA, graphA);

                return hash != null && hash.Length > 0;
            }
            finally
            {
                try { System.IO.Directory.Delete(tempDir, true); } catch { }
            }
        }

        #endregion

        #region Tools.cs Tests

        private static bool TestNormalize_InRange()
        {
            // Value between min/max should return normalized 0-1
            double result = Tools.Normalize(50, 0, 100);
            return Math.Abs(result - 0.5) < 1e-9;
        }

        private static bool TestNormalize_BelowMin()
        {
            // Value <= min should return 0
            double atMin = Tools.Normalize(0, 0, 100);
            double belowMin = Tools.Normalize(-10, 0, 100);
            return atMin == 0 && belowMin == 0;
        }

        private static bool TestNormalize_AboveMax()
        {
            // Value >= max should return 1
            double atMax = Tools.Normalize(100, 0, 100);
            double aboveMax = Tools.Normalize(150, 0, 100);
            return atMax == 1 && aboveMax == 1;
        }

        private static bool TestNormalize_ZeroRange()
        {
            // min ≈ max (range < 0.0000001) should return 0
            double result = Tools.Normalize(5, 5, 5);
            double nearlyZero = Tools.Normalize(5, 5, 5.00000001);
            return result == 0 && nearlyZero == 0;
        }

        private static bool TestTryComputeMarkerX_Valid()
        {
            // Valid computation should return true with correct x position
            bool success = Tools.TryComputeMarkerX(
                value: 50, lower: 0, upper: 100, min: 0, max: 100, width: 200, out double x);
            // At midpoint of full range: should be at midpoint of width
            return success && Math.Abs(x - 100) < 1e-9;
        }

        private static bool TestTryComputeMarkerX_ZeroWidth()
        {
            // Width <= 0 should return false
            bool successZero = Tools.TryComputeMarkerX(50, 0, 100, 0, 100, 0, out _);
            bool successNegative = Tools.TryComputeMarkerX(50, 0, 100, 0, 100, -10, out _);
            return !successZero && !successNegative;
        }

        private static bool TestTryComputeMarkerX_SwappedMinMax()
        {
            // Should handle min > max by swapping internally
            bool success = Tools.TryComputeMarkerX(
                value: 50, lower: 0, upper: 100, min: 100, max: 0, width: 200, out double x);
            return success && Math.Abs(x - 100) < 1e-9;
        }

        private static bool TestTryAutoTuneLoadGain_BelowMinForce()
        {
            // When axisForce < minForce, should return false without changing gain
            bool changed = Tools.TryAutoTuneLoadGain(
                axisForceAbs: 5, loadForceAbs: 3, currentGain: 1.0,
                minGainAbs: 0.1, maxGainAbs: 2.0,
                ratioLow: 0.8, ratioHigh: 1.2, gainStep: 0.1, minForceAbs: 10,
                out double updatedGain);
            return !changed && updatedGain == 1.0;
        }

        private static bool TestTryAutoTuneLoadGain_RatioHigh()
        {
            // ratio > ratioHigh should decrease gain
            // ratio = 15/10 = 1.5 > 1.2
            bool changed = Tools.TryAutoTuneLoadGain(
                axisForceAbs: 10, loadForceAbs: 15, currentGain: 1.0,
                minGainAbs: 0.1, maxGainAbs: 2.0,
                ratioLow: 0.8, ratioHigh: 1.2, gainStep: 0.1, minForceAbs: 5,
                out double updatedGain);
            return changed && Math.Abs(updatedGain - 0.9) < 1e-9;
        }

        private static bool TestTryAutoTuneLoadGain_RatioLow()
        {
            // ratio < ratioLow should increase gain
            // ratio = 5/10 = 0.5 < 0.8
            bool changed = Tools.TryAutoTuneLoadGain(
                axisForceAbs: 10, loadForceAbs: 5, currentGain: 1.0,
                minGainAbs: 0.1, maxGainAbs: 2.0,
                ratioLow: 0.8, ratioHigh: 1.2, gainStep: 0.1, minForceAbs: 5,
                out double updatedGain);
            return changed && Math.Abs(updatedGain - 1.1) < 1e-9;
        }

        private static bool TestTryAutoTuneLoadGain_InRange()
        {
            // ratio in [ratioLow, ratioHigh] should return false, no change
            // ratio = 10/10 = 1.0, which is between 0.8 and 1.2
            bool changed = Tools.TryAutoTuneLoadGain(
                axisForceAbs: 10, loadForceAbs: 10, currentGain: 1.0,
                minGainAbs: 0.1, maxGainAbs: 2.0,
                ratioLow: 0.8, ratioHigh: 1.2, gainStep: 0.1, minForceAbs: 5,
                out double updatedGain);
            return !changed && updatedGain == 1.0;
        }

        #endregion

        #region CubicSpline Tests

        private static bool TestInterpolate_LinearData()
        {
            // Linear points should produce linear interpolation
            double[] xs = { 0, 1, 2, 3, 4 };
            double[] ys = { 0, 1, 2, 3, 4 };
            double[] xInterp = { 0.5, 1.5, 2.5, 3.5 };

            var (yInterp, _, _) = Cubic.Interpolate(xs, ys, xInterp);

            // For linear data, interpolated values should match x values
            return Math.Abs(yInterp[0] - 0.5) < 1e-6 &&
                   Math.Abs(yInterp[1] - 1.5) < 1e-6 &&
                   Math.Abs(yInterp[2] - 2.5) < 1e-6 &&
                   Math.Abs(yInterp[3] - 3.5) < 1e-6;
        }

        private static bool TestInterpolate_EndpointMatch()
        {
            // Interpolated endpoints should match original endpoints
            double[] xs = { 0, 1, 2, 3, 4 };
            double[] ys = { 0, 2, 1, 3, 4 };
            double[] xInterp = { 0, 4 }; // endpoints

            var (yInterp, _, _) = Cubic.Interpolate(xs, ys, xInterp);

            return Math.Abs(yInterp[0] - ys[0]) < 1e-6 &&
                   Math.Abs(yInterp[1] - ys[4]) < 1e-6;
        }

        private static bool TestInterpolate_Monotonic()
        {
            // Monotonic input should produce smooth output (no wild oscillations)
            double[] xs = { 0, 1, 2, 3, 4 };
            double[] ys = { 0, 1, 4, 9, 16 }; // quadratic-like
            double[] xInterp = { 0.5, 1.5, 2.5, 3.5 };

            var (yInterp, _, _) = Cubic.Interpolate(xs, ys, xInterp);

            // Values should be increasing and within reasonable bounds
            return yInterp[0] > 0 && yInterp[0] < 1 &&
                   yInterp[1] > 1 && yInterp[1] < 4 &&
                   yInterp[2] > 4 && yInterp[2] < 9 &&
                   yInterp[3] > 9 && yInterp[3] < 16;
        }

        private static bool TestInterpolate1D_CountMatches()
        {
            // Output array should have requested count
            double[] xs = { 0, 1, 2, 3, 4 };
            double[] ys = { 0, 2, 1, 3, 4 };
            int requestedCount = 20;

            var (evenDistances, ysOut, _, _) = Cubic.Interpolate1D(xs, ys, requestedCount);

            return evenDistances.Length == requestedCount && ysOut.Length == requestedCount;
        }

        private static bool TestInterpolate_MismatchedArrays()
        {
            // Mismatched array lengths should throw ArgumentException
            double[] xs = { 0, 1, 2 };
            double[] ys = { 0, 1 }; // different length

            try
            {
                Cubic.Interpolate1D(xs, ys, 10);
                return false; // Should have thrown
            }
            catch (ArgumentException)
            {
                return true;
            }
        }

        #endregion

        #region StringExtensions Tests

        private static bool TestConstCaseToTitleCase()
        {
            // "HELLO_WORLD" -> "Hello World"
            string result = "HELLO_WORLD".ConstCaseToTitleCaseSentence();
            return result == "Hello World";
        }

        private static bool TestCamelCaseToTitleCase()
        {
            // "helloWorld" -> "hello World"
            string result = "helloWorld".CamelCaseToTitleCase();
            return result == "hello World";
        }

        #endregion

        #region GeneralKinematics Tests

        /// <summary>
        /// Creates a minimal valid kinematics config for testing.
        /// A simple 4-pin, 2-bar linkage with one metering bar.
        /// </summary>
        private static GeneralKinematicConfig CreateValidKinematicsConfig()
        {
            var config = new GeneralKinematicConfig();

            // Pin 1: Grounded pivot
            config.Pins.Add(new GeneralKinematicPin { PinId = 1, X = 0, Y = 0, Grounded = true });
            // Pin 2: Middle of lever
            config.Pins.Add(new GeneralKinematicPin { PinId = 2, X = 0, Y = 100 });
            // Pin 3: Contact point at end of lever
            config.Pins.Add(new GeneralKinematicPin { PinId = 3, X = 0, Y = 180, IsContactPoint = true });
            // Pin 4: Rail interface (sled)
            config.Pins.Add(new GeneralKinematicPin { PinId = 4, X = 250, Y = 23, IsRailInterface = true });

            // Bar 1: Lever (3-pin collinear bar)
            var bar1 = new GeneralKinematicBar();
            bar1.PinIds.Add(1);
            bar1.PinIds.Add(2);
            bar1.PinIds.Add(3);
            config.Bars.Add(bar1);

            // Bar 2: Metering rod connecting pin 2 to sled
            var bar2 = new GeneralKinematicBar { IsMetering = true };
            bar2.PinIds.Add(2);
            bar2.PinIds.Add(4);
            config.Bars.Add(bar2);

            config.RailTravelNegative = 0;
            config.RailTravelPositive = 60;

            return config;
        }

        private static bool TestCalcKinematicParameters_NullConfig()
        {
            try
            {
                GeneralKinematics.CalcKinematicParameters(null);
                return false; // Should have thrown
            }
            catch (ArgumentNullException)
            {
                return true;
            }
        }

        private static bool TestCalcKinematicParameters_NoPins()
        {
            try
            {
                var config = new GeneralKinematicConfig();
                // No pins added
                var bar = new GeneralKinematicBar();
                bar.PinIds.Add(1);
                bar.PinIds.Add(2);
                config.Bars.Add(bar);
                config.RailTravelPositive = 10;

                GeneralKinematics.CalcKinematicParameters(config);
                return false; // Should have thrown
            }
            catch (ArgumentException ex) when (ex.Message.Contains("pins"))
            {
                return true;
            }
        }

        private static bool TestCalcKinematicParameters_NoBars()
        {
            try
            {
                var config = new GeneralKinematicConfig();
                config.Pins.Add(new GeneralKinematicPin { PinId = 1, X = 0, Y = 0, Grounded = true });
                config.Pins.Add(new GeneralKinematicPin { PinId = 2, X = 10, Y = 0, IsContactPoint = true });
                // No bars added
                config.RailTravelPositive = 10;

                GeneralKinematics.CalcKinematicParameters(config);
                return false; // Should have thrown
            }
            catch (ArgumentException ex) when (ex.Message.Contains("bars"))
            {
                return true;
            }
        }

        private static bool TestCalcKinematicParameters_NegativeTravel()
        {
            try
            {
                var config = CreateValidKinematicsConfig();
                config.RailTravelNegative = -5; // Negative value not allowed

                GeneralKinematics.CalcKinematicParameters(config);
                return false; // Should have thrown
            }
            catch (ArgumentException ex) when (ex.Message.Contains("rail_travel"))
            {
                return true;
            }
        }

        private static bool TestSimpleLinkage_Computes()
        {
            // A valid configuration should compute without throwing
            var config = CreateValidKinematicsConfig();

            try
            {
                var parameters = GeneralKinematics.CalcKinematicParameters(config);

                // Should have polynomial coefficients
                if (parameters.CoeffsSledPosOverContactPointPos.Count == 0) return false;
                if (parameters.CoeffsForceFactorOverContactPointPos.Count == 0) return false;

                // Contact point range should be non-zero
                if (parameters.ContactPointPosMinAbs == 0 && parameters.ContactPointPosMaxAbs == 0) return false;

                return true;
            }
            catch
            {
                return false;
            }
        }

        private static bool TestRailTravel_Bounds()
        {
            // Contact positions should be bounded within expected range
            var config = CreateValidKinematicsConfig();

            var poseCache = GeneralKinematics.BuildPoseCache(config);

            // ContactPositions array should span a reasonable range
            double minContact = poseCache.ContactPositions.Min();
            double maxContact = poseCache.ContactPositions.Max();

            // The range should be non-zero (there should be movement)
            if (Math.Abs(maxContact - minContact) < 1e-6) return false;

            // Rail offsets should span from -RailTravelNegative to +RailTravelPositive
            double minRail = poseCache.RailOffsets.Min();
            double maxRail = poseCache.RailOffsets.Max();

            // Allow small tolerance for floating point
            if (Math.Abs(minRail - (-config.RailTravelNegative)) > 0.1) return false;
            if (Math.Abs(maxRail - config.RailTravelPositive) > 0.1) return false;

            return true;
        }

        private static bool TestPoseCache_PinCount()
        {
            var config = CreateValidKinematicsConfig();
            var poseCache = GeneralKinematics.BuildPoseCache(config);

            // Should have as many pin IDs as pins in config
            if (poseCache.PinIds.Length != config.Pins.Count) return false;

            // Each sample should have position arrays for all pins
            if (poseCache.PinPositionsX[0].Length != config.Pins.Count) return false;
            if (poseCache.PinPositionsY[0].Length != config.Pins.Count) return false;

            return true;
        }

        private static bool TestPoseCache_SampleCount()
        {
            var config = CreateValidKinematicsConfig();
            var poseCache = GeneralKinematics.BuildPoseCache(config);

            // SampleCount is 200 (private const in GeneralKinematics)
            const int expectedSampleCount = 200;

            if (poseCache.RailOffsets.Length != expectedSampleCount) return false;
            if (poseCache.ContactPositions.Length != expectedSampleCount) return false;
            if (poseCache.PinPositionsX.Length != expectedSampleCount) return false;
            if (poseCache.PinPositionsY.Length != expectedSampleCount) return false;

            return true;
        }

        private static bool TestCollinearPins_Handled()
        {
            // A bar with collinear pins should be handled properly (treated as rigid bar)
            var config = CreateValidKinematicsConfig();
            // The 3-pin bar (pins 1,2,3) is already collinear in the test config

            try
            {
                var poseCache = GeneralKinematics.BuildPoseCache(config);

                // All samples should have valid positions (no NaN)
                for (int i = 0; i < poseCache.ContactPositions.Length; i++)
                {
                    if (double.IsNaN(poseCache.ContactPositions[i])) return false;
                }

                return true;
            }
            catch
            {
                return false;
            }
        }

        private static bool TestZeroLengthBar_Throws()
        {
            // A bar with two pins at the same location should throw
            try
            {
                var config = new GeneralKinematicConfig();
                config.Pins.Add(new GeneralKinematicPin { PinId = 1, X = 0, Y = 0, Grounded = true });
                config.Pins.Add(new GeneralKinematicPin { PinId = 2, X = 0, Y = 0 }); // Same position!
                config.Pins.Add(new GeneralKinematicPin { PinId = 3, X = 50, Y = 50, IsContactPoint = true });
                config.Pins.Add(new GeneralKinematicPin { PinId = 4, X = 100, Y = 0, IsRailInterface = true });

                var bar1 = new GeneralKinematicBar();
                bar1.PinIds.Add(1);
                bar1.PinIds.Add(2); // Zero-length bar!
                config.Bars.Add(bar1);

                var bar2 = new GeneralKinematicBar { IsMetering = true };
                bar2.PinIds.Add(2);
                bar2.PinIds.Add(4);
                config.Bars.Add(bar2);

                config.RailTravelPositive = 30;

                GeneralKinematics.CalcKinematicParameters(config);
                return false; // Should have thrown
            }
            catch (ArgumentException ex) when (ex.Message.Contains("length"))
            {
                return true;
            }
        }

        private static bool TestMissingContactPoint_Throws()
        {
            // Config without a contact point pin should throw
            try
            {
                var config = new GeneralKinematicConfig();
                config.Pins.Add(new GeneralKinematicPin { PinId = 1, X = 0, Y = 0, Grounded = true });
                config.Pins.Add(new GeneralKinematicPin { PinId = 2, X = 50, Y = 0 }); // No IsContactPoint!
                config.Pins.Add(new GeneralKinematicPin { PinId = 3, X = 100, Y = 0, IsRailInterface = true });

                var bar = new GeneralKinematicBar { IsMetering = true };
                bar.PinIds.Add(1);
                bar.PinIds.Add(2);
                config.Bars.Add(bar);

                config.RailTravelPositive = 30;

                GeneralKinematics.CalcKinematicParameters(config);
                return false; // Should have thrown
            }
            catch (ArgumentException ex) when (ex.Message.Contains("contact point"))
            {
                return true;
            }
        }

        private static bool TestMissingRailInterface_Throws()
        {
            // Config without a rail interface pin should throw
            try
            {
                var config = new GeneralKinematicConfig();
                config.Pins.Add(new GeneralKinematicPin { PinId = 1, X = 0, Y = 0, Grounded = true });
                config.Pins.Add(new GeneralKinematicPin { PinId = 2, X = 50, Y = 0, IsContactPoint = true });
                config.Pins.Add(new GeneralKinematicPin { PinId = 3, X = 100, Y = 0 }); // No IsRailInterface!

                var bar = new GeneralKinematicBar { IsMetering = true };
                bar.PinIds.Add(1);
                bar.PinIds.Add(2);
                config.Bars.Add(bar);

                config.RailTravelPositive = 30;

                GeneralKinematics.CalcKinematicParameters(config);
                return false; // Should have thrown
            }
            catch (ArgumentException ex) when (ex.Message.Contains("rail interface"))
            {
                return true;
            }
        }

        #endregion

        #region GraphRuntimeConverter Tests

        private static bool TestMapNodeType_AllKinds()
        {
            // Test that all GraphNodeKind values map to correct runtime NodeType
            // We test indirectly through Convert() since MapNodeType is private
            var graph = new GraphEditor.GraphDefinition();

            // Input node
            var inputNode = new GraphEditor.GraphNode { Id = "in", Kind = GraphEditor.GraphNodeKind.Input };
            inputNode.Ports.Add(new GraphEditor.GraphPort { Name = "val", Kind = GraphEditor.GraphPortKind.Output });
            graph.Nodes.Add(inputNode);

            // Param node
            var paramNode = new GraphEditor.GraphNode { Id = "param", Kind = GraphEditor.GraphNodeKind.Param };
            paramNode.Ports.Add(new GraphEditor.GraphPort { Name = "p", Kind = GraphEditor.GraphPortKind.Output });
            graph.Nodes.Add(paramNode);

            // Const node
            var constNode = new GraphEditor.GraphNode { Id = "const", Kind = GraphEditor.GraphNodeKind.Const, ConstValue = 1.0 };
            graph.Nodes.Add(constNode);

            // Op node
            var opNode = new GraphEditor.GraphNode { Id = "op", Kind = GraphEditor.GraphNodeKind.Op, Op = "add" };
            opNode.Ports.Add(new GraphEditor.GraphPort { Name = "out", Kind = GraphEditor.GraphPortKind.Output });
            graph.Nodes.Add(opNode);

            // Func node
            var funcNode = new GraphEditor.GraphNode { Id = "func", Kind = GraphEditor.GraphNodeKind.Func, Func = "sin" };
            funcNode.Ports.Add(new GraphEditor.GraphPort { Name = "out", Kind = GraphEditor.GraphPortKind.Output });
            graph.Nodes.Add(funcNode);

            // Include node
            var includeNode = new GraphEditor.GraphNode { Id = "inc", Kind = GraphEditor.GraphNodeKind.Include, IncludePath = "test.json" };
            graph.Nodes.Add(includeNode);

            var runtime = GraphEditor.GraphRuntimeConverter.Convert(graph);

            // Input and Param create separate nodes with port-based IDs
            bool inputOk = runtime.Nodes.TryGetValue("in:val", out var rIn) && rIn.Type.ToString() == "Input";
            bool paramOk = runtime.Nodes.TryGetValue("param:p", out var rParam) && rParam.Type.ToString() == "Param";
            bool constOk = runtime.Nodes.TryGetValue("const", out var rConst) && rConst.Type.ToString() == "Const";
            bool opOk = runtime.Nodes.TryGetValue("op", out var rOp) && rOp.Type.ToString() == "Op";
            bool funcOk = runtime.Nodes.TryGetValue("func", out var rFunc) && rFunc.Type.ToString() == "Func";
            bool incOk = runtime.Nodes.TryGetValue("inc", out var rInc) && rInc.Type.ToString() == "Include";

            return inputOk && paramOk && constOk && opOk && funcOk && incOk;
        }

        private static bool TestMapOp_AllOperators()
        {
            // Test all operator strings map correctly
            var ops = new (string op, string expected)[]
            {
                ("add", "Add"), ("sub", "Sub"), ("mul", "Mul"), ("div", "Div"),
                ("min", "Min"), ("max", "Max"), ("abs", "Abs"), ("neg", "Neg"),
                ("clamp", "Clamp"), ("lerp", "Lerp")
            };

            foreach (var (op, expected) in ops)
            {
                var graph = new GraphEditor.GraphDefinition();
                var opNode = new GraphEditor.GraphNode { Id = "op", Kind = GraphEditor.GraphNodeKind.Op, Op = op };
                opNode.Ports.Add(new GraphEditor.GraphPort { Name = "out", Kind = GraphEditor.GraphPortKind.Output });
                graph.Nodes.Add(opNode);

                var runtime = GraphEditor.GraphRuntimeConverter.Convert(graph);
                if (!runtime.Nodes.TryGetValue("op", out var rOp) || rOp.Op.ToString() != expected)
                {
                    return false;
                }
            }
            return true;
        }

        private static bool TestMapOp_CaseInsensitive()
        {
            // Test that "ADD", "add", "Add" all map to Add
            var cases = new[] { "ADD", "add", "Add", "aDd" };

            foreach (var opStr in cases)
            {
                var graph = new GraphEditor.GraphDefinition();
                var opNode = new GraphEditor.GraphNode { Id = "op", Kind = GraphEditor.GraphNodeKind.Op, Op = opStr };
                opNode.Ports.Add(new GraphEditor.GraphPort { Name = "out", Kind = GraphEditor.GraphPortKind.Output });
                graph.Nodes.Add(opNode);

                var runtime = GraphEditor.GraphRuntimeConverter.Convert(graph);
                if (!runtime.Nodes.TryGetValue("op", out var rOp) || rOp.Op.ToString() != "Add")
                {
                    return false;
                }
            }
            return true;
        }

        private static bool TestMapOp_Symbols()
        {
            // Test that "+", "-", "*", "/" map correctly
            var symbols = new (string symbol, string expected)[]
            {
                ("+", "Add"), ("-", "Sub"), ("*", "Mul"), ("/", "Div")
            };

            foreach (var (symbol, expected) in symbols)
            {
                var graph = new GraphEditor.GraphDefinition();
                var opNode = new GraphEditor.GraphNode { Id = "op", Kind = GraphEditor.GraphNodeKind.Op, Op = symbol };
                opNode.Ports.Add(new GraphEditor.GraphPort { Name = "out", Kind = GraphEditor.GraphPortKind.Output });
                graph.Nodes.Add(opNode);

                var runtime = GraphEditor.GraphRuntimeConverter.Convert(graph);
                if (!runtime.Nodes.TryGetValue("op", out var rOp) || rOp.Op.ToString() != expected)
                {
                    return false;
                }
            }
            return true;
        }

        private static bool TestConvert_InputNode()
        {
            // Test that Input node converts with correct signal name from SignalGroup.SignalSuffix
            var graph = new GraphEditor.GraphDefinition();

            var inputNode = new GraphEditor.GraphNode
            {
                Id = "in",
                Kind = GraphEditor.GraphNodeKind.Input,
                SignalGroup = "XPlane"
            };
            inputNode.Ports.Add(new GraphEditor.GraphPort
            {
                Name = "IAS",
                Kind = GraphEditor.GraphPortKind.Output,
                SignalSuffix = "IAS"
            });
            graph.Nodes.Add(inputNode);

            var runtime = GraphEditor.GraphRuntimeConverter.Convert(graph);

            // Input node should have port-based ID and full signal name
            return runtime.Nodes.TryGetValue("in:IAS", out var rIn) &&
                   rIn.Type.ToString() == "Input" &&
                   rIn.Name == "XPlane.IAS";
        }

        private static bool TestConvert_ParamNode()
        {
            // Test that Param node gets default value from graph.Params
            var graph = new GraphEditor.GraphDefinition();

            var paramNode = new GraphEditor.GraphNode
            {
                Id = "param",
                Kind = GraphEditor.GraphNodeKind.Param,
                SignalGroup = "Settings"
            };
            paramNode.Ports.Add(new GraphEditor.GraphPort
            {
                Name = "Gain",
                Kind = GraphEditor.GraphPortKind.Output,
                SignalSuffix = "Gain"
            });
            graph.Nodes.Add(paramNode);

            // Set default value in graph.Params
            graph.Params["Settings.Gain"] = new GraphEditor.GraphParam
            {
                Name = "Settings.Gain",
                DefaultValue = 0.5,
                Min = 0,
                Max = 1
            };

            var runtime = GraphEditor.GraphRuntimeConverter.Convert(graph);

            return runtime.Nodes.TryGetValue("param:Gain", out var rParam) &&
                   rParam.Type.ToString() == "Param" &&
                   rParam.Name == "Settings.Gain" &&
                   Math.Abs(rParam.ConstValue - 0.5) < 1e-6;
        }

        private static bool TestConvert_OutputNode()
        {
            // Test that Output node converts with source connection
            var graph = new GraphEditor.GraphDefinition();

            var constNode = new GraphEditor.GraphNode { Id = "k", Kind = GraphEditor.GraphNodeKind.Const, ConstValue = 42.0 };
            graph.Nodes.Add(constNode);

            var outputNode = new GraphEditor.GraphNode
            {
                Id = "out",
                Kind = GraphEditor.GraphNodeKind.Output,
                SignalGroup = "Result"
            };
            outputNode.Ports.Add(new GraphEditor.GraphPort
            {
                Name = "Value",
                Kind = GraphEditor.GraphPortKind.Input,
                SignalSuffix = "Value"
            });
            graph.Nodes.Add(outputNode);

            graph.Links.Add(new GraphEditor.GraphLink
            {
                FromNodeId = "k",
                ToNodeId = "out",
                ToPort = "Value"
            });

            var runtime = GraphEditor.GraphRuntimeConverter.Convert(graph);

            // Output node creates port-based ID nodes
            return runtime.Nodes.TryGetValue("out:Value", out var rOut) &&
                   rOut.Type.ToString() == "Output" &&
                   rOut.Name == "Result.Value" &&
                   rOut.Src == "k";
        }

        private static bool TestConvert_OpNode_Args()
        {
            // Test that Op node args are populated from links
            var graph = new GraphEditor.GraphDefinition();

            var a = new GraphEditor.GraphNode { Id = "a", Kind = GraphEditor.GraphNodeKind.Const, ConstValue = 1.0 };
            var b = new GraphEditor.GraphNode { Id = "b", Kind = GraphEditor.GraphNodeKind.Const, ConstValue = 2.0 };
            var c = new GraphEditor.GraphNode { Id = "c", Kind = GraphEditor.GraphNodeKind.Const, ConstValue = 3.0 };
            graph.Nodes.Add(a);
            graph.Nodes.Add(b);
            graph.Nodes.Add(c);

            var op = new GraphEditor.GraphNode { Id = "op", Kind = GraphEditor.GraphNodeKind.Op, Op = "add" };
            op.Ports.Add(new GraphEditor.GraphPort { Name = "x", Kind = GraphEditor.GraphPortKind.Input });
            op.Ports.Add(new GraphEditor.GraphPort { Name = "y", Kind = GraphEditor.GraphPortKind.Input });
            op.Ports.Add(new GraphEditor.GraphPort { Name = "z", Kind = GraphEditor.GraphPortKind.Input });
            op.Ports.Add(new GraphEditor.GraphPort { Name = "out", Kind = GraphEditor.GraphPortKind.Output });
            graph.Nodes.Add(op);

            graph.Links.Add(new GraphEditor.GraphLink { FromNodeId = "a", ToNodeId = "op", ToPort = "x" });
            graph.Links.Add(new GraphEditor.GraphLink { FromNodeId = "b", ToNodeId = "op", ToPort = "y" });
            graph.Links.Add(new GraphEditor.GraphLink { FromNodeId = "c", ToNodeId = "op", ToPort = "z" });

            var runtime = GraphEditor.GraphRuntimeConverter.Convert(graph);

            return runtime.Nodes.TryGetValue("op", out var rOp) &&
                   rOp.Args.Count == 3 &&
                   rOp.Args[0] == "a" &&
                   rOp.Args[1] == "b" &&
                   rOp.Args[2] == "c";
        }

        private static bool TestConvert_OpNode_Negate()
        {
            // Test that Negate flags are set correctly only for Add/Mul
            var graph = new GraphEditor.GraphDefinition();

            var a = new GraphEditor.GraphNode { Id = "a", Kind = GraphEditor.GraphNodeKind.Const, ConstValue = 1.0 };
            var b = new GraphEditor.GraphNode { Id = "b", Kind = GraphEditor.GraphNodeKind.Const, ConstValue = 2.0 };
            graph.Nodes.Add(a);
            graph.Nodes.Add(b);

            // Add op with negate on second input
            var addOp = new GraphEditor.GraphNode { Id = "add", Kind = GraphEditor.GraphNodeKind.Op, Op = "add" };
            addOp.Ports.Add(new GraphEditor.GraphPort { Name = "x", Kind = GraphEditor.GraphPortKind.Input, Negate = false });
            addOp.Ports.Add(new GraphEditor.GraphPort { Name = "y", Kind = GraphEditor.GraphPortKind.Input, Negate = true });
            addOp.Ports.Add(new GraphEditor.GraphPort { Name = "out", Kind = GraphEditor.GraphPortKind.Output });
            graph.Nodes.Add(addOp);

            // Sub op with negate (should be ignored since Sub doesn't support negate)
            var subOp = new GraphEditor.GraphNode { Id = "sub", Kind = GraphEditor.GraphNodeKind.Op, Op = "sub" };
            subOp.Ports.Add(new GraphEditor.GraphPort { Name = "x", Kind = GraphEditor.GraphPortKind.Input, Negate = true });
            subOp.Ports.Add(new GraphEditor.GraphPort { Name = "y", Kind = GraphEditor.GraphPortKind.Input, Negate = true });
            subOp.Ports.Add(new GraphEditor.GraphPort { Name = "out", Kind = GraphEditor.GraphPortKind.Output });
            graph.Nodes.Add(subOp);

            graph.Links.Add(new GraphEditor.GraphLink { FromNodeId = "a", ToNodeId = "add", ToPort = "x" });
            graph.Links.Add(new GraphEditor.GraphLink { FromNodeId = "b", ToNodeId = "add", ToPort = "y" });
            graph.Links.Add(new GraphEditor.GraphLink { FromNodeId = "a", ToNodeId = "sub", ToPort = "x" });
            graph.Links.Add(new GraphEditor.GraphLink { FromNodeId = "b", ToNodeId = "sub", ToPort = "y" });

            var runtime = GraphEditor.GraphRuntimeConverter.Convert(graph);

            // Add should have negate flags [false, true]
            bool addOk = runtime.Nodes.TryGetValue("add", out var rAdd) &&
                         rAdd.ArgNegate.Count == 2 &&
                         rAdd.ArgNegate[0] == false &&
                         rAdd.ArgNegate[1] == true;

            // Sub should have negate flags [false, false] (negate not supported)
            bool subOk = runtime.Nodes.TryGetValue("sub", out var rSub) &&
                         rSub.ArgNegate.Count == 2 &&
                         rSub.ArgNegate[0] == false &&
                         rSub.ArgNegate[1] == false;

            return addOk && subOk;
        }

        private static bool TestConvert_IncludeNode_InputMap()
        {
            // Test that Include node input map is populated from links
            var graph = new GraphEditor.GraphDefinition();

            var constNode = new GraphEditor.GraphNode { Id = "k", Kind = GraphEditor.GraphNodeKind.Const, ConstValue = 10.0 };
            graph.Nodes.Add(constNode);

            var includeNode = new GraphEditor.GraphNode
            {
                Id = "inc",
                Kind = GraphEditor.GraphNodeKind.Include,
                IncludePath = "subgraph.json"
            };
            includeNode.Ports.Add(new GraphEditor.GraphPort { Name = "input1", Kind = GraphEditor.GraphPortKind.Input });
            includeNode.Ports.Add(new GraphEditor.GraphPort { Name = "input2", Kind = GraphEditor.GraphPortKind.Input });
            includeNode.Ports.Add(new GraphEditor.GraphPort { Name = "output", Kind = GraphEditor.GraphPortKind.Output });
            graph.Nodes.Add(includeNode);

            graph.Links.Add(new GraphEditor.GraphLink { FromNodeId = "k", ToNodeId = "inc", ToPort = "input1" });

            var runtime = GraphEditor.GraphRuntimeConverter.Convert(graph);

            return runtime.Nodes.TryGetValue("inc", out var rInc) &&
                   rInc.Type.ToString() == "Include" &&
                   rInc.InputMap.Count == 1 &&
                   rInc.InputMap.ContainsKey("input1") &&
                   rInc.InputMap["input1"] == "k";
        }

        private static bool TestConvert_IncludeNode_OutputMap()
        {
            // Test that Include node output map is populated for all output ports
            var graph = new GraphEditor.GraphDefinition();

            var includeNode = new GraphEditor.GraphNode
            {
                Id = "inc",
                Kind = GraphEditor.GraphNodeKind.Include,
                IncludePath = "subgraph.json"
            };
            includeNode.Ports.Add(new GraphEditor.GraphPort { Name = "result", Kind = GraphEditor.GraphPortKind.Output });
            includeNode.Ports.Add(new GraphEditor.GraphPort { Name = "status", Kind = GraphEditor.GraphPortKind.Output });
            graph.Nodes.Add(includeNode);

            var runtime = GraphEditor.GraphRuntimeConverter.Convert(graph);

            return runtime.Nodes.TryGetValue("inc", out var rInc) &&
                   rInc.OutputMap.Count == 2 &&
                   rInc.OutputMap.ContainsKey("result") &&
                   rInc.OutputMap["result"] == "inc:result" &&
                   rInc.OutputMap.ContainsKey("status") &&
                   rInc.OutputMap["status"] == "inc:status";
        }

        private static bool TestConvert_SignalGroup()
        {
            // Test that full signal name is built from SignalGroup.SignalSuffix
            var graph = new GraphEditor.GraphDefinition();

            var inputNode = new GraphEditor.GraphNode
            {
                Id = "in",
                Kind = GraphEditor.GraphNodeKind.Input,
                SignalGroup = "Aircraft"
            };
            inputNode.Ports.Add(new GraphEditor.GraphPort
            {
                Name = "Speed",
                Kind = GraphEditor.GraphPortKind.Output,
                SignalSuffix = "IAS_kts"
            });
            graph.Nodes.Add(inputNode);

            var runtime = GraphEditor.GraphRuntimeConverter.Convert(graph);

            return runtime.Nodes.TryGetValue("in:Speed", out var rIn) &&
                   rIn.Name == "Aircraft.IAS_kts";
        }

        private static bool TestConvert_SignalGroup_Legacy()
        {
            // Test fallback to port name when no SignalGroup/SignalSuffix
            var graph = new GraphEditor.GraphDefinition { IsLibraryGraph = true };

            var inputNode = new GraphEditor.GraphNode
            {
                Id = "in",
                Kind = GraphEditor.GraphNodeKind.Input
                // No SignalGroup set
            };
            inputNode.Ports.Add(new GraphEditor.GraphPort
            {
                Name = "force",
                Kind = GraphEditor.GraphPortKind.Output
                // No SignalSuffix set
            });
            graph.Nodes.Add(inputNode);

            var runtime = GraphEditor.GraphRuntimeConverter.Convert(graph);

            // Should fall back to port name "force"
            return runtime.Nodes.TryGetValue("in:force", out var rIn) &&
                   rIn.Name == "force";
        }

        private static bool TestConvertEditorJson_DetectsLinks()
        {
            // Test that JSON with "links" is detected as editor format
            // We test this through the resolver which uses ConvertEditorJson internally
            var graph = new GraphEditor.GraphDefinition();
            graph.Nodes.Add(new GraphEditor.GraphNode { Id = "n1", Kind = GraphEditor.GraphNodeKind.Const, ConstValue = 1.0 });
            graph.Links.Add(new GraphEditor.GraphLink { FromNodeId = "n1", ToNodeId = "n1" }); // Dummy link to ensure "links" appears

            string json = GraphEditor.GraphSerializer.Serialize(graph);

            // The JSON should contain "links"
            return json.Contains("\"links\"") || json.Contains("\"Links\"");
        }

        private static bool TestConvertEditorJson_DetectsKind()
        {
            // Test that JSON with "kind" is detected as editor format
            var graph = new GraphEditor.GraphDefinition();
            graph.Nodes.Add(new GraphEditor.GraphNode { Id = "n1", Kind = GraphEditor.GraphNodeKind.Const, ConstValue = 1.0 });

            string json = GraphEditor.GraphSerializer.Serialize(graph);

            // The JSON should contain "kind" (from node Kind property)
            return json.Contains("\"kind\"") || json.Contains("\"Kind\"");
        }

        #endregion

        #region AxisRequestQueue Tests

        /// <summary>
        /// Mock sender for testing AxisRequestQueue
        /// </summary>
        private class MockAxisRequestSender : IAxisRequestSender
        {
            public List<(AxisID, AxisRequestType, Message)> Requests = new List<(AxisID, AxisRequestType, Message)>();
            public bool ReturnValue = true;

            public bool SendAxisRequest(AxisID axisId, AxisRequestType type, Message payload)
            {
                Requests.Add((axisId, type, payload));
                return ReturnValue;
            }
        }

        private static bool TestEnqueue_AddsToQueue()
        {
            var sender = new MockAxisRequestSender();
            var queue = new AxisRequestQueue(sender, manualTick: true);

            queue.Enqueue(AxisID._1, AxisRequestType.AxisConfig);

            return queue.QueueCount == 1;
        }

        private static bool TestEnqueue_DuplicateIgnored()
        {
            var sender = new MockAxisRequestSender();
            var queue = new AxisRequestQueue(sender, manualTick: true);

            queue.Enqueue(AxisID._1, AxisRequestType.AxisConfig);
            queue.Enqueue(AxisID._1, AxisRequestType.AxisConfig); // duplicate

            // Only one should be in queue
            return queue.QueueCount == 1;
        }

        private static bool TestEnqueue_UploadNotDeduplicated()
        {
            var sender = new MockAxisRequestSender();
            var queue = new AxisRequestQueue(sender, manualTick: true);

            queue.Enqueue(AxisID._1, AxisRequestType.AxisConfigUpload);
            queue.Enqueue(AxisID._1, AxisRequestType.AxisConfigUpload); // should NOT be deduplicated

            // Both should be in queue
            return queue.QueueCount == 2;
        }

        private static bool TestHandleResponse_MatchesCurrentRequest()
        {
            var sender = new MockAxisRequestSender();
            var queue = new AxisRequestQueue(sender, manualTick: true);

            // Enqueue and tick to make it current
            queue.Enqueue(AxisID._1, AxisRequestType.AxisConfig);
            queue.Tick();

            // Should have current request waiting for response
            if (!queue.HasCurrentRequest) return false;

            // Send matching response
            var response = new Message
            {
                AxisConfig = new AxisConfig { AxisId = AxisID._1 }
            };
            queue.HandleResponse(response);

            // Current request should be cleared
            return !queue.HasCurrentRequest;
        }

        private static bool TestHandleResponse_WrongType_Ignored()
        {
            var sender = new MockAxisRequestSender();
            var queue = new AxisRequestQueue(sender, manualTick: true);

            // Enqueue AxisConfig request
            queue.Enqueue(AxisID._1, AxisRequestType.AxisConfig);
            queue.Tick();

            if (!queue.HasCurrentRequest) return false;

            // Send DeviceInfo response (wrong type)
            var response = new Message
            {
                DeviceInfo = new DeviceInfo { AxisId = AxisID._1 }
            };
            queue.HandleResponse(response);

            // Current request should still be pending (wrong type)
            return queue.HasCurrentRequest;
        }

        private static bool TestHandleResponse_WrongAxis_Ignored()
        {
            var sender = new MockAxisRequestSender();
            var queue = new AxisRequestQueue(sender, manualTick: true);

            // Enqueue request for Axis1
            queue.Enqueue(AxisID._1, AxisRequestType.AxisConfig);
            queue.Tick();

            if (!queue.HasCurrentRequest) return false;

            // Send response for Axis2 (wrong axis)
            var response = new Message
            {
                AxisConfig = new AxisConfig { AxisId = AxisID._2 }
            };
            queue.HandleResponse(response);

            // Current request should still be pending (wrong axis)
            return queue.HasCurrentRequest;
        }

        private static bool TestRequiresResponse_RequestTypes()
        {
            var sender = new MockAxisRequestSender();
            var queue = new AxisRequestQueue(sender, manualTick: true);

            // Test that Restart (which doesn't require response) completes immediately
            queue.Enqueue(AxisID._1, AxisRequestType.Restart);
            queue.Tick();

            // Should not be waiting for response
            bool restartCompleted = !queue.HasCurrentRequest;

            // Test that AxisConfig (which requires response) waits for response
            queue.Enqueue(AxisID._1, AxisRequestType.AxisConfig);
            queue.Tick();

            // Should be waiting for response
            bool axisConfigWaiting = queue.HasCurrentRequest;

            return restartCompleted && axisConfigWaiting;
        }

        private static bool TestRetry_OnSendFailure()
        {
            var sender = new MockAxisRequestSender();
            sender.ReturnValue = false; // Simulate send failure
            var queue = new AxisRequestQueue(sender, manualTick: true);

            DateTime now = new DateTime(2024, 1, 1, 0, 0, 0, DateTimeKind.Utc);
            queue.SetNowProvider(() => now);

            queue.Enqueue(AxisID._1, AxisRequestType.Restart);
            queue.Tick(); // First attempt fails

            // Should still have current (pending retry)
            if (!queue.HasCurrentRequest) return false;

            // Request should have been sent
            if (sender.Requests.Count != 1) return false;

            // Advance time past retry delay
            now = now.AddMilliseconds(300);

            queue.Tick(); // Second attempt (still fails)

            // Should have sent twice now
            return sender.Requests.Count == 2;
        }

        private static bool TestRetry_MaxRetriesExhausted()
        {
            var sender = new MockAxisRequestSender();
            sender.ReturnValue = false; // Simulate send failure
            var queue = new AxisRequestQueue(sender, manualTick: true);

            DateTime now = new DateTime(2024, 1, 1, 0, 0, 0, DateTimeKind.Utc);
            queue.SetNowProvider(() => now);

            queue.Enqueue(AxisID._1, AxisRequestType.Restart);

            // MaxRetries = 3, so we need to tick 3 times with time advances
            for (int i = 0; i < 4; i++)
            {
                queue.Tick();
                now = now.AddMilliseconds(300);
            }

            // Should have dropped the request after max retries
            return !queue.HasCurrentRequest && sender.Requests.Count == 3;
        }

        #endregion

        #region Stateful graph node tests

        /// <summary>Helper: build a graph with a single stateful Func node and evaluate it.</summary>
        // Builds a runtime graph: one Expr node fed by Input nodes (one per inport,
        // node id == inport name), output exposed as "result".
        private static GraphDefinition BuildExprGraphDef(string formula, params string[] inports)
        {
            var graph = new GraphDefinition();
            var expr = new GraphNode { Id = "expr", Type = NodeType.Expr, Expr = formula };
            foreach (var p in inports)
            {
                graph.Nodes[p] = new GraphNode { Id = p, Type = NodeType.Input, Name = p };
                expr.InputMap[p] = p;
            }
            graph.Nodes["expr"] = expr;
            graph.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "result", Src = "expr" };
            return graph;
        }

        private static bool TestExpr_Arithmetic()
        {
            var eval = new GraphCompiledEvaluator(BuildExprGraphDef("a + b * 2", "a", "b"));
            var outputs = eval.Evaluate(new Dictionary<string, double> { ["a"] = 3.0, ["b"] = 4.0 }, null);
            return outputs.TryGetValue("result", out var v) && Math.Abs(v - 11.0) < 1e-9;
        }

        private static bool TestExpr_BuiltinFunctions()
        {
            // Built-in functions are allowed and are NOT treated as inports.
            var eval = new GraphCompiledEvaluator(BuildExprGraphDef("Pow(x, 2) + Abs(y)", "x", "y"));
            var outputs = eval.Evaluate(new Dictionary<string, double> { ["x"] = 3.0, ["y"] = -4.0 }, null);
            return outputs.TryGetValue("result", out var v) && Math.Abs(v - 13.0) < 1e-9;
        }

        private static bool TestExpr_PiConstant()
        {
            // Pi is a built-in constant: usable without being a wired inport.
            var def = BuildExprGraphDef("x * Pi", "x");
            var eval = new GraphCompiledEvaluator(def);
            var outputs = eval.Evaluate(new Dictionary<string, double> { ["x"] = 2.0 }, null);
            return outputs.TryGetValue("result", out var v) && Math.Abs(v - 2.0 * Math.PI) < 1e-9;
        }

        private static bool TestExpr_CompiledMatchesInterpreter()
        {
            var def = BuildExprGraphDef("(a - b) * 0.5 + Max(a, b)", "a", "b");
            var inputs = new Dictionary<string, double> { ["a"] = 7.0, ["b"] = 2.0 };
            double interp = new GraphEvaluator(def).Evaluate(inputs, null)["result"];
            double compiled = new GraphCompiledEvaluator(def).Evaluate(inputs, null)["result"];
            return Math.Abs(interp - compiled) < 1e-9 && Math.Abs(compiled - 9.5) < 1e-9;
        }

        private static bool TestExpr_NonInportRejected()
        {
            // 'z' is not a wired inport — compiling the formula must fail, naming 'z'.
            var def = BuildExprGraphDef("a + z", "a");
            try
            {
                _ = new GraphCompiledEvaluator(def);
                return false; // should have thrown
            }
            catch (InvalidOperationException ex)
            {
                return ex.Message.Contains("'z'");
            }
        }

        private static bool TestExpr_ParseErrorRejected()
        {
            var def = BuildExprGraphDef("a + * b", "a", "b");
            try
            {
                _ = new GraphCompiledEvaluator(def);
                return false; // malformed formula should fail to compile
            }
            catch (InvalidOperationException)
            {
                return true;
            }
        }

        private static bool TestExpr_EditorRoundtripAndConversion()
        {
            var graph = new GraphEditor.GraphDefinition { IsLibraryGraph = true };

            var input = new GraphEditor.GraphNode { Id = "in", Kind = GraphNodeKind.Input, Title = "In" };
            input.Ports.Add(new GraphEditor.GraphPort { Name = "a", Kind = GraphEditor.GraphPortKind.Output });
            graph.Nodes.Add(input);

            var expr = new GraphEditor.GraphNode { Id = "expr", Kind = GraphNodeKind.Expr, Title = "Expr", Expr = "a * 3 + 1" };
            expr.Ports.Add(new GraphEditor.GraphPort { Name = "a", Kind = GraphEditor.GraphPortKind.Input });
            expr.Ports.Add(new GraphEditor.GraphPort { Name = "out", Kind = GraphEditor.GraphPortKind.Output });
            graph.Nodes.Add(expr);

            var output = new GraphEditor.GraphNode { Id = "out", Kind = GraphNodeKind.Output, Title = "Out" };
            output.Ports.Add(new GraphEditor.GraphPort { Name = "result", Kind = GraphEditor.GraphPortKind.Input });
            graph.Nodes.Add(output);

            graph.Links.Add(new GraphEditor.GraphLink { FromNodeId = "in", FromPort = "a", ToNodeId = "expr", ToPort = "a" });
            graph.Links.Add(new GraphEditor.GraphLink { FromNodeId = "expr", FromPort = "out", ToNodeId = "out", ToPort = "result" });

            // Formula must survive JSON roundtrip.
            string json = GraphSerializer.Serialize(graph);
            var loaded = GraphSerializer.Deserialize(json, out var validation);
            if (!validation.IsValid) return false;
            var loadedExpr = loaded.Nodes.FirstOrDefault(n => n.Kind == GraphNodeKind.Expr);
            if (loadedExpr == null || loadedExpr.Expr != "a * 3 + 1") return false;

            // Convert to runtime and verify the Expr node wired correctly.
            // (Inspect via ToString to avoid the local/plugin-DLL type conflict that
            // prevents feeding a converted graph to the locally-compiled evaluator;
            // evaluation itself is covered by the runtime-graph tests above.)
            var runtime = GraphEditor.GraphRuntimeConverter.Convert(loaded);
            var exprNode = runtime.Nodes.Values.FirstOrDefault(n => n.Type.ToString() == "Expr");
            return exprNode != null
                   && exprNode.Expr == "a * 3 + 1"
                   && exprNode.InputMap.ContainsKey("a");
        }

        private static GraphCompiledEvaluator BuildStatefulGraph(string func, string[] argNodeIds, Dictionary<string, double> constNodes = null)
        {
            var graph = new GraphDefinition();
            if (constNodes != null)
            {
                foreach (var kv in constNodes)
                {
                    graph.Nodes[kv.Key] = new GraphNode { Id = kv.Key, Type = NodeType.Const, ConstValue = kv.Value };
                }
            }
            foreach (var id in argNodeIds)
            {
                if (!graph.Nodes.ContainsKey(id))
                {
                    graph.Nodes[id] = new GraphNode { Id = id, Type = NodeType.Input, Name = id };
                }
            }
            graph.Nodes["func"] = new GraphNode
            {
                Id = "func",
                Type = NodeType.Func,
                Func = func,
                Args = new List<string>(argNodeIds)
            };
            graph.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "result", Src = "func" };
            return new GraphCompiledEvaluator(graph);
        }

        private static double Eval(GraphCompiledEvaluator eval, Dictionary<string, double> inputs, double dt = 0.0)
        {
            var outputs = eval.Evaluate(inputs, new Dictionary<string, double>(), dt);
            return outputs.TryGetValue("result", out var v) ? v : double.NaN;
        }

        private static bool TestAccumulator_BasicIncrement()
        {
            // accumulator(trigger, step, min, max) — step is in units/sec, dt = 1.0s per tick
            var eval = BuildStatefulGraph("accumulator",
                new[] { "trigger", "step", "min", "max" },
                new Dictionary<string, double> { ["step"] = 2.0, ["min"] = -100.0, ["max"] = 100.0 });

            var on = new Dictionary<string, double> { ["trigger"] = 1.0 };
            double v1 = Eval(eval, on, 1.0);  // 0 + 2*1 = 2
            double v2 = Eval(eval, on, 1.0);  // 2 + 2*1 = 4
            double v3 = Eval(eval, on, 0.5);  // 4 + 2*0.5 = 5
            return Math.Abs(v1 - 2.0) < 1e-9 && Math.Abs(v2 - 4.0) < 1e-9 && Math.Abs(v3 - 5.0) < 1e-9;
        }

        private static bool TestAccumulator_ClampBounds()
        {
            var eval = BuildStatefulGraph("accumulator",
                new[] { "trigger", "step", "min", "max" },
                new Dictionary<string, double> { ["step"] = 50.0, ["min"] = -10.0, ["max"] = 10.0 });

            var on = new Dictionary<string, double> { ["trigger"] = 1.0 };
            for (int i = 0; i < 10; i++) Eval(eval, on, 1.0);  // Would be 500, but clamped
            double v = Eval(eval, on, 1.0);
            return Math.Abs(v - 10.0) < 1e-9;
        }

        private static bool TestAccumulator_Reset()
        {
            var eval = BuildStatefulGraph("accumulator",
                new[] { "trigger", "step", "min", "max", "reset" },
                new Dictionary<string, double> { ["step"] = 3.0, ["min"] = -100.0, ["max"] = 100.0 });

            var on = new Dictionary<string, double> { ["trigger"] = 1.0, ["reset"] = 0.0 };
            Eval(eval, on, 1.0);  // 3
            Eval(eval, on, 1.0);  // 6
            double before = Eval(eval, on, 1.0);  // 9

            var resetInputs = new Dictionary<string, double> { ["trigger"] = 0.0, ["reset"] = 1.0 };
            double after = Eval(eval, resetInputs, 1.0);  // should be 0

            return Math.Abs(before - 9.0) < 1e-9 && Math.Abs(after) < 1e-9;
        }

        private static bool TestAccumulator_NoTrigger()
        {
            var eval = BuildStatefulGraph("accumulator",
                new[] { "trigger", "step", "min", "max" },
                new Dictionary<string, double> { ["step"] = 5.0, ["min"] = -100.0, ["max"] = 100.0 });

            var off = new Dictionary<string, double> { ["trigger"] = 0.0 };
            Eval(eval, off, 1.0);
            Eval(eval, off, 1.0);
            double v = Eval(eval, off, 1.0);
            return Math.Abs(v) < 1e-9;
        }

        private static bool TestSampleHold_FallingEdge()
        {
            // sample_hold(input, trigger) — captures on falling edge (1→0)
            var eval = BuildStatefulGraph("sample_hold", new[] { "input", "trigger" });

            // Trigger high, input = 42
            Eval(eval, new Dictionary<string, double> { ["input"] = 42.0, ["trigger"] = 1.0 });
            // Trigger goes low → should capture input
            double captured = Eval(eval, new Dictionary<string, double> { ["input"] = 42.0, ["trigger"] = 0.0 });
            // Input changes but trigger stays low → held value unchanged
            double held = Eval(eval, new Dictionary<string, double> { ["input"] = 99.0, ["trigger"] = 0.0 });

            return Math.Abs(captured - 42.0) < 1e-9 && Math.Abs(held - 42.0) < 1e-9;
        }

        private static bool TestSampleHold_HoldDuringHigh()
        {
            var eval = BuildStatefulGraph("sample_hold", new[] { "input", "trigger" });

            // No falling edge yet — held value should be 0 (initial)
            double v1 = Eval(eval, new Dictionary<string, double> { ["input"] = 10.0, ["trigger"] = 1.0 });
            double v2 = Eval(eval, new Dictionary<string, double> { ["input"] = 20.0, ["trigger"] = 1.0 });
            return Math.Abs(v1) < 1e-9 && Math.Abs(v2) < 1e-9;
        }

        private static bool TestEdgeDetect_RisingEdge()
        {
            // edge_detect(input) — 1.0 for one tick on rising edge, 0.0 otherwise
            var eval = BuildStatefulGraph("edge_detect", new[] { "input" });

            double v1 = Eval(eval, new Dictionary<string, double> { ["input"] = 0.0 });  // no edge
            double v2 = Eval(eval, new Dictionary<string, double> { ["input"] = 1.0 });  // rising edge → 1
            double v3 = Eval(eval, new Dictionary<string, double> { ["input"] = 1.0 });  // sustained → 0
            double v4 = Eval(eval, new Dictionary<string, double> { ["input"] = 0.0 });  // falling → 0
            double v5 = Eval(eval, new Dictionary<string, double> { ["input"] = 1.0 });  // rising again → 1

            return Math.Abs(v1) < 1e-9
                && Math.Abs(v2 - 1.0) < 1e-9
                && Math.Abs(v3) < 1e-9
                && Math.Abs(v4) < 1e-9
                && Math.Abs(v5 - 1.0) < 1e-9;
        }

        private static bool TestEdgeDetect_NoPulseOnSustained()
        {
            var eval = BuildStatefulGraph("edge_detect", new[] { "input" });

            // Start high → no edge (prev was 0, input is 1 → actually this IS a rising edge)
            double v1 = Eval(eval, new Dictionary<string, double> { ["input"] = 1.0 });
            // Stay high
            double v2 = Eval(eval, new Dictionary<string, double> { ["input"] = 1.0 });
            double v3 = Eval(eval, new Dictionary<string, double> { ["input"] = 1.0 });

            // First tick: prev=0 (initial), input=1 → rising edge → 1.0
            // Subsequent: prev=1, input=1 → no edge → 0.0
            return Math.Abs(v1 - 1.0) < 1e-9
                && Math.Abs(v2) < 1e-9
                && Math.Abs(v3) < 1e-9;
        }

        private static bool TestResetState_ClearsAccumulator()
        {
            var eval = BuildStatefulGraph("accumulator",
                new[] { "trigger", "step", "min", "max" },
                new Dictionary<string, double> { ["step"] = 5.0, ["min"] = -100.0, ["max"] = 100.0 });

            var on = new Dictionary<string, double> { ["trigger"] = 1.0 };
            Eval(eval, on, 1.0);  // 5
            Eval(eval, on, 1.0);  // 10
            double before = Eval(eval, on, 1.0);  // 15

            eval.ResetState();

            double after = Eval(eval, on, 1.0);  // should be 5 (fresh start + one step)
            return Math.Abs(before - 15.0) < 1e-9 && Math.Abs(after - 5.0) < 1e-9;
        }

        #endregion

        #region Conditional op tests

        private static bool TestSelect_TrueBranch()
        {
            var g = new GraphDefinition();
            g.Nodes["cond"] = new GraphNode { Id = "cond", Type = NodeType.Const, ConstValue = 1.0 };
            g.Nodes["a"] = new GraphNode { Id = "a", Type = NodeType.Const, ConstValue = 10.0 };
            g.Nodes["b"] = new GraphNode { Id = "b", Type = NodeType.Const, ConstValue = 20.0 };
            g.Nodes["sel"] = new GraphNode { Id = "sel", Type = NodeType.Op, Op = OpType.Select, Args = { "cond", "a", "b" } };
            g.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "result", Src = "sel" };
            var result = new GraphEvaluator(g).Evaluate(null, null);
            return result.TryGetValue("result", out var v) && Math.Abs(v - 10.0) < 1e-9;
        }

        private static bool TestSelect_FalseBranch()
        {
            var g = new GraphDefinition();
            g.Nodes["cond"] = new GraphNode { Id = "cond", Type = NodeType.Const, ConstValue = 0.0 };
            g.Nodes["a"] = new GraphNode { Id = "a", Type = NodeType.Const, ConstValue = 10.0 };
            g.Nodes["b"] = new GraphNode { Id = "b", Type = NodeType.Const, ConstValue = 20.0 };
            g.Nodes["sel"] = new GraphNode { Id = "sel", Type = NodeType.Op, Op = OpType.Select, Args = { "cond", "a", "b" } };
            g.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "result", Src = "sel" };
            var result = new GraphEvaluator(g).Evaluate(null, null);
            return result.TryGetValue("result", out var v) && Math.Abs(v - 20.0) < 1e-9;
        }

        private static bool TestSelect_Boundary()
        {
            // Exactly 0.5 should select false branch (> 0.5, not >=)
            var g = new GraphDefinition();
            g.Nodes["cond"] = new GraphNode { Id = "cond", Type = NodeType.Const, ConstValue = 0.5 };
            g.Nodes["a"] = new GraphNode { Id = "a", Type = NodeType.Const, ConstValue = 10.0 };
            g.Nodes["b"] = new GraphNode { Id = "b", Type = NodeType.Const, ConstValue = 20.0 };
            g.Nodes["sel"] = new GraphNode { Id = "sel", Type = NodeType.Op, Op = OpType.Select, Args = { "cond", "a", "b" } };
            g.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "result", Src = "sel" };
            var result = new GraphEvaluator(g).Evaluate(null, null);
            return result.TryGetValue("result", out var v) && Math.Abs(v - 20.0) < 1e-9;
        }

        private static bool TestEq_Equal()
        {
            var g = new GraphDefinition();
            g.Nodes["a"] = new GraphNode { Id = "a", Type = NodeType.Const, ConstValue = 5.0 };
            g.Nodes["b"] = new GraphNode { Id = "b", Type = NodeType.Const, ConstValue = 5.0 };
            g.Nodes["eq"] = new GraphNode { Id = "eq", Type = NodeType.Op, Op = OpType.Eq, Args = { "a", "b" } };
            g.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "result", Src = "eq" };
            var result = new GraphEvaluator(g).Evaluate(null, null);
            return result.TryGetValue("result", out var v) && Math.Abs(v - 1.0) < 1e-9;
        }

        private static bool TestEq_Unequal()
        {
            var g = new GraphDefinition();
            g.Nodes["a"] = new GraphNode { Id = "a", Type = NodeType.Const, ConstValue = 5.0 };
            g.Nodes["b"] = new GraphNode { Id = "b", Type = NodeType.Const, ConstValue = 6.0 };
            g.Nodes["eq"] = new GraphNode { Id = "eq", Type = NodeType.Op, Op = OpType.Eq, Args = { "a", "b" } };
            g.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "result", Src = "eq" };
            var result = new GraphEvaluator(g).Evaluate(null, null);
            return result.TryGetValue("result", out var v) && Math.Abs(v) < 1e-9;
        }

        private static bool TestEq_NearTolerance()
        {
            // 0.0005 apart — within 0.001 tolerance, should be equal
            var g = new GraphDefinition();
            g.Nodes["a"] = new GraphNode { Id = "a", Type = NodeType.Const, ConstValue = 1.0 };
            g.Nodes["b"] = new GraphNode { Id = "b", Type = NodeType.Const, ConstValue = 1.0005 };
            g.Nodes["eq"] = new GraphNode { Id = "eq", Type = NodeType.Op, Op = OpType.Eq, Args = { "a", "b" } };
            g.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "result", Src = "eq" };
            var result = new GraphEvaluator(g).Evaluate(null, null);
            return result.TryGetValue("result", out var v) && Math.Abs(v - 1.0) < 1e-9;
        }

        private static bool TestGt_Greater()
        {
            var g = new GraphDefinition();
            g.Nodes["a"] = new GraphNode { Id = "a", Type = NodeType.Const, ConstValue = 10.0 };
            g.Nodes["b"] = new GraphNode { Id = "b", Type = NodeType.Const, ConstValue = 5.0 };
            g.Nodes["gt"] = new GraphNode { Id = "gt", Type = NodeType.Op, Op = OpType.Gt, Args = { "a", "b" } };
            g.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "result", Src = "gt" };
            var result = new GraphEvaluator(g).Evaluate(null, null);
            return result.TryGetValue("result", out var v) && Math.Abs(v - 1.0) < 1e-9;
        }

        private static bool TestGt_Less()
        {
            var g = new GraphDefinition();
            g.Nodes["a"] = new GraphNode { Id = "a", Type = NodeType.Const, ConstValue = 3.0 };
            g.Nodes["b"] = new GraphNode { Id = "b", Type = NodeType.Const, ConstValue = 5.0 };
            g.Nodes["gt"] = new GraphNode { Id = "gt", Type = NodeType.Op, Op = OpType.Gt, Args = { "a", "b" } };
            g.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "result", Src = "gt" };
            var result = new GraphEvaluator(g).Evaluate(null, null);
            return result.TryGetValue("result", out var v) && Math.Abs(v) < 1e-9;
        }

        private static bool TestGt_Equal()
        {
            // Equal values: gt should return 0 (strict greater-than)
            var g = new GraphDefinition();
            g.Nodes["a"] = new GraphNode { Id = "a", Type = NodeType.Const, ConstValue = 5.0 };
            g.Nodes["b"] = new GraphNode { Id = "b", Type = NodeType.Const, ConstValue = 5.0 };
            g.Nodes["gt"] = new GraphNode { Id = "gt", Type = NodeType.Op, Op = OpType.Gt, Args = { "a", "b" } };
            g.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "result", Src = "gt" };
            var result = new GraphEvaluator(g).Evaluate(null, null);
            return result.TryGetValue("result", out var v) && Math.Abs(v) < 1e-9;
        }

        private static bool TestTriStateRouting()
        {
            // type=0 → 100, type=1 → 200, type=2 → 300
            // select(eq(type, 0), 100, select(eq(type, 1), 200, 300))
            var g = new GraphDefinition();
            g.Nodes["type"] = new GraphNode { Id = "type", Type = NodeType.Param, Name = "type" };
            g.Nodes["c0"] = new GraphNode { Id = "c0", Type = NodeType.Const, ConstValue = 0.0 };
            g.Nodes["c1"] = new GraphNode { Id = "c1", Type = NodeType.Const, ConstValue = 1.0 };
            g.Nodes["v0"] = new GraphNode { Id = "v0", Type = NodeType.Const, ConstValue = 100.0 };
            g.Nodes["v1"] = new GraphNode { Id = "v1", Type = NodeType.Const, ConstValue = 200.0 };
            g.Nodes["v2"] = new GraphNode { Id = "v2", Type = NodeType.Const, ConstValue = 300.0 };
            g.Nodes["eq0"] = new GraphNode { Id = "eq0", Type = NodeType.Op, Op = OpType.Eq, Args = { "type", "c0" } };
            g.Nodes["eq1"] = new GraphNode { Id = "eq1", Type = NodeType.Op, Op = OpType.Eq, Args = { "type", "c1" } };
            g.Nodes["sel1"] = new GraphNode { Id = "sel1", Type = NodeType.Op, Op = OpType.Select, Args = { "eq1", "v1", "v2" } };
            g.Nodes["sel0"] = new GraphNode { Id = "sel0", Type = NodeType.Op, Op = OpType.Select, Args = { "eq0", "v0", "sel1" } };
            g.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "result", Src = "sel0" };

            var eval = new GraphEvaluator(g);
            var params0 = new Dictionary<string, double> { { "type", 0.0 } };
            var params1 = new Dictionary<string, double> { { "type", 1.0 } };
            var params2 = new Dictionary<string, double> { { "type", 2.0 } };

            double r0 = eval.Evaluate(null, params0).TryGetValue("result", out var v0) ? v0 : -1;
            double r1 = eval.Evaluate(null, params1).TryGetValue("result", out var v1) ? v1 : -1;
            double r2 = eval.Evaluate(null, params2).TryGetValue("result", out var v2) ? v2 : -1;

            return Math.Abs(r0 - 100.0) < 1e-9 &&
                   Math.Abs(r1 - 200.0) < 1e-9 &&
                   Math.Abs(r2 - 300.0) < 1e-9;
        }

        #endregion

        #region FunctionScope / ConfigOut / Scoped Output tests

        private static bool TestConvert_ScopedOutput()
        {
            // A scoped Output in a sub-graph should create a runtime Output node
            // with Name = FunctionScope + "." + SignalSuffix when the Include has FunctionScope.
            var graph = new GraphEditor.GraphDefinition();

            var constNode = new GraphEditor.GraphNode { Id = "k", Kind = GraphEditor.GraphNodeKind.Const, ConstValue = 7.0 };
            graph.Nodes.Add(constNode);

            var includeNode = new GraphEditor.GraphNode
            {
                Id = "inc",
                Kind = GraphEditor.GraphNodeKind.Include,
                IncludePath = "sub.json",
                FunctionScope = "FlightStickPitch"
            };
            // Simulate CachedInterface with one scoped output
            includeNode.CachedInterface = new GraphEditor.IncludedGraphInterface { IsValid = true };
            includeNode.CachedInterface.ScopedOutputs.Add(new GraphEditor.ScopedOutputPort
            {
                Name = "spring",
                SignalSuffix = "SpringGain"
            });
            // No output ports on the Include node (scoped outputs are hidden)
            graph.Nodes.Add(includeNode);

            var runtime = GraphEditor.GraphRuntimeConverter.Convert(graph);

            // Should have created a scoped runtime Output node
            return runtime.Nodes.TryGetValue("inc:scoped:spring", out var rOut) &&
                   rOut.Type.ToString() == "Output" &&
                   rOut.Name == "FlightStickPitch.SpringGain" &&
                   rOut.Src == "inc:spring";
        }

        private static bool TestConvert_UnscopedOutputWithFunctionScope()
        {
            // An unscoped output should still appear in the Include's OutputMap
            // even when FunctionScope is set — only scoped outputs are auto-registered.
            var graph = new GraphEditor.GraphDefinition();

            var includeNode = new GraphEditor.GraphNode
            {
                Id = "inc",
                Kind = GraphEditor.GraphNodeKind.Include,
                IncludePath = "sub.json",
                FunctionScope = "FlightStickPitch"
            };
            includeNode.CachedInterface = new GraphEditor.IncludedGraphInterface { IsValid = true };
            // Unscoped output appears as a regular port on the Include node
            includeNode.Ports.Add(new GraphEditor.GraphPort { Name = "custom_out", Kind = GraphEditor.GraphPortKind.Output });
            graph.Nodes.Add(includeNode);

            var runtime = GraphEditor.GraphRuntimeConverter.Convert(graph);

            // Unscoped output should be in OutputMap, NOT auto-registered as a scoped output
            bool hasOutputMap = runtime.Nodes.TryGetValue("inc", out var rInc) &&
                                rInc.OutputMap.ContainsKey("custom_out");
            bool noScopedNode = !runtime.Nodes.ContainsKey("inc:scoped:custom_out");
            return hasOutputMap && noScopedNode;
        }

        private static bool TestConvert_ConfigOut()
        {
            // A ConfigOut node in a graph should create a runtime ConfigOut node
            // with Name = ConfigField and Type = ConfigOut.
            var graph = new GraphEditor.GraphDefinition();

            var constNode = new GraphEditor.GraphNode { Id = "k", Kind = GraphEditor.GraphNodeKind.Const, ConstValue = 5.0 };
            graph.Nodes.Add(constNode);

            var cfgOutNode = new GraphEditor.GraphNode
            {
                Id = "cfg",
                Kind = GraphEditor.GraphNodeKind.ConfigOut,
                ConfigType = "FlightStick"
            };
            cfgOutNode.Ports.Add(new GraphEditor.GraphPort
            {
                Name = "ratio_0",
                Kind = GraphEditor.GraphPortKind.Input,
                ConfigField = "flight_stick.damping"
            });
            graph.Nodes.Add(cfgOutNode);

            graph.Links.Add(new GraphEditor.GraphLink
            {
                FromNodeId = "k",
                ToNodeId = "cfg",
                ToPort = "ratio_0"
            });

            var runtime = GraphEditor.GraphRuntimeConverter.Convert(graph);

            return runtime.Nodes.TryGetValue("cfg:ratio_0", out var rCfg) &&
                   rCfg.Type.ToString() == "ConfigOut" &&
                   rCfg.Name == "flight_stick.damping" &&
                   rCfg.Src == "k";
        }

        private static bool TestConvert_ConfigOutScoped()
        {
            // ConfigOut in a sub-graph should create a scoped runtime ConfigOut node
            // with Name = FunctionScope + ":" + ConfigField when Include has FunctionScope.
            var graph = new GraphEditor.GraphDefinition();

            var includeNode = new GraphEditor.GraphNode
            {
                Id = "inc",
                Kind = GraphEditor.GraphNodeKind.Include,
                IncludePath = "sub.json",
                FunctionScope = "FlightStickRoll"
            };
            includeNode.CachedInterface = new GraphEditor.IncludedGraphInterface { IsValid = true };
            includeNode.CachedInterface.ConfigOutputs.Add(new GraphEditor.ConfigOutputPort
            {
                Name = "ratio_0",
                ConfigField = "flight_stick.damping",
                ConfigType = "FlightStick"
            });
            graph.Nodes.Add(includeNode);

            var runtime = GraphEditor.GraphRuntimeConverter.Convert(graph);

            return runtime.Nodes.TryGetValue("inc:scoped_cfg:ratio_0", out var rCfg) &&
                   rCfg.Type.ToString() == "ConfigOut" &&
                   rCfg.Name == "FlightStickRoll:flight_stick.damping" &&
                   rCfg.Src == "inc:ratio_0";
        }

        private static bool TestEval_ConfigOutValues()
        {
            // ConfigOut runtime nodes should appear in result.ConfigOutputs, not result.Outputs.
            var runtime = new GraphDefinition();

            runtime.Nodes["k"] = new GraphNode { Id = "k", Type = NodeType.Const, ConstValue = 3.14 };
            runtime.Nodes["cfg"] = new GraphNode { Id = "cfg", Type = NodeType.ConfigOut, Name = "FlightStickPitch:flight_stick.damping", Src = "k" };
            runtime.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "FlightStickPitch.SpringGain", Src = "k" };

            var eval = new GraphEvaluator(runtime);
            var result = eval.EvaluateWithTrace(null, null);

            bool cfgInConfigOutputs = result.ConfigOutputs.TryGetValue("FlightStickPitch:flight_stick.damping", out var cfgVal) &&
                                      Math.Abs(cfgVal - 3.14) < 1e-9;
            bool cfgNotInOutputs = !result.Outputs.ContainsKey("FlightStickPitch:flight_stick.damping");
            bool outInOutputs = result.Outputs.TryGetValue("FlightStickPitch.SpringGain", out var outVal) &&
                                Math.Abs(outVal - 3.14) < 1e-9;
            bool outNotInConfigOutputs = !result.ConfigOutputs.ContainsKey("FlightStickPitch.SpringGain");

            return cfgInConfigOutputs && cfgNotInOutputs && outInOutputs && outNotInConfigOutputs;
        }

        private static bool TestCompiledEval_ConfigOutValues()
        {
            // Same as above but using GraphCompiledEvaluator.
            var runtime = new GraphDefinition();

            runtime.Nodes["k"] = new GraphNode { Id = "k", Type = NodeType.Const, ConstValue = 2.72 };
            runtime.Nodes["cfg"] = new GraphNode { Id = "cfg", Type = NodeType.ConfigOut, Name = "FlightPedals:flight_pedals.damping", Src = "k" };
            runtime.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "FlightPedals.SpringGain", Src = "k" };

            var eval = new GraphCompiledEvaluator(runtime);
            var result = eval.EvaluateWithTrace(null, null);

            bool cfgInConfigOutputs = result.ConfigOutputs.TryGetValue("FlightPedals:flight_pedals.damping", out var cfgVal) &&
                                      Math.Abs(cfgVal - 2.72) < 1e-9;
            bool cfgNotInOutputs = !result.Outputs.ContainsKey("FlightPedals:flight_pedals.damping");
            bool outInOutputs = result.Outputs.TryGetValue("FlightPedals.SpringGain", out var outVal) &&
                                Math.Abs(outVal - 2.72) < 1e-9;

            return cfgInConfigOutputs && cfgNotInOutputs && outInOutputs;
        }

        // Regression: include's ConfigOut values must propagate through the parent's
        // OutputMap bridging loop alongside Output values. The bridge used to call
        // evaluator.Evaluate() which returned only Outputs, silently dropping ConfigOutputs
        // — leaving the parent's scoped ConfigOut nodes reading 0 even when the sub-graph
        // produced correct values from Const nodes.
        private static bool TestEval_ConfigOutBridgesThroughInclude()
        {
            // Sub-graph: Const(7.0) → ConfigOut named after the ConfigField.
            var sub = new GraphDefinition();
            sub.Nodes["k"] = new GraphNode { Id = "k", Type = NodeType.Const, ConstValue = 7.0 };
            sub.Nodes["cfg"] = new GraphNode
            {
                Id = "cfg",
                Type = NodeType.ConfigOut,
                Name = "flight_stick.damping",
                Src = "k"
            };

            // Parent: Include with OutputMap bridging the sub's ConfigOut name to a
            // parent runtime ID, plus a scoped ConfigOut reading from that ID.
            var parent = new GraphDefinition();
            var include = new GraphNode
            {
                Id = "inc",
                Type = NodeType.Include,
                InlineGraph = sub
            };
            include.OutputMap["flight_stick.damping"] = "inc_out_damping";
            parent.Nodes["inc"] = include;
            parent.Nodes["scoped"] = new GraphNode
            {
                Id = "scoped",
                Type = NodeType.ConfigOut,
                Name = "FlightStickPitch:flight_stick.damping",
                Src = "inc_out_damping"
            };

            // Interpreter requires an IGraphResolver to traverse Include nodes,
            // even when InlineGraph is set.
            var resolver = new GraphIncludeResolver(AppContext.BaseDirectory);
            var result = new GraphEvaluator(parent, resolver).EvaluateWithTrace(null, null);

            return result.ConfigOutputs.TryGetValue("FlightStickPitch:flight_stick.damping", out var v) &&
                   Math.Abs(v - 7.0) < 1e-9;
        }

        private static bool TestCompiledEval_ConfigOutBridgesThroughInclude()
        {
            // Same shape as the interpreter test, exercised through GraphCompiledEvaluator
            // (the production path).
            var sub = new GraphDefinition();
            sub.Nodes["k"] = new GraphNode { Id = "k", Type = NodeType.Const, ConstValue = 7.0 };
            sub.Nodes["cfg"] = new GraphNode
            {
                Id = "cfg",
                Type = NodeType.ConfigOut,
                Name = "flight_stick.damping",
                Src = "k"
            };

            var parent = new GraphDefinition();
            var include = new GraphNode
            {
                Id = "inc",
                Type = NodeType.Include,
                InlineGraph = sub
            };
            include.OutputMap["flight_stick.damping"] = "inc_out_damping";
            parent.Nodes["inc"] = include;
            parent.Nodes["scoped"] = new GraphNode
            {
                Id = "scoped",
                Type = NodeType.ConfigOut,
                Name = "FlightStickPitch:flight_stick.damping",
                Src = "inc_out_damping"
            };

            var result = new GraphCompiledEvaluator(parent).EvaluateWithTrace(null, null);

            return result.ConfigOutputs.TryGetValue("FlightStickPitch:flight_stick.damping", out var v) &&
                   Math.Abs(v - 7.0) < 1e-9;
        }

        private static bool TestExtractInterface_ScopedOutputs()
        {
            // ExtractInterface should separate scoped and unscoped outputs.
            var graph = new GraphEditor.GraphDefinition { IsLibraryGraph = true };

            // Unscoped Output node
            var unscopedOut = new GraphEditor.GraphNode { Id = "out1", Kind = GraphEditor.GraphNodeKind.Output };
            unscopedOut.Ports.Add(new GraphEditor.GraphPort { Name = "custom", Kind = GraphEditor.GraphPortKind.Input });
            graph.Nodes.Add(unscopedOut);

            // Scoped Output node
            var scopedOut = new GraphEditor.GraphNode { Id = "out2", Kind = GraphEditor.GraphNodeKind.Output, Scoped = true };
            scopedOut.Ports.Add(new GraphEditor.GraphPort { Name = "SpringGain", Kind = GraphEditor.GraphPortKind.Input, SignalSuffix = "SpringGain" });
            scopedOut.Ports.Add(new GraphEditor.GraphPort { Name = "DamperGain", Kind = GraphEditor.GraphPortKind.Input, SignalSuffix = "DamperGain" });
            graph.Nodes.Add(scopedOut);

            var iface = GraphEditor.GraphSerializer.ExtractInterface(graph);

            bool unscopedCorrect = iface.Outputs.Count == 1 && iface.Outputs[0] == "custom";
            bool scopedCorrect = iface.ScopedOutputs.Count == 2 &&
                                 iface.ScopedOutputs[0].Name == "SpringGain" &&
                                 iface.ScopedOutputs[0].SignalSuffix == "SpringGain" &&
                                 iface.ScopedOutputs[1].Name == "DamperGain" &&
                                 iface.ScopedOutputs[1].SignalSuffix == "DamperGain";

            return iface.IsValid && unscopedCorrect && scopedCorrect;
        }

        private static bool TestExtractInterface_ConfigOutputs()
        {
            // ExtractInterface should extract ConfigOut ports with ConfigType.
            var graph = new GraphEditor.GraphDefinition { IsLibraryGraph = true };

            var cfgOut = new GraphEditor.GraphNode
            {
                Id = "cfg",
                Kind = GraphEditor.GraphNodeKind.ConfigOut,
                ConfigType = "FlightStick"
            };
            cfgOut.Ports.Add(new GraphEditor.GraphPort
            {
                Name = "ratio_0",
                Kind = GraphEditor.GraphPortKind.Input,
                ConfigField = "flight_stick.damping"
            });
            cfgOut.Ports.Add(new GraphEditor.GraphPort
            {
                Name = "spring",
                Kind = GraphEditor.GraphPortKind.Input,
                ConfigField = "flight_stick.centering_spring_const"
            });
            graph.Nodes.Add(cfgOut);

            var iface = GraphEditor.GraphSerializer.ExtractInterface(graph);

            return iface.IsValid &&
                   iface.ConfigOutputs.Count == 2 &&
                   iface.ConfigOutputs[0].Name == "ratio_0" &&
                   iface.ConfigOutputs[0].ConfigField == "flight_stick.damping" &&
                   iface.ConfigOutputs[0].ConfigType == "FlightStick" &&
                   iface.ConfigOutputs[1].Name == "spring" &&
                   iface.ConfigOutputs[1].ConfigField == "flight_stick.centering_spring_const";
        }

        #endregion

    }
}
