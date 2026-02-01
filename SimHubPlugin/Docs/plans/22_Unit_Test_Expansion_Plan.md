# Unit Test Expansion Plan

Date: 2026-01-31
Status: Complete (All Phases)

## Goal

Expand unit test coverage from the current 77 tests to cover critical untested components, prioritizing pure logic classes that are easy to test and high-value for catching regressions.

## Current State

**Test infrastructure:** Custom lightweight test runner (`TestCommon/TestRunner.cs`)
**Test location:** `SimHubPlugin/GraphTest/GraphTestRunner.cs`
**Current coverage:** 77 tests covering graph runtime, parameter migration, serialization roundtrips

### Well-Tested Areas
- Graph runtime evaluation (30+ tests)
- Parameter migration and resolution (8+ tests)
- Graph serialization/deserialization roundtrips
- Include validation, cyclic detection
- Graph hash computation

### Major Gaps
| Component | Lines | Testability | Priority |
|-----------|-------|-------------|----------|
| GeneralKinematics | 1,382 | High (pure math) | High |
| CubicSpline | 111 | High (pure function) | High |
| Tools utilities | 124 | High (pure functions) | High |
| GraphRuntimeConverter | 278 | High (no UI deps) | Medium |
| AxisRequestQueue | 226 | Medium (needs mock) | Medium |

## Implementation Phases

### Phase 1: Pure Function Tests (Quick Wins)

#### 1.1 Tools.cs Tests

| Test | Description |
|------|-------------|
| `TestNormalize_InRange` | Value between min/max returns normalized 0-1 |
| `TestNormalize_BelowMin` | Value ≤ min returns 0 |
| `TestNormalize_AboveMax` | Value ≥ max returns 1 |
| `TestNormalize_ZeroRange` | min ≈ max returns 0 |
| `TestTryComputeMarkerX_Valid` | Returns true with correct x position |
| `TestTryComputeMarkerX_ZeroWidth` | Returns false |
| `TestTryComputeMarkerX_SwappedMinMax` | Handles min > max |
| `TestTryAutoTuneLoadGain_BelowMinForce` | Returns false, gain unchanged |
| `TestTryAutoTuneLoadGain_RatioHigh` | Decreases gain |
| `TestTryAutoTuneLoadGain_RatioLow` | Increases gain |
| `TestTryAutoTuneLoadGain_InRange` | Returns false, no change |

#### 1.2 CubicSpline Tests

| Test | Description |
|------|-------------|
| `TestInterpolate_LinearData` | Linear points produce linear interpolation |
| `TestInterpolate_EndpointMatch` | Interpolated endpoints match original |
| `TestInterpolate_Monotonic` | Monotonic input produces smooth output |
| `TestInterpolate1D_CountMatches` | Output array has requested count |
| `TestInterpolate_MismatchedArrays` | Throws ArgumentException |

#### 1.3 StringExtensions Tests

| Test | Description |
|------|-------------|
| `TestConstCaseToTitleCase` | "HELLO_WORLD" → "Hello World" |
| `TestCamelCaseToTitleCase` | "helloWorld" → "hello World" |

### Phase 2: GeneralKinematics Tests

The kinematics module is critical for FFB accuracy. Testing strategy:

#### 2.1 Input Validation Tests

| Test | Description |
|------|-------------|
| `TestCalcKinematicParameters_NullConfig` | Throws ArgumentNullException |
| `TestCalcKinematicParameters_NoPins` | Throws ArgumentException |
| `TestCalcKinematicParameters_NoBars` | Throws ArgumentException |
| `TestCalcKinematicParameters_NegativeTravel` | Throws ArgumentException |

#### 2.2 Known Geometry Tests

Create test fixtures with known geometries and expected outputs:

| Test | Description |
|------|-------------|
| `TestSimpleLinkage_ContactPositions` | 2-bar linkage with known contact curve |
| `TestSimpleLinkage_ForceRatio` | Known force amplification ratio |
| `TestRailTravel_Bounds` | Contact positions within travel limits |
| `TestPoseCache_PinCount` | Pose cache has correct pin count |
| `TestPoseCache_SampleCount` | 200 samples as configured |

#### 2.3 Edge Case Tests

| Test | Description |
|------|-------------|
| `TestCollinearPins_Detection` | Near-collinear pins handled |
| `TestZeroLengthBar_Handling` | Very short bars don't cause divide-by-zero |

### Phase 3: GraphRuntimeConverter Tests

#### 3.1 Node Type Mapping

| Test | Description |
|------|-------------|
| `TestMapNodeType_AllKinds` | Each GraphNodeKind maps correctly |
| `TestMapOp_AllOperators` | Each op string maps to correct OpType |
| `TestMapOp_CaseInsensitive` | "ADD", "add", "Add" all work |
| `TestMapOp_Symbols` | "+", "-", "*", "/" map correctly |

#### 3.2 Conversion Tests

| Test | Description |
|------|-------------|
| `TestConvert_InputNode` | Input node becomes runtime Input |
| `TestConvert_ParamNode` | Param node gets default value from Params dict |
| `TestConvert_OutputNode` | Output node with source connection |
| `TestConvert_OpNode_Args` | Op node args populated from links |
| `TestConvert_OpNode_Negate` | Negate flags set correctly for Add/Mul |
| `TestConvert_IncludeNode_InputMap` | Include input map populated |
| `TestConvert_IncludeNode_OutputMap` | Include output map populated |
| `TestConvert_SignalGroup` | Full signal name built from group.suffix |
| `TestConvert_SignalGroup_Legacy` | Falls back to port name if no group |

#### 3.3 Editor Format Detection

| Test | Description |
|------|-------------|
| `TestConvertEditorJson_DetectsLinks` | JSON with "links" detected as editor format |
| `TestConvertEditorJson_DetectsKind` | JSON with "kind" detected as editor format |
| `TestConvertEditorJson_IgnoresRuntime` | Runtime JSON returns null |

### Phase 4: AxisRequestQueue Tests (Requires Refactoring)

The current implementation has a hard dependency on `DiyFfbPluginUI`. To test:

#### 4.1 Refactoring (Optional)

Extract interface for the send callback:
```csharp
public interface IAxisRequestSender
{
    bool SendAxisRequest(AxisID axisId, AxisRequestType type, Message payload);
}
```

#### 4.2 Tests (After Refactoring)

| Test | Description |
|------|-------------|
| `TestEnqueue_AddsToQueue` | Request added to queue |
| `TestEnqueue_DuplicateIgnored` | Same axis+type not added twice |
| `TestEnqueue_UploadNotDeduplicated` | Upload requests always added |
| `TestHandleResponse_MatchesCurrentRequest` | Matching response clears current |
| `TestHandleResponse_WrongType_Ignored` | Mismatched type doesn't clear |
| `TestHandleResponse_WrongAxis_Ignored` | Mismatched axis doesn't clear |
| `TestRequiresResponse_RequestTypes` | Correct types require response |
| `TestRetry_OnSendFailure` | Retry after send failure |
| `TestRetry_MaxRetriesExhausted` | Request dropped after max retries |

### Phase 5: Regression Tests for Recent Bugs

These require extracting testable logic from UI code:

#### 5.1 Potential Extractions

| Bug | Testable Logic |
|-----|----------------|
| Node deletion selection | Selection state management (could extract to SelectionManager) |
| Inspector stale events | DataContext validation logic |
| CloseTab save dialog | Dirty state + shared graph detection logic |

#### 5.2 Without Refactoring

Add integration-style tests using the existing graph structure:

| Test | Description |
|------|-------------|
| `TestGraphUsageScanner_SharedGraph` | Already has tests, extend if needed |
| `TestGraphEditorTab_DirtyState` | Verify dirty tracking (if logic extracted) |

## File Changes

| File | Changes |
|------|---------|
| `GraphTest/GraphTestRunner.cs` | Add new test methods |
| `SimHubPlugin/Tools.cs` | No changes (already testable) |
| `SimHubPlugin/CubicSpline.cs` | No changes (already testable) |
| `SimHubPlugin/GeneralKinematics.cs` | No changes (already testable) |
| `GraphEditor/GraphRuntimeConverter.cs` | No changes (already testable) |
| `SimHubPlugin/AxisRequestQueue.cs` | Optional: extract interface |

## Test Data

Create test fixtures in `GraphTest/TestData/`:

```
TestData/
  Kinematics/
    simple_2bar.json          # Known 2-bar linkage geometry
    pedal_geometry.json       # Representative pedal config
  Graphs/
    editor_simple.json        # Simple editor-format graph
    runtime_simple.json       # Equivalent runtime format
```

## Verification

1. Build succeeds
2. All 77 existing tests still pass
3. New tests pass
4. Test count increases to ~120+ tests

## Prioritized Implementation Order

| Priority | Component | Est. Tests | Rationale |
|----------|-----------|------------|-----------|
| 1 | Tools.cs | 11 | Quick win, pure functions |
| 2 | CubicSpline | 5 | Quick win, critical for curves |
| 3 | StringExtensions | 2 | Quick win |
| 4 | GraphRuntimeConverter | 15 | Medium effort, catches conversion bugs |
| 5 | GeneralKinematics | 12 | Medium effort, high value for FFB |
| 6 | AxisRequestQueue | 9 | Requires refactoring, lower priority |

**Total new tests:** ~54

## Risks

| Risk | Mitigation |
|------|------------|
| Kinematics tests fragile due to floating-point | Use tolerance-based assertions |
| AxisRequestQueue refactoring scope creep | Make interface extraction minimal |
| Test data maintenance burden | Keep fixtures minimal, derive from existing configs |

## Out of Scope

- UI automation tests (WPF testing framework complexity)
- Integration tests with actual serial devices
- Performance/stress tests
- Code coverage metrics tooling
