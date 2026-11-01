package meshbool

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// These private replay refusals are immutable and exist before a budget starts.
var (
	errBrakeInvalidEarPolygon    = fmt.Errorf("%w: replay ear polygon has fewer than three vertices", decaderr.ErrUnsupported)
	errBrakeEarStalled           = fmt.Errorf(`%w: exact ear clipping stalled on a boolean subdivision polygon`, decaderr.ErrBooleanFailed)
	errBrakeEarClockwise         = fmt.Errorf(`%w: a boolean subdivision polygon closed clockwise`, decaderr.ErrBooleanFailed)
	errBrakeContactBatchCapacity = fmt.Errorf("%w: invalid replay contact batch capacity", decaderr.ErrUnsupported)
	errBrakeNoContactEdge        = fmt.Errorf("%w: replay facet has no contact edge", decaderr.ErrUnsupported)
	errBrakeUnanchored           = fmt.Errorf("%w: unanchored replay region is unavailable", decaderr.ErrUnsupported)
	errBrakeDeferredVertex       = fmt.Errorf("%w: deferred replay vertex is unavailable", decaderr.ErrUnsupported)
	errBrakeDeferredProbe        = fmt.Errorf("%w: deferred replay chain probe is unavailable", decaderr.ErrUnsupported)
	errBrakeInvalidChain         = fmt.Errorf("%w: replay chain has fewer than two vertices", decaderr.ErrUnsupported)
	errBrakeParityRays           = fmt.Errorf("%w: every parity ray was ambiguous", decaderr.ErrBooleanFailed)
)

// Static replay refusals do not allocate when geometry is declined.
var (
	errReplayCutterNeedsTheSharedReplayBudget          = fmt.Errorf("%w: cutter needs the shared replay budget", decaderr.ErrUnsupported)
	errReplayMissingReplayCutterBudget                 = fmt.Errorf("%w: missing replay cutter budget", decaderr.ErrUnsupported)
	errReplayMissingReplayCutterReservation            = fmt.Errorf("%w: missing replay cutter reservation", decaderr.ErrUnsupported)
	errReplayMissingReplayVertexTable                  = fmt.Errorf("%w: missing replay vertex table", decaderr.ErrUnsupported)
	errReplayNegativeCutterArraySize                   = fmt.Errorf("%w: negative cutter array size", decaderr.ErrUnsupported)
	errReplayNegativeCutterRecordCount                 = fmt.Errorf("%w: negative cutter record count", decaderr.ErrUnsupported)
	errReplayCutterRecordCountOverflow                 = fmt.Errorf("%w: cutter record count overflow", decaderr.ErrUnsupported)
	errReplayIncompleteReplayCutter                    = fmt.Errorf("%w: incomplete replay cutter", decaderr.ErrUnsupported)
	errReplayAContactChainLiesOnASubdivision           = fmt.Errorf(`%w: a contact chain lies on a subdivision boundary`, decaderr.ErrBooleanFailed)
	errReplayAContactChainEscapedEverySubdivisionPiece = fmt.Errorf(`%w: a contact chain escaped every subdivision piece`, decaderr.ErrBooleanFailed)
	errReplayLoopOpeningDidNotConverge                 = fmt.Errorf(`%w: loop opening did not converge`, decaderr.ErrBooleanFailed)
	errReplayInvalidCutterEdge                         = fmt.Errorf("%w: invalid cutter edge", decaderr.ErrBooleanFailed)
	errReplayAContactChainDanglesMidFacet              = fmt.Errorf(`%w: a contact chain dangles mid-facet`, decaderr.ErrBooleanFailed)
	errReplayAClosedContactChainHasNoWidth             = fmt.Errorf(`%w: a closed contact chain has no width`, decaderr.ErrBooleanFailed)
	errReplayNoVertexFreeSplitLineFound                = fmt.Errorf(`%w: no vertex-free split line found`, decaderr.ErrBooleanFailed)
	errReplayAChainEndpointIsMissingFromIts            = fmt.Errorf(`%w: a chain endpoint is missing from its region boundary`, decaderr.ErrBooleanFailed)
	errReplayAChainSplitProducedANonPositive           = fmt.Errorf(`%w: a chain split produced a non-positive region`, decaderr.ErrBooleanFailed)
	errReplayAChainLiesOnAFreshlySplit                 = fmt.Errorf(`%w: a chain lies on a freshly split boundary`, decaderr.ErrBooleanFailed)
	errReplayAChainEscapedBothSplitRegions             = fmt.Errorf(`%w: a chain escaped both split regions`, decaderr.ErrBooleanFailed)
	errReplayAChainEndpointLiesOnNoRegion              = fmt.Errorf(`%w: a chain endpoint lies on no region boundary`, decaderr.ErrBooleanFailed)
	errReplayARegionTriangulatedToNothing              = fmt.Errorf(`%w: a region triangulated to nothing`, decaderr.ErrBooleanFailed)
	errReplayMismatchedReplayKeys                      = fmt.Errorf("%w: mismatched replay keys", decaderr.ErrUnsupported)
	errReplayBoundedOutputExceedsFacets                = fmt.Errorf("%w: bounded output exceeds %d facets", decaderr.ErrUnsupported, brakeReplayFacetLimit)
	errReplayARegionProbeLandedOnItsContact            = fmt.Errorf("%w: a region probe landed on its contact plane", decaderr.ErrBooleanFailed)
	errReplayReplayParityNeedsAReplayBudget            = fmt.Errorf("%w: replay parity needs a replay budget", decaderr.ErrUnsupported)
	errReplayMissingParityReplayBudget                 = fmt.Errorf("%w: missing parity replay budget", decaderr.ErrUnsupported)
	errReplayMissingReplayParityOwner                  = fmt.Errorf("%w: missing replay parity owner", decaderr.ErrUnsupported)
	errReplayReplayParitySize                          = fmt.Errorf("%w: replay parity size", decaderr.ErrUnsupported)
	errReplayReplayParityRequiresPreparedExactFacets   = fmt.Errorf("%w: replay parity requires prepared exact facets", decaderr.ErrUnsupported)
	errReplayInvalidBoundedTable                       = fmt.Errorf("%w: invalid bounded table", decaderr.ErrUnsupported)
	errReplayBoundedTableSizeOverflow                  = fmt.Errorf("%w: bounded table size overflow", decaderr.ErrUnsupported)
	errReplayInvalidBoundedTableCapacity               = fmt.Errorf("%w: invalid bounded table capacity", decaderr.ErrUnsupported)
	errReplayInvalidBoundedTableLookup                 = fmt.Errorf("%w: invalid bounded table lookup", decaderr.ErrUnsupported)
	errReplayBoundedComparisonOverflow                 = fmt.Errorf("%w: bounded comparison overflow", decaderr.ErrUnsupported)
	errReplayBoundedTableIsFull                        = fmt.Errorf("%w: bounded table is full", decaderr.ErrUnsupported)
	errReplayBoundedTableCapacityExceeded              = fmt.Errorf("%w: bounded table capacity exceeded", decaderr.ErrUnsupported)
	errReplayDuplicatedBoundedTableEntry               = fmt.Errorf("%w: duplicated bounded table entry", decaderr.ErrBooleanFailed)
	errReplayReplaySliceCapacityOverflow               = fmt.Errorf("%w: replay slice capacity overflow", decaderr.ErrUnsupported)
	errReplayBoundedSliceSizeOverflow                  = fmt.Errorf("%w: bounded slice size overflow", decaderr.ErrUnsupported)
	errReplayBoundedSliceCapacityOverflow              = fmt.Errorf("%w: bounded slice capacity overflow", decaderr.ErrUnsupported)
)

var (
	errReplayReplayPairCutsNeedASharedBudget          = fmt.Errorf("%w: replay pair cuts need a shared budget", decaderr.ErrUnsupported)
	errReplayPairCutWorkBudgetExceeded                = fmt.Errorf("%w: pair cut work budget exceeded", decaderr.ErrUnsupported)
	errReplayPairCutRecordBudgetExceeded              = fmt.Errorf("%w: pair cut record budget exceeded", decaderr.ErrUnsupported)
	errReplayPairCutByteReservationOverflow           = fmt.Errorf("%w: pair cut byte reservation overflow", decaderr.ErrUnsupported)
	errReplayMissingReplayCollectorBudget             = fmt.Errorf("%w: missing replay collector budget", decaderr.ErrUnsupported)
	errReplayReplayPairCutsRequireThePrivateCollector = fmt.Errorf("%w: replay pair cuts require the private collector", decaderr.ErrUnsupported)
	errReplayReplayPairCutsNeedPreparedFacetSlab      = fmt.Errorf("%w: replay pair cuts need prepared facet slab bounds", decaderr.ErrUnsupported)
	errReplayInvalidContactFacet                      = fmt.Errorf("%w: invalid contact facet", decaderr.ErrUnsupported)
)

// Contact causes are shared immutable errors; each replay owns its typed wrapper.
var (
	errBrakeBranchContactCause   = fmt.Errorf(`%w: intersection curves branch on a facet — the exact predicates cannot classify a tangent contact`, decaderr.ErrUnsupported)
	errBrakeBoundaryContactCause = fmt.Errorf(`%w: an operand facet touches the other operand's boundary — the exact predicates cannot classify a tangent contact`, decaderr.ErrUnsupported)
)

func (tc *brakeTriCutter) branchContactRefusal() error {
	if tc.contactRefusal.Err == nil {
		tc.contactRefusal = BooleanExpectedError{Kind: BooleanExpectedContact, Operand: -1, Err: errBrakeBranchContactCause}
	}
	return &tc.contactRefusal
}

func (b *PairCutsBudget) boundaryContactRefusal() error {
	if b.contactRefusal.Err == nil {
		b.contactRefusal = BooleanExpectedError{Kind: BooleanExpectedContact, Operand: -1, Err: errBrakeBoundaryContactCause}
	}
	return &b.contactRefusal
}

// Helper refusals retain their diagnostic while avoiding failure allocations.
var (
	errReplayBoundedStitchFacetCapExceeded              = fmt.Errorf("%w: bounded stitch facet cap exceeded", decaderr.ErrUnsupported)
	errReplayTheStitchedBoundaryHasNoExtent             = fmt.Errorf(`%w: the stitched boundary has no extent`, decaderr.ErrBooleanFailed)
	errReplayInvalidConformEdge                         = fmt.Errorf("%w: invalid conform edge", decaderr.ErrUnsupported)
	errReplayInvalidConformVertex                       = fmt.Errorf("%w: invalid conform vertex", decaderr.ErrUnsupported)
	errReplayAConformingPolygonLostItsCorners           = fmt.Errorf(`%w: a conforming polygon lost its corners`, decaderr.ErrBooleanFailed)
	errReplayAConformingPolygonIsDegenerate             = fmt.Errorf(`%w: a conforming polygon is degenerate`, decaderr.ErrBooleanFailed)
	errReplayMissingReplayContactMemoInputs             = fmt.Errorf("%w: missing replay contact memo inputs", decaderr.ErrUnsupported)
	errReplayProximityReplayNeedsASharedBudget          = fmt.Errorf("%w: proximity replay needs a shared budget", decaderr.ErrUnsupported)
	errReplayReplayCapContactProofIsUnavailable         = fmt.Errorf("%w: replay cap contact proof is unavailable", decaderr.ErrUnsupported)
	errReplayTooManyFaceGroups                          = fmt.Errorf("%w: too many face groups", decaderr.ErrUnsupported)
	errReplayAnOperandSTessellationStatesNoDisplacement = fmt.Errorf(`%w: an operand's tessellation states no displacement bound for one of its own source faces`, decaderr.ErrBooleanFailed)
	errReplayTheCertifiedConeCapHasAHeld                = fmt.Errorf("%w: the certified cone cap has a held facet contact", decaderr.ErrUnsupported)
	errReplayBoundedStitchNeedsASharedBudget            = fmt.Errorf("%w: bounded stitch needs a shared budget", decaderr.ErrUnsupported)
	errReplayMissingStitchReplayBudget                  = fmt.Errorf("%w: missing stitch replay budget", decaderr.ErrUnsupported)
	errReplayTheOperationLeavesNoBoundaryAtAll          = fmt.Errorf(`%w: the operation leaves no boundary at all`, decaderr.ErrBooleanFailed)
	errReplayAKeptFacetCollapsed                        = fmt.Errorf(`%w: a kept facet collapsed`, decaderr.ErrBooleanFailed)
	errReplayTheConformingPassDidNotConverge            = fmt.Errorf(`%w: the conforming pass did not converge`, decaderr.ErrBooleanFailed)
	errReplayTheStitchedBoundaryDoesNotClose            = fmt.Errorf(`%w: the stitched boundary does not close`, decaderr.ErrBooleanFailed)
	errReplayNonfiniteRoundedReplayVertex               = fmt.Errorf("%w: nonfinite rounded replay vertex", decaderr.ErrUnsupported)
	errReplayTheWholeResultCollapsedUnderRounding       = fmt.Errorf(`%w: the whole result collapsed under rounding`, decaderr.ErrBooleanFailed)
	errReplayBoundedStitchReachRepairIsUnavailable      = fmt.Errorf("%w: bounded stitch reach repair is unavailable", decaderr.ErrUnsupported)
	errReplayTheRoundedBoundaryDoesNotClose             = fmt.Errorf(`%w: the rounded boundary does not close`, decaderr.ErrBooleanFailed)
	errReplayInvalidReplayDroppedFacets                 = fmt.Errorf("%w: invalid replay dropped facets", decaderr.ErrUnsupported)
	errReplayStitchEdgeCapacityOverflow                 = fmt.Errorf("%w: stitch edge capacity overflow", decaderr.ErrUnsupported)
	errReplayAFacetCrossesAPlaneMoreThan                = fmt.Errorf("%w: a facet crosses a plane more than twice", decaderr.ErrBooleanFailed)
	errReplayNoncoplanarContactHasNoIntersectionLine    = fmt.Errorf("%w: noncoplanar contact has no intersection line", decaderr.ErrUnsupported)
	errReplayProductObjectExceedsItsFixedEnvelope       = fmt.Errorf("%w: product object exceeds its fixed envelope", decaderr.ErrUnsupported)
	errReplayFanIncidenceOverflow                       = fmt.Errorf("%w: fan incidence overflow", decaderr.ErrUnsupported)
	errReplayFanStorageOverflow                         = fmt.Errorf("%w: fan storage overflow", decaderr.ErrUnsupported)
	errReplayFanWorkOverflow                            = fmt.Errorf("%w: fan work overflow", decaderr.ErrUnsupported)
	errReplayInvalidFanVertex                           = fmt.Errorf("%w: invalid fan vertex", decaderr.ErrUnsupported)
	errReplayInvalidReplayEmbeddingInput                = fmt.Errorf("%w: invalid replay embedding input", decaderr.ErrUnsupported)
	errReplayInvalidReplayFacet                         = fmt.Errorf("%w: invalid replay facet", decaderr.ErrUnsupported)
	errReplayOrientationPlaneWorkOverflow               = fmt.Errorf("%w: orientation plane work overflow", decaderr.ErrUnsupported)
	errReplayHeldSATWorkOverflow                        = fmt.Errorf("%w: held SAT work overflow", decaderr.ErrUnsupported)
	errReplayReplayEmbeddingTreeSize                    = fmt.Errorf("%w: replay embedding tree size", decaderr.ErrUnsupported)
	errReplayBoundedMeshPreparationFacetCapExceeded     = fmt.Errorf("%w: bounded mesh preparation facet cap exceeded", decaderr.ErrUnsupported)
	errReplayAMeshVertexIsNotFinite                     = fmt.Errorf("%w: a mesh vertex is not finite", decaderr.ErrBooleanFailed)
	errReplayExactPointByteReservationOverflow          = fmt.Errorf("%w: exact point byte reservation overflow", decaderr.ErrUnsupported)
	errReplayInvalidOperandFacet                        = fmt.Errorf("%w: invalid operand facet", decaderr.ErrBooleanFailed)
	errReplayExactNormalByteReservationOverflow         = fmt.Errorf("%w: exact normal byte reservation overflow", decaderr.ErrUnsupported)
	errReplayAnOperandHoldsACollapsedFacet              = fmt.Errorf("%w: an operand holds a collapsed facet", decaderr.ErrUnsupported)
	errReplayMissingReplayEdgeOwner                     = fmt.Errorf("%w: missing replay edge owner", decaderr.ErrUnsupported)
	errReplayAnInPlaneFacetEdgeHasNo                    = fmt.Errorf("%w: an in-plane facet edge has no twin facet", decaderr.ErrBooleanFailed)
	errReplayReplayGapInputs                            = fmt.Errorf("%w: replay gap inputs", decaderr.ErrUnsupported)
	errReplayReplayGapRecord                            = fmt.Errorf("%w: replay gap record", decaderr.ErrUnsupported)
	errReplayAResultVertexSDisplacementBoundIs          = fmt.Errorf("%w: a result vertex's displacement bound is not finite", decaderr.ErrUnsupported)
	errReplayMissingReplayKeyBudget                     = fmt.Errorf("%w: missing replay key budget", decaderr.ErrUnsupported)
	errReplayExactKeyNeedsReplayBudget                  = fmt.Errorf("%w: exact key needs replay budget", decaderr.ErrUnsupported)
	errReplayKeyMemoOverflow                            = fmt.Errorf("%w: key memo overflow", decaderr.ErrUnsupported)
	errReplayKeyRehashWorkOverflow                      = fmt.Errorf("%w: key rehash work overflow", decaderr.ErrUnsupported)
)

var (
	errReplayInvalidReplayRepairInput        = fmt.Errorf("%w: invalid replay repair input", decaderr.ErrUnsupported)
	errReplayInvalidReplayRepairFacet        = fmt.Errorf("%w: invalid replay repair facet", decaderr.ErrUnsupported)
	errReplayReplayExactLiftWorkOverflow     = fmt.Errorf("%w: replay exact lift work overflow", decaderr.ErrUnsupported)
	errReplayInvalidReplayExactPoint         = fmt.Errorf("%w: invalid replay exact point", decaderr.ErrUnsupported)
	errReplayReplayRepairGridIsNotFinite     = fmt.Errorf("%w: replay repair grid is not finite", decaderr.ErrUnsupported)
	errReplayReplayFloatBoxRepairDidNotClear = fmt.Errorf("%w: replay float-box repair did not clear the fold", decaderr.ErrUnsupported)
	errReplayMissingCachedExactCoordinate    = fmt.Errorf("%w: missing cached exact coordinate", decaderr.ErrUnsupported)
	errReplayInvalidPreparedReplayGeometry   = fmt.Errorf("%w: invalid prepared replay geometry", decaderr.ErrUnsupported)
	errReplayInvalidReplayFloatBox           = fmt.Errorf("%w: invalid replay float box", decaderr.ErrUnsupported)
	errReplayNonFiniteReplayFloatBox         = fmt.Errorf("%w: non-finite replay float box", decaderr.ErrUnsupported)
)
