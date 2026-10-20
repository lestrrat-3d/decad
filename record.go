package decad

import (
	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/chainrecord"
	"github.com/lestrrat-3d/decad/internal/coilshell"
	"github.com/lestrrat-3d/decad/internal/cupwall"
	"github.com/lestrrat-3d/decad/internal/draftrecord"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/patchchain"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/stitchrecord"
	"github.com/lestrrat-3d/decad/internal/sweepmitre"
	"github.com/lestrrat-3d/decad/internal/sweeprecord"
	"github.com/lestrrat-3d/decad/internal/twistsweep"
)

// Structural curve records stay in internal packages. Their aliases keep the
// evaluator's local names short without adding them to decad's public API.

type planeRecord = sectionrecord.PlaneRecord

// planePoint is the evaluator's plane-local coordinate. Point2 is its public
// spelling; both share one underlying struct, so a value assigns either way.
type planePoint = sectionrecord.Point2

// Point2 is a plane-local coordinate in millimetres.
type Point2 struct {
	U float64 `json:"u"`
	V float64 `json:"v"`
}

func point2ToRecordSlice(points []planePoint) []sectionrecord.Point2 {
	if points == nil {
		return nil
	}
	out := make([]sectionrecord.Point2, len(points))
	copy(out, points)
	return out
}

type profileRecord = momentinput.Profile

// prismPayload is the evaluator's own record of a straight extrude: the
// internal/prismrecord payload, which prismrecord.Build evaluates over
// topology nodes and which implements topo.Payload (docs/evaluator-design.md
// §5, §8).
type prismPayload = prismrecord.Payload

// stackedPrismPayload is the evaluator's own record of a stacked prism: the
// internal/stackedrecord payload, which stackedrecord.Build evaluates over
// topology nodes and which implements topo.Payload
// (docs/stacked-prism-design.md, docs/evaluator-design.md §8).
type stackedPrismPayload = stackedrecord.Payload

// chainPayload is chain Extrude's own record of a ribbon body: the
// internal/chainrecord payload, which chainrecord.Build evaluates over
// topology nodes and which implements topo.Payload
// (docs/surface-design.md §13.4, docs/evaluator-design.md §8).
type chainPayload = chainrecord.Payload

// cupPayload is the evaluator's own record of a cup: the internal/cupwall
// payload, which cupwall.Build evaluates over topology nodes and which
// implements topo.Payload (docs/modify-reach-design.md §9.1,
// docs/evaluator-design.md §8). cupView is the two-prism reading it re-derives.
type (
	cupPayload = cupwall.Payload
	cupView    = cupwall.View
)

// roleShellCap names a cup's pocket floor.
const roleShellCap = cupwall.RoleShellCap

// patchPayload and bodyPatchPayload are the evaluator's own records of a
// PatchProfile body and a PatchEdges body: the internal/patchchain payloads,
// which patchchain.Build and patchchain.BuildBody evaluate over topology nodes
// and which implement topo.Payload. bodyPatchChain is one proven chain of the
// latter (docs/surface-design.md §5, docs/evaluator-design.md §8).
type (
	patchPayload     = patchchain.Payload
	bodyPatchPayload = patchchain.BodyPayload
	bodyPatchChain   = patchchain.Chain
)

// revolvePayload is the evaluator's own record of a revolved body, and
// chainRevolvePayload chain Revolve's record of a shell: the
// internal/revolveplan payloads, which revolveplan.Build and
// revolveplan.BuildChain evaluate over topology nodes and which implement
// topo.Payload (docs/evaluator-design.md §6, §8; docs/surface-design.md §13.4).
type (
	revolvePayload      = revolveplan.Payload
	chainRevolvePayload = revolveplan.ChainPayload
)

// mitredSweepPayload is the evaluator's record of a mitred polyline sweep: the
// internal/sweepmitre payload, which sweepmitre.Build evaluates over topology
// nodes and which implements topo.Payload (docs/sweep-design.md §16,
// docs/evaluator-design.md §8).
type mitredSweepPayload = sweepmitre.Payload

// coilPayload is the evaluator's record of a coil: the internal/coilshell
// payload, which coilshell.BuildBody evaluates over topology nodes and which
// implements topo.Payload (docs/helix-design.md, docs/evaluator-design.md §8).
type coilPayload = coilshell.Payload

// loftPayload and chainLoftPayload are the evaluator's records of a Loft and of
// a LoftChain ribbon: the internal/loftrecord payloads, which loftrecord.Build
// and loftrecord.BuildChain evaluate over topology nodes and which implement
// topo.Payload (docs/loft-design.md, docs/evaluator-design.md §8).
type (
	loftPayload      = loftrecord.Payload
	chainLoftPayload = loftrecord.ChainPayload
)

// twistedSweepPayload and compositeTwistedSweepPayload are the evaluator's
// records of a twisted Sweep along one line and along a line and a cardinal
// quarter arc: the internal/twistsweep payloads, which twistsweep.Eval
// evaluates over topology nodes and which implement topo.Payload
// (docs/sweep-design.md §17, docs/evaluator-design.md §8).
type (
	twistedSweepPayload          = twistsweep.Payload
	compositeTwistedSweepPayload = twistsweep.CompositePayload
)

// sweepPayload and chainSweepPayload are the evaluator's records of a profile
// Sweep and a chain Sweep, and chainSweepSpanPayload is one chain span:
// the internal/sweeprecord payloads, which sweeprecord's
// builders evaluate over topology nodes and whose body records implement
// topo.Payload (docs/sweep-design.md, docs/evaluator-design.md §8).
type (
	sweepPayload      = sweeprecord.Payload
	chainSweepPayload = sweeprecord.ChainPayload
)

// stitchPayload and unstitchPayload are the evaluator's records of a Stitch
// and of one Unstitch piece: the internal/stitchrecord payloads, which
// stitchrecord.Build and stitchrecord.BuildUnstitchFace evaluate over
// topology nodes and which implement topo.Payload (docs/surface-design.md §6,
// docs/evaluator-design.md §8).
type (
	stitchPayload   = stitchrecord.Payload
	unstitchPayload = stitchrecord.UnstitchPayload
)

// draftPayload and twoSidedDraftPayload are the evaluator's records of a
// tapered Extrude or face Draft and of a two-sided taper: the
// internal/draftrecord payloads, which draftrecord.Build and
// draftrecord.BuildTwoSided evaluate over topology nodes and which implement
// topo.Payload (docs/draft-design.md, docs/evaluator-design.md §8).
type (
	draftPayload         = draftrecord.Payload
	twoSidedDraftPayload = draftrecord.TwoSidedPayload
)

// capBlendPayload is the evaluator's record of a complete-cap-loop chamfer
// and of the band view a draft or brep loop band builds through: the
// internal/capblend payload, which capblend.Build evaluates over topology
// nodes and which implements topo.Payload (docs/modify-reach-design.md §8.3,
// docs/evaluator-design.md §8). The other names are its setbacks and its
// loft source.
type (
	capBlendPayload   = capblend.Payload
	capSetback        = capblend.Setback
	loftCapBandSource = capblend.LoftSource
)

// facetedPayload is the evaluator's record of a mesh-built body, and
// facetGroup one source face's provenance in it: the internal/facetedrecord
// payload, which facetedrecord.BuildWithProof assembles over topology nodes
// with one face per connected patch and which implements topo.Payload
// (docs/evaluator-design.md §8, §9).
type (
	facetedPayload = facetedrecord.Payload
	facetGroup     = facetedrecord.Group
)

// The faceted payload's source records and the support types they hold are
// internal/facetedrecord types; these aliases keep the evaluator's local names.
type (
	loftFaceCapBounds          = facetedrecord.LoftFaceCapBounds
	loftFacesCapRecord         = facetedrecord.LoftFacesCapRecord
	loftFacesToeRecord         = facetedrecord.LoftFacesToeRecord
	loftFacesTrimRecord        = facetedrecord.LoftFacesTrimRecord
	loftMirrorJoinRecord       = facetedrecord.LoftMirrorJoinRecord
	loftMirrorPlacementRecord  = facetedrecord.LoftMirrorPlacementRecord
	pointConeLimit             = facetedrecord.PointConeLimit
	pointConeTrimRecord        = facetedrecord.PointConeTrimRecord
	pointPlaneLimit            = facetedrecord.PointPlaneLimit
	pointSectionRecord         = facetedrecord.PointSectionRecord
	pointSectionSource         = facetedrecord.PointSectionSource
	prismUnionFilletRecord     = facetedrecord.PrismUnionFilletRecord
	revolveLoftSectorPlacement = facetedrecord.RevolveLoftSectorPlacement
	revolveSectionCertificate  = facetedrecord.RevolveSectionCertificate
	revolveSectionSpan         = facetedrecord.RevolveSectionSpan
	spiralCombineSource        = facetedrecord.SpiralCombineSource
	spiralFirstJoinSource      = facetedrecord.SpiralFirstJoinSource
)

type loopRecord = sectionrecord.LoopRecord

type chainRecord = sectionrecord.ChainRecord

type curveSegment = sectionrecord.CurveSegment

type lineSeg = sectionrecord.LineSeg

type circleSeg = sectionrecord.CircleSeg

type arcSeg = sectionrecord.ArcSeg

type fitSplineSeg = sectionrecord.FitSplineSeg

// allEdgeChamferPayload is the evaluator's record of the box all-edge chamfer
// of modify-general §4.3b: the internal/breprecord payload, which
// breprecord.BuildAllEdgeChamfer evaluates over topology nodes and which
// implements topo.Payload (docs/evaluator-design.md §8).
type allEdgeChamferPayload = breprecord.AllEdgeChamferPayload

// brepPayload is the evaluator's record of an analytically trimmed body
// (docs/general-boolean-design.md §4): the internal/breprecord payload, which
// breprecord.Build evaluates over topology nodes and which implements
// topo.Payload (docs/evaluator-design.md §8). The other names are its faces,
// an A1 result's slabs, a route L band and its kind, and the shared topology
// reading.
type (
	brepPayload  = breprecord.Payload
	brepFace     = breprecord.Face
	brepBandKind = breprecord.BandKind
	brepTopology = breprecord.Topology
)

// brepSplit is one side-line split: a level and its displacement. brepEmbed
// maps one face frame's local axes onto the reference frame's by a signed
// permutation.
type (
	brepSplit = brepgeom.Split
	brepEmbed = brepgeom.Embed
)
