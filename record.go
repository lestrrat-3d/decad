package decad

import (
	"github.com/lestrrat-3d/decad/internal/chainrecord"
	"github.com/lestrrat-3d/decad/internal/coilshell"
	"github.com/lestrrat-3d/decad/internal/cupwall"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/patchchain"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/sweepmitre"
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

func point2FromRecordSlice(points []sectionrecord.Point2) []planePoint {
	if points == nil {
		return nil
	}
	out := make([]planePoint, len(points))
	copy(out, points)
	return out
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

type loopRecord = sectionrecord.LoopRecord

type chainRecord = sectionrecord.ChainRecord

type curveSegment = sectionrecord.CurveSegment

type lineSeg = sectionrecord.LineSeg

type circleSeg = sectionrecord.CircleSeg

type arcSeg = sectionrecord.ArcSeg

type fitSplineSeg = sectionrecord.FitSplineSeg

func cloneLoopRecord(loop loopRecord) loopRecord { return sectionrecord.CloneLoopRecord(loop) }
func normalizeSegment(segment curveSegment) (curveSegment, error) {
	return sectionrecord.NormalizeSegment(segment)
}
