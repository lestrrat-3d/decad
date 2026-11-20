package booleaneval

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/prismtess"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

var errStationsInapplicable = errors.New("native prism stations do not match")

type stationOperand struct {
	body  *topo.Body
	mesh  *topo.Mesh
	chord float64
}
type stationMesher struct {
	OperandMesher
	plate, disk stationOperand
	rims        []meshbool.AxialRim
	budget      *proofbound.ReplayBudget
}

func (m *stationMesher) matches(a, b *topo.Body, ma, mb *topo.Mesh) bool {
	return a == m.plate.body && b == m.disk.body && ma == m.plate.mesh && mb == m.disk.mesh || a == m.disk.body && b == m.plate.body && ma == m.disk.mesh && mb == m.plate.mesh
}

func (m *stationMesher) TessellateOperand(ctx context.Context, b *topo.Body, chord float64) (*topo.Mesh, bool, error) {
	for _, op := range []stationOperand{m.plate, m.disk} {
		if op.body == b && op.chord == chord {
			return op.mesh, false, nil
		}
	}
	return m.OperandMesher.TessellateOperand(ctx, b, chord)
}

func stationProfile(p prismrecord.Payload) bool {
	return !p.SurfaceResult && p.Xform == r3.Identity() && p.SectionDelta == 0 && p.Z0Delta == 0 && p.Z1Delta == 0 && p.Frame.U() == (r3.Vec{X: 1}) && p.Frame.V() == (r3.Vec{Z: 1}) && len(p.Profile.Holes) == 0 && len(p.Profile.Outer.Segments) > 0 && len(p.Profile.Outer.Segments) <= 16
}
func stationSealed(ctx context.Context, b *topo.Body, p prismrecord.Payload, budget *proofbound.ReplayBudget) (bool, error) {
	if len(b.Lumps) != 1 || b.Lumps[0] == nil || len(b.Lumps[0].Shells) != 1 || b.Lumps[0].Shells[0] == nil || len(b.Lumps[0].Shells[0].Faces) != len(p.Profile.Outer.Segments)+2 {
		return false, nil
	}
	for _, f := range b.Lumps[0].Shells[0].Faces {
		if f == nil {
			return false, nil
		}
		ok, err := b.ReplayPrismBuiltFace(ctx, f, budget)
		if !ok || err != nil {
			return false, err
		}
	}
	return b.ReplayPrismBuildPayload(ctx, p.Frame, p.Xform, p.Z0, p.Z1, p.Z0Delta, p.Z1Delta, p.SectionDelta, p.Z0Denoted, p.Z1Denoted, []sectionrecord.AnalyticProfileView{p.Profile.AnalyticSealView()}, budget)
}

// Limit the chording before the ordinary prism builder allocates. Only
// native complete lines and circular walks are eligible for this trial.
func stationBuild(ctx context.Context, b *topo.Body, p prismrecord.Payload, chord float64, disk bool, budget *proofbound.ReplayBudget) (*topo.Mesh, error) {
	total := 0
	for _, seg := range p.Profile.Outer.Segments {
		w, err := boundarywalk.WalkOf(seg, freeform.NewFreeformWork())
		if err != nil {
			return nil, err
		}
		n := 1
		if _, ok := seg.(sectionrecord.ArcSeg); ok {
			n, _, err = tessellation.ChordCount(w, chord/2, tessellation.ChordWalkMin(w))
			if err != nil {
				return nil, err
			}
		}
		total += n
	}
	if total < 3 || total > 1024 {
		return nil, errStationsInapplicable
	}
	if err := budget.ReserveBundle(ctx, uint64(total)*256, uint64(total)*40, uint64(total)*4096); err != nil {
		return nil, err
	}
	role := func(loop, seg int) string {
		if disk {
			return "side(0,0)"
		}
		return fmt.Sprintf("side(%d,%d)", loop, seg)
	}
	m, err := prismtess.Build(ctx, b, p, role, chord, true, nil, 0)
	if err != nil {
		return nil, err
	}
	if err := topobuild.RequireSingleShellWindingReplay(ctx, m, b.Lumps, budget); err != nil {
		return nil, err
	}
	_, ok, err := facetproof.AuditFacetedMeshReplay(ctx, budget.WorkBudget(ctx), m.Vertices, m.Triangles)
	if err != nil || !ok {
		return nil, err
	}
	ok, err = stationEmbedded(ctx, m, budget)
	if err != nil || !ok {
		return nil, err
	}
	m.BoundaryOK = true
	return m, nil
}

func stationArc(p prismrecord.Payload) (int, sectionrecord.ArcSeg, bool) {
	index := -1
	var arc sectionrecord.ArcSeg
	for i, seg := range p.Profile.Outer.Segments {
		switch s := seg.(type) {
		case sectionrecord.LineSeg:
			if !((s.TStart == 0 && s.TEnd == 1) || (s.TStart == 1 && s.TEnd == 0)) {
				return -1, arc, false
			}
		case sectionrecord.ArcSeg:
			if !((s.TStart == 0 && s.TEnd == 1) || (s.TStart == 1 && s.TEnd == 0)) {
				return -1, arc, false
			}
			if s.Center == (sectionrecord.Point2{}) {
				if index != -1 || s.TStart != 0 {
					return -1, arc, false
				}
				index, arc = i, s
			}
		default:
			return -1, arc, false
		}
	}
	return index, arc, index >= 0
}

// A rounded flange and a native disk can have genuinely distinct radii less
// than one float ulp apart. Independent chords alternately cross along their
// common angular span, creating cap islands. Use the flange's two endpoint
// directions as the disk's two arc divisions. A small outward displacement
// of the disk's held polygon makes the two strict native crossings visible
// at float precision. The candidate records the full radius difference.
func matchedPrismUnion(ctx context.Context, a, b *topo.Body, meshes OperandMesher, chordScale float64) (Evaluation, bool, error) {
	if a == nil || b == nil || !a.HasStrictPrismBuildSeal() || !b.HasStrictPrismBuildSeal() {
		return Evaluation{}, false, nil
	}
	plate, disk := a, b
	p, ok := a.Payload.(prismrecord.Payload)
	q, ok2 := b.Payload.(prismrecord.Payload)
	if !ok || !ok2 || !stationProfile(p) || !stationProfile(q) {
		return Evaluation{}, false, nil
	}
	if len(p.Profile.Outer.Segments) == 1 {
		plate, disk, p, q = b, a, q, p
	}
	if len(q.Profile.Outer.Segments) != 1 || p.Frame.Origin().X != q.Frame.Origin().X || p.Frame.Origin().Z != q.Frame.Origin().Z {
		return Evaluation{}, false, nil
	}
	circle, ok := q.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	if !ok || !circle.CCW || circle.TStart != 0 || circle.TEnd != 1 || circle.Center != (sectionrecord.Point2{}) || circle.Radius.Unit() != units.Millimeter {
		return Evaluation{}, false, nil
	}
	radius := circle.Radius.Base()
	if !(radius > 0) || proofbound.IsNonFinite(radius) {
		return Evaluation{}, false, nil
	}
	index, arc, ok := stationArc(p)
	if !ok {
		return Evaluation{}, false, nil
	}
	budget := proofbound.NewStrictReplayBudget()
	for _, op := range []struct {
		b *topo.Body
		p prismrecord.Payload
	}{{plate, p}, {disk, q}} {
		yes, e := stationSealed(ctx, op.b, op.p, budget)
		if e != nil || !yes {
			return Evaluation{}, false, e
		}
	}
	rims, ok, err := stationRoots(ctx, plate, disk, p, q, index, arc, radius, budget)
	if !ok || err != nil {
		return Evaluation{}, false, err
	}
	capDelta, diameter, err := booloperand.PairChord(ctx, a, b)
	if err != nil {
		return Evaluation{}, false, err
	}
	size := chordDiameter(ctx, meshbool.OpUnion, a, b, diameter)
	read := func(b *topo.Body) float64 { return operandChord(ctx, b, capDelta*chordScale, size, chordScale) }
	ap, bp := read(plate), read(disk)
	tol := min(ap, bp) / 2
	// An offset of order chord²/r keeps even a tangent line's intersection
	// displacement at the scale of the chord tolerance. The exact native root
	// check below determines the bound independently of this proposal.
	scale := 1 + (tol/radius)*(tol/radius)/4
	if proofbound.IsNonFinite(scale) {
		return Evaluation{}, false, nil
	}
	arc.Start.U *= scale
	arc.Start.V *= scale
	arc.End.U *= scale
	arc.End.V *= scale
	if proofbound.IsNonFinite(arc.Start.U) || proofbound.IsNonFinite(arc.Start.V) || proofbound.IsNonFinite(arc.End.U) || proofbound.IsNonFinite(arc.End.V) {
		return Evaluation{}, false, nil
	}
	rs := stationCoords(arc.Start).radius2()
	r2 := stationMul(stationR(radius), stationR(radius))
	if rs.Cmp(stationCoords(arc.End).radius2()) != 0 || rs.Cmp(r2) <= 0 {
		return Evaluation{}, false, nil
	}
	// (R'²-R²)/R bounds R'-R without using a rounded square root. Both
	// surrogate arcs have exactly the same squared radius, so their union is
	// the full circle and this radial transport covers caps, walls and trims.
	delta := proofbound.RatFloatUp(new(big.Rat).Quo(stationSub(rs, r2), stationR(radius)))
	if !(delta > 0 && delta < tol/2) || math.IsInf(scale, 0) {
		return Evaluation{}, false, nil
	}
	complement := arc
	complement.Start, complement.End = arc.End, arc.Start
	q.Profile.Outer.Segments = []sectionrecord.CurveSegment{arc, complement}
	p.Walks = nil
	q.Walks = nil
	q.SectionDelta = delta
	pm, err := stationBuild(ctx, plate, p, tol, false, budget)
	if pm == nil || err != nil {
		return Evaluation{}, false, err
	}
	dm, err := stationBuild(ctx, disk, q, tol, true, budget)
	if dm == nil || err != nil {
		return Evaluation{}, false, err
	}
	arcFace, err := stationRole(ctx, plate, fmt.Sprintf("side(0,%d)", index), budget)
	if err != nil || arcFace == nil {
		return Evaluation{}, false, err
	}
	ok, err = stationHeldCrossings(ctx, pm, dm, arcFace, rims, budget)
	if !ok || err != nil {
		return Evaluation{}, false, err
	}
	if plate == b {
		for i := range rims {
			r := &rims[i]
			r.Side, r.Disk = r.Disk, r.Side
		}
	}
	alt := &stationMesher{meshes, stationOperand{plate, pm, ap}, stationOperand{disk, dm, bp}, rims, budget}
	result, err := evaluate(ctx, meshbool.OpUnion, a, b, alt, chordScale, nil, nil, &operandTwins{}, nil, nil, nil, nil, 0)
	if err != nil {
		return Evaluation{}, false, err
	}
	return result, radialOverlap(a, b, result), nil
}

func stationTrialDeclined(err error) bool {
	var expected *meshbool.BooleanExpectedError
	var limit *proofbound.WorkLimitError
	return errors.As(err, &expected) || errors.As(err, &limit) || errors.Is(err, freeform.ErrTooManyChords) || errors.Is(err, errStationsInapplicable)
}
