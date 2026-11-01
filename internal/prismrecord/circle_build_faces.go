package prismrecord

import (
	"context"
	"fmt"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// circleBuildFaces names the actual faces minted by the primitive builder.
// It does not by itself authenticate the payload, placement or source history.
type circleBuildFaces struct {
	body                 *topo.Body
	wall, start, end     *topo.Face
	circle               sectionrecord.CircleSeg
	frame                r3.Frame
	placement            r3.Transform
	z0, z1               float64
	z0denoted, z1denoted prismplacement.LevelDenotation
	doc                  *topo.Document
	origin               provenance.Ref
	walks                *momentinput.ProfileWalks
	walk                 circleWalkSeal
	tokens               [3]topo.PrismFaceToken
	lump                 *topo.Lump
	shell                *topo.FaceShell
}

type circleWalkSeal struct {
	values                         [29]uint64
	kind                           survey2d.WalkKind
	closed, reversed, interpolated bool
}

func circleWalkSignature(w survey2d.SegmentWalk) circleWalkSeal {
	values := [29]float64{w.StartU, w.StartV, w.EndU, w.EndV, w.StartBound.U, w.StartBound.V,
		w.EndBound.U, w.EndBound.V, w.TanInU, w.TanInV, w.TanOutU, w.TanOutV, w.TanInBound,
		w.TanOutBound, w.Length, w.LengthBound, w.LengthUpper, w.CoordUpper, w.AxisRadiusUpper,
		w.AxisMomentUpper, w.StartVBound, w.EndVBound, w.CVBound, w.CU, w.CV, w.Radius, w.RadiusBound, w.Th0, w.Th1}
	var out circleWalkSeal
	for i, v := range values {
		out.values[i] = math.Float64bits(v)
	}
	out.kind, out.closed, out.reversed, out.interpolated = w.Kind, w.Closed, w.Reversed, w.FitInterpolated
	return out
}

func (w *circleBuildFaces) captureWalk(p Payload) bool {
	if p.Walks == nil || !p.Walks.Metered || p.Walks.Spent != 0 || p.Walks.ReconstructionSpent != 0 ||
		len(p.Walks.Outer) != 1 || len(p.Walks.Holes) != 0 || len(p.Walks.ReadCharges) != 0 ||
		len(p.Walks.Profile.Holes) != 0 || len(p.Walks.Profile.Outer.Segments) != 1 {
		return false
	}
	segment, ok := p.Walks.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	if !ok || !sameCircleSegment(segment, w.circle) {
		return false
	}
	walk := p.Walks.Outer[0]
	radius, err := segment.Radius.In(units.Millimeter)
	if err != nil || walk.Kind != survey2d.WalkCircular || !walk.Closed || len(walk.Spans) != 0 ||
		walk.CU != segment.Center.U || walk.CV != segment.Center.V || walk.Radius != radius ||
		walk.Th0 != 0 || walk.Th1 != 2*math.Pi {
		return false
	}
	w.walk = circleWalkSignature(walk)
	return true
}

func (w *circleBuildFaces) matchesWalk(p Payload) bool {
	if p.Walks == nil || p.Walks != w.walks || !p.Walks.Metered || p.Walks.Spent != 0 || p.Walks.ReconstructionSpent != 0 ||
		len(p.Walks.Outer) != 1 || len(p.Walks.Holes) != 0 || len(p.Walks.ReadCharges) != 0 ||
		len(p.Walks.Profile.Holes) != 0 || len(p.Walks.Profile.Outer.Segments) != 1 || len(p.Walks.Outer[0].Spans) != 0 {
		return false
	}
	segment, ok := p.Walks.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	return ok && sameCircleSegment(segment, w.circle) && circleWalkSignature(p.Walks.Outer[0]) == w.walk
}

// CircleBuildWitness retains construction-time payload and face identities.
type CircleBuildWitness struct{ inner *circleBuildFaces }

// SealCircleBuildWitness records an eligible body immediately after its producer.
func SealCircleBuildWitness(body *topo.Body) *CircleBuildWitness {
	if body == nil {
		return nil
	}
	p, ok := body.Payload.(Payload)
	if !ok || p.SurfaceResult || len(p.Profile.Holes) != 0 || len(p.Profile.Outer.Segments) != 1 || len(body.Lumps) != 1 || body.Lumps[0] == nil || len(body.Lumps[0].Shells) != 1 || body.Lumps[0].Shells[0] == nil {
		return nil
	}
	c, ok := p.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	if !ok || !c.CCW || c.TStart != 0 || c.TEnd != 1 {
		return nil
	}
	f := body.Lumps[0].Shells[0].Faces
	if len(f) != 3 {
		return nil
	}
	w := &circleBuildFaces{body: body, wall: f[0], start: f[1], end: f[2], circle: c, frame: p.Frame, placement: p.Xform, z0: p.Z0, z1: p.Z1, z0denoted: p.Z0Denoted, z1denoted: p.Z1Denoted, doc: body.Doc, origin: body.Origin, walks: p.Walks}
	w.lump, w.shell = body.Lumps[0], body.Lumps[0].Shells[0]
	if !w.captureWalk(p) {
		return nil
	}
	for i, face := range f {
		if face == nil {
			return nil
		}
		token, valid := face.PrismConstruction()
		kind, loop, seg := topo.PrismFaceWall, 0, 0
		if i == 1 {
			kind, loop, seg = topo.PrismFaceCapStart, -1, -1
		}
		if i == 2 {
			kind, loop, seg = topo.PrismFaceCapEnd, -1, -1
		}
		if !valid || !token.Matches(face, body, body.Origin.Producer, kind, loop, seg) || !token.MatchesSealed(face) {
			return nil
		}
		w.tokens[i] = token
	}
	return &CircleBuildWitness{inner: w}
}

func (w *circleBuildFaces) matchesTopology(ctx context.Context, body *topo.Body, budget *proofbound.ReplayBudget) (bool, error) {
	if len(body.Lumps) != 1 || body.Lumps[0] != w.lump || w.lump == nil || len(w.lump.Shells) != 1 || w.lump.Shells[0] != w.shell || w.shell == nil {
		return false, nil
	}
	shell := body.Lumps[0].Shells[0]
	if shell.Open || shell.Void || len(shell.Faces) != 3 {
		return false, nil
	}
	faces := [3]*topo.Face{w.wall, w.start, w.end}
	for i, f := range faces {
		if shell.Faces[i] != f || !w.tokens[i].MatchesSealed(f) {
			return false, nil
		}
		valid, err := w.tokens[i].ReplayMatchesTopology(ctx, f, budget)
		if err != nil || !valid {
			return false, err
		}
	}
	return true, nil
}

var errCircleBuildFaces = fmt.Errorf("%w: circle construction faces unavailable", decaderr.ErrUnsupported)

// This covers a fixed one-circle record, three faces, four loops and their
// edge/vertex fields. Rational level checks are charged separately.
const circleIdentityWork = 4096

// CircleBuildFacesSnapshot seals builder face identities and the held circle
// record. Walk contents, displacement bounds, topology and history need proof.
type CircleBuildFacesSnapshot struct {
	budget *proofbound.ReplayBudget
	build  *circleBuildFaces
	faces  [3]*topo.Face
}

// CaptureCircleBuildFaces captures identities from the original builder.
func CaptureCircleBuildFaces(ctx context.Context, body *topo.Body, witness *CircleBuildWitness, budget *proofbound.ReplayBudget) (*CircleBuildFacesSnapshot, error) {
	if budget == nil || !budget.IsCircleReplay() || body == nil {
		return nil, errCircleBuildFaces
	}
	if err := budget.SpendWork(ctx, circleIdentityWork); err != nil {
		return nil, err
	}
	if witness == nil {
		return nil, errCircleBuildFaces
	}
	w := witness.inner
	if w == nil || w.body != body || w.wall == nil || w.start == nil || w.end == nil ||
		w.wall == w.start || w.wall == w.end || w.start == w.end ||
		w.wall.Body != body || w.start.Body != body || w.end.Body != body || !w.matchesRecord(body) {
		return nil, errCircleBuildFaces
	}
	valid, err := w.matchesTopology(ctx, body, budget)
	if err != nil {
		return nil, err
	}
	if !valid {
		return nil, errCircleBuildFaces
	}
	if err := w.validateLevels(ctx, body, budget); err != nil {
		return nil, err
	}
	if err := budget.ReserveRecords(ctx, 1); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(CircleBuildFacesSnapshot{}))); err != nil {
		return nil, err
	}
	return &CircleBuildFacesSnapshot{budget: budget, build: w, faces: [3]*topo.Face{w.wall, w.start, w.end}}, nil
}

// Faces rechecks the body and builder witness under the capturing meter.
func (s *CircleBuildFacesSnapshot) Faces(ctx context.Context, body *topo.Body, budget *proofbound.ReplayBudget) ([3]*topo.Face, error) {
	if s == nil || budget == nil || budget != s.budget || body == nil || s.build == nil {
		return [3]*topo.Face{}, errCircleBuildFaces
	}
	if err := budget.SpendWork(ctx, circleIdentityWork); err != nil {
		return [3]*topo.Face{}, err
	}
	w := s.build
	if w.body != body || [3]*topo.Face{w.wall, w.start, w.end} != s.faces ||
		w.wall.Body != body || w.start.Body != body || w.end.Body != body || !w.matchesRecord(body) {
		return [3]*topo.Face{}, errCircleBuildFaces
	}
	valid, err := w.matchesTopology(ctx, body, budget)
	if err != nil {
		return [3]*topo.Face{}, err
	}
	if !valid {
		return [3]*topo.Face{}, errCircleBuildFaces
	}
	if err := w.validateLevels(ctx, body, budget); err != nil {
		return [3]*topo.Face{}, err
	}
	return s.faces, nil
}

// SameCircleSource compares the held source record across a placement. It does
// not authenticate the placement operation or bound its rounding.
func SameCircleSource(ctx context.Context, a, b *CircleBuildFacesSnapshot, source, placed *topo.Body, budget *proofbound.ReplayBudget) (bool, error) {
	if _, err := a.Faces(ctx, source, budget); err != nil {
		return false, err
	}
	if _, err := b.Faces(ctx, placed, budget); err != nil {
		return false, err
	}
	if err := budget.SpendWork(ctx, 64); err != nil {
		return false, err
	}
	x, y := a.build, b.build
	return x.doc == y.doc && sameCircleSegment(x.circle, y.circle) && sameCircleFrame(x.frame, y.frame) &&
		math.Float64bits(x.z0) == math.Float64bits(y.z0) && math.Float64bits(x.z1) == math.Float64bits(y.z1) &&
		x.walks == y.walks && x.walk == y.walk, nil
}

// matchesRecord accepts the zero-displacement circle record only. A valid
// placement is captured verbatim; its accumulated rounding is not proven here.
func (w *circleBuildFaces) matchesRecord(body *topo.Body) bool {
	p, ok := body.Payload.(Payload)
	if !ok || !body.Solid || body.Doc != w.doc || body.Origin != w.origin || len(body.OperationWarnings) != 0 || p.SurfaceResult ||
		len(p.Profile.Holes) != 0 || len(p.Profile.Outer.Segments) != 1 || len(p.BlendSegs) != 0 ||
		p.BlendKind != "" || len(p.Upstream) != 0 || math.Float64bits(p.SectionDelta) != 0 ||
		math.Float64bits(p.Z0Delta) != 0 || math.Float64bits(p.Z1Delta) != 0 ||
		p.Z0Denoted != w.z0denoted || p.Z1Denoted != w.z1denoted ||
		!sameCircleFrame(p.Frame, w.frame) || !sameCirclePlacement(p.Xform, w.placement) ||
		math.Float64bits(p.Z0) != math.Float64bits(w.z0) || math.Float64bits(p.Z1) != math.Float64bits(w.z1) || !w.matchesWalk(p) ||
		!p.Frame.IsValid() || !p.Xform.IsValid() || proofbound.IsNonFinite(p.Z0) || proofbound.IsNonFinite(p.Z1) || p.Z1 <= p.Z0 {
		return false
	}
	circle, ok := p.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	return ok && sameCircleSegment(circle, w.circle) && circle.CCW && circle.TStart == 0 && circle.TEnd == 1 &&
		!proofbound.IsNonFinite(circle.Center.U) && !proofbound.IsNonFinite(circle.Center.V) &&
		!proofbound.IsNonFinite(circle.Radius.Base()) && circle.Radius.Base() > 0
}

func (w *circleBuildFaces) validateLevels(ctx context.Context, body *topo.Body, budget *proofbound.ReplayBudget) error {
	p, ok := body.Payload.(Payload)
	if !ok {
		return errCircleBuildFaces
	}
	if w.start == nil || w.end == nil || w.start.CapRecord == nil || w.end.CapRecord == nil {
		return errCircleBuildFaces
	}
	if err := prismplacement.ValidateZeroLevelReplay(ctx, p.Z0, p.Z0Denoted, budget); err != nil {
		return err
	}
	if err := prismplacement.ValidateZeroLevelReplay(ctx, p.Z1, p.Z1Denoted, budget); err != nil {
		return err
	}
	if err := prismplacement.ValidateHeldRationalReplay(ctx, p.Z0, w.start.CapRecord.ZDenoted, budget); err != nil {
		return err
	}
	return prismplacement.ValidateHeldRationalReplay(ctx, p.Z1, w.end.CapRecord.ZDenoted, budget)
}

func sameCircleSegment(a, b sectionrecord.CircleSeg) bool {
	return a.Radius == b.Radius && a.CCW == b.CCW && math.Float64bits(a.Center.U) == math.Float64bits(b.Center.U) &&
		math.Float64bits(a.Center.V) == math.Float64bits(b.Center.V) &&
		math.Float64bits(a.TStart) == math.Float64bits(b.TStart) && math.Float64bits(a.TEnd) == math.Float64bits(b.TEnd)
}

func sameCircleVec(a, b r3.Vec) bool {
	return math.Float64bits(a.X) == math.Float64bits(b.X) && math.Float64bits(a.Y) == math.Float64bits(b.Y) &&
		math.Float64bits(a.Z) == math.Float64bits(b.Z)
}

func sameCircleFrame(a, b r3.Frame) bool {
	return sameCircleVec(a.Origin(), b.Origin()) && sameCircleVec(a.U(), b.U()) && sameCircleVec(a.V(), b.V()) && sameCircleVec(a.N(), b.N())
}

func sameCirclePlacement(a, b r3.Transform) bool {
	x, y := a.Basis(), b.Basis()
	return sameCircleVec(a.Translation(), b.Translation()) && sameCircleVec(x.EX, y.EX) &&
		sameCircleVec(x.EY, y.EY) && sameCircleVec(x.EZ, y.EZ)
}
