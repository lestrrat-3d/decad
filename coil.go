package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/coil"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// This file is docs/helix-design.md's entry point: the CoilOption surface,
// Table CS's gates in §4's order, the coilPayload that records a coil and
// its placement. internal/coilshell builds §5's held shell and coil_body.go
// publishes Table CB's topology and Table CM's readings.

// coilStationsPerTurn is §5.2's station density: 256 stations per turn, a
// power of two, keeps the chord departure ρ·(π/256)²/2 ≈ 7.5e-5·ρ two decades
// below the default Verify tolerance.
const coilStationsPerTurn = 256

// coilArcChordsPerTurn is the number of chords a full turn of a circular
// profile segment is cut into (docs/helix-design.md §5.4, §11 PR 3): an arc
// of sweep Δ takes ⌈coilArcChordsPerTurn·Δ/2π⌉. Every chord cell enters the
// crossing audit beside the helix stations, so the count is held where a
// round-wire spring of a few turns stays inside the audit's pair budget.
const coilArcChordsPerTurn = 16

// maxCoilStations is Table CS row CS8's station ceiling: 128 turns at
// coilStationsPerTurn.
const maxCoilStations = 1 << 15

// maxCoilFacets is Table CS row CS8's facet ceiling over the wall and cap
// triangles together. Every triangle enters the crossing audit's exact pair
// classification and every station a 200-bit trig enclosure, so the
// ceiling bounds the build's time and memory before anything is allocated.
const maxCoilFacets = 1 << 20

// CoilOption configures Coil. Sealed: WithLeftHand is the one option.
// WithSurfaceResult does not implement it, so a coil is always a solid
// (Table CS row CS11).
type CoilOption interface {
	coilOption()
}

type leftHandOption struct{}

func (leftHandOption) coilOption() {}

// WithLeftHand turns the section left-handed about the axis direction while
// it advances along it. The default is right-handed. Passing it twice is
// ErrDegenerate.
func WithLeftHand() CoilOption {
	return leftHandOption{}
}

func decodeCoilOptions(opts []CoilOption) (bool, error) {
	leftHand := false
	for _, raw := range opts {
		if raw == nil {
			return false, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
		if _, ok := raw.(leftHandOption); !ok {
			return false, fmt.Errorf(`%w: the coil option is not a decad coil option (%T)`, ErrDegenerate, raw)
		}
		if leftHand {
			return false, fmt.Errorf(`%w: WithLeftHand was passed more than once`, ErrDegenerate)
		}
		leftHand = true
	}
	return leftHand, nil
}

// coilPayload is the evaluator's record of a coil (docs/helix-design.md):
// the authenticated profile and its plane frame, the axis resolved into the
// plane with the side that puts the profile on its positive side, the pitch,
// the turn count, the hand and the accumulated placement — what a placement
// re-runs §5 from — plus the build that record produced.
//
// verts is the held vertex table, station-major (§5.3); vertexBound is each
// vertex's β of §5.4 and delta the largest of them; maxRound is the largest
// station rounding; tris is the globally oriented held triangle set, walls
// first, then capStart and capEnd.
type coilPayload struct {
	profile  profileRecord
	frame    r3.Frame
	line     axisLine2
	side     int
	pitch    float64
	turns    float64
	leftHand bool
	xform    r3.Transform

	verts       []r3.Vec
	vertexBound []float64
	tris        [][3]int
	delta       float64
	maxRound    float64
}

func (cp coilPayload) transform() r3.Transform { return cp.xform }

// placed re-runs §5 from the record under the composed motion (§5.6, Table CD
// row CD7): the stations, the held table, β, δ, the orientation and the audit
// are rebuilt, so δ never accumulates across placements. Volume and Area
// read the record's closed forms under the composed map (§5.3), so they move
// only by that map's determinant and defect.
func (cp coilPayload) placed(ctx context.Context, d *Document, ref producerID, composed r3.Transform) (*Body, error) {
	next := coilPayload{
		profile:  cp.profile,
		frame:    cp.frame,
		line:     cp.line,
		side:     cp.side,
		pitch:    cp.pitch,
		turns:    cp.turns,
		leftHand: cp.leftHand,
		xform:    composed,
	}
	return evalCoil(ctx, d, ref, next)
}

// Coil moves the closed profile p of sketch s along the screw motion about
// axis (docs/helix-design.md): turns full rotations about the axis,
// right-handed about the axis direction, advancing pitch along that
// direction per rotation. The section stays in the axis plane throughout, so
// a thread profile drawn beside the axis cuts the thread it draws. The axis
// is Revolve's own vocabulary — SketchLine, ConstructionAxis or EdgeAxis —
// and MUST lie in the sketch plane.
//
// pitch MUST be a finite positive length and turns a finite positive
// dimensionless count, whole or fractional (ErrUnitKind for the wrong kind,
// ErrNotFinite for a non-finite value, ErrDegenerate at or below zero). The
// profile MUST lie strictly on one side of the axis: a profile proven to
// touch or cross it is ErrDegenerate, and one whose side the axis's own
// rounding leaves undecided is ErrUnsupported. With one turn or more, the
// profile's extent along the axis MUST stay below the pitch, or two turns
// would meet (ErrUnsupported). Every profile segment MUST be a whole LineSeg,
// ArcSeg or CircleSeg (ErrUnsupported); an arc is held as 16 chords per turn
// of its sweep. A coil needing more than 32768 stations at 256 per turn, or
// more than 1048576 triangles, is ErrUnsupported, as is one whose held shell
// crosses itself where two turns pass closer than the station chords
// resolve, or whose crossing audit passes its triangle or pair ceiling.
//
// The result is a solid: two planar caps, one faceted wall per profile
// segment spanning every turn, read through Table CM's closed forms and, on
// an arc, docs/helix-design.md §11.1's area bracket. A failed call leaves
// the document untouched.
func (d *Document) Coil(ctx context.Context, s *sketch.Sketch, p *sketch.Profile, axis Axis, pitch, turns units.Value, opts ...CoilOption) (*Body, error) {
	// CS1: nils, owned options and option arity.
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a coil`, ErrDegenerate)
	}
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	if s == nil || p == nil {
		return nil, fmt.Errorf(`%w: Coil requires a non-nil sketch and profile`, ErrDegenerate)
	}
	axis, err := normalizeAxis(axis)
	if err != nil {
		return nil, err
	}
	leftHand, err := decodeCoilOptions(opts)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	// CS2: the profile seam, with the live-profile area falsifier every
	// profile-fed build runs.
	profile, plane, profileArea, err := recordProfile(s, p)
	if err != nil {
		return nil, err
	}
	work := freeform.NewFreeformWork()
	if _, err := falsifyRecordedArea(profile, profileArea, work); err != nil {
		return nil, err
	}

	// CS4: pitch and turns.
	pitchMM, err := coilMagnitude(pitch, units.Length, units.Millimeter, "the coil pitch")
	if err != nil {
		return nil, err
	}
	turnCount, err := coilMagnitude(turns, units.Dimensionless, units.One, "the coil turn count")
	if err != nil {
		return nil, err
	}

	// CS3: the axis, resolved into the plane exactly as Revolve resolves it.
	evalAxis := axis
	if ea, ok := axis.(EdgeAxis); ok {
		line, err := d.resolveEdgeAxis(ea)
		if err != nil {
			return nil, err
		}
		evalAxis = line
	}
	frame, err := r3.NewFrame(plane.Origin, plane.U, plane.V)
	if err != nil {
		return nil, fmt.Errorf(`%w: the recorded plane is degenerate: %s`, ErrDegenerate, err)
	}
	line, err := axisInPlane(evalAxis, frame)
	if err != nil {
		return nil, err
	}

	// CS5 and CS6.
	side, err := coilSide(ctx, profile, line, pitchMM, turnCount, work)
	if err != nil {
		return nil, err
	}

	// CS7 and CS8.
	prof, err := coil.Loops(profile.Outer, profile.Holes, coilArcChordsPerTurn)
	if err != nil {
		return nil, err
	}
	if err := coilPreflight(turnCount, len(prof.Pts), len(prof.LoopIdx)-1); err != nil {
		return nil, err
	}

	body, err := evalCoil(ctx, d, d.nextProducerID(), coilPayload{
		profile:  profile,
		frame:    frame,
		line:     line,
		side:     side,
		pitch:    pitchMM,
		turns:    turnCount,
		leftHand: leftHand,
		xform:    r3.Identity(),
	})
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(body)
	return body, nil
}

// coilMagnitude is Table CS row CS4 over one value: its kind, its
// finiteness, then its sign. Zero is ErrDegenerate rather than a reduction:
// a zero pitch names a Revolve, and a zero turn count names no solid.
func coilMagnitude(v units.Value, kind units.Kind, unit units.Unit, what string) (float64, error) {
	if v.Kind() != kind {
		return 0, fmt.Errorf(`%w: %s must be a %s, got %s`, ErrUnitKind, what, kind, v.Kind())
	}
	m, err := v.In(unit)
	if err != nil || math.IsNaN(m) || math.IsInf(m, 0) {
		return 0, fmt.Errorf(`%w: %s is not finite`, ErrNotFinite, what)
	}
	if m <= 0 {
		if what == "the coil pitch" {
			return 0, fmt.Errorf(`%w: %s must be positive, got %s; a zero pitch is a Revolve`, ErrDegenerate, what, v)
		}
		return 0, fmt.Errorf(`%w: %s must be positive, got %s`, ErrDegenerate, what, v)
	}
	return m, nil
}

// coilSide is Table CS rows CS5 and CS6 (§5.1): it reads the profile's
// radial and axial extremes about the resolved axis, decides the side
// through revolveaxis.ResolveSide, and requires the near-axis extreme proven
// strictly positive. With one turn or more it requires the axial extent's
// upper bound below the pitch (CP5). Both are reject-only readings off the
// profile's own record; neither admits a profile the proof does not cover.
func coilSide(ctx context.Context, profile profileRecord, line axisLine2, pitch, turns float64, work *freeform.FreeformWork) (int, error) {
	nU, nV := -line.dV, line.dU
	rlo, rhi, rBound, err := boundaryExtremesBoundedContext(ctx, profile, nU, nV, work, nil)
	if err != nil {
		return 0, err
	}
	zlo, zhi, zBound, err := boundaryExtremesBoundedContext(ctx, profile, line.dU, line.dV, work, nil)
	if err != nil {
		return 0, err
	}
	coordUpper, err := momentinput.CoordinateEnvelope(profile, work, nil)
	if err != nil {
		return 0, err
	}
	resolved, err := revolveaxis.ResolveSide(revolveaxis.Line2{
		AU: line.aU, AV: line.aV, AUBound: line.aUBound, AVBound: line.aVBound,
		DU: line.dU, DV: line.dV, DUBound: line.dUBound, DVBound: line.dVBound,
	}, revolveaxis.SideExtremes{
		RLo: rlo, RHi: rhi, RBound: rBound,
		ZLo: zlo, ZHi: zhi, ZBound: zBound,
		CoordUpper: coordUpper,
	})
	if err != nil {
		return 0, err
	}
	switch proofbound.AdmitAbove(resolved.Near, 0) {
	case proofbound.SurvReject:
		return 0, fmt.Errorf(`%w: the coil profile touches its axis; a point on the axis sweeps to a segment of it and pinches the wall`, ErrDegenerate)
	case proofbound.SurvStraddle:
		return 0, fmt.Errorf(`%w: the coil profile's distance from its axis is known only to ±%v mm, which does not prove it off the axis`, ErrUnsupported, resolved.Near.Bound)
	}
	if turns >= 1 && resolved.AxialExtentUpper >= pitch {
		return 0, fmt.Errorf(`%w: the coil profile reaches %v mm along its axis, at or past the %v mm pitch, so its %v turns would meet; a profile narrower than one pitch is required at one turn or more`,
			ErrUnsupported, resolved.AxialExtentUpper, pitch, turns)
	}
	return int(resolved.Side), nil
}

// coilPreflight is Table CS row CS8, decided from counts alone before any
// allocation.
func coilPreflight(turns float64, vertices, holes int) error {
	n, ok := coil.StationCount(proofarith.FloatRat(turns), coilStationsPerTurn)
	if !ok || n > maxCoilStations {
		return fmt.Errorf(`%w: a coil of %v turns needs more than the %d-station ceiling at %d stations per turn`,
			ErrUnsupported, turns, maxCoilStations, coilStationsPerTurn)
	}
	walls := 2 * n * int64(vertices)
	caps := 2 * int64(vertices+2*holes-2)
	if walls+caps > maxCoilFacets {
		return fmt.Errorf(`%w: the coil's %d triangles exceed the %d-triangle ceiling`, ErrUnsupported, walls+caps, maxCoilFacets)
	}
	return nil
}
