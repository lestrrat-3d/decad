package apitest_test

import (
	"math"
	"sort"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These tests cover docs/modify-reach-design.md §13's "Options", "Tangent
// chain" and "Asymmetric chamfer" lists through the public API.
//
// Legs shown to fail before these fixtures were accepted: dropping the
// tangent expansion from Chamfer sends every chain build red (the seed reads
// as a partial cap loop); returning DegYes from the oracle's undecided band,
// in both the tangent and the face-normal test, builds the near-tangent slot
// instead of refusing SX2; assigning the
// positional distance to the arriving walk regardless of the reference face
// sends the reference-swap and revolve tests red (the two feet trade
// places); skipping the deep copy in WithAsymmetricChamfer sends both
// selector-copy subtests red, and sharing the caller's branch list with spare
// capacity sends the union one red; and resolving the reference without the
// one-face-per-edge count builds the dual and extra references.

// slotBody extrudes a stadium: straight walls from (0,0) to (10,0) and from
// (10,10) to (0,10), a semicircle about (0,5) on the left, and an arc from
// (10,0) to (10,10) about rightCenter on the right. rightCenter (10,5) makes
// every join an exactly tangent one.
// slotHeight is every slot's sweep height.
const slotHeight = 10.0

func slotBody(t *testing.T, rightCenter [2]float64) (*decad.Document, *decad.Body) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	fixed := func(x, y float64) *sketch.Point {
		p := s.CreatePoint(x, y)
		s.Fix(p)
		return p
	}
	p0, p1, p2, p3 := fixed(0, 0), fixed(10, 0), fixed(10, 10), fixed(0, 10)
	cr, cl := fixed(rightCenter[0], rightCenter[1]), fixed(0, 5)
	s.CreateLine(p0, p1)
	s.CreateArc(cr, p1, p2)
	s.CreateLine(p2, p3)
	s.CreateArc(cl, p3, p0)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(slotHeight), Dir: decad.Along})
	require.NoError(t, err)
	return doc, body
}

// slotSeed names the one straight top-cap edge of a slot from (0,0) to
// (10,0), asserting exactly one match.
func slotSeed(body *decad.Body) *decad.EdgeQuery {
	return decad.Edges(
		decad.CreatedBy(decad.CapEnd(body)),
		decad.ParallelTo(r3.NewVec(1, 0, 0)),
		decad.EndpointAt(r3.NewVec(0, 0, slotHeight)),
	).Exactly(1)
}

// ovalBody extrudes, by slotHeight, a four-arc oval whose joins are exactly tangent: arcs of
// radius 5 about (±3, 0) and of radius 10 about (0, ∓4), meeting at (±6, ±4)
// — each join lies on the line through its two centres, a 3-4-5 triangle.
func ovalBody(t *testing.T) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	fixed := func(x, y float64) *sketch.Point {
		p := s.CreatePoint(x, y)
		s.Fix(p)
		return p
	}
	ne, nw, sw, se := fixed(6, 4), fixed(-6, 4), fixed(-6, -4), fixed(6, -4)
	s.CreateArc(fixed(0, -4), ne, nw)
	s.CreateArc(fixed(-3, 0), nw, sw)
	s.CreateArc(fixed(0, 4), sw, se)
	s.CreateArc(fixed(3, 0), se, ne)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(slotHeight), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

func volumeMM3(t *testing.T, b *decad.Body) float64 {
	t.Helper()
	v, err := b.Volume()
	require.NoError(t, err)
	mm3, err := v.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	return mm3
}

func TestModifyOptionGates(t *testing.T) {
	t.Parallel()
	t.Run("NilOption", func(t *testing.T) {
		doc, box := filletBox(t)
		_, err := box.Fillet(t.Context(), verticalEdges(), units.Millimeters(5), nil)
		require.ErrorIs(t, err, decad.ErrDegenerate)
		_, err = box.Chamfer(t.Context(), verticalEdges(), units.Millimeters(5), nil)
		require.ErrorIs(t, err, decad.ErrDegenerate)
		_, err = box.Shell(t.Context(), decad.Faces(decad.FaceCreatedBy(decad.CapEnd(box))), units.Millimeters(5), nil)
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.Equal(t, []*decad.Body{box}, doc.Bodies())
	})
	t.Run("DuplicateAsymmetric", func(t *testing.T) {
		doc, box := filletBox(t)
		ref := decad.Faces(decad.Facing(r3.NewVec(1, 0, 0)))
		_, err := box.Chamfer(t.Context(), verticalEdges(), units.Millimeters(3),
			decad.WithAsymmetricChamfer(ref, units.Millimeters(5)),
			decad.WithAsymmetricChamfer(ref, units.Millimeters(5)))
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.ErrorContains(t, err, "SX1")
		require.Equal(t, []*decad.Body{box}, doc.Bodies())
	})
	t.Run("ContradictorySense", func(t *testing.T) {
		doc, box := filletBox(t)
		_, err := box.Shell(t.Context(), decad.Faces(decad.FaceCreatedBy(decad.CapEnd(box))), units.Millimeters(5),
			decad.WithShellSense(decad.Inward), decad.WithShellSense(decad.Outward))
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.ErrorContains(t, err, "SX1")
		require.Equal(t, []*decad.Body{box}, doc.Bodies())
	})
	t.Run("RepeatedSameIntent", func(t *testing.T) {
		_, box := filletBox(t)
		cup, err := box.Shell(t.Context(), decad.Faces(decad.FaceCreatedBy(decad.CapEnd(box))), units.Millimeters(5),
			decad.WithShellSense(decad.Outward), decad.WithShellSense(decad.Outward))
		require.NoError(t, err)
		// Outward: the dilated section — the plate, a 5 mm band along its
		// 320 mm perimeter and four quarter disks at its corners — over
		// [−5, 20], less the original box.
		decadtest.MeasuresVolume(t, cup, units.CubicMillimeters((100*60+5*320+25*math.Pi)*25-100*60*20), decadtest.Within(units.CubicMillimeters(1e-6)))

		_, slot := slotBody(t, [2]float64{10, 5})
		once := volumeMM3(t, mustChamfer(t, slot, slotSeed(slot), 2, decad.WithTangentChain(), decad.WithTangentChain()))
		_, slot2 := slotBody(t, [2]float64{10, 5})
		require.Equal(t, once, volumeMM3(t, mustChamfer(t, slot2, slotSeed(slot2), 2, decad.WithTangentChain())))
	})
	t.Run("OtherDistanceGates", func(t *testing.T) {
		doc, box := filletBox(t)
		ref := decad.Faces(decad.Facing(r3.NewVec(1, 0, 0)))
		for _, other := range []units.Value{units.Millimeters(0), units.Millimeters(-1), units.Degrees(3)} {
			_, err := box.Chamfer(t.Context(), cornerEdge(), units.Millimeters(3), decad.WithAsymmetricChamfer(ref, other))
			require.Error(t, err)
		}
		require.Equal(t, []*decad.Body{box}, doc.Bodies())
	})
}

func mustChamfer(t *testing.T, b *decad.Body, sel decad.EdgeSelector, d float64, opts ...decad.ChamferOption) *decad.Body {
	t.Helper()
	out, err := b.Chamfer(t.Context(), sel, units.Millimeters(d), opts...)
	require.NoError(t, err)
	return out
}

func TestShellNoOpenings(t *testing.T) {
	t.Parallel()
	t.Run("NilSelectorAccepted", func(t *testing.T) {
		// The option lifts base S16's nil-selector refusal: a hole-free prism
		// builds its closed shell (reach BX5), through a nil selector and a
		// typed nil alike.
		var typedNil *decad.FaceQuery
		for _, sel := range []decad.FaceSelector{nil, typedNil} {
			doc, box := filletBox(t)
			closed, err := box.Shell(t.Context(), sel, units.Millimeters(5), decad.WithNoOpenings())
			require.NoError(t, err)
			require.Len(t, closed.Shells(), 2)
			require.Equal(t, []*decad.Body{closed}, doc.Bodies())
		}
	})
	t.Run("NilSelectorWithoutOption", func(t *testing.T) {
		_, box := filletBox(t)
		_, err := box.Shell(t.Context(), nil, units.Millimeters(5))
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
	t.Run("SelectorConflicts", func(t *testing.T) {
		doc, box := filletBox(t)
		_, err := box.Shell(t.Context(), decad.Faces(decad.FaceCreatedBy(decad.CapEnd(box))), units.Millimeters(5), decad.WithNoOpenings())
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.ErrorContains(t, err, "SX1")
		require.Equal(t, []*decad.Body{box}, doc.Bodies())
	})
	t.Run("Revolve", func(t *testing.T) {
		// A partial turn's closed shell keeps both angular caps (SX8); a full
		// turn's builds (revolve_shell_test.go).
		doc := decad.New()
		partial := revolveMeridian(t, doc, shaftMeridian, halfTurn)
		_, err := partial.Shell(t.Context(), nil, units.Millimeters(1), decad.WithNoOpenings())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "SX8")
		full := revolveMeridian(t, doc, shaftMeridian, decad.FullRevolution{})
		closed, err := full.Shell(t.Context(), nil, units.Millimeters(1), decad.WithNoOpenings())
		require.NoError(t, err)
		require.Equal(t, []*decad.Body{partial, closed}, doc.Bodies())
	})
	t.Run("HoledSection", func(t *testing.T) {
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		rect := s.CreateRectangle(0, 0, 40, 40)
		s.Fix(rect.A)
		c := s.CreatePoint(20, 20)
		s.Fix(c)
		s.CreateCircle(c, 5)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		doc := decad.New()
		var holed *decad.Body
		for _, p := range s.Profiles() {
			if len(p.Holes) == 1 {
				holed, err = doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
				require.NoError(t, err)
			}
		}
		require.NotNil(t, holed)
		_, err = holed.Shell(t.Context(), nil, units.Millimeters(2), decad.WithNoOpenings())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "SX8")
	})
}

func TestTangentChainSlotCapLoop(t *testing.T) {
	t.Parallel()
	const h, d = slotHeight, 2.0
	_, slot := slotBody(t, [2]float64{10, 5})

	// Without the chain the one seed is a partial cap loop: SX4.
	_, err := slot.Chamfer(t.Context(), slotSeed(slot), units.Millimeters(d))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "only part of a cap loop")

	// Exactly(1) holds on the seed; the line→arc→line→arc chain then expands
	// it to the whole top loop, which builds the cap-loop chamfer.
	chained := mustChamfer(t, slot, slotSeed(slot), d, decad.WithTangentChain())
	requireManifold(t, chained)

	// Pappus: the removed ring has the right-triangle section d²/2, its
	// centroid d/3 inside the wall, so the straight walls remove d²/2·2L and
	// the two semicircles d²/2·2π(R − d/3).
	const L, R = 10.0, 5.0
	removed := d * d / 2 * (2*L + 2*math.Pi*(R-d/3))
	decadtest.MeasuresVolume(t, chained, units.CubicMillimeters((2*R*L+math.Pi*R*R)*h-removed),
		decadtest.Within(units.CubicMillimeters(1e-9)))

	// The expansion selects the same edges the complete loop names: the two
	// bodies are built from one selection and measure bit for bit alike.
	_, whole := slotBody(t, [2]float64{10, 5})
	explicit := mustChamfer(t, whole, capLoopEdges(whole), d)
	require.Equal(t, volumeMM3(t, explicit), volumeMM3(t, chained))
	require.Len(t, chained.Faces(), len(explicit.Faces()))
}

func TestTangentChainOvalArcArc(t *testing.T) {
	t.Parallel()
	const h, d = slotHeight, 1.0
	oval := ovalBody(t)
	seed := decad.Edges(decad.CreatedBy(decad.CapEnd(oval)), decad.EndpointAt(r3.NewVec(6, 4, h)), decad.EndpointAt(r3.NewVec(-6, 4, h))).Exactly(1)
	chained := mustChamfer(t, oval, seed, d, decad.WithTangentChain())
	requireManifold(t, chained)

	// Pappus per arc: the removed triangle d²/2 at radius R − d/3 over the
	// arc's angle; the four angles are 2·atan2(4,3) about the radius-5
	// centres and 2·atan2(3,4) about the radius-10 ones.
	small, big := 2*math.Atan2(4, 3), 2*math.Atan2(3, 4)
	// Green's theorem per arc: ½(r²θ + c × (end − start)).
	area := 100*big + 25*small - 24
	removed := d * d / 2 * (2*small*(5-d/3) + 2*big*(10-d/3))
	decadtest.MeasuresVolume(t, chained, units.CubicMillimeters(area*h-removed), decadtest.Within(units.CubicMillimeters(1e-9)))

	whole := ovalBody(t)
	require.Equal(t, volumeMM3(t, mustChamfer(t, whole, capLoopEdges(whole), d)), volumeMM3(t, chained))
}

func TestTangentChainNearTangentStops(t *testing.T) {
	t.Parallel()
	// The right arc's centre sits 0.1 inside the tangent position, so both
	// of its joins turn left by about 0.02 rad — far above the oracle's
	// noise floor, a proven "no". (sketch itself refuses a region whose join
	// turns by much less than that and much more than the oracle's floor.)
	// The chain from the bottom wall runs through the exact left semicircle
	// to the top wall and stops at both near-tangent joins: three of the four
	// top edges, a partial loop.
	doc, slot := slotBody(t, [2]float64{9.9, 5})
	_, err := slot.Chamfer(t.Context(), slotSeed(slot), units.Millimeters(1), decad.WithTangentChain())
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "only part of a cap loop")
	require.NotContains(t, err.Error(), "SX2")
	require.Equal(t, []*decad.Body{slot}, doc.Bodies())
}

func TestTangentChainUndecidedRefused(t *testing.T) {
	t.Parallel()
	// The right arc's centre sits 1e-10 inside the tangent position: the joins'
	// cross product is exactly nonzero and below the oracle's noise floor, so
	// the continuation is undecided — SX2, never a silent stop.
	doc, slot := slotBody(t, [2]float64{9.9999999999, 5})
	_, err := slot.Chamfer(t.Context(), slotSeed(slot), units.Millimeters(1), decad.WithTangentChain())
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "SX2")
	_, err = slot.Fillet(t.Context(), slotSeed(slot), units.Millimeters(1), decad.WithTangentChain())
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "SX2")
	require.Equal(t, []*decad.Body{slot}, doc.Bodies())
}

// cornerEdge names the lateral edge of the filletBox plate at (100, 60).
func cornerEdge() *decad.EdgeQuery {
	return decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.EndpointAt(r3.NewVec(100, 60, 0))).Exactly(1)
}

// chamferFeet returns the bevel walls' vertices on the z = 0 cap, sorted.
func chamferFeet(t *testing.T, b *decad.Body) []r3.Vec {
	t.Helper()
	var out []r3.Vec
	for _, f := range b.Faces() {
		bevel := false
		for _, o := range f.Origins() {
			bevel = bevel || strings.HasPrefix(o.Role, "chamfer(")
		}
		if !bevel {
			continue
		}
		seen := map[r3.Vec]struct{}{}
		for _, e := range f.Edges() {
			for _, v := range []*decad.Vertex{e.Start(), e.End()} {
				p := v.Position().Value
				if _, ok := seen[p]; ok || p.Z != 0 {
					continue
				}
				seen[p] = struct{}{}
				out = append(out, p)
			}
		}
	}
	sort.Slice(out, func(i, j int) bool {
		if out[i].X != out[j].X {
			return out[i].X < out[j].X
		}
		return out[i].Y < out[j].Y
	})
	return out
}

func TestAsymmetricChamferFeet(t *testing.T) {
	t.Parallel()
	const h = filletBoxHeight
	t.Run("ReferenceTakesThePositionalDistance", func(t *testing.T) {
		// d = 3 across the x = 100 wall, 5 across the y = 60 wall: the feet
		// sit 3 down the first and 5 back along the second.
		_, box := filletBox(t)
		out := mustChamfer(t, box, cornerEdge(), 3,
			decad.WithAsymmetricChamfer(decad.Faces(decad.Facing(r3.NewVec(1, 0, 0))), units.Millimeters(5)))
		requireManifold(t, out)
		require.Equal(t, []r3.Vec{r3.NewVec(95, 60, 0), r3.NewVec(100, 57, 0)}, chamferFeet(t, out))
		decadtest.MeasuresVolume(t, out, units.CubicMillimeters(100*60*h-3*5/2.0*h), decadtest.Exactly())
	})
	t.Run("ReferenceSwapsTheDistances", func(t *testing.T) {
		_, box := filletBox(t)
		out := mustChamfer(t, box, cornerEdge(), 3,
			decad.WithAsymmetricChamfer(decad.Faces(decad.Facing(r3.NewVec(0, 1, 0))), units.Millimeters(5)))
		require.Equal(t, []r3.Vec{r3.NewVec(97, 60, 0), r3.NewVec(100, 55, 0)}, chamferFeet(t, out))
		decadtest.MeasuresVolume(t, out, units.CubicMillimeters(100*60*h-3*5/2.0*h), decadtest.Exactly())
	})
	t.Run("OneReferenceFacePerEdge", func(t *testing.T) {
		// Four lateral edges, two reference walls: x = 0 and x = 100 each
		// border two corners, and each corner takes 3 along its x wall and 5
		// along its y wall.
		_, box := filletBox(t)
		out := mustChamfer(t, box, verticalEdges(), 3,
			decad.WithAsymmetricChamfer(decad.Faces(decad.NormalTo(r3.NewVec(1, 0, 0))), units.Millimeters(5)))
		requireManifold(t, out)
		require.Equal(t, []r3.Vec{
			r3.NewVec(0, 3, 0), r3.NewVec(0, 57, 0), r3.NewVec(5, 0, 0), r3.NewVec(5, 60, 0),
			r3.NewVec(95, 0, 0), r3.NewVec(95, 60, 0), r3.NewVec(100, 3, 0), r3.NewVec(100, 57, 0),
		}, chamferFeet(t, out))
		decadtest.MeasuresVolume(t, out, units.CubicMillimeters(100*60*h-4*7.5*h), decadtest.Exactly())
	})
	t.Run("OtherDistanceKeptExactly", func(t *testing.T) {
		// A quarter inch is 6.35 mm; the y = 60 foot lands at 100 − 6.35.
		_, box := filletBox(t)
		out := mustChamfer(t, box, cornerEdge(), 3,
			decad.WithAsymmetricChamfer(decad.Faces(decad.Facing(r3.NewVec(1, 0, 0))), units.Inches(0.25)))
		feet := chamferFeet(t, out)
		require.Len(t, feet, 2)
		require.InDelta(t, 100-6.35, feet[0].X, 1e-12)
		require.Equal(t, 60.0, feet[0].Y)
		require.Equal(t, r3.NewVec(100, 57, 0), feet[1])
	})
	t.Run("UnionSelectorDeepCopy", func(t *testing.T) {
		// Four lateral edges and a two-branch union naming the x = 0 and
		// x = 100 walls. After the option copied it, the caller adds a branch
		// naming the y = 60 wall (which would give two corners two reference
		// faces) and asserts a count the union never meets; the chamfer
		// resolves the copy, both branches and its own Exactly(2).
		_, box := filletBox(t)
		ref := decad.Faces(decad.Facing(r3.NewVec(-1, 0, 0))).Or(decad.Facing(r3.NewVec(1, 0, 0))).Exactly(2)
		opt := decad.WithAsymmetricChamfer(ref, units.Millimeters(5))
		ref.Or(decad.Facing(r3.NewVec(0, 1, 0))).Exactly(7)
		out := mustChamfer(t, box, verticalEdges(), 3, opt)
		require.Equal(t, []r3.Vec{
			r3.NewVec(0, 3, 0), r3.NewVec(0, 57, 0), r3.NewVec(5, 0, 0), r3.NewVec(5, 60, 0),
			r3.NewVec(95, 0, 0), r3.NewVec(95, 60, 0), r3.NewVec(100, 3, 0), r3.NewVec(100, 57, 0),
		}, chamferFeet(t, out))
		decadtest.MeasuresVolume(t, out, units.CubicMillimeters(100*60*h-4*7.5*h), decadtest.Exactly())
	})
	t.Run("SelectorDeepCopy", func(t *testing.T) {
		// The caller's query changes after the option copied it; the chamfer
		// resolves the copy.
		_, box := filletBox(t)
		ref := decad.Faces(decad.Facing(r3.NewVec(1, 0, 0)))
		opt := decad.WithAsymmetricChamfer(ref, units.Millimeters(5))
		ref.Exactly(7)
		out := mustChamfer(t, box, cornerEdge(), 3, opt)
		require.Equal(t, []r3.Vec{r3.NewVec(95, 60, 0), r3.NewVec(100, 57, 0)}, chamferFeet(t, out))
	})
}

func TestAsymmetricChamferReferenceRefused(t *testing.T) {
	t.Parallel()
	tests := []struct {
		name string
		ref  decad.FaceSelector
		want error
	}{
		{name: "Missing", ref: decad.Faces(decad.Facing(r3.NewVec(-1, 0, 0))), want: decad.ErrCardinality},
		{name: "Dual", ref: decad.Faces(decad.Planar()), want: decad.ErrCardinality},
		{name: "Extra", ref: decad.Faces(decad.NormalTo(r3.NewVec(1, 0, 0))), want: decad.ErrCardinality},
		{name: "Nil", ref: nil, want: decad.ErrDegenerate},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			doc, box := filletBox(t)
			_, err := box.Chamfer(t.Context(), cornerEdge(), units.Millimeters(3), decad.WithAsymmetricChamfer(tc.ref, units.Millimeters(5)))
			require.ErrorIs(t, err, tc.want)
			require.Equal(t, []*decad.Body{box}, doc.Bodies(), `a refused chamfer leaves the receiver live`)
		})
	}
}

func TestAsymmetricChamferOverrunEachSide(t *testing.T) {
	t.Parallel()
	ref := decad.Faces(decad.Facing(r3.NewVec(1, 0, 0)))
	// The x = 100 wall is 60 long and the y = 60 wall 100 long; each setback
	// faces S6 on its own wall.
	for _, c := range []struct{ d, other float64 }{{61, 5}, {3, 101}} {
		doc, box := filletBox(t)
		_, err := box.Chamfer(t.Context(), cornerEdge(), units.Millimeters(c.d), decad.WithAsymmetricChamfer(ref, units.Millimeters(c.other)))
		require.ErrorIs(t, err, decad.ErrUnsupported, `d=%v other=%v`, c.d, c.other)
		require.Equal(t, []*decad.Body{box}, doc.Bodies())
	}
	// Both setbacks fit: 59 and 99 build.
	_, box := filletBox(t)
	_, err := box.Chamfer(t.Context(), cornerEdge(), units.Millimeters(59), decad.WithAsymmetricChamfer(ref, units.Millimeters(99)))
	require.NoError(t, err)
}

func TestAsymmetricChamferRepeatable(t *testing.T) {
	t.Parallel()
	run := func() (float64, []r3.Vec) {
		_, box := filletBox(t)
		out := mustChamfer(t, box, cornerEdge(), 3,
			decad.WithAsymmetricChamfer(decad.Faces(decad.Facing(r3.NewVec(1, 0, 0))), units.Millimeters(5)), decad.WithTangentChain())
		return volumeMM3(t, out), chamferFeet(t, out)
	}
	v1, f1 := run()
	v2, f2 := run()
	require.Equal(t, v1, v2)
	require.Equal(t, f1, f2)
}

func TestAsymmetricChamferRevolveJunction(t *testing.T) {
	t.Parallel()
	// The shaft's concave shoulder corner at (z, ρ) = (10, 5). The reference
	// face's own side role picks either the shoulder plane or the journal
	// cylinder; d = 1 runs along the referenced wall and 3 along the other.
	for _, refPlane := range []bool{true, false} {
		doc := decad.New()
		shaft := revolveMeridian(t, doc, shaftMeridian, decad.FullRevolution{})
		edges, err := concaveJunctions().SelectEdges(shaft)
		require.NoError(t, err)
		require.Len(t, edges, 1)
		var ref decad.FeatureRef
		for _, f := range edges[0].Faces() {
			_, plane := f.Surface().(decad.Plane)
			if plane != refPlane {
				continue
			}
			for _, o := range f.Origins() {
				if strings.HasPrefix(o.Role, "side(") {
					ref = o
				}
			}
		}
		require.NotEmpty(t, ref.Role)
		out := mustChamfer(t, shaft, concaveJunctions(), 1,
			decad.WithAsymmetricChamfer(decad.Faces(decad.FaceCreatedBy(ref)), units.Millimeters(3)))
		requireManifold(t, out)
		// The added triangle has legs 1 and 3 and area 3/2. Along the shoulder
		// plane the leg runs in ρ, along the journal in z, so the centroid's
		// ρ is 5 + 1/3 with the plane referenced and 5 + 3/3 otherwise.
		rho := 5 + 3.0/3
		if refPlane {
			rho = 5 + 1.0/3
		}
		decadtest.MeasuresVolume(t, out, units.CubicMillimeters(2*math.Pi*(shaftQ+1.5*rho)))
	}
}

func TestAsymmetricChamferOnBrepStraightEdge(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name      string
		normal    r3.Vec
		frontFoot r3.Vec
		sideFoot  r3.Vec
	}{
		{name: "front reference", normal: r3.NewVec(0, -1, 0),
			frontFoot: r3.NewVec(38, 0, 0), sideFoot: r3.NewVec(40, 3, 0)},
		{name: "side reference", normal: r3.NewVec(1, 0, 0),
			frontFoot: r3.NewVec(37, 0, 0), sideFoot: r3.NewVec(40, 2, 0)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			// The plate and boss form one brep. Swapping the reference face
			// swaps the feet of a 2 mm by 3 mm bevel at its far vertical edge.
			w := sketch.NewWorld()
			doc := decad.New()
			block := func(x1, z, height float64) *decad.Body {
				plane, err := w.CreateOffsetPlane(w.XY(), z)
				require.NoError(t, err)
				s, err := w.CreateSketch(plane)
				require.NoError(t, err)
				rect := s.CreateRectangle(0, 0, x1, 20)
				s.Fix(rect.A)
				_, err = s.Solve(t.Context())
				require.NoError(t, err)
				b, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
				require.NoError(t, err)
				return b
			}
			part, err := decad.Union(t.Context(), block(40, 0, 10), block(10, 10, 10))
			require.NoError(t, err)
			edge := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.EndpointAt(r3.NewVec(40, 0, 0))).Exactly(1)
			refFace := planeFacing(t, part, tc.normal, r3.NewVec(40, 0, 0))
			reference := decad.Faces(decad.FaceCreatedBy(refFace.Origins()[0]))
			beveled, err := part.Chamfer(t.Context(), edge, units.Millimeters(2),
				decad.WithAsymmetricChamfer(reference, units.Millimeters(3)))
			require.NoError(t, err)
			require.Equal(t, []*decad.Body{beveled}, doc.Bodies())
			volume, err := beveled.Volume()
			require.NoError(t, err)
			require.InDelta(t, 9970, volume.Value.Base(), 1e-8)
			require.Equal(t, decad.Exact, volume.Exactness)
			for _, want := range []r3.Vec{tc.frontFoot, tc.sideFoot} {
				found := false
				for _, vertex := range beveled.Vertices() {
					if vertex.Position().Value.Sub(want).Len() < 1e-9 {
						found = true
						break
					}
				}
				require.True(t, found, "bevel foot %v is a result vertex", want)
			}
			mesh, err := beveled.Tessellate(t.Context(), units.Millimeters(0.1))
			require.NoError(t, err)
			require.True(t, mesh.VolumeVerified())
		})
	}
}

func TestAsymmetricChamferOnBrepRestatedEndFaces(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBodyAtZ(t, doc, 0, 0, 40, 20, 0, 10)
	boss := boxBodyAtZ(t, doc, 0, 0, 10, 20, 10, 10)
	part, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)
	// Both x-end faces are swept walls in the boolean record. Route E
	// restates them as planes before assigning the bottom face's setback.
	edge := decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0)),
		decad.EndpointAt(r3.NewVec(0, 0, 0))).Exactly(1)
	bottom := planeFacing(t, part, r3.NewVec(0, 0, -1), r3.NewVec(0, 0, 0))
	result, err := part.Chamfer(t.Context(), edge, units.Millimeters(2),
		decad.WithAsymmetricChamfer(decad.Faces(decad.FaceCreatedBy(bottom.Origins()[0])), units.Millimeters(3)))
	require.NoError(t, err)
	volume, err := result.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, volume.Exactness)
	require.InDelta(t, 9880, volume.Value.Base(), 1e-8)
	mesh, err := result.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.True(t, mesh.VolumeVerified())
}

func TestAsymmetricChamferOnBrepLoop(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name          string
		sideRef       bool
		piCoefficient float64
	}{
		{name: "cap reference", piCoefficient: 7.5},
		{name: "side references", sideRef: true, piCoefficient: 7},
	} {
		t.Run(tc.name, func(t *testing.T) {
			part := roundedDrilledPlate(t)
			doc := part.Document()
			top := planeFacing(t, part, r3.NewVec(0, 0, 1), r3.NewVec(0, 0, 20))
			loop := top.Loops()[0]
			reference := decad.Faces(decad.FaceCreatedBy(top.Origins()[0]))
			if tc.sideRef {
				reference = nil
				for _, edge := range loop.Edges() {
					for _, face := range edge.Faces() {
						if face == top {
							continue
						}
						if reference == nil {
							reference = decad.Faces(decad.FaceCreatedBy(face.Origins()[0]))
						} else {
							reference.Or(decad.FaceCreatedBy(face.Origins()[0]))
						}
					}
				}
			}
			result, err := part.Chamfer(t.Context(), loopEdgeQuery(loop), units.Millimeters(1.5),
				decad.WithAsymmetricChamfer(reference, units.Millimeters(2)))
			require.NoError(t, err)
			require.Equal(t, []*decad.Body{result}, doc.Bodies())
			volume, err := result.Volume()
			require.NoError(t, err)
			// The 40 by 20 mm top loop has 3 mm corner arcs. Its chamfer
			// removes ds * (perimeter * dc/2 - pi * dc^2/3) from 15280 mm^3.
			require.InDelta(t, 15136-tc.piCoefficient*math.Pi,
				volume.Value.Base(), volume.Bound.Base()+1e-8)
			mesh, err := result.Tessellate(t.Context(), units.Millimeters(0.1))
			require.NoError(t, err)
			require.True(t, mesh.VolumeVerified())
		})
	}
}

func TestAsymmetricChamferOnBrepLoopSideOverrun(t *testing.T) {
	t.Parallel()
	part := roundedDrilledPlate(t)
	doc := part.Document()
	top := planeFacing(t, part, r3.NewVec(0, 0, 1), r3.NewVec(0, 0, 20))
	_, err := part.Chamfer(t.Context(), loopEdgeQuery(top.Loops()[0]), units.Millimeters(1.5),
		decad.WithAsymmetricChamfer(decad.Faces(decad.FaceCreatedBy(top.Origins()[0])), units.Millimeters(20)))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "SX7")
	require.Equal(t, []*decad.Body{part}, doc.Bodies())
}

func TestAsymmetricChamferOnStackedLoop(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name    string
		sideRef bool
		volume  float64
	}{
		{name: "cap reference", volume: 14766},
		{name: "side references", sideRef: true, volume: 14768},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc := decad.New()
			plate := boxBody(t, doc, 0, 0, 40, 40, 10)
			pocket, err := decad.Cut(t.Context(), plate, boxBodyAtZ(t, doc, 10, 15, 30, 25, 5, 5))
			require.NoError(t, err)
			top := planeFacing(t, pocket, r3.NewVec(0, 0, 1), r3.NewVec(0, 0, 10))
			loop := top.Loops()[0]
			reference := decad.Faces(decad.FaceCreatedBy(top.Origins()[0]))
			if tc.sideRef {
				reference = nil
				for _, edge := range loop.Edges() {
					for _, face := range edge.Faces() {
						if face == top {
							continue
						}
						if reference == nil {
							reference = decad.Faces(decad.FaceCreatedBy(face.Origins()[0]))
						} else {
							reference.Or(decad.FaceCreatedBy(face.Origins()[0]))
						}
					}
				}
			}
			result, err := pocket.Chamfer(t.Context(), loopEdgeQuery(loop), units.Millimeters(1.5),
				decad.WithAsymmetricChamfer(reference, units.Millimeters(2)))
			require.NoError(t, err)
			require.Equal(t, []*decad.Body{result}, doc.Bodies())
			volume, err := result.Volume()
			require.NoError(t, err)
			// The 40 mm square's top rim removes a triangular corner band.
			require.InDelta(t, tc.volume, volume.Value.Base(), volume.Bound.Base()+1e-8)
			mesh, err := result.Tessellate(t.Context(), units.Millimeters(0.1))
			require.NoError(t, err)
			require.True(t, mesh.VolumeVerified())
		})
	}
}

func TestAsymmetricChamferOnStackedEdge(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 40, 40, 10)
	pocket, err := decad.Cut(t.Context(), plate, boxBodyAtZ(t, doc, 10, 15, 30, 25, 5, 5))
	require.NoError(t, err)
	edge := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)),
		decad.EndpointAt(r3.NewVec(0, 0, 0))).Exactly(1)
	front := planeFacing(t, pocket, r3.NewVec(0, -1, 0), r3.NewVec(0, 0, 0))
	result, err := pocket.Chamfer(t.Context(), edge, units.Millimeters(1),
		decad.WithAsymmetricChamfer(decad.Faces(decad.FaceCreatedBy(front.Origins()[0])), units.Millimeters(2)))
	require.NoError(t, err)
	volume, err := result.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, volume.Exactness)
	require.InDelta(t, 14990, volume.Value.Base(), 1e-8)
	mesh, err := result.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.True(t, mesh.VolumeVerified())
}
