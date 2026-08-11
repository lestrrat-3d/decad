package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is the PLACEMENT half of a band patch's surface-departure bound
// (docs/modify-reach-design.md §8.3). The other half — a non-tangential miter
// corner, which leaves the two directrices sweeping different windows — is
// covered in capblend_survey_test.go. This one covers the half that has nothing
// to do with the windows at all: every world coordinate the build emits is a
// ROUNDED image of the plane-local number it denotes, and the roundings are
// independent, so a band whose two windows genuinely coincide still leaves its
// own `Cone` tag once it is placed. A bound read off the plane-local windows
// alone is identical placed or not and publishes a zero there, which is an
// assertion rather than a measurement.
//
// Every reading below is taken from HELD numbers only: a ruling's own two
// endpoint vertices, a directrix's own centre, and the tag's own origin. Each
// difference is between two coordinates within a factor of two of one another,
// so the subtraction is exact and the reconstruction carries no cancellation of
// its own to be confused with the departure it measures. Nothing is sampled by
// finite differences and nothing is evaluated at a point the test computed.

// bandRulingEnd is one corner of a chamfer band's built ruled surface: the
// vertex itself, the directrix passing through it, and the straight ruling
// leaving it. The two together span the built surface's own tangent plane
// there, which is what the patch's published `Cone` is judged against.
type bandRulingEnd struct {
	at      r3.Vec
	center  r3.Vec
	axis    r3.Vec
	rulingA r3.Vec
	rulingB r3.Vec
}

// tangentPlaneNormal is the built surface's own unit normal at this corner: the
// directrix tangent (axis crossed into the radial arm) crossed into the ruling.
// The sign is taken from the published normal, which may decide it — the two
// are far closer than a right angle everywhere this file reads them, so the
// sign is never the thing under test.
func (e bandRulingEnd) tangentPlaneNormal(t *testing.T, published r3.Vec) r3.Vec {
	t.Helper()
	tangent := e.axis.Cross(e.at.Sub(e.center))
	n, ok := tangent.Cross(e.rulingB.Sub(e.rulingA)).Normalize()
	require.True(t, ok, "the built ruled patch has a tangent plane at its own corner")
	if n.Dot(published) < 0 {
		n = n.Scale(-1)
	}
	return n
}

// generatorSine is how far the published straight ruling leaves the published
// cone's own generator through this corner — zero exactly when the ruling IS a
// generator, which is what an unrounded build gives and what the `Cone` tag
// claims.
func (e bandRulingEnd) generatorSine(origin r3.Vec) float64 {
	ruling := e.rulingB.Sub(e.rulingA)
	generator := e.at.Sub(origin)
	return ruling.Cross(generator).Len() / (ruling.Len() * generator.Len())
}

// bandRulingEnds reads every corner of one circular band patch off its OWN
// public boundary: two `Arc3` directrices and two `Line3` rulings, paired
// through the boundary vertices they share.
func bandRulingEnds(t *testing.T, f *decad.Face) []bandRulingEnd {
	t.Helper()
	type arcEdge struct {
		arc        decad.Arc3
		start, end *decad.Vertex
	}
	var arcs []arcEdge
	var lines []*decad.Edge
	for _, ce := range f.Loops()[0].CoEdges() {
		e := ce.Edge()
		switch c := e.Curve().(type) {
		case decad.Arc3:
			arcs = append(arcs, arcEdge{arc: c, start: e.Start(), end: e.End()})
		case decad.Line3:
			lines = append(lines, e)
		}
	}
	require.Len(t, arcs, 2, "a circular band patch runs between two circular directrices")
	require.Len(t, lines, 2, "and is bounded by two straight rulings")

	holder := func(v *decad.Vertex) (arcEdge, bool) {
		for _, a := range arcs {
			if a.start == v || a.end == v {
				return a, true
			}
		}
		return arcEdge{}, false
	}
	var out []bandRulingEnd
	for _, ln := range lines {
		a, okA := holder(ln.Start())
		b, okB := holder(ln.End())
		require.True(t, okA && okB, "each ruling joins one directrix to the other")
		for _, side := range []struct {
			v *decad.Vertex
			a arcEdge
		}{{ln.Start(), a}, {ln.End(), b}} {
			out = append(out, bandRulingEnd{
				at:      side.v.Position().Value,
				center:  side.a.arc.Center,
				axis:    side.a.arc.Axis,
				rulingA: ln.Start().Position().Value,
				rulingB: ln.End().Position().Value,
			})
		}
	}
	return out
}

// bandConePatches is every circular chamfer band patch of a chamfered body.
func bandConePatches(t *testing.T, b *decad.Body) []*decad.Face {
	t.Helper()
	var out []*decad.Face
	for _, f := range b.Faces() {
		if f.Surface().Kind() != decad.KindCone {
			continue
		}
		out = append(out, f)
	}
	require.NotEmpty(t, out)
	return out
}

// tangentFilletChamfer is the finding's own fixture: an l x w rectangle swept
// h, all four lateral corners filleted to radius r — so every join between a
// straight wall and a circular one is TANGENT and the two directrices of every
// band patch sweep the same window — then chamfered on its end cap.
//
// The section is drawn at the sketch ORIGIN and carried out by the placement.
// Drawing a small section at large sketch coordinates leaves the arrangement's
// own weld band a handful of ulps of margin, which each platform's arithmetic
// can land either side of; a placement displaces the built coordinates exactly
// as drawing it out there would, and welds nothing.
func tangentFilletChamfer(t *testing.T, motion *r3.Transform) *decad.Body {
	t.Helper()
	const l, w, h, r, d = 8.0, 6.0, 4.0, 1.0, 0.25
	sk := sketch.NewWorld()
	s, err := sk.CreateSketch(sk.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, l, w)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)

	doc := decad.New()
	box, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	rounded, err := box.Fillet(decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Convex()), units.Millimeters(r))
	require.NoError(t, err)
	chamfered, err := rounded.Chamfer(capLoopEdges(rounded), units.Millimeters(d))
	require.NoError(t, err)
	if motion == nil {
		return chamfered
	}
	placed, err := chamfered.Placed(*motion)
	require.NoError(t, err)
	return placed
}

// tangentBandMotions is the three placements this file reads the same band
// under: none, a rotation about the world origin, and that rotation carried far
// out. The rounding of a placed coordinate scales with its own magnitude, so
// the departure the middle case shows is small and the far one's is large,
// which is exactly the difference a plane-local bound cannot see.
func tangentBandMotions(t *testing.T) (r3.Transform, r3.Transform) {
	t.Helper()
	spin, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(1e5, -3e5, 7e4))
	require.NoError(t, err)
	far, err := spin.Then(shift)
	require.NoError(t, err)
	return spin, far
}

// TestCapBlendPlacedTangentBandNormalCarriesItsOwnBound is this file's central
// claim. A tangent-join band's own windows coincide, so its patches carry no
// window skew at all — and its built ruled surface still leaves the `Cone` it
// publishes, because the rulings' endpoints, the directrices' centres and the
// tag's own origin are separately rounded images of the placed frame. The
// published `Face.NormalAt` bound must enclose that departure at every corner
// of every patch, under every placement.
func TestCapBlendPlacedTangentBandNormalCarriesItsOwnBound(t *testing.T) {
	spin, far := tangentBandMotions(t)
	for _, tc := range []struct {
		name   string
		motion *r3.Transform
	}{
		{name: `unplaced`},
		{name: `rotated about the origin`, motion: &spin},
		{name: `rotated and placed far out`, motion: &far},
	} {
		t.Run(tc.name, func(t *testing.T) {
			body := tangentFilletChamfer(t, tc.motion)
			checked := 0
			for _, f := range bandConePatches(t, body) {
				cone, ok := f.Surface().(decad.Cone)
				require.True(t, ok)
				for _, end := range bandRulingEnds(t, f) {
					published, err := f.NormalAt(end.at)
					require.NoError(t, err)
					built := end.tangentPlaneNormal(t, published.Value)
					gap := built.Sub(published.Value).Len()
					require.LessOrEqual(t, gap, published.Bound.Mag(),
						"the built ruled surface's normal is %v from the published %v, past the bound %v it publishes (generator sine %v)",
						built, published.Value, published.Bound.Mag(), end.generatorSine(cone.Origin))
					checked++
				}
			}
			require.Equal(t, 16, checked, "four tangent fillets, two rulings each, two ends per ruling")
		})
	}
}

// TestCapBlendPlacedTangentBandDepartureGrowsWithPlacement pins the causal path
// the bound above has to charge for. The SAME band, whose plane-local windows
// are byte for byte the same under every placement, leaves its `Cone` tag by
// orders more once it is placed — measured two independent ways, on held
// coordinates alone: the built tangent plane's own normal against the published
// one, and the published straight ruling against the published cone's own
// generator through the same corner. A bound derived from the windows would be
// unmoved across these three rows, and zero in all of them.
func TestCapBlendPlacedTangentBandDepartureGrowsWithPlacement(t *testing.T) {
	spin, far := tangentBandMotions(t)
	worst := func(motion *r3.Transform) (float64, float64, float64) {
		body := tangentFilletChamfer(t, motion)
		normalGap, generator, bound := 0.0, 0.0, 0.0
		for _, f := range bandConePatches(t, body) {
			cone, ok := f.Surface().(decad.Cone)
			require.True(t, ok)
			for _, end := range bandRulingEnds(t, f) {
				published, err := f.NormalAt(end.at)
				require.NoError(t, err)
				built := end.tangentPlaneNormal(t, published.Value)
				normalGap = math.Max(normalGap, built.Sub(published.Value).Len())
				generator = math.Max(generator, end.generatorSine(cone.Origin))
				bound = math.Max(bound, published.Bound.Mag())
			}
		}
		return normalGap, generator, bound
	}

	_, flatGen, flatBound := worst(nil)
	spunGap, spunGen, spunBound := worst(&spin)
	farGap, farGen, farBound := worst(&far)

	// Audited: unplaced generator sine 6.3e-16 and bound 1.5e-15; rotated at the
	// origin 3.2e-15 and 1.2e-14; rotated and far out 1.6e-10 and 2.7e-10, with
	// the built surface's own normal 7.4e-11 from the published one there.
	require.Less(t, flatGen, 1e-14,
		"an unplaced build's rulings are the cone's own generators to within its own arithmetic")
	require.Greater(t, farGen, 1e3*spunGen,
		"a placed build's leave the generator by orders more once the placement carries the band far out")
	require.Greater(t, farGap, 1e-11, "and the built surface's own normal leaves the published one with it")
	require.Greater(t, farGap, 1e3*spunGap, "at the same scaling")
	// The published bound has to follow, or a decision taken against the
	// published direction is decided against a direction the surface has not
	// got. Bounds that did not move across these rows would be the defect this
	// file exists for.
	require.Greater(t, spunBound, flatBound)
	require.Greater(t, farBound, 1e3*spunBound)
	require.Greater(t, farBound, farGap)
}
