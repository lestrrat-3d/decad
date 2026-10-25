package revolveshell_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/coneaxis"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/lestrrat-3d/decad/internal/sectioncert"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// revolveFixtureAbout is revolveFixture about the sketch line from (0, 0) to
// end, recording the axis vector a public Revolve records
// (coneaxis.RecordedVec).
func revolveFixtureAbout(t *testing.T, outline []sectionrecord.Point2, end sectionrecord.Point2) *topo.Body {
	t.Helper()
	s, p := sectionFixture(t, outline)
	profile, plane, _, err := sectioncert.RecordProfile(s, p)
	require.NoError(t, err)
	frame, err := r3.NewFrame(plane.Origin, plane.U, plane.V)
	require.NoError(t, err)
	input := revolveaxis.SketchLine{EndU: end.U, EndV: end.V}
	line, err := revolveaxis.AxisInPlane(input, frame)
	require.NoError(t, err)
	work := freeform.NewFreeformWork()
	ax, side, err := revolveplan.ResolveAxisSide(t.Context(), profile, line, work)
	require.NoError(t, err)
	full := revolveangle.Sweep{Phi0: revolveangle.Zero(),
		Phi1: revolveangle.Angle{Rad: new(big.Rat), Turn: big.NewRat(1, 1)}}
	body, err := revolveplan.Build(t.Context(), topo.Env{Doc: &topo.Document{}, Ref: 1, Alloc: topotest.Alloc{}},
		revolveplan.Payload{Profile: profile, Frame: frame, Ax: ax, Phi1: 2 * math.Pi, Full: true,
			Den: full, Xform: r3.Identity(), RadialProof: ax.RadialProof,
			AxisVec: coneaxis.RecordedVec(frame, input, ax, side)}, work)
	require.NoError(t, err)
	return body
}

const ringPrec = 512

func ringBig(v float64) *big.Float { return new(big.Float).SetPrec(ringPrec).SetFloat64(v) }

// circleReading returns the axial h = p·Â and radial ρ = |p − hÂ| of p about
// the exact unit axis Â along a, in 512-bit arithmetic.
func circleReading(p, a r3.Vec) (*big.Float, *big.Float) {
	pv := [3]*big.Float{ringBig(p.X), ringBig(p.Y), ringBig(p.Z)}
	av := [3]*big.Float{ringBig(a.X), ringBig(a.Y), ringBig(a.Z)}
	dot := func(x, y [3]*big.Float) *big.Float {
		out := new(big.Float).SetPrec(ringPrec)
		for i := range x {
			out.Add(out, new(big.Float).SetPrec(ringPrec).Mul(x[i], y[i]))
		}
		return out
	}
	norm := new(big.Float).SetPrec(ringPrec).Sqrt(dot(av, av))
	h := new(big.Float).SetPrec(ringPrec).Quo(dot(pv, av), norm)
	rhoSq := new(big.Float).SetPrec(ringPrec).Sub(dot(pv, pv), new(big.Float).SetPrec(ringPrec).Mul(h, h))
	if rhoSq.Sign() < 0 {
		rhoSq.SetInt64(0)
	}
	return h, new(big.Float).SetPrec(ringPrec).Sqrt(rhoSq)
}

// circleDistance is the exact distance from v to the circle that plane point
// p sweeps about a: √((h_v − h_p)² + (ρ_v − ρ_p)²).
func circleDistance(v, p, a r3.Vec) *big.Float {
	hv, rv := circleReading(v, a)
	hp, rp := circleReading(p, a)
	dh := new(big.Float).SetPrec(ringPrec).Sub(hv, hp)
	dr := new(big.Float).SetPrec(ringPrec).Sub(rv, rp)
	sum := new(big.Float).SetPrec(ringPrec).Add(new(big.Float).SetPrec(ringPrec).Mul(dh, dh),
		new(big.Float).SetPrec(ringPrec).Mul(dr, dr))
	return sum.Sqrt(sum)
}

// A general section ring places every vertex on the circle its recorded
// plane point sweeps about the exact axis, within the SweptPointGap it
// charges, and the ring's returned bound covers every vertex and the chord sag
// of the largest true radius (agent-docs/revolve-axis-design.md §5.7). The hub
// is the six-span hub of the sector tests on the XY plane about the line to
// (1, 2) or to (1, −2): each design point (z, r) is the plane point
// z/2·(1, ±2) + r/2·(∓2, 1). About (1, −2) every plane-local V is negative,
// so no reading of V as the radius passes.
func TestBuildRevolvedSectionRingAboutAxis(t *testing.T) {
	t.Parallel()
	for _, sign := range []float64{1, -1} {
		ringAboutAxis(t, sign)
	}
}

func ringAboutAxis(t *testing.T, sign float64) {
	t.Helper()
	design := [][2]float64{{6, 0}, {9, 0}, {9, 3}, {8, 4}, {5, 2.5}, {6, 1.5}}
	outline := make([]sectionrecord.Point2, len(design))
	for i, p := range design {
		outline[i] = sectionrecord.Point2{U: p[0]/2 - sign*p[1], V: sign*p[0] + p[1]/2}
	}
	hub := revolveFixtureAbout(t, outline, sectionrecord.Point2{U: 1, V: 2 * sign})
	candidates := revolveshell.RootCandidates(hub)
	require.NotEmpty(t, candidates)
	section := candidates[0]
	require.NotNil(t, section.General)
	axis := section.Axis.Vec()
	require.Equal(t, r3.Vec{X: 1, Y: 2 * sign}, axis)
	const stations = 64
	angles := make([]float64, stations)
	for j := range angles {
		angles[j] = 2 * math.Pi * float64(j) / stations
	}
	mesh := &facetedrecord.JoinedMesh{}
	ring, sag, err := revolveshell.BuildRevolvedSectionRing(mesh, section, angles, nil, 0.1)
	require.NoError(t, err)
	sagBound := ringBig(sag)
	moved := 0
	maxRadius := new(big.Float).SetPrec(ringPrec)
	for i, span := range section.Spans {
		p := span.Start
		_, r := section.Axis.PlaneScaled(section.Frame, p.U, p.V)
		onAxis := r.Sign() == 0
		world := section.Frame.ToWorldUV(p.U, p.V)
		if _, radius := circleReading(world, axis); radius.Cmp(maxRadius) > 0 {
			maxRadius = radius
		}
		require.Len(t, ring.Rings[i], map[bool]int{false: stations, true: 1}[onAxis])
		for j, index := range ring.Rings[i] {
			v := mesh.Verts[index]
			held, gap, ok := revolveshell.RingVertexAbout(section, p, onAxis, angles[j])
			require.True(t, ok)
			require.Equal(t, held, v, "span %d station %d", i, j)
			distance := circleDistance(v, world, axis)
			if distance.Sign() > 0 {
				moved++
			}
			require.LessOrEqual(t, distance.Cmp(ringBig(gap)), 0,
				"span %d station %d lies %s off its circle, gap %g", i, j, distance.Text('g', 6), gap)
			require.LessOrEqual(t, distance.Cmp(sagBound), 0)
		}
	}
	require.Positive(t, moved, "some vertex must round off its circle")
	// A chord of angular step 2π/64 at radius ρ leaves its arc by up to
	// ρ·step²/8.
	step := ringBig(2 * math.Pi / stations)
	chord := new(big.Float).SetPrec(ringPrec).Mul(maxRadius, new(big.Float).SetPrec(ringPrec).Mul(step, step))
	chord.Quo(chord, ringBig(8))
	require.LessOrEqual(t, chord.Cmp(sagBound), 0, "the ring bound %g must cover the chord sag %s", sag,
		chord.Text('g', 6))
}
