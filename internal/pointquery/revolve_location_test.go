package pointquery

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// loopOf joins the corners into a loop of whole lines.
func loopOf(corners ...pt) sectionrecord.LoopRecord {
	var segments []sectionrecord.CurveSegment
	for i, c := range corners {
		segments = append(segments, line(c, corners[(i+1)%len(corners)]))
	}
	return sectionrecord.LoopRecord{Segments: segments}
}

// revolveOf revolves profile a full turn about the plane's v axis, oriented
// so u ≥ 0 is the material side (ρ = u). The XY plane's map takes the
// meridian point (u, v) at angle φ to (u·cos φ, v, u·sin φ): a solid about
// the world y axis.
func revolveOf(t *testing.T, profile momentinput.Profile) revolveplan.Payload {
	t.Helper()
	pp := payloadOf(t, momentinput.Profile{})
	return revolveplan.Payload{
		Profile: profile, Frame: pp.Frame, Xform: pp.Xform, Full: true, Den: revolveangle.FullTurn(),
		Ax: revolveaxis.Frame{DV: -1, SnapTol: 1e-9},
	}
}

// cylinderMeridian is the rectangle u ∈ [0, 10], v ∈ [0, 20]: a cylinder of
// radius 10 about the y axis over y ∈ [0, 20].
func cylinderMeridian() momentinput.Profile {
	return momentinput.Profile{Outer: loopOf(pt{U: 0, V: 0}, pt{U: 10, V: 0}, pt{U: 10, V: 20}, pt{U: 0, V: 20})}
}

func requireRevolveLocations(t *testing.T, rp revolveplan.Payload, cases []prismCase) {
	t.Helper()
	requireLocationCases(t, cases, func(at r3.Vec) (Location, bool, error) {
		return LocateRevolve(t.Context(), at, rp)
	})
}

func TestLocateRevolveCylinder(t *testing.T) {
	t.Parallel()
	requireRevolveLocations(t, revolveOf(t, cylinderMeridian()), []prismCase{
		{name: "wall", at: r3.NewVec(10, 5, 0), want: OnBoundary},
		{name: "wall across", at: r3.NewVec(0, 5, -10), want: OnBoundary},
		{name: "wall at (6, 8)", at: r3.NewVec(6, 5, 8), want: OnBoundary},
		{name: "material", at: r3.NewVec(3, 5, 4), want: Inside},
		{name: "beside", at: r3.NewVec(8, 5, 8), want: Outside},
		{name: "irrational radius", at: r3.NewVec(7, 5, 7), want: Inside},
		{name: "axis", at: r3.NewVec(0, 5, 0), want: Inside},
		{name: "cap centre", at: r3.NewVec(0, 0, 0), want: OnBoundary},
		{name: "far cap centre", at: r3.NewVec(0, 20, 0), want: OnBoundary},
		{name: "axis past the cap", at: r3.NewVec(0, -1e-9, 0), want: Outside},
		{name: "rim", at: r3.NewVec(0, 20, 10), want: OnBoundary},
	})
}

// TestLocateRevolvePie revolves the quarter pie of radius 20 into a
// hemisphere over y ∈ [0, 20]. The pole is an axis point on the arc, and a
// point on the arc's chord is left to the mesh.
func TestLocateRevolvePie(t *testing.T) {
	t.Parallel()
	requireRevolveLocations(t, revolveOf(t, momentinput.Profile{Outer: pieLoop(quarterArc())}), []prismCase{
		{name: "pole", at: r3.NewVec(0, 20, 0), want: OnBoundary},
		{name: "under the pole", at: r3.NewVec(0, 20-1e-9, 0), want: Inside},
		{name: "axis", at: r3.NewVec(0, 10, 0), want: Inside},
		{name: "flat face centre", at: r3.NewVec(0, 0, 0), want: OnBoundary},
		{name: "sphere", at: r3.NewVec(0, 16, 12), want: OnBoundary},
		{name: "1e-9 outside the sphere", at: r3.NewVec(0, 16, 12+1e-9), want: Outside},
		{name: "chord", at: r3.NewVec(10, 10, 0), want: Undecided},
	})
}

// TestLocateRevolveTilted revolves the cylinder meridian about the same axis
// on a plane turned about the world z axis: the frame is orthonormal only to
// rounding, and the exact solve reads its held axes. An axis point stays on
// the axis, and a wall point is decided against the denoted wall.
func TestLocateRevolveTilted(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(3, 4, 0), r3.NewVec(-4, 3, 0))
	require.NoError(t, err)
	rp := revolveOf(t, cylinderMeridian())
	rp.Frame = frame
	v := frame.V()
	requireRevolveLocations(t, rp, []prismCase{
		{name: "axis", at: r3.NewVec(5*v.X, 5*v.Y, 0), want: Inside},
		{name: "beside the axis", at: r3.NewVec(5*v.X, 5*v.Y, 9), want: Inside},
		{name: "off the wall", at: r3.NewVec(5*v.X, 5*v.Y, 10.001), want: Outside},
	})
}

// TestLocateRevolveLeavesOpenRecords covers every gate that hands a revolve
// back to the mesh.
func TestLocateRevolveLeavesOpenRecords(t *testing.T) {
	t.Parallel()
	at := r3.NewVec(3, 5, 4)
	radianSweep := revolveangle.Sweep{
		Phi0: revolveangle.Zero(),
		Phi1: revolveangle.Angle{Rad: new(big.Rat).SetFloat64(2 * math.Pi), Turn: new(big.Rat)},
	}
	span := proofbound.PointInterval(big.NewRat(1, 1))
	gates := map[string]func(*revolveplan.Payload){
		"partial sweep":        func(rp *revolveplan.Payload) { rp.Full = false },
		"surface result":       func(rp *revolveplan.Payload) { rp.SurfaceResult = true },
		"section displacement": func(rp *revolveplan.Payload) { rp.SectionDelta = 1e-12 },
		"radian sweep":         func(rp *revolveplan.Payload) { rp.Den = radianSweep },
		"no denotation":        func(rp *revolveplan.Payload) { rp.Den = revolveangle.Sweep{} },
		"derived span": func(rp *revolveplan.Payload) {
			rp.Den.Phi1 = revolveangle.Angle{Rad: new(big.Rat), Turn: big.NewRat(1, 1), Span: &span}
		},
		"turn and a radian part": func(rp *revolveplan.Payload) {
			rp.Den.Phi1 = revolveangle.Angle{Rad: big.NewRat(1, 1000), Turn: big.NewRat(1, 1)}
		},
		"anchor u bound":          func(rp *revolveplan.Payload) { rp.Ax.AUBound = 1e-16 },
		"anchor v bound":          func(rp *revolveplan.Payload) { rp.Ax.AVBound = 1e-16 },
		"direction u bound":       func(rp *revolveplan.Payload) { rp.Ax.DUBound = 1e-16 },
		"direction v bound":       func(rp *revolveplan.Payload) { rp.Ax.DVBound = 1e-16 },
		"radial admission":        func(rp *revolveplan.Payload) { rp.Ax.RadialAdmitAllow = 1e-16 },
		"snap allowance":          func(rp *revolveplan.Payload) { rp.Ax.Snap.Area = 1e-16 },
		"zero direction":          func(rp *revolveplan.Payload) { rp.Ax.DV = 0 },
		"singular placement":      func(rp *revolveplan.Payload) { rp.Xform = r3.Transform{} },
		"negative snap tolerance": func(rp *revolveplan.Payload) { rp.Ax.SnapTol = -1 },
	}
	for name, gate := range gates {
		rp := revolveOf(t, cylinderMeridian())
		gate(&rp)
		_, decided, err := LocateRevolve(t.Context(), at, rp)
		require.NoError(t, err, name)
		require.False(t, decided, name)
	}

	o := pt{U: 0, V: 0}
	fragment := sectionrecord.LineSeg{Start: pt{U: -10, V: 0}, End: pt{U: 10, V: 0}, TStart: 0.5, TEnd: 1}
	gap := loopOf(o, pt{U: 10, V: 0}, pt{U: 10, V: 20}, pt{U: 0, V: 20})
	gap.Segments[1] = line(pt{U: 10, V: 0}, pt{U: 10, V: math.Nextafter(20, 21)})
	bulge := sectionrecord.ArcSeg{Center: pt{U: -1, V: 10}, Start: pt{U: 10, V: 0}, End: pt{U: 10, V: 20}, TStart: 0, TEnd: 1}
	meridians := map[string]sectionrecord.LoopRecord{
		"line fragment": {Segments: []sectionrecord.CurveSegment{
			fragment, line(pt{U: 10, V: 0}, pt{U: 10, V: 20}), line(pt{U: 10, V: 20}, pt{U: 0, V: 20}), line(pt{U: 0, V: 20}, o),
		}},
		"open junction":      gap,
		"point across axis":  loopOf(pt{U: -1, V: 0}, pt{U: 10, V: 0}, pt{U: 10, V: 20}, pt{U: -1, V: 20}),
		"point in snap band": loopOf(pt{U: 1e-10, V: 0}, pt{U: 10, V: 0}, pt{U: 10, V: 20}, pt{U: 1e-10, V: 20}),
		"centre across axis": {Segments: []sectionrecord.CurveSegment{
			line(o, pt{U: 10, V: 0}), bulge, line(pt{U: 10, V: 20}, pt{U: 0, V: 20}), line(pt{U: 0, V: 20}, o),
		}},
	}
	for name, loop := range meridians {
		_, decided, err := LocateRevolve(t.Context(), at, revolveOf(t, momentinput.Profile{Outer: loop}))
		require.NoError(t, err, name)
		require.False(t, decided, name)
	}
}

// TestLocateRevolveHornCircle reads a meridian circle of radius 2 about
// (2, 10), touching the axis at (0, 10): a horn torus the build refuses. Its
// axis point lies on the circle, so the axis rule reads it on the boundary,
// as the swept set has it.
func TestLocateRevolveHornCircle(t *testing.T) {
	t.Parallel()
	horn := sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{wholeCircle(pt{U: 2, V: 10}, 2, true)}}
	requireRevolveLocations(t, revolveOf(t, momentinput.Profile{Outer: horn}), []prismCase{
		{name: "axis point on the circle", at: r3.NewVec(0, 10, 0), want: OnBoundary},
		{name: "axis beside the circle", at: r3.NewVec(0, 11, 0), want: Outside},
		{name: "tube", at: r3.NewVec(0, 10, 2), want: Inside},
	})
}
