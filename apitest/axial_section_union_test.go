package apitest_test

import (
	"math"
	"strconv"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

// ribSection draws a closed section bounded by a line, a fit spline, an arc, a fit
// spline, a line and an arc about the origin, turned by rot: a lobe standing on
// the circle of radius r.
func lobeLoft(t testing.TB, doc *decad.Document, r, rot, half float64, height float64) *decad.Body {
	t.Helper()
	return flankLobeLoft(t, doc, r, rot, half, height, func(f float64) (float64, float64) {
		return r + 0.35 + f*0.65, half * (1 - 0.4*f)
	})
}

// flankLobeLoft lofts the lobe section of lobeLoft with its fit flanks drawn by
// flank: fit point f ∈ [0, 1] of the side at angle half sits at radius and angle
// flank(f), and the opposite side mirrors it. The radial lines run from the circle
// of radius r to flank(0).
func flankLobeLoft(t testing.TB, doc *decad.Document, r, rot, half float64, height float64,
	flank func(f float64) (float64, float64)) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	section := func(plane *sketch.Plane) (*sketch.Sketch, *sketch.Profile) {
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		c := s.Origin()
		pt := func(rad, a float64) *sketch.Point {
			p := s.CreatePoint(rad*math.Cos(a+rot), rad*math.Sin(a+rot))
			s.Fix(p)
			return p
		}
		rr, rl := pt(r, -half), pt(r, half)
		const fit = 5
		right, left := make([]*sketch.Point, fit), make([]*sketch.Point, fit)
		for i := range fit {
			rad, a := flank(float64(i) / float64(fit-1))
			right[i], left[i] = pt(rad, -a), pt(rad, a)
		}
		s.CreateLine(rr, right[0])
		_, err = s.CreateFitSpline(right...)
		require.NoError(t, err)
		s.CreateArc(c, right[fit-1], left[fit-1])
		_, err = s.CreateFitSpline(left...)
		require.NoError(t, err)
		s.CreateLine(left[0], rl)
		s.CreateArc(c, rr, rl)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		require.Len(t, s.Profiles(), 1)
		return s, s.Profiles()[0]
	}
	s0, p0 := section(w.XY())
	s1, p1 := section(splitTopPlane(t, w, height))
	b, err := decad.Loft(t.Context(), doc, s0, p0, s1, p1)
	require.NoError(t, err)
	return b
}

// BenchmarkAxialSectionUnion17Lobes measures seventeen free-form lofts and
// their complete axial union, matching the integration case above.
func BenchmarkAxialSectionUnion17Lobes(b *testing.B) {
	const r, height, n = 3.0, 2.0, 17
	for b.Loop() {
		doc := decad.New()
		disc := circleBodyAtZ(b, doc, 0, r, 0, height)
		lobes := make([]*decad.Body, n)
		sum := 0.0
		for k := range lobes {
			lobes[k] = lobeLoft(b, doc, r, 2*math.Pi*float64(k)/n,
				0.4*math.Pi/float64(n), height)
			volume, err := lobes[k].Volume()
			if err != nil {
				b.Fatal(err)
			}
			sum += volume.Value.Base()
		}
		joined, err := decad.Union(b.Context(), disc, lobes...)
		if err != nil {
			b.Fatal(err)
		}
		if !joined.IsSolid() {
			b.Fatal("union of seventeen lobes is not solid")
		}
		volume, err := joined.Volume()
		if err != nil {
			b.Fatal(err)
		}
		want := math.Pi*r*r*height + sum
		require.InDelta(b, want, volume.Value.Base(), volume.Bound.Base()+1e-9)
		mesh, err := joined.Tessellate(b.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
		if err != nil {
			b.Fatal(err)
		}
		require.True(b, mesh.BoundaryVerified())
		require.True(b, mesh.VolumeVerified())
	}
}

func TestAxialSectionUnionLobesOnADisc(t *testing.T) {
	t.Parallel()
	const r, height = 3.0, 2.0
	for _, n := range []int{8, 10, 12, 17} {
		t.Run(strconv.Itoa(n)+" lobes", func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			disc := circleBodyAtZ(t, doc, 0, r, 0, height)
			lobes := make([]*decad.Body, n)
			sum := 0.0
			for k := range lobes {
				lobes[k] = lobeLoft(t, doc, r, 2*math.Pi*float64(k)/float64(n), 0.4*math.Pi/float64(n), height)
				v, err := lobes[k].Volume()
				require.NoError(t, err)
				sum += v.Value.Base()
			}
			joined, err := decad.Union(t.Context(), disc, lobes...)
			require.NoError(t, err)
			require.True(t, joined.IsSolid())
			v, err := joined.Volume()
			require.NoError(t, err)
			want := math.Pi*r*r*height + sum
			require.InDelta(t, want, v.Value.Base(), v.Bound.Base()+1e-9)
			mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
		})
	}
}

// TestAxialSectionUnionFinishes joins ten lobes to a disc, then bores it and chamfers
// both outer caps. A concave root fillet needs a loft fillet at a line and circle
// fragment corner, which the loft fillet does not take, so it is not part of this run.
func TestAxialSectionUnionFinishes(t *testing.T) {
	t.Parallel()
	const r, height, n = 3.0, 2.0, 10
	doc := decad.New()
	disc := circleBodyAtZ(t, doc, 0, r, 0, height)
	lobes := make([]*decad.Body, n)
	for k := range lobes {
		lobes[k] = lobeLoft(t, doc, r, 2*math.Pi*float64(k)/float64(n), 0.4*math.Pi/float64(n), height)
	}
	joined, err := decad.Union(t.Context(), disc, lobes...)
	require.NoError(t, err)
	v0, err := joined.Volume()
	require.NoError(t, err)

	bore := circleBodyAtZ(t, doc, 0, 0.8, -1, height+2)
	bored, err := decad.Cut(t.Context(), joined, bore)
	require.NoError(t, err)
	bv, err := bored.Volume()
	require.NoError(t, err)
	require.InDelta(t, v0.Value.Base()-math.Pi*0.8*0.8*height, bv.Value.Base(), v0.Bound.Base()+bv.Bound.Base())

	caps := decad.Edges(decad.OuterLoopOf(decad.CapStart(bored))).Or(decad.OuterLoopOf(decad.CapEnd(bored)))
	chamfered, err := decad.Chamfer(t.Context(), bored, caps, units.Millimeters(0.03))
	require.NoError(t, err)
	cv, err := chamfered.Volume()
	require.NoError(t, err)
	require.Less(t, cv.Value.Base(), bv.Value.Base(), "a chamfer removes material")
	require.True(t, chamfered.IsSolid())
}

// tongueLoft draws a section of a base line inside a plate, two vertical sides and a fit
// spline arch over the plate's top edge, and lofts it between two identical planes.
func tongueLoft(t *testing.T, doc *decad.Document, height float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	section := func(plane *sketch.Plane) (*sketch.Sketch, *sketch.Profile) {
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		pt := func(x, y float64) *sketch.Point {
			p := s.CreatePoint(x, y)
			s.Fix(p)
			return p
		}
		bl, br, tr, tl := pt(4, 4), pt(8, 4), pt(8, 7), pt(4, 7)
		s.CreateLine(bl, br)
		s.CreateLine(br, tr)
		_, err = s.CreateFitSpline(tr, pt(7, 7.8), pt(6, 8.1), pt(5, 7.8), tl)
		require.NoError(t, err)
		s.CreateLine(tl, bl)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		require.Len(t, s.Profiles(), 1)
		return s, s.Profiles()[0]
	}
	s0, p0 := section(w.XY())
	s1, p1 := section(splitTopPlane(t, w, height))
	b, err := decad.Loft(t.Context(), doc, s0, p0, s1, p1)
	require.NoError(t, err)
	return b
}

// TestAxialSectionUnionPlateAndTongue joins a rectangular plate and a tongue with a
// curved top that crosses the plate's top edge. The overlap is a 4 by 2 rectangle,
// so the joined volume is the two volumes less 8 times the height.
func TestAxialSectionUnionPlateAndTongue(t *testing.T) {
	t.Parallel()
	const height = 5.0
	doc := decad.New()
	plate := polyPrismBody(t, doc, [][2]float64{{0, 0}, {12, 0}, {12, 6}, {0, 6}}, height)
	tongue := tongueLoft(t, doc, height)
	pv, err := plate.Volume()
	require.NoError(t, err)
	tv, err := tongue.Volume()
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), plate, tongue)
	require.NoError(t, err)
	v, err := joined.Volume()
	require.NoError(t, err)
	want := pv.Value.Base() + tv.Value.Base() - 8*height
	require.InDelta(t, want, v.Value.Base(), v.Bound.Base()+pv.Bound.Base()+tv.Bound.Base()+1e-9)
	require.True(t, joined.IsSolid())
}

// TestAxialSectionUnionLiftedLobeKeepsCapLevel joins eight lobes to a disc,
// one lobe lifted by 1e-15 mm, and extrudes a pin from the XY plane up to the
// join's end cap. The join loft's caps lie on the reference's section plane
// exactly, but the joined record stands for operands whose end planes do not
// all sit there: the lifted lobe's top rounds to a float above 2. The pin's
// top vertices must state a bound covering that gap. Shown to fail with the
// join plane gap dropped from the cap level (stateCapLevels given 0): the top
// vertices read Exact at z = 2.
func TestAxialSectionUnionLiftedLobeKeepsCapLevel(t *testing.T) {
	t.Parallel()
	const r, height, n = 3.0, 2.0, 8
	doc := decad.New()
	disc := circleBodyAtZ(t, doc, 0, r, 0, height)
	lift, err := r3.Translation(r3.NewVec(0, 0, 1e-15))
	require.NoError(t, err)
	lobes := make([]*decad.Body, n)
	liftedTop := 0.0
	for k := range lobes {
		lobes[k] = lobeLoft(t, doc, r, 2*math.Pi*float64(k)/float64(n), 0.4*math.Pi/float64(n), height)
		if k != 0 {
			continue
		}
		lobes[k], err = decad.Transform(t.Context(), lobes[k], lift)
		require.NoError(t, err)
		for _, v := range lobes[k].Vertices() {
			liftedTop = max(liftedTop, v.Position().Value.Z)
		}
	}
	require.Greater(t, liftedTop, height, "the lifted lobe's top rounds above the disc's")
	joined, err := decad.Union(t.Context(), disc, lobes...)
	require.NoError(t, err)

	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	s.CreateCircle(s.Origin(), 0.5)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	pin, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.ToFace{Body: joined, Face: capEndFace(joined)})
	require.NoError(t, err)
	tops := 0
	for _, v := range pin.Vertices() {
		p := v.Position()
		if p.Value.Z < height/2 {
			continue
		}
		tops++
		require.Equal(t, decad.Approximate, p.Exactness)
		require.GreaterOrEqual(t, p.Bound.Base(), liftedTop-height, "the top must cover the lifted lobe's end plane")
	}
	require.Positive(t, tops)
}

// TestAxialSectionUnionExactTurnsKeepExactCaps joins a lobe and its three
// quarter-turn copies to a disc and extrudes a pin up to the join's end cap.
// r3 keeps the z axis of every quarter turn exact (measured), so the join's
// plane gap is 0 and the pin's top vertices read Exact at z = 2. Shown to fail
// with the old JoinDelta == 0 gate: the vertices read Approximate, bound 5.8e-15.
func TestAxialSectionUnionExactTurnsKeepExactCaps(t *testing.T) {
	t.Parallel()
	const r, height, n = 3.0, 2.0, 4
	doc := decad.New()
	disc := circleBodyAtZ(t, doc, 0, r, 0, height)
	lobe := lobeLoft(t, doc, r, 0, 0.4*math.Pi/n, height)
	copies, err := decad.PatternCopies(t.Context(), lobe,
		decad.CircularPattern{Center: r3.Vec{}, Axis: r3.NewVec(0, 0, 1), Count: n})
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), disc, append([]*decad.Body{lobe}, copies...)...)
	require.NoError(t, err)

	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	s.CreateCircle(s.Origin(), 0.5)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	pin, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.ToFace{Body: joined, Face: capEndFace(joined)})
	require.NoError(t, err)
	tops := 0
	for _, v := range pin.Vertices() {
		p := v.Position()
		if p.Value.Z < height/2 {
			continue
		}
		tops++
		require.Equal(t, decad.Exact, p.Exactness)
		require.Zero(t, p.Bound.Base())
		require.Equal(t, height, p.Value.Z)
	}
	require.Positive(t, tops)
}

// TestAxialSectionUnionCounterboreRimChamfers joins a lobe and 16 turned copies
// to a disc, cuts a blind 0.5 mm counterbore into the rear cap and chamfers its
// rim. The rim is a Circle3, which Chamfer takes. The disc radius 12 is the
// smallest measured one (of 3, 6, 12) whose loft displacement exceeds the circle
// rim's arithmetic allowance, so the old gate reaches the declined branch. Shown
// to fail with the old JoinDelta == 0 gate: the rim reads decad.FacetedCurve.
func TestAxialSectionUnionCounterboreRimChamfers(t *testing.T) {
	t.Parallel()
	const r, height, n, bore, setback = 12.0, 2.0, 17, 0.5, 0.2
	doc := decad.New()
	disc := circleBodyAtZ(t, doc, 0, r, 0, height)
	lobe := lobeLoft(t, doc, r, 0, 0.4*math.Pi/n, height)
	copies, err := decad.PatternCopies(t.Context(), lobe,
		decad.CircularPattern{Center: r3.Vec{}, Axis: r3.NewVec(0, 0, 1), Count: n})
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), disc, append([]*decad.Body{lobe}, copies...)...)
	require.NoError(t, err)

	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), -1)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	centre := s.CreatePoint(0, 0)
	s.Fix(centre)
	s.CreateCircle(centre, bore)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	tool, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
	require.NoError(t, err)
	cut, err := decad.Cut(t.Context(), joined, tool)
	require.NoError(t, err)

	var rim *decad.Edge
	rims := 0
	for _, e := range cut.Edges() {
		p := e.Start().Position().Value
		if math.Abs(p.Z) > 1e-9 || math.Abs(math.Hypot(p.X, p.Y)-bore) > 1e-3 {
			continue
		}
		rim = e
		rims++
	}
	require.Equal(t, 1, rims)
	c, ok := rim.Curve().(decad.Circle3)
	require.True(t, ok, "the rim reads %T", rim.Curve())
	require.InDelta(t, bore, c.Radius.Base(), 1e-12)
	require.InDelta(t, 0, c.Center.X, 1e-12)
	require.InDelta(t, 0, c.Center.Y, 1e-12)
	require.InDelta(t, 0, c.Center.Z, 1e-12)

	v0, err := cut.Volume()
	require.NoError(t, err)
	chamfered, err := decad.Chamfer(t.Context(), cut,
		decad.Edges(decad.EndpointAt(rim.Start().Position().Value), decad.Circular()).Exactly(1),
		units.Millimeters(setback))
	require.NoError(t, err)
	require.True(t, chamfered.IsSolid())
	v1, err := chamfered.Volume()
	require.NoError(t, err)
	want := math.Pi * (bore + setback/3) * setback * setback
	require.InDelta(t, want, v0.Value.Base()-v1.Value.Base(), 0.01*want)
}
