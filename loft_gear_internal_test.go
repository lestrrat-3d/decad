package decad

import (
	"fmt"
	"math"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

// This file holds the helical spur gear fixture docs/loft-gear-bounds-design.md
// measures: module 2, 20° pressure angle, 10 mm face, 20° helix twist, and
// one fit spline through fitPoints involute points per flank.

// loftGear is one spur gear's tooth geometry.
type loftGear struct {
	module, teeth, pressure float64
	fitPoints               int
}

// loftGearZ is the design's gear with z teeth and five fit points per flank.
func loftGearZ(z float64) loftGear {
	return loftGear{module: 2, teeth: z, pressure: 20 * math.Pi / 180, fitPoints: 5}
}

// radii returns the base, root and tip radii.
func (g loftGear) radii() (float64, float64, float64) {
	r := g.module * g.teeth / 2
	return r * math.Cos(g.pressure), r - 1.25*g.module, r + g.module
}

// flank returns one involute flank as (radius, angle) fit points from the
// base circle to the tip circle.
func (g loftGear) flank() [][2]float64 {
	rb, _, ra := g.radii()
	thetaB := math.Pi/(2*g.teeth) + (math.Tan(g.pressure) - g.pressure)
	tmax := math.Sqrt((ra/rb)*(ra/rb) - 1)
	out := make([][2]float64, g.fitPoints)
	for k := range out {
		tt := tmax * float64(k) / float64(g.fitPoints-1)
		out[k] = [2]float64{rb * math.Sqrt(1+tt*tt), -(thetaB - (tt - math.Atan(tt)))}
	}
	return out
}

func loftGearPolar(s *sketch.Sketch, r, ang, rot float64) *sketch.Point {
	p := s.CreatePoint(r*math.Cos(ang+rot), r*math.Sin(ang+rot))
	s.Fix(p)
	return p
}

// sketchOn draws count consecutive teeth on plane: per tooth a root line, a
// fit-spline flank, a tip arc, the mirrored flank and a root line, joined by
// root arcs. One tooth closes with a line across its root.
func (g loftGear) sketchOn(t *testing.T, w *sketch.World, plane *sketch.Plane, count int) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	_, rf, _ := g.radii()
	fl := g.flank()
	thetaB := -fl[0][1]
	origin := s.CreatePoint(0, 0)
	s.Fix(origin)
	pitch := 2 * math.Pi / g.teeth
	var firstRoot, prevRoot *sketch.Point
	for k := range count {
		rot := float64(k) * pitch
		rootR := loftGearPolar(s, rf, -thetaB, rot)
		right := make([]*sketch.Point, len(fl))
		for i, p := range fl {
			right[i] = loftGearPolar(s, p[0], p[1], rot)
		}
		left := make([]*sketch.Point, len(fl))
		for i, p := range fl {
			left[len(fl)-1-i] = loftGearPolar(s, p[0], -p[1], rot)
		}
		rootL := loftGearPolar(s, rf, thetaB, rot)
		s.CreateLine(rootR, right[0])
		_, err = s.CreateFitSpline(right...)
		require.NoError(t, err)
		s.CreateArc(origin, right[len(right)-1], left[0])
		_, err = s.CreateFitSpline(left...)
		require.NoError(t, err)
		s.CreateLine(left[len(left)-1], rootL)
		if prevRoot != nil {
			s.CreateArc(origin, prevRoot, rootR)
		} else {
			firstRoot = rootR
		}
		prevRoot = rootL
	}
	if count == 1 {
		s.CreateLine(prevRoot, firstRoot)
	} else {
		s.CreateArc(origin, prevRoot, firstRoot)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	return s, profiles[0]
}

// loftGearSketches draws count teeth on z = 0 and again on z = 10, the second
// turned by the 20° helix's twist over the face width.
func loftGearSketches(t *testing.T, g loftGear, count int) (*sketch.Sketch, *sketch.Profile, *sketch.Sketch, *sketch.Profile) {
	t.Helper()
	const height = 10.0
	twist := height * math.Tan(20*math.Pi/180) / (g.module * g.teeth / 2)
	w := sketch.NewWorld()
	f0, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	f1, err := r3.NewFrame(r3.NewVec(0, 0, height), r3.NewVec(math.Cos(twist), math.Sin(twist), 0), r3.NewVec(-math.Sin(twist), math.Cos(twist), 0))
	require.NoError(t, err)
	pl0, err := w.CreatePlaneFromFrame(f0)
	require.NoError(t, err)
	pl1, err := w.CreatePlaneFromFrame(f1)
	require.NoError(t, err)
	s0, p0 := g.sketchOn(t, w, pl0, count)
	s1, p1 := g.sketchOn(t, w, pl1, count)
	return s0, p0, s1, p1
}

// loftGearAssembly runs evalLoft's prefix up to assembleLoft over count teeth
// of g and returns the assembled triangle set with its structure.
func loftGearAssembly(t *testing.T, g loftGear, count int) loftAssembly {
	t.Helper()
	s0, p0, s1, p1 := loftGearSketches(t, g, count)
	profile0, plane0, _, err := recordProfile(s0, p0)
	require.NoError(t, err)
	profile1, plane1, _, err := recordProfile(s1, p1)
	require.NoError(t, err)
	work0, work1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
	offsets, walks0, walks1, target, err := loftmesh.ValidateLoftRecords(profile0, profile1, plane0, plane1, nil, loftRecordAreas(t, profile0, profile1), work0, work1)
	require.NoError(t, err)
	pairs, _, _, stationRound, err := loftmesh.PairRecords(profile0, profile1, offsets, walks0, walks1, target, work0, work1)
	require.NoError(t, err)
	frame0, err := r3.NewFrame(plane0.Origin, plane0.U, plane0.V)
	require.NoError(t, err)
	frame1, err := r3.NewFrame(plane1.Origin, plane1.U, plane1.V)
	require.NoError(t, err)
	a, err := assembleLoft(t.Context(), pairs, frame0, frame1, plane0, r3.Identity(), stationRound)
	require.NoError(t, err)
	return a
}

// loftGearCase is one gear fixture: count teeth of the z = teeth gear.
type loftGearCase struct {
	name  string
	teeth float64
	count int
	// heavy marks a full outline too slow for the race shards: it skips under
	// -short and under the race detector (docs/loft-gear-bounds-design.md §7).
	heavy bool
}

// loftGearTeeth are the one-tooth fixtures and loftGearOutlines the full
// outlines of docs/loft-gear-bounds-design.md §8.
var (
	loftGearTeeth = []loftGearCase{
		{name: "tooth z8", teeth: 8, count: 1},
		{name: "tooth z20", teeth: 20, count: 1},
		{name: "tooth z40", teeth: 40, count: 1},
	}
	loftGearOutlines = []loftGearCase{
		{name: "outline z8", teeth: 8, count: 8},
		{name: "outline z20", teeth: 20, count: 20, heavy: true},
		{name: "outline z40", teeth: 40, count: 40, heavy: true},
	}
)

func (c loftGearCase) skipHeavy(t *testing.T) {
	t.Helper()
	if !c.heavy {
		return
	}
	if testing.Short() {
		t.Skipf("the %s build takes seconds; run without -short", c.name)
	}
	if raceDetector {
		t.Skipf("the %s build is too slow under the race detector for the race shards", c.name)
	}
}

// loftGearDenseLoop samples one recorded loop densely in walk order: perSeg
// points per ArcSeg at even steps of its recorded range, and perSeg per
// converted Bézier span of a free-form segment (denseWalkSamples). A LineSeg
// contributes its start point alone, and faceted marks that sample: the loft
// walls a LineSeg pair by the triangle pair through its four corners
// (docs/loft-design.md §5), not by a ruled patch. Every sample lies on the
// recorded curve.
func loftGearDenseLoop(t *testing.T, loop LoopRecord, perSeg int) ([]Point2, []bool) {
	t.Helper()
	var out []Point2
	var faceted []bool
	for _, seg := range loop.Segments {
		switch s := seg.(type) {
		case LineSeg:
			u, v := recordPointAt(t, seg, s.TStart)
			out = append(out, pt(u, v))
			faceted = append(faceted, true)
		case ArcSeg:
			for k := range perSeg {
				u, v := recordPointAt(t, seg, s.TStart+(s.TEnd-s.TStart)*float64(k)/float64(perSeg))
				out = append(out, pt(u, v))
				faceted = append(faceted, false)
			}
		default:
			samples := denseWalkSamples(t, seg, perSeg)
			out = append(out, samples...)
			for range samples {
				faceted = append(faceted, false)
			}
		}
	}
	return out, faceted
}

// loftGearReference is the true gear loft's volume, centroid and surface
// area. Both sections record the same local curve and the loft pairs each
// point with the same local point on the other section, so lifting one dense
// sample through both frames gives matched rulings. The closed surface is the
// ruled patch over every curved sample interval, the loft's own triangle pair
// (diagonal from bottom sample i to top sample i+1, as assembleLoft splits a
// cell) over every LineSeg, and the two planar caps. Volume and first moment
// come from the divergence theorem over that surface: ∮ (p·n)/3 and
// ∮ p(p·n)/4, which 4-point Gauss–Legendre integrates exactly on a ruled
// patch, whose integrands are cubic in each parameter.
func loftGearReference(t *testing.T, lp loftPayload) ruledReferenceReadings {
	t.Helper()
	dense, faceted := loftGearDenseLoop(t, lp.profile0.Outer, 512)
	x0 := liftSamples(dense, lp.frame0)
	x1 := liftSamples(dense, lp.frame1)
	n := len(dense)

	var vol, wall float64
	var moment r3.Vec
	triangle := func(a, b, c r3.Vec) {
		v := a.Dot(b.Cross(c)) / 6
		vol += v
		moment = moment.Add(a.Add(b).Add(c).Scale(v / 4))
	}
	gauss := [4][2]float64{
		{0.5 - 0.3399810435848563/2, 0.6521451548625461 / 2},
		{0.5 + 0.3399810435848563/2, 0.6521451548625461 / 2},
		{0.5 - 0.8611363115940526/2, 0.3478548451374538 / 2},
		{0.5 + 0.8611363115940526/2, 0.3478548451374538 / 2},
	}
	for i := range n {
		j := (i + 1) % n
		a, b, c, d := x0[i], x0[j], x1[i], x1[j]
		if faceted[i] {
			triangle(a, b, d)
			triangle(a, d, c)
			wall += b.Sub(a).Cross(d.Sub(a)).Len()/2 + d.Sub(a).Cross(c.Sub(a)).Len()/2
			continue
		}
		for _, gu := range gauss {
			for _, gl := range gauss {
				u, l := gu[0], gl[0]
				w := gu[1] * gl[1]
				p := a.Scale((1 - u) * (1 - l)).Add(b.Scale(u * (1 - l))).Add(c.Scale((1 - u) * l)).Add(d.Scale(u * l))
				xu := b.Sub(a).Scale(1 - l).Add(d.Sub(c).Scale(l))
				xl := c.Sub(a).Scale(1 - u).Add(d.Sub(b).Scale(u))
				cross := xu.Cross(xl)
				flux := p.Dot(cross)
				vol += w * flux / 3
				moment = moment.Add(p.Scale(w * flux / 4))
				wall += w * cross.Len()
			}
		}
	}
	// The caps are the planar polygons through the samples, fanned from the
	// first sample of each and oriented against the walls.
	capArea := 0.0
	for i := 1; i+1 < n; i++ {
		triangle(x0[0], x0[i+1], x0[i])
		triangle(x1[0], x1[i], x1[i+1])
		capArea += x0[i].Sub(x0[0]).Cross(x0[i+1].Sub(x0[0])).Z / 2
	}
	if vol < 0 {
		vol, moment = -vol, moment.Scale(-1)
	}
	return ruledReferenceReadings{
		volume:   vol,
		centroid: moment.Scale(1 / vol),
		area:     wall + 2*math.Abs(capArea),
	}
}

// requireLoftGearSound builds c through Document.Loft and asserts what
// docs/loft-gear-bounds-design.md §8 requires of a gear fixture: Verify reads
// Sound, every reading's ratio to its reference is below the default 1e-3
// tolerance, the station count is inside stationCap(P), and Volume, Centroid
// and Area each enclose the dense-sample reference. It logs the build time.
func requireLoftGearSound(t *testing.T, c loftGearCase) {
	t.Helper()
	s0, p0, s1, p1 := loftGearSketches(t, loftGearZ(c.teeth), c.count)
	doc := New()
	start := time.Now()
	body, err := doc.Loft(t.Context(), s0, p0, s1, p1)
	built := time.Since(start)
	require.NoError(t, err)
	report, err := doc.Verify(t.Context())
	verified := time.Since(start) - built
	require.NoError(t, err)
	require.Equal(t, Sound, report.Status, "the gear loft must verify Sound at the default tolerance")

	ratio, binding := loftBodyBindingRatio(t, t.Context(), body)
	require.Less(t, ratio, toleranceRel, "every reading must sit inside the default tolerance")

	lp := body.payload.(loftPayload)
	p := uint64(len(lp.profile0.Outer.Segments))
	stations := lp.walls / 2
	require.Less(t, stations, loftmesh.StationCap(p), "the build must settle inside stationCap(P)")

	ref := loftGearReference(t, lp)
	vol, err := body.Volume()
	require.NoError(t, err)
	cen, err := body.Centroid()
	require.NoError(t, err)
	area, err := body.Area()
	require.NoError(t, err)
	t.Logf("%s: P=%d stations=%d build=%s verify=%s binding=%s ratio=%.3g", c.name, p, stations, built, verified, binding, ratio)
	t.Logf("  Volume %.10g ± %.3g, reference %.10g (residual %.3g)", vol.Value.Base(), vol.Bound.Base(), ref.volume, math.Abs(vol.Value.Base()-ref.volume))
	t.Logf("  Centroid %v ± %.3g, reference %v (residual %.3g)", cen.Value, cen.Bound.Base(), ref.centroid, cen.Value.Sub(ref.centroid).Len())
	t.Logf("  Area %.10g ± %.3g, reference %.10g (residual %.3g)", area.Value.Base(), area.Bound.Base(), ref.area, math.Abs(area.Value.Base()-ref.area))
	require.LessOrEqual(t, math.Abs(vol.Value.Base()-ref.volume), vol.Bound.Base(), "Volume must enclose the dense reference")
	require.LessOrEqual(t, cen.Value.Sub(ref.centroid).Len(), cen.Bound.Base(), "Centroid must enclose the dense reference")
	require.LessOrEqual(t, math.Abs(area.Value.Base()-ref.area), area.Bound.Base(), "Area must enclose the dense reference")
}

// TestLoftGearToothVerifiesSound builds one tooth of the z = 8, 20 and 40
// helical gears (docs/loft-gear-bounds-design.md §8) and requires each to
// verify Sound with readings enclosing the dense-sample reference. A tooth
// needs at most 132 stations and fits the default work ceiling, so this test
// guards the readings; the outline test below guards the ceilings.
//
// Shown to fail first: deleting LoftChordedAllow.WallLeg from Volume's
// allowance fails every row on the Volume enclosure. Deleting the ruled area
// leg (AreaExcess) from Area's bound leaves every row green: the remaining
// legs still enclose the reference.
func TestLoftGearToothVerifiesSound(t *testing.T) {
	t.Parallel()
	for _, c := range loftGearTeeth {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			requireLoftGearSound(t, c)
		})
	}
}

// TestLoftGearOutlineVerifiesSound builds the full z = 8, 20 and 40 helical
// gear outlines (docs/loft-gear-bounds-design.md §8). Each must build inside
// the raised ceilings and verify Sound with readings enclosing the
// dense-sample reference. z = 20 and z = 40 skip under -short and under the
// race detector.
//
// Shown to fail first: a fixed 500-station cap refuses every outline (S15 at
// z = 8 and 20, R7 at z = 40, whose work raise the cap also sizes); 32
// stations per paired segment instead of 64 refuses z = 8 with S15, its flanks
// needing 48 cells against a share of 47; leaving the work ceiling at its
// 1 << 20 default refuses z = 8 with R7 while its walks resolve; restoring
// the 1 << 26 reconstruction ceiling refuses z = 40 before its loft starts;
// deleting LoftChordedAllow.WallLeg from Volume's allowance fails z = 8 on the
// Volume enclosure.
func TestLoftGearOutlineVerifiesSound(t *testing.T) {
	t.Parallel()
	for _, c := range loftGearOutlines {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			c.skipHeavy(t)
			requireLoftGearSound(t, c)
		})
	}
}

// TestLoftStationWalkWorkPerStation runs Loft's own prefix on each gear
// fixture with the two records' counters in hand. It asserts the raise
// loftmesh.ValidateLoftRecords applies before it resolves a walk,
// loftmesh.StationWorkLimit over the counter's spend after the area
// falsifier, and that the walk resolution and the station walk together stay
// below loftmesh.StationWorkUnits per station on every case, which is what
// sizes the raise (docs/loft-gear-bounds-design.md §7).
func TestLoftStationWalkWorkPerStation(t *testing.T) {
	t.Parallel()
	for _, c := range append(append([]loftGearCase(nil), loftGearTeeth...), loftGearOutlines...) {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			c.skipHeavy(t)
			s0, sp0, s1, sp1 := loftGearSketches(t, loftGearZ(c.teeth), c.count)
			profile0, plane0, area0, err := recordProfile(s0, sp0)
			require.NoError(t, err)
			profile1, plane1, area1, err := recordProfile(s1, sp1)
			require.NoError(t, err)
			work0, work1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
			a0, err := falsifyRecordedArea(profile0, area0, work0)
			require.NoError(t, err)
			a1, err := falsifyRecordedArea(profile1, area1, work1)
			require.NoError(t, err)
			works := []*freeform.FreeformWork{work0, work1}
			before := []uint64{work0.Spent, work1.Spent}

			offsets, walks0, walks1, target, err := loftmesh.ValidateLoftRecords(profile0, profile1, plane0, plane1, nil, [2]float64{a0, a1}, work0, work1)
			require.NoError(t, err)
			p := uint64(len(profile0.Outer.Segments))
			for i, w := range works {
				require.Equal(t, loftmesh.StationWorkLimit(before[i], p), w.WorkLimit(),
					"loftmesh.ValidateLoftRecords raises record %d's ceiling to StationWorkLimit over its spend", i)
				require.Greater(t, w.WorkLimit(), freeform.FreeformWorkLimit)
			}

			pairs, _, _, _, err := loftmesh.PairRecords(profile0, profile1, offsets, walks0, walks1, target, work0, work1)
			require.NoError(t, err)
			stations := 0
			for _, pair := range pairs {
				stations += len(pair.V)
			}
			for i, w := range works {
				perStation := (w.Spent - before[i]) / uint64(stations) //nolint:gosec // stations is a positive count.
				t.Logf("%s record %d: stations=%d walk work=%d per station=%d spent=%d limit=%d",
					c.name, i, stations, w.Spent-before[i], perStation, w.Spent, w.WorkLimit())
				require.Less(t, perStation, uint64(loftmesh.StationWorkUnits),
					"record %d's walks and station walk must charge less than StationWorkUnits per station", i)
			}
		})
	}
}

// TestLoftStationWalkRefusesAtTheRaisedWorkLimit is R7 at the raised
// ceiling. Each gear tooth record's counter is raised by loftmesh.ValidateLoftRecords,
// then left with half the station walk's measured cost: the walk runs past
// the default 1 << 20 (the counters already hold more than that) and refuses
// with ErrUnsupported exactly at the raised limit.
//
// Shown to fail first: with RaiseLimit removed from loftmesh.ValidateLoftRecords, the
// first assertion fails, and the walk refuses on its first charge rather than
// at the raised limit.
func TestLoftStationWalkRefusesAtTheRaisedWorkLimit(t *testing.T) {
	t.Parallel()
	s0, sp0, s1, sp1 := loftGearSketches(t, loftGearZ(8), 1)
	profile0, plane0, _, err := recordProfile(s0, sp0)
	require.NoError(t, err)
	profile1, plane1, _, err := recordProfile(s1, sp1)
	require.NoError(t, err)
	areas := loftRecordAreas(t, profile0, profile1)

	measure := func() (uint64, uint64) {
		work0, work1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
		offsets, walks0, walks1, target, err := loftmesh.ValidateLoftRecords(profile0, profile1, plane0, plane1, nil, areas, work0, work1)
		require.NoError(t, err)
		before0, before1 := work0.Spent, work1.Spent
		_, _, _, _, err = loftmesh.PairRecords(profile0, profile1, offsets, walks0, walks1, target, work0, work1)
		require.NoError(t, err)
		return work0.Spent - before0, work1.Spent - before1
	}
	cost0, cost1 := measure()
	require.Positive(t, cost0)
	require.Positive(t, cost1)

	work0, work1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
	offsets, walks0, walks1, target, err := loftmesh.ValidateLoftRecords(profile0, profile1, plane0, plane1, nil, areas, work0, work1)
	require.NoError(t, err)
	require.Greater(t, work0.WorkLimit(), freeform.FreeformWorkLimit+cost0, "the raise must leave room past the default ceiling")
	require.Greater(t, work1.WorkLimit(), freeform.FreeformWorkLimit+cost1, "the raise must leave room past the default ceiling")
	work0.Spent = work0.WorkLimit() - cost0/2
	work1.Spent = work1.WorkLimit() - cost1/2

	start := time.Now()
	_, _, _, _, err = loftmesh.PairRecords(profile0, profile1, offsets, walks0, walks1, target, work0, work1) //nolint:dogsled // only the refusal matters here.
	require.ErrorIs(t, err, ErrUnsupported)
	refused := work0
	if work1.Spent == work1.WorkLimit() {
		refused = work1
	}
	require.Equal(t, refused.WorkLimit(), refused.Spent, "the refused counter is spent to its raised limit")
	require.ErrorContains(t, err, fmt.Sprintf("fixed work budget of %d", refused.WorkLimit()),
		"the refusal names the raised ceiling, not the default")
	require.Less(t, time.Since(start), 10*time.Second, "the refusal comes within the walk's own bounded cost")
}
