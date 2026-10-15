package decad

import (
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/twistsweep"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func roundedTwistLocalAreaSource(t *testing.T) (*Document, *Body, twistedSweepPayload) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	point := func(x, y float64) *sketch.Point {
		p := s.CreatePoint(x, y)
		s.Fix(p)
		return p
	}
	p := [8]*sketch.Point{
		point(-1.5, -1), point(1.5, -1), point(2, -0.5), point(2, 0.5),
		point(1.5, 1), point(-1.5, 1), point(-2, 0.5), point(-2, -0.5),
	}
	s.CreateLine(p[0], p[1])
	s.CreateArc(point(1.5, -0.5), p[1], p[2])
	s.CreateLine(p[2], p[3])
	s.CreateArc(point(1.5, 0.5), p[3], p[4])
	s.CreateLine(p[4], p[5])
	s.CreateArc(point(-1.5, 0.5), p[5], p[6])
	s.CreateLine(p[6], p[7])
	s.CreateArc(point(-1.5, -0.5), p[7], p[0])
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var profile *sketch.Profile
	for _, candidate := range s.Profiles() {
		if candidate.Valid && len(candidate.Outer) == 8 && len(candidate.Holes) == 0 {
			profile = candidate
			break
		}
	}
	require.NotNil(t, profile)
	path, err := NewPath(r3.Vec{}, LineTo{End: r3.NewVec(0, 0, 5)})
	require.NoError(t, err)
	doc := New()
	body, err := Sweep(t.Context(), doc, s, profile, path, WithSweepTwist(units.Degrees(30)))
	require.NoError(t, err)
	payload, ok := body.n.Payload.(twistedSweepPayload)
	require.True(t, ok)
	return doc, body, payload
}

// This uses the same authenticated Sketch profile and private Loft producer
// as the public Sweep. The local claim must be much smaller than charging
// both whole positive wall areas for a later Boolean subset.
func TestTwistLocalAreaRealArcCell(t *testing.T) {
	doc, body, source := roundedTwistLocalAreaSource(t)
	poly, err := twistsweep.TwistPolygonOf(source.profile)
	require.NoError(t, err)
	angleIv, ok := revolveangle.FromValue(source.angle).Enclosure()
	require.True(t, ok)
	angle, err := source.angle.In(units.Radian)
	require.NoError(t, err)
	baseCells, refinedCells, err := stationedTwistCellCounts(poly,
		proofbound.RatFloatUp(proofbound.IntervalAbsUpper(angleIv)))
	require.NoError(t, err)
	var cells int
	var stations []straightTwistStation
	var held loftPayload
	for _, candidate := range []int{refinedCells, baseCells} {
		candidateStations, stationErr := straightTwistStations(source.path.End.Z, angle,
			angleIv, poly, candidate)
		require.NoError(t, stationErr)
		part, buildErr := bodyResult(loftrecord.Build(t.Context(), envOf(documentNode(doc), body.n.Origin.Producer), loftPayload{
			Profile0: source.profile, Profile1: source.profile,
			Plane0: candidateStations[0].Plane, Plane1: candidateStations[1].Plane,
			Frame0: candidateStations[0].Frame, Frame1: candidateStations[1].Frame,
			Xform: source.xform, RecordArea: [2]float64{source.recordArea, source.recordArea},
			RetainTwistAreaCells: true,
		}, proofbound.NewWorkBudget(t.Context()), freeform.NewFreeformWork(), freeform.NewFreeformWork()))
		require.NoError(t, buildErr)
		candidateHeld, ok := part.n.Payload.(loftPayload)
		require.True(t, ok)
		n := len(candidateHeld.Segment[0])
		if len(source.mesh.n.Vertices) == (candidate+1)*n &&
			slices.Equal(source.mesh.n.Vertices[:2*n], candidateHeld.Verts) {
			cells, stations, held = candidate, candidateStations, candidateHeld
			break
		}
	}
	require.Positive(t, cells, "the real Sweep must use one certified station count")
	require.Len(t, held.TwistAreaCells, 1)
	n := len(held.Segment[0])
	j := -1
	for k, segment := range held.Segment[0] {
		if segment == 1 {
			j = k
			break
		}
	}
	require.GreaterOrEqual(t, j, 0)
	claim := held.TwistAreaCells[0][j]
	vLo, vHi := held.Verts[j], held.Verts[(j+1)%n]
	wLo, wHi := held.Verts[n+j], held.Verts[n+(j+1)%n]
	e := proofbound.AbsSumUpper(held.Delta,
		math.Max(stations[0].PointError, stations[1].PointError))
	bound := twistsweep.TwistWallCellAreaL1(vLo, vHi, wLo, wHi, claim, e,
		poly.MaxRadiusL1, proofbound.DivUpper(proofbound.RatFloatUp(proofbound.IntervalAbsUpper(angleIv)), float64(cells)),
		proofbound.DivUpper(source.path.End.Z, float64(cells)))
	require.True(t, !proofbound.IsNonFinite(bound))
	t.Logf("cells=%d source_segment=%d cell=%d bound=%g arc=%g energy=%g matched=%g endpoint=%g",
		cells, held.Segment[0][j], j, bound, claim.ArcLength, claim.Energy, claim.Matched, e)
	require.Less(t, bound, 2.0)
	// Independently integrate the area-density difference on this actual
	// quarter-circle cell. Midpoint sampling is a regression witness, not the
	// certificate: twistWallCellAreaL1 proves the bound for every parameter.
	arcCells := 0
	for _, segment := range held.Segment[0] {
		if segment == 1 {
			arcCells++
		}
	}
	require.Positive(t, arcCells)
	const samples = 128
	turn := angle / float64(cells)
	arcTurn := math.Pi / (2 * float64(arcCells))
	heldArea0 := vHi.Sub(vLo).Cross(wHi.Sub(vHi)).Len()
	heldArea1 := wHi.Sub(wLo).Cross(wLo.Sub(vLo)).Len()
	sampled, sampledSubset := 0.0, 0.0
	for si := range samples {
		s := (float64(si) + 0.5) / samples
		phi := -math.Pi/2 + s*arcTurn
		p := r3.NewVec(1.5+0.5*math.Cos(phi), -0.5+0.5*math.Sin(phi), 0)
		dp := r3.NewVec(-0.5*arcTurn*math.Sin(phi), 0.5*arcTurn*math.Cos(phi), 0)
		for ui := range samples {
			u := (float64(ui) + 0.5) / samples
			sin, cos := math.Sincos(u * turn)
			ts := r3.NewVec(cos*dp.X-sin*dp.Y, sin*dp.X+cos*dp.Y, 0)
			tu := r3.NewVec(-turn*(sin*p.X+cos*p.Y), turn*(cos*p.X-sin*p.Y),
				source.path.End.Z/float64(cells))
			heldArea := heldArea0
			if u > s {
				heldArea = heldArea1
			}
			gap := math.Abs(ts.Cross(tu).Len() - heldArea)
			sampled += gap / (samples * samples)
			if s > 0.25 && u < 0.75 {
				sampledSubset += gap / (samples * samples)
			}
		}
	}
	t.Logf("sampled absolute density=%g sampled subset=%g", sampled, sampledSubset)
	require.Positive(t, sampledSubset)
	require.Less(t, sampled, bound)
	require.Less(t, sampledSubset, bound)
	heldCells := make([]loftPayload, cells)
	heldCells[0] = held
	for i := 1; i < cells; i++ {
		part, err := bodyResult(loftrecord.Build(t.Context(), envOf(documentNode(doc), body.n.Origin.Producer), loftPayload{
			Profile0: source.profile, Profile1: source.profile,
			Plane0: stations[i].Plane, Plane1: stations[i+1].Plane,
			Frame0: stations[i].Frame, Frame1: stations[i+1].Frame,
			Xform: source.xform, RecordArea: [2]float64{source.recordArea, source.recordArea},
			RetainTwistAreaCells: true,
		}, proofbound.NewWorkBudget(t.Context()), freeform.NewFreeformWork(), freeform.NewFreeformWork()))
		require.NoError(t, err)
		heldCells[i], ok = part.n.Payload.(loftPayload)
		require.True(t, ok)
	}
	for i, cell := range heldCells {
		for k := range cell.Verts {
			require.Equal(t, source.mesh.n.Vertices[i*n+k], cell.Verts[k])
		}
	}
	global := 0
	for i, cell := range heldCells {
		for k, tri := range cell.Tris {
			if k >= cell.Walls && k < cell.Walls+cell.CapStartCount && i != 0 ||
				k >= cell.Walls+cell.CapStartCount && i != cells-1 {
				continue
			}
			require.Less(t, global, len(source.mesh.n.Triangles))
			require.Equal(t, [3]int{i*n + tri[0], i*n + tri[1], i*n + tri[2]},
				source.mesh.n.Triangles[global])
			global++
		}
	}
	require.Equal(t, len(source.mesh.n.Triangles), global)
	wallBound, err := twistMixedWallAreaL1(heldCells, stations, poly,
		proofbound.RatFloatUp(proofbound.IntervalAbsUpper(angleIv)), source.path.End.Z)
	require.NoError(t, err)
	t.Logf("all wall-cell absolute area allowance=%g", wallBound)
	require.Less(t, wallBound, 20.0)
	comparison, _, heldByFace, err := compositeTwistMesh(t.Context(), body, heldCells,
		len(poly.Points), make([]int, cells), source.delta, source.volSymDiff)
	require.NoError(t, err)
	require.Equal(t, source.mesh.n.Vertices, comparison.n.Vertices)
	require.Equal(t, source.mesh.n.Triangles, comparison.n.Triangles)
	localSlack, err := twistMixedAreaSlackLocal(source.profile, poly, body, heldByFace, wallBound)
	require.NoError(t, err)
	t.Logf("local mixed area allowance=%g", localSlack)
	require.Equal(t, localSlack, source.mesh.n.AreaSlack)
	require.Less(t, localSlack, 50.0)
	corrupt := append([]loftPayload(nil), heldCells...)
	corrupt[0].Side = append([]uint8(nil), corrupt[0].Side...)
	corrupt[0].Side[2*j+1] = 0
	_, err = twistMixedWallAreaL1(corrupt, stations, poly,
		proofbound.RatFloatUp(proofbound.IntervalAbsUpper(angleIv)), source.path.End.Z)
	require.ErrorIs(t, err, ErrUnsupported)
	corrupt[0] = heldCells[0]
	corrupt[0].Tris = append([][3]int(nil), corrupt[0].Tris...)
	corrupt[0].Tris[2*j+1][2] = corrupt[0].Tris[2*j+1][1]
	_, err = twistMixedWallAreaL1(corrupt, stations, poly,
		proofbound.RatFloatUp(proofbound.IntervalAbsUpper(angleIv)), source.path.End.Z)
	require.ErrorIs(t, err, ErrUnsupported)
	corrupt[0] = heldCells[0]
	corrupt[0].TwistAreaCells = nil
	_, err = twistMixedWallAreaL1(corrupt, stations, poly,
		proofbound.RatFloatUp(proofbound.IntervalAbsUpper(angleIv)), source.path.End.Z)
	require.ErrorIs(t, err, ErrUnsupported)
	corrupt[0] = heldCells[0]
	corrupt[0].TwistAreaCells = [][]loftrecord.TwistAreaCellClaims{
		append([]loftrecord.TwistAreaCellClaims(nil), heldCells[0].TwistAreaCells[0]...),
	}
	corrupt[0].TwistAreaCells[0][j].Matched = -1
	_, err = twistMixedWallAreaL1(corrupt, stations, poly,
		proofbound.RatFloatUp(proofbound.IntervalAbsUpper(angleIv)), source.path.End.Z)
	require.ErrorIs(t, err, ErrUnsupported)
}
