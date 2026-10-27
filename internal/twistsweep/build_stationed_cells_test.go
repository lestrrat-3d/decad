package twistsweep

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// stationedCase is one straight twist the cell differential test builds.
type stationedCase struct {
	name    string
	profile profileRecord
	deg     float64
	height  float64
	xform   r3.Transform
	// refine adds the refinement's cell count; optional also runs each count
	// as an optional attempt, which checks the first cell's work projection.
	refine, optional bool
	// race keeps the case in the smaller set a race build runs.
	race bool
}

func polygonRecord(points ...[2]float64) profileRecord {
	segs := make([]sectionrecord.CurveSegment, len(points))
	for i, p := range points {
		q := points[(i+1)%len(points)]
		segs[i] = sectionrecord.LineSeg{
			Start: sectionrecord.Point2{U: p[0], V: p[1]},
			End:   sectionrecord.Point2{U: q[0], V: q[1]}, TEnd: 1,
		}
	}
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}}
}

// roundedRecord is a 8 by 6 rectangle whose corners are quarter arcs of
// radius 1.5, wound counter-clockwise.
func roundedRecord() profileRecord {
	pt := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	line := func(a, b sectionrecord.Point2) sectionrecord.CurveSegment {
		return sectionrecord.LineSeg{Start: a, End: b, TEnd: 1}
	}
	arc := func(c, a, b sectionrecord.Point2) sectionrecord.CurveSegment {
		return sectionrecord.ArcSeg{Center: c, Start: a, End: b, TEnd: 1}
	}
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		line(pt(-2.5, -3), pt(2.5, -3)), arc(pt(2.5, -1.5), pt(2.5, -3), pt(4, -1.5)),
		line(pt(4, -1.5), pt(4, 1.5)), arc(pt(2.5, 1.5), pt(4, 1.5), pt(2.5, 3)),
		line(pt(2.5, 3), pt(-2.5, 3)), arc(pt(-2.5, 1.5), pt(-2.5, 3), pt(-4, 1.5)),
		line(pt(-4, 1.5), pt(-4, -1.5)), arc(pt(-2.5, -1.5), pt(-4, -1.5), pt(-2.5, -3)),
	}}}
}

func placedXform(t *testing.T) r3.Transform {
	t.Helper()
	turn, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	require.NoError(t, err)
	move, err := r3.Translation(r3.NewVec(15.5, -2.25, 9))
	require.NoError(t, err)
	placed, err := turn.Then(move)
	require.NoError(t, err)
	return placed
}

func stationedCases(t *testing.T) []stationedCase {
	t.Helper()
	bore := polygonRecord([2]float64{-7.7, -2.375}, [2]float64{7.7, -2.375},
		[2]float64{7.7, 2.375}, [2]float64{-7.7, 2.375})
	hexagon := polygonRecord([2]float64{3, 0}, [2]float64{1.5, 2.6}, [2]float64{-1.5, 2.6},
		[2]float64{-3, 0}, [2]float64{-1.5, -2.6}, [2]float64{1.5, -2.6})
	placed := placedXform(t)
	return []stationedCase{
		{"bore 30", bore, 30, 6, r3.Identity(), true, false, true},
		{"bore 87 placed", bore, 87, 6, placed, true, false, false},
		{"bore 120", bore, 120, 6, r3.Identity(), false, false, false},
		{"bore -75 placed", bore, -75, 6, placed, true, false, false},
		{"hexagon 200", hexagon, 200, 10, r3.Identity(), false, false, false},
		{"rounded 30", roundedRecord(), 30, 5, r3.Identity(), false, true, true},
		{"rounded 80 placed", roundedRecord(), 80, 5, placed, false, true, false},
	}
}

// stationedCellCounts lists the cell counts the build reaches for poly: the
// stationed base and finer counts and, with refine, the refinement count of a
// straight polygon. Two cells is the smallest count the composite mesh
// accepts.
func stationedCellCounts(t *testing.T, poly TwistPolygon, upper float64, refine bool) []int {
	t.Helper()
	base, finer, err := StationedTwistCellCounts(poly, upper)
	require.NoError(t, err)
	counts := []int{2, base}
	if finer != base {
		counts = append(counts, finer)
	}
	if refine {
		refined, err := refinedCellCount(poly, upper)
		require.NoError(t, err)
		counts = append(counts, refined)
	}
	return counts
}

// TestBuildStationedCellsMatchesSerial builds each twist's cells with
// buildStationedCells and with the serial reference loop, then finishes both
// into bodies, and requires the same error or bit-identical bodies. A race
// build runs the cases marked race, a refined polygon and an arc profile,
// without their optional attempts.
func TestBuildStationedCellsMatchesSerial(t *testing.T) {
	env := topo.Env{Alloc: topotest.Alloc{}}
	for _, tc := range stationedCases(t) {
		if raceDetector && !tc.race {
			continue
		}
		t.Run(tc.name, func(t *testing.T) {
			poly, err := TwistPolygonOf(tc.profile)
			require.NoError(t, err, tc.name)
			angle := units.Degrees(tc.deg)
			theta, err := angle.In(units.Radian)
			require.NoError(t, err)
			thetaIv, ok := revolveangle.FromValue(angle).Enclosure()
			require.True(t, ok)
			absTheta := proofbound.IntervalAbsUpper(thetaIv)
			charge, err := placementCharge(tc.xform)
			require.NoError(t, err)
			recordArea, _ := poly.Area.Float64()
			path := &sweepinput.Path{End: r3.NewVec(0, 0, tc.height)}
			plane := planeRecord{U: r3.NewVec(1, 0, 0), V: r3.NewVec(0, 1, 0)}
			for _, cells := range stationedCellCounts(t, poly, proofbound.RatFloatUp(absTheta), tc.refine) {
				attempts := []bool{false}
				if tc.optional && !raceDetector {
					attempts = append(attempts, true)
				}
				for _, optional := range attempts {
					label := fmt.Sprintf("%s cells=%d optional=%v", tc.name, cells, optional)
					stations, err := StraightTwistStations(tc.height, theta, thetaIv, poly, cells)
					require.NoError(t, err, label)
					finish := func(parts []compositesweep.SpanPart, held []loftrecord.Payload, err error) (*topo.Body, error) {
						if err != nil {
							return nil, err
						}
						return finishStationedCells(t.Context(), env, tc.profile, plane, path, angle, recordArea,
							tc.xform, poly, thetaIv, absTheta, charge, stations, parts, held)
					}
					got, gotErr := finish(buildStationedCells(t.Context(), env, tc.profile, tc.xform, recordArea,
						poly, stations, cells, optional))
					want, wantErr := finish(legacyBuildStationedCells(t.Context(), env, tc.profile, tc.xform, recordArea,
						poly, stations, cells, optional))
					if wantErr != nil {
						require.Error(t, gotErr, label)
						require.Equal(t, wantErr.Error(), gotErr.Error(), label)
						continue
					}
					require.NoError(t, gotErr, label)
					requireSameStationedBody(t, label, want, got)
				}
			}
		})
	}
}

func requireSameBits(t *testing.T, label string, want, got float64) {
	t.Helper()
	require.Equalf(t, math.Float64bits(want), math.Float64bits(got), "%s: %v against %v", label, got, want)
}

func requireSameVecs(t *testing.T, label string, want, got []r3.Vec) {
	t.Helper()
	require.Lenf(t, got, len(want), "%s: vertex count", label)
	for i := range want {
		requireSameBits(t, fmt.Sprintf("%s vertex %d x", label, i), want[i].X, got[i].X)
		requireSameBits(t, fmt.Sprintf("%s vertex %d y", label, i), want[i].Y, got[i].Y)
		requireSameBits(t, fmt.Sprintf("%s vertex %d z", label, i), want[i].Z, got[i].Z)
	}
}

// requireSameStationedBody compares the published readings of two stationed
// bodies field by field, so a failure names one field and two scalars.
func requireSameStationedBody(t *testing.T, label string, want, got *topo.Body) {
	t.Helper()
	wp, ok := want.Payload.(Payload)
	require.True(t, ok, label)
	gp, ok := got.Payload.(Payload)
	require.True(t, ok, label)
	requireSameBits(t, label+" delta", wp.Delta, gp.Delta)
	requireSameBits(t, label+" area slack", wp.AreaSlack, gp.AreaSlack)
	requireSameBits(t, label+" volume tube", wp.VolSymDiff, gp.VolSymDiff)
	wm, gm := wp.Mesh, gp.Mesh
	requireSameVecs(t, label+" mesh", wm.Vertices, gm.Vertices)
	require.Lenf(t, gm.Triangles, len(wm.Triangles), "%s: triangle count", label)
	for i := range wm.Triangles {
		require.Equalf(t, wm.Triangles[i], gm.Triangles[i], "%s: triangle %d", label, i)
		wf, gf := wm.Source[i], gm.Source[i]
		require.Equalf(t, wf.Origins[0].Role, gf.Origins[0].Role, "%s: triangle %d source", label, i)
		requireSameBits(t, fmt.Sprintf("%s triangle %d face bound", label, i), wm.FaceBound[wf], gm.FaceBound[gf])
	}
	requireSameBits(t, label+" mesh bound", wm.Bound, gm.Bound)
	requireSameBits(t, label+" mesh coordinate bound", wm.CoordBound, gm.CoordBound)
	requireSameBits(t, label+" mesh area slack", wm.AreaSlack, gm.AreaSlack)
	requireSameBits(t, label+" mesh volume tube", wm.VolSymDiff, gm.VolSymDiff)
	for name, pair := range map[string][2]float64{
		"volume":       {want.Volume.Value.Base(), got.Volume.Value.Base()},
		"volume bound": {want.Volume.Bound.Base(), got.Volume.Bound.Base()},
		"area":         {want.Area.Value.Base(), got.Area.Value.Base()},
		"area bound":   {want.Area.Bound.Base(), got.Area.Bound.Base()},
		"centre bound": {want.Centroid.Bound.Base(), got.Centroid.Bound.Base()},
		"box bound":    {want.Bounds.Bound.Base(), got.Bounds.Bound.Base()},
	} {
		requireSameBits(t, label+" "+name, pair[0], pair[1])
	}
	requireSameVecs(t, label+" centre and box", []r3.Vec{want.Centroid.Value, want.Bounds.Min, want.Bounds.Max},
		[]r3.Vec{got.Centroid.Value, got.Bounds.Min, got.Bounds.Max})
	wantFaces, gotFaces := topobuild.Faces(want), topobuild.Faces(got)
	require.Lenf(t, gotFaces, len(wantFaces), "%s: face count", label)
	for i := range wantFaces {
		require.Equalf(t, wantFaces[i].Origins[0].Role, gotFaces[i].Origins[0].Role, "%s: face %d role", label, i)
		requireSameBits(t, fmt.Sprintf("%s face %d area", label, i), wantFaces[i].Area, gotFaces[i].Area)
		requireSameBits(t, fmt.Sprintf("%s face %d area bound", label, i), wantFaces[i].AreaBound, gotFaces[i].AreaBound)
	}
	wantEdges, gotEdges := topobuild.Edges(want), topobuild.Edges(got)
	require.Lenf(t, gotEdges, len(wantEdges), "%s: edge count", label)
	for i := range wantEdges {
		requireSameBits(t, fmt.Sprintf("%s edge %d length", label, i), wantEdges[i].Length, gotEdges[i].Length)
		requireSameBits(t, fmt.Sprintf("%s edge %d length bound", label, i), wantEdges[i].LengthBound, gotEdges[i].LengthBound)
	}
	wantVerts, gotVerts := topobuild.Vertices(want), topobuild.Vertices(got)
	require.Lenf(t, gotVerts, len(wantVerts), "%s: vertex node count", label)
	for i := range wantVerts {
		requireSameVecs(t, fmt.Sprintf("%s vertex node %d", label, i),
			[]r3.Vec{wantVerts[i].Position}, []r3.Vec{gotVerts[i].Position})
		requireSameBits(t, fmt.Sprintf("%s vertex node %d bound", label, i),
			wantVerts[i].Bound.Base(), gotVerts[i].Bound.Base())
	}
}
