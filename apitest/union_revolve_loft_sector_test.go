package apitest_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

const splitCapFirstRole = "side(0,1)"

// TestUnionTaperedHubAndRadialLug uses independent real Sketch producers for
// the Revolve section and the trimmed point-Loft sector. The extra line on
// the hub cap is exactly collinear and must keep both source origins.
func TestUnionTaperedHubAndRadialLug(t *testing.T) {
	doc, hub, trimmed, lugSketch, splitCap := taperedHubAndRadialLug(t, 9, 7.5, 4.3, false)
	_ = lugSketch
	hubVolume, err := hub.Volume()
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), hub, trimmed)
	require.NoError(t, err)
	mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	volume, err := joined.Volume()
	require.NoError(t, err)
	require.Greater(t, volume.Value.Base()-volume.Bound.Base(),
		hubVolume.Value.Base()+hubVolume.Bound.Base())
	require.Equal(t, 2, revolveLugEuler(mesh))
	requireRevolveLugSourceFaces(t, joined, splitCap)
	require.Contains(t, doc.Bodies(), joined)
}

func TestUnionTaperedHubAndFittedRadialLug(t *testing.T) {
	_, hub, trimmed, _, splitCap := taperedHubAndRadialLug(t, 9, 7.5, 4.3, true)
	hubVolume, err := hub.Volume()
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), hub, trimmed)
	require.NoError(t, err)
	mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	volume, err := joined.Volume()
	require.NoError(t, err)
	require.Greater(t, volume.Value.Base()-volume.Bound.Base(),
		hubVolume.Value.Base()+hubVolume.Bound.Base())
	require.Equal(t, 2, revolveLugEuler(mesh))
	requireRevolveLugSourceFaces(t, joined, splitCap)
}

// The outer shoulder adds a non-collinear meridian face away from the root.
// Public Union must preserve that face while joining the same real lug.
func TestUnionShoulderedHubAndRadialLug(t *testing.T) {
	_, hub, trimmed := shoulderedHubAndRadialLug(t)
	hubVolume, err := hub.Volume()
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), hub, trimmed)
	require.NoError(t, err)
	mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	volume, err := joined.Volume()
	require.NoError(t, err)
	require.Greater(t, volume.Value.Base()-volume.Bound.Base(),
		hubVolume.Value.Base()+hubVolume.Bound.Base())
	require.Equal(t, 2, revolveLugEuler(mesh))
}

// A sloped cap adjacent to the root remains an authenticated meridian span.
func TestUnionSlopedHubCapAndRadialLug(t *testing.T) {
	_, hub, trimmed, _, splitCap := taperedHubAndRadialLug(t, 8.99, 7.5, 4.3, false)
	hubVolume, err := hub.Volume()
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), hub, trimmed)
	require.NoError(t, err)
	mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Equal(t, 2, revolveLugEuler(mesh))
	volume, err := joined.Volume()
	require.NoError(t, err)
	require.Greater(t, volume.Value.Base()-volume.Bound.Base(),
		hubVolume.Value.Base()+hubVolume.Bound.Base())
	requireRevolveLugSourceFaces(t, joined, splitCap)
}

func TestUnionShoulderedHubAndTwoRadialLugs(t *testing.T) {
	_, hub, trimmed := shoulderedHubAndRadialLug(t)
	hubVolume, err := hub.Volume()
	require.NoError(t, err)
	turn, err := r3.Rotation(r3.Vec{X: 1}, units.Degrees(180))
	require.NoError(t, err)
	second, err := decad.Copy(t.Context(), trimmed, decad.WithCopyPlacement(turn))
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), hub, trimmed)
	require.NoError(t, err)
	joined, err = decad.Union(t.Context(), joined, second)
	require.NoError(t, err)
	mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Equal(t, 2, revolveLugEuler(mesh))
	volume, err := joined.Volume()
	require.NoError(t, err)
	require.Greater(t, volume.Value.Base()-volume.Bound.Base(),
		hubVolume.Value.Base()+hubVolume.Bound.Base())
}

// A six-span hub also has a previously admitted serial construction route.
// The public build keeps that route available while the wider sector class
// uses the generic constructor.
func TestUnionSixSpanHubSerialRadialLugs(t *testing.T) {
	doc, sourceHub, trimmed, _, sourceFaces := taperedHubAndRadialLug(t, 9, 7.5, 4.3, false)
	require.NotNil(t, sourceHub)
	require.Len(t, sourceFaces, 2)
	world := sketch.NewWorld()
	section, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	outline := []decad.Point2{{U: 6}, {U: 9}, {U: 9, V: 3},
		{U: 8, V: 4}, {U: 5, V: 2.5}, {U: 6, V: 1.5}}
	points := make([]*sketch.Point, len(outline))
	for i, point := range outline {
		points[i] = section.CreatePoint(point.U, point.V)
		section.Fix(points[i])
	}
	for i := range points {
		section.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = section.Solve(t.Context())
	require.NoError(t, err)
	profiles := section.Profiles()
	require.Len(t, profiles, 1)
	hub, err := decad.Revolve(t.Context(), doc, section, profiles[0],
		decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{U: 1}}, decad.FullRevolution{})
	require.NoError(t, err)
	hubVolume, err := hub.Volume()
	require.NoError(t, err)
	placed := make([]*decad.Body, 0, 2)
	for _, degrees := range []float64{120, 240} {
		turn, turnErr := r3.Rotation(r3.Vec{X: 1}, units.Degrees(degrees))
		require.NoError(t, turnErr)
		placedCopy, copyErr := decad.Copy(t.Context(), trimmed, decad.WithCopyPlacement(turn))
		require.NoError(t, copyErr)
		placed = append(placed, placedCopy)
	}
	joined, err := decad.Union(t.Context(), hub, trimmed)
	require.NoError(t, err)
	for _, placedCopy := range placed {
		joined, err = decad.Union(t.Context(), joined, placedCopy)
		require.NoError(t, err)
	}
	mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Equal(t, 2, revolveLugEuler(mesh))
	volume, err := joined.Volume()
	require.NoError(t, err)
	require.Greater(t, volume.Value.Base()-volume.Bound.Base(),
		hubVolume.Value.Base()+hubVolume.Bound.Base())
}

func TestUnionShoulderedHubAndTwoRadialLugsVariadic(t *testing.T) {
	_, hub, trimmed := shoulderedHubAndRadialLug(t)
	turn, err := r3.Rotation(r3.Vec{X: 1}, units.Degrees(180))
	require.NoError(t, err)
	second, err := decad.Copy(t.Context(), trimmed, decad.WithCopyPlacement(turn))
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), hub, trimmed, second)
	require.NoError(t, err)
	mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Equal(t, 2, revolveLugEuler(mesh))
}

func TestUnionShoulderedHubAndThreeRadialLugsVariadic(t *testing.T) {
	_, hub, trimmed := shoulderedHubAndRadialLug(t)
	hubVolume, err := hub.Volume()
	require.NoError(t, err)
	placed := make([]*decad.Body, 2)
	for i, degrees := range []float64{120, 240} {
		turn, err := r3.Rotation(r3.Vec{X: 1}, units.Degrees(degrees))
		require.NoError(t, err)
		placed[i], err = decad.Copy(t.Context(), trimmed, decad.WithCopyPlacement(turn))
		require.NoError(t, err)
	}
	joined, err := decad.Union(t.Context(), hub, trimmed, placed[0], placed[1])
	require.NoError(t, err)
	mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Equal(t, 2, revolveLugEuler(mesh))
	volume, err := joined.Volume()
	require.NoError(t, err)
	require.Greater(t, volume.Value.Base()-volume.Bound.Base(),
		hubVolume.Value.Base()+hubVolume.Bound.Base())
}

func TestUnionShoulderedHubWithNearAxisCopy(t *testing.T) {
	for _, serial := range []bool{false, true} {
		name := "variadic"
		if serial {
			name = "serial"
		}
		t.Run(name, func(t *testing.T) {
			_, hub, trimmed := shoulderedHubAndRadialLug(t)
			hubVolume, err := hub.Volume()
			require.NoError(t, err)
			turn, err := r3.Rotation(r3.NewVec(1, 0, 0), units.Radians(2*math.Pi*5/16))
			require.NoError(t, err)
			copyOfLug, err := decad.Copy(t.Context(), trimmed, decad.WithCopyPlacement(turn))
			require.NoError(t, err)
			var joined *decad.Body
			if serial {
				joined, err = decad.Union(t.Context(), hub, trimmed)
				require.NoError(t, err)
				joined, err = decad.Union(t.Context(), joined, copyOfLug)
			} else {
				joined, err = decad.Union(t.Context(), hub, trimmed, copyOfLug)
			}
			require.NoError(t, err)
			mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.1),
				decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
			require.Equal(t, 2, revolveLugEuler(mesh))
			volume, err := joined.Volume()
			require.NoError(t, err)
			require.Greater(t, volume.Value.Base()-volume.Bound.Base(),
				hubVolume.Value.Base()+hubVolume.Bound.Base())
		})
	}
}

func TestUnionNearAxisCopyOverlapLeavesInputsLive(t *testing.T) {
	doc, hub, trimmed := shoulderedHubAndRadialLug(t)
	farTurn, err := r3.Rotation(r3.NewVec(1, 0, 0), units.Radians(2*math.Pi*5/16))
	require.NoError(t, err)
	farCopy, err := decad.Copy(t.Context(), trimmed, decad.WithCopyPlacement(farTurn))
	require.NoError(t, err)
	nearTurn, err := r3.Rotation(r3.NewVec(1, 0, 0), units.Degrees(5))
	require.NoError(t, err)
	nearCopy, err := decad.Copy(t.Context(), trimmed, decad.WithCopyPlacement(nearTurn))
	require.NoError(t, err)
	before := doc.Bodies()
	_, err = decad.Union(t.Context(), hub, trimmed, farCopy, nearCopy)
	require.ErrorContains(t, err, "complete sector sources are not separate")
	require.Equal(t, before, doc.Bodies())
}

func TestUnionShoulderedHubOverlappingLugsLeaveInputsLive(t *testing.T) {
	doc, hub, trimmed := shoulderedHubAndRadialLug(t)
	turn, err := r3.Rotation(r3.Vec{X: 1}, units.Degrees(5))
	require.NoError(t, err)
	second, err := decad.Copy(t.Context(), trimmed, decad.WithCopyPlacement(turn))
	require.NoError(t, err)
	before := doc.Bodies()
	_, err = decad.Union(t.Context(), hub, trimmed, second)
	require.Error(t, err)
	require.Equal(t, before, doc.Bodies())
}

func shoulderedHubAndRadialLug(t *testing.T) (*decad.Document, *decad.Body, *decad.Body) {
	t.Helper()
	doc, priorHub, trimmed, sourceSketch, sourceRoles := taperedHubAndRadialLug(t, 9, 7.5, 4.3, false)
	require.NotNil(t, priorHub)
	require.NotNil(t, sourceSketch)
	require.Len(t, sourceRoles, 2)
	world := sketch.NewWorld()
	section, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	outline := []decad.Point2{{U: 6}, {U: 9}, {U: 9.2, V: 1.5},
		{U: 9, V: 3}, {U: 8, V: 4}, {U: 5, V: 2.5}, {U: 6, V: 1.5}}
	points := make([]*sketch.Point, len(outline))
	for i, point := range outline {
		points[i] = section.CreatePoint(point.U, point.V)
		section.Fix(points[i])
	}
	for i := range points {
		section.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = section.Solve(t.Context())
	require.NoError(t, err)
	profiles := section.Profiles()
	require.Len(t, profiles, 1)
	hub, err := decad.Revolve(t.Context(), doc, section, profiles[0],
		decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{U: 1}}, decad.FullRevolution{})
	require.NoError(t, err)
	require.Greater(t, len(hub.Faces()), 5, "the shoulder must add an effective meridian side")
	return doc, hub, trimmed
}

func requireRevolveLugSourceFaces(t *testing.T, joined *decad.Body, splitCap []decad.FeatureRef) {
	t.Helper()
	var splitCapFace, rootConeFace bool
	for _, face := range joined.Faces() {
		origins := face.Origins()
		if _, ok := face.Surface().(decad.Cone); ok {
			rootConeFace = true
		}
		seen := 0
		for _, origin := range origins {
			for _, source := range splitCap {
				if origin == source {
					seen++
				}
			}
		}
		if seen == len(splitCap) {
			splitCapFace = true
		}
	}
	require.True(t, splitCapFace, "joined cap must retain both exact source lines")
	require.True(t, rootConeFace, "joined root must retain its analytic cone")
}

func taperedHubAndRadialLug(t *testing.T, splitU, toeApex, rootRadius float64, fitted bool) (
	*decad.Document, *decad.Body, *decad.Body, *sketch.Sketch, []decad.FeatureRef,
) {
	t.Helper()
	doc := decad.New()
	world := sketch.NewWorld()
	section, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	outline := []decad.Point2{{U: 6}, {U: 9}, {U: splitU, V: 1.5},
		{U: 9, V: 3}, {U: 8, V: 4}, {U: 5, V: 2.5}, {U: 6, V: 1.5}}
	points := make([]*sketch.Point, len(outline))
	for i, point := range outline {
		points[i] = section.CreatePoint(point.U, point.V)
		section.Fix(points[i])
	}
	for i := range points {
		section.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = section.Solve(t.Context())
	require.NoError(t, err)
	profiles := section.Profiles()
	require.Len(t, profiles, 1)
	hub, err := decad.Revolve(t.Context(), doc, section, profiles[0],
		decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{U: 1}}, decad.FullRevolution{})
	require.NoError(t, err)
	var splitCap []decad.FeatureRef
	for _, face := range hub.Faces() {
		origins := face.Origins()
		if len(origins) == 2 && origins[0].Role == splitCapFirstRole && origins[1].Role == "side(0,2)" {
			splitCap = origins
		}
	}
	if splitU == 9 {
		require.Len(t, splitCap, 2)
	}

	lugWorld := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.NewVec(10, 0, 0),
		r3.NewVec(0, -1, 0), r3.NewVec(0, 0, -1))
	require.NoError(t, err)
	plane, err := lugWorld.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	lugSketch, err := lugWorld.CreateSketch(plane)
	require.NoError(t, err)
	rootLow := lugSketch.CreatePoint(-rootRadius, -0.4)
	flankLow := lugSketch.CreatePoint(-5.5, -0.4)
	tipLow := lugSketch.CreatePoint(-6, -0.4)
	tipHigh := lugSketch.CreatePoint(-6, 0.4)
	flankHigh := lugSketch.CreatePoint(-5.5, 0.4)
	rootHigh := lugSketch.CreatePoint(-rootRadius, 0.4)
	center := lugSketch.CreatePoint(0, 0)
	for _, point := range []*sketch.Point{rootLow, flankLow, tipLow,
		tipHigh, flankHigh, rootHigh, center} {
		lugSketch.Fix(point)
	}
	if fitted {
		midLow := lugSketch.CreatePoint(-(rootRadius+5.5)/2, -0.48)
		lugSketch.Fix(midLow)
		_, err = lugSketch.CreateFitSpline(rootLow, midLow, flankLow)
		require.NoError(t, err)
	} else {
		lugSketch.CreateLine(rootLow, flankLow)
	}
	lugSketch.CreateLine(flankLow, tipLow)
	lugSketch.CreateArc(center, tipHigh, tipLow)
	lugSketch.CreateLine(tipHigh, flankHigh)
	if fitted {
		midHigh := lugSketch.CreatePoint(-(rootRadius+5.5)/2, 0.48)
		lugSketch.Fix(midHigh)
		_, err = lugSketch.CreateFitSpline(flankHigh, midHigh, rootHigh)
		require.NoError(t, err)
	} else {
		lugSketch.CreateLine(flankHigh, rootHigh)
	}
	lugSketch.CreateArc(center, rootHigh, rootLow)
	_, err = lugSketch.Solve(t.Context())
	require.NoError(t, err)
	lugProfiles := lugSketch.Profiles()
	require.Len(t, lugProfiles, 1)
	lug, err := decad.LoftFromPoint(t.Context(), doc, r3.Vec{}, lugSketch, lugProfiles[0])
	require.NoError(t, err)
	toeTrim, err := decad.Cut(t.Context(), lug, radialLugConeTool(t, doc, toeApex))
	require.NoError(t, err)
	trimmed, err := decad.Intersect(t.Context(), toeTrim, radialLugConeTool(t, doc, 12))
	require.NoError(t, err)
	return doc, hub, trimmed, lugSketch, splitCap
}

// radialLugConeTool is a real Sketch and Revolve producer kept with this
// neutral Union test so the older point-cone test fixture can be removed.
func radialLugConeTool(t *testing.T, doc *decad.Document, apex float64) *decad.Body {
	t.Helper()
	world := sketch.NewWorld()
	section, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	points := []*sketch.Point{
		section.CreatePoint(-2, 0), section.CreatePoint(-2, apex+2),
		section.CreatePoint(apex, 0),
	}
	for _, point := range points {
		section.Fix(point)
	}
	for i := range points {
		section.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = section.Solve(t.Context())
	require.NoError(t, err)
	profiles := section.Profiles()
	require.Len(t, profiles, 1)
	tool, err := decad.Revolve(t.Context(), doc, section, profiles[0],
		decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{U: 1}}, decad.FullRevolution{})
	require.NoError(t, err)
	return tool
}

func TestUnionTaperedHubRadialLugRefusals(t *testing.T) {
	for _, test := range []struct {
		name       string
		splitU     float64
		toeApex    float64
		rootRadius float64
		fitted     bool
		stale      bool
		cancel     bool
	}{
		{name: "foreign toe cone", splitU: 9, toeApex: 7.6, rootRadius: 4.3},
		{name: "near root contact", splitU: 9, toeApex: 7.5, rootRadius: 4.98397},
		{name: "fitted near root contact", splitU: 9, toeApex: 7.5,
			rootRadius: 4.98397, fitted: true},
		{name: "stale source", splitU: 9, toeApex: 7.5, rootRadius: 4.3, stale: true},
		{name: "fitted stale source", splitU: 9, toeApex: 7.5,
			rootRadius: 4.3, fitted: true, stale: true},
		{name: "canceled", splitU: 9, toeApex: 7.5, rootRadius: 4.3, cancel: true},
	} {
		t.Run(test.name, func(t *testing.T) {
			doc, hub, trimmed, source, _ := taperedHubAndRadialLug(t,
				test.splitU, test.toeApex, test.rootRadius, test.fitted)
			if test.stale {
				source.CreatePoint(0, 0)
			}
			ctx := t.Context()
			if test.cancel {
				var stop context.CancelFunc
				ctx, stop = context.WithCancel(ctx)
				stop()
			}
			before := doc.Bodies()
			_, err := decad.Union(ctx, hub, trimmed)
			require.Error(t, err)
			require.Equal(t, before, doc.Bodies())
		})
	}
}

func revolveLugEuler(mesh *decad.Mesh) int {
	edges := make(map[[2]int]struct{})
	for _, triangle := range mesh.Triangles() {
		for i := range 3 {
			a, b := triangle[i], triangle[(i+1)%3]
			if a > b {
				a, b = b, a
			}
			edges[[2]int{a, b}] = struct{}{}
		}
	}
	return len(mesh.Vertices()) - len(edges) + len(mesh.Triangles())
}
