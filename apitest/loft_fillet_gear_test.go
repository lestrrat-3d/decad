package apitest_test

import (
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

type gearPoint struct{ x, y float64 }

func TestLoftFilletRectangleCorner(t *testing.T) {
	w := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 10), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	upper, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	profile := func(plane *sketch.Plane) (*sketch.Sketch, *sketch.Profile) {
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		p := []*sketch.Point{
			s.CreatePoint(0, 0), s.CreatePoint(10, 0),
			s.CreatePoint(10, 10), s.CreatePoint(0, 10),
		}
		for i := range p {
			s.CreateLine(p[i], p[(i+1)%len(p)])
		}
		s.Fix(p[0])
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		require.Len(t, s.Profiles(), 1)
		return s, s.Profiles()[0]
	}
	s0, p0 := profile(w.XY())
	s1, p1 := profile(upper)
	body, err := decad.New().Loft(t.Context(), s0, p0, s1, p1)
	require.NoError(t, err)
	query := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)),
		decad.EndpointAt(r3.NewVec(0, 0, 0))).Exactly(1)
	rounded, err := body.Fillet(t.Context(), query, units.Millimeters(1))
	require.NoError(t, err)
	volume, err := rounded.Volume()
	require.NoError(t, err)
	require.InDelta(t, 1000-10*(1-math.Pi/4), volume.Value.Base(), 0.02)
	filletRoles := 0
	for _, face := range rounded.Faces() {
		for _, origin := range face.Origins() {
			if strings.HasPrefix(origin.Role, "fillet(0,") {
				filletRoles++
			}
		}
	}
	require.Positive(t, filletRoles)
	shift, err := r3.Translation(r3.NewVec(20, 0, 0))
	require.NoError(t, err)
	placedCopy, err := rounded.PlacedCopy(t.Context(), shift)
	require.NoError(t, err)
	copyFilletRoles := 0
	for _, face := range placedCopy.Faces() {
		for _, origin := range face.Origins() {
			if strings.HasPrefix(origin.Role, "fillet(0,") {
				copyFilletRoles++
			}
		}
	}
	require.Equal(t, filletRoles, copyFilletRoles)
}

func gearInvolute(base, radius float64) gearPoint {
	t := math.Tan(math.Acos(base / radius))
	return gearPoint{base * (math.Cos(t) + t*math.Sin(t)), base * (math.Sin(t) - t*math.Cos(t))}
}

func gearRotate(p gearPoint, angle float64) gearPoint {
	s, c := math.Sincos(angle)
	return gearPoint{p.x*c - p.y*s, p.x*s + p.y*c}
}

func gearRootBlend(unit gearPoint, root, radius, sign float64) (gearPoint, gearPoint, gearPoint) {
	leg := math.Sqrt(root*root + 2*root*radius)
	normal := gearPoint{-unit.y, unit.x}
	center := gearPoint{leg*unit.x + sign*radius*normal.x, leg*unit.y + sign*radius*normal.y}
	onRoot := gearPoint{root * center.x / (root + radius), root * center.y / (root + radius)}
	onLine := gearPoint{leg * unit.x, leg * unit.y}
	return center, onRoot, onLine
}

func gearProfile(t *testing.T, world *sketch.World, plane *sketch.Plane, preRounded bool) (*sketch.Sketch, *sketch.Profile, []gearPoint) {
	t.Helper()
	const teeth, samples = 17, 15
	const pressure = 20 * math.Pi / 180
	pitch, root, tip := float64(teeth)/2, (float64(teeth)-2.5)/2, (float64(teeth)+2)/2
	base := pitch * math.Cos(pressure)
	blendRadius := 0.9 * root * (math.Pi/teeth - 2*(math.Tan(pressure)-pressure)) / 2
	atPitch := gearInvolute(base, pitch)
	rotation := math.Pi/(2*teeth) - math.Atan2(-atPitch.y, atPitch.x)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	fixed := func(p gearPoint) *sketch.Point {
		pt := s.CreatePoint(p.x, p.y)
		s.Fix(pt)
		return pt
	}
	var firstRoot, previousRoot *sketch.Point
	var roots []gearPoint
	for tooth := range teeth {
		angle := float64(tooth) * 2 * math.Pi / teeth
		left := make([]*sketch.Point, samples)
		right := make([]*sketch.Point, samples)
		var leftStart, rightStart gearPoint
		for i := range samples {
			radius := base + (tip-base)*float64(i)/float64(samples-1)
			p := gearInvolute(base, radius)
			p.y = -p.y
			p = gearRotate(p, rotation)
			if i == 0 {
				leftStart = p
			}
			left[i] = fixed(gearRotate(p, angle))
			p.y = -p.y
			if i == 0 {
				rightStart = p
			}
			right[i] = fixed(gearRotate(p, angle))
		}
		leftXY := gearRotate(gearPoint{leftStart.x * root / base, leftStart.y * root / base}, angle)
		rightXY := gearRotate(gearPoint{rightStart.x * root / base, rightStart.y * root / base}, angle)
		var leftCenter, rightCenter, leftLineXY, rightLineXY gearPoint
		if preRounded {
			rightCenter, rightXY, rightLineXY = gearRootBlend(
				gearRotate(gearPoint{1, 0}, angle-rotation), root, blendRadius, -1)
			leftCenter, leftXY, leftLineXY = gearRootBlend(
				gearRotate(gearPoint{1, 0}, angle+rotation), root, blendRadius, +1)
		}
		leftRoot, rightRoot := fixed(leftXY), fixed(rightXY)
		roots = append(roots, leftXY, rightXY)
		if previousRoot != nil {
			s.CreateArc(center, previousRoot, rightRoot)
		} else {
			firstRoot = rightRoot
		}
		if preRounded {
			rightLine := fixed(rightLineXY)
			s.CreateArc(fixed(rightCenter), rightLine, rightRoot)
			s.CreateLine(rightLine, right[0])
		} else {
			s.CreateLine(rightRoot, right[0])
		}
		_, err = s.CreateFitSpline(right...)
		require.NoError(t, err)
		s.CreateArc(center, right[samples-1], left[samples-1])
		_, err = s.CreateFitSpline(left...)
		require.NoError(t, err)
		if preRounded {
			leftLine := fixed(leftLineXY)
			s.CreateLine(left[0], leftLine)
			s.CreateArc(fixed(leftCenter), leftRoot, leftLine)
		} else {
			s.CreateLine(left[0], leftRoot)
		}
		previousRoot = leftRoot
	}
	s.CreateArc(center, previousRoot, firstRoot)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	require.True(t, s.Profiles()[0].Valid)
	return s, s.Profiles()[0], roots
}

func TestLoftFilletDefaultGearRoots(t *testing.T) {
	const teeth = 17
	const root = (teeth - 2.5) / 2.0
	const pressure = 20 * math.Pi / 180
	radius := 0.9 * root * (math.Pi/teeth - 2*(math.Tan(pressure)-pressure)) / 2
	world := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 10), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	upper, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s0, p0, rootPoints := gearProfile(t, world, world.XY(), false)
	s1, p1, _ := gearProfile(t, world, upper, false)
	body, err := decad.New().Loft(t.Context(), s0, p0, s1, p1)
	require.NoError(t, err)
	before, err := body.Volume()
	require.NoError(t, err)

	axial, err := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))).SelectEdges(body)
	require.NoError(t, err)
	var roots []r3.Vec
	for _, edge := range axial {
		point := edge.Start().Position().Value
		for _, expected := range rootPoints {
			if math.Hypot(point.X-expected.x, point.Y-expected.y) < 1e-8 {
				roots = append(roots, point)
				break
			}
		}
	}
	require.Equal(t, 2*teeth, len(roots))
	_, err = body.Fillet(t.Context(), decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)),
		decad.EndpointAt(roots[0])).Exactly(1), units.Millimeters(100))
	require.Error(t, err)
	var splineStation r3.Vec
	for _, edge := range axial {
		point := edge.Start().Position().Value
		r := math.Hypot(point.X, point.Y)
		if r > 8.1 && r < 9.3 {
			splineStation = point
			break
		}
	}
	require.NotEqual(t, r3.Vec{}, splineStation)
	_, err = body.Fillet(t.Context(), decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)),
		decad.EndpointAt(splineStation)).Exactly(1), units.Millimeters(radius))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	query := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.EndpointAt(roots[0]))
	for _, point := range roots[1:] {
		query.Or(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.EndpointAt(point))
	}
	rounded, err := body.Fillet(t.Context(), query.Exactly(2*teeth), units.Millimeters(radius))
	require.NoError(t, err)
	require.True(t, rounded.IsSolid())
	after, err := rounded.Volume()
	require.NoError(t, err)
	require.Greater(t, after.Value.Base(), before.Value.Base())
	referenceWorld := sketch.NewWorld()
	referenceUpper, err := referenceWorld.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	referenceSketch0, referenceProfile0, _ := gearProfile(t, referenceWorld, referenceWorld.XY(), true)
	referenceSketch1, referenceProfile1, _ := gearProfile(t, referenceWorld, referenceUpper, true)
	reference, err := decad.New().Loft(t.Context(), referenceSketch0, referenceProfile0,
		referenceSketch1, referenceProfile1)
	require.NoError(t, err)
	referenceVolume, err := reference.Volume()
	require.NoError(t, err)
	require.InDelta(t, referenceVolume.Value.Base(), after.Value.Base(), 0.1)
	require.InDelta(t, referenceVolume.Value.Base(), after.Value.Base(),
		referenceVolume.Bound.Base()+after.Bound.Base()+0.02)
	roles := map[string]struct{}{}
	for _, face := range rounded.Faces() {
		for _, origin := range face.Origins() {
			if strings.HasPrefix(origin.Role, "fillet(0,") {
				roles[origin.Role] = struct{}{}
			}
		}
	}
	require.Equal(t, 2*teeth, len(roles))
	remaining, err := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))).SelectEdges(rounded)
	require.NoError(t, err)
	for _, edge := range remaining {
		point := edge.Start().Position().Value
		for _, rootPoint := range roots {
			require.NotEqual(t, rootPoint, point)
		}
	}
}
