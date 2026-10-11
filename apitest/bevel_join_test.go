package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// bevelReferenceSection is the six-line 8/8 reference blank section from
// the gallery bevel-gear producer. It keeps the producer's float operations,
// including the small mismatch between its finite root line and public cone.
func bevelReferenceSection() ([]decad.Point2, float64) {
	angle := math.Atan2(4, 4)
	coneDist := 4 / math.Sin(angle)
	width := math.Min(math.Hypot(8, 8)/6, 0.95*coneDist*0.5)
	baseHeight := 1.0
	heelStation := coneDist*math.Cos(angle) + baseHeight
	heelRadius := 4 - baseHeight/math.Tan(angle)
	dedStation := coneDist*math.Cos(angle) + 1.25*math.Sin(angle)
	dedRadius := 4 - 1.25*math.Cos(angle)
	apexToDed := math.Hypot(coneDist, 1.25)
	rootLength := width * apexToDed / coneDist
	toeFactor := 1 - rootLength/apexToDed
	toeRootStation, toeRootRadius := toeFactor*dedStation, toeFactor*dedRadius
	toeRadius := 4 - width/math.Sin(angle)
	toeSlide := (toeRootRadius - toeRadius) / math.Cos(angle)
	toeStation := toeRootStation + toeSlide*math.Sin(angle)
	return []decad.Point2{
		{U: toeStation}, {U: heelStation},
		{U: heelStation, V: heelRadius},
		{U: dedStation, V: dedRadius},
		{U: toeRootStation, V: toeRootRadius},
		{U: toeStation, V: toeRadius},
	}, toeRootStation + toeRootRadius*math.Tan(angle)
}

func bevelReferenceBlank(t *testing.T, doc *decad.Document) (*decad.Body, float64) {
	t.Helper()
	section, toeApex := bevelReferenceSection()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	points := make([]*sketch.Point, len(section))
	for i, p := range section {
		points[i] = s.CreatePoint(p.U, p.V)
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	blank, err := doc.Revolve(s, profiles[0],
		decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{U: 1}},
		decad.FullRevolution{})
	require.NoError(t, err)
	return blank, toeApex
}

func bevelInvolute(base, radius float64) (float64, float64) {
	alpha := math.Acos(base / radius)
	t := math.Tan(alpha)
	return base * (math.Cos(t) + t*math.Sin(t)),
		base * (math.Sin(t) - t*math.Cos(t))
}

func bevelReferenceTooth(t *testing.T, doc *decad.Document) (*decad.Body, *sketch.Sketch) {
	t.Helper()
	angle := math.Atan2(4, 4)
	coneDist := 4 / math.Sin(angle)
	center := r3.NewVec(coneDist/math.Cos(angle), 0, 0)
	frame, err := r3.NewFrame(center,
		r3.NewVec(math.Sin(angle), -math.Cos(angle), 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	w := sketch.NewWorld()
	plane, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	pitch := 4 / math.Cos(angle)
	base := pitch * math.Cos(20*math.Pi/180)
	tip := pitch + 1
	root := pitch - 1.3625
	virtualTeeth := 8 / math.Cos(angle)
	px, py := bevelInvolute(base, pitch)
	rotation := math.Pi/(2*virtualTeeth) - math.Atan2(-py, px)
	left, right := make([]*sketch.Point, 5), make([]*sketch.Point, 5)
	fixed := func(x, y float64) *sketch.Point {
		p := s.CreatePoint(x, y)
		s.Fix(p)
		return p
	}
	for i := range left {
		radius := base + (tip-base)*float64(i)/float64(len(left)-1)
		x, y := bevelInvolute(base, radius)
		for side := range 2 {
			sy := -y
			rx := x*math.Cos(rotation) - sy*math.Sin(rotation)
			ry := x*math.Sin(rotation) + sy*math.Cos(rotation)
			if side == 1 {
				ry = -ry
			}
			p := fixed(rx*math.Cos(math.Pi)-ry*math.Sin(math.Pi),
				rx*math.Sin(math.Pi)+ry*math.Cos(math.Pi))
			if side == 0 {
				left[i] = p
			} else {
				right[i] = p
			}
		}
	}
	_, err = s.CreateFitSpline(right...)
	require.NoError(t, err)
	origin := fixed(0, 0)
	s.CreateArc(origin, right[len(right)-1], left[len(left)-1])
	_, err = s.CreateFitSpline(left...)
	require.NoError(t, err)
	rightRoot := fixed(right[0].X()*root/base, right[0].Y()*root/base)
	leftRoot := fixed(left[0].X()*root/base, left[0].Y()*root/base)
	s.CreateLine(rightRoot, right[0])
	s.CreateLine(left[0], leftRoot)
	s.CreateArc(origin, rightRoot, leftRoot)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	tooth, err := doc.LoftFromPoint(t.Context(), r3.Vec{}, s, profiles[0])
	require.NoError(t, err)
	return tooth, s
}

func TestBevelOneToothJoinPreservesSourceFacesAndVerifiedSolid(t *testing.T) {
	doc := decad.New()
	blank, toeApex := bevelReferenceBlank(t, doc)
	tooth, source := bevelReferenceTooth(t, doc)
	trimmed, err := decad.Cut(t.Context(), tooth, pointConeTool(t, doc, toeApex))
	require.NoError(t, err)
	trimmed, err = decad.Intersect(t.Context(), trimmed, pointConeTool(t, doc, 8))
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), blank, trimmed)
	require.NoError(t, err)
	require.True(t, joined.IsSolid())
	mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.LessOrEqual(t, mesh.Bound().Base(), 0.1)
	blankVolume, err := blank.Volume()
	require.NoError(t, err)
	joinedVolume, err := joined.Volume()
	require.NoError(t, err)
	require.Greater(t, joinedVolume.Value.Base()-joinedVolume.Bound.Base(),
		blankVolume.Value.Base()+blankVolume.Bound.Base())
	var fits int
	for _, face := range joined.Faces() {
		if _, ok := face.Surface().(decad.NURBSSurface); ok {
			fits++
		}
	}
	require.GreaterOrEqual(t, fits, 2)
	// The same held tooth can no longer use its former certificate after the
	// Sketch revision changes. The generic mesh Union is too coarse here.
	source.CreatePoint(20, 20)
	_, err = decad.Union(t.Context(), blank, trimmed)
	require.Error(t, err)
}

func TestBevelTwoToothJoinReusesOneRootRing(t *testing.T) {
	doc := decad.New()
	blank, toeApex := bevelReferenceBlank(t, doc)
	tooth, _ := bevelReferenceTooth(t, doc)
	trimmed, err := decad.Cut(t.Context(), tooth, pointConeTool(t, doc, toeApex))
	require.NoError(t, err)
	trimmed, err = decad.Intersect(t.Context(), trimmed, pointConeTool(t, doc, 8))
	require.NoError(t, err)
	turn, err := r3.Rotation(r3.NewVec(1, 0, 0), units.Degrees(45))
	require.NoError(t, err)
	second, err := trimmed.PlacedCopy(t.Context(), turn)
	require.NoError(t, err)
	firstJoin, err := decad.Union(t.Context(), blank, trimmed)
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), firstJoin, second)
	require.NoError(t, err)
	mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	firstVolume, err := firstJoin.Volume()
	require.NoError(t, err)
	joinedVolume, err := joined.Volume()
	require.NoError(t, err)
	blankVolume, err := blank.Volume()
	require.NoError(t, err)
	require.Greater(t, joinedVolume.Value.Base(), firstVolume.Value.Base()+1)
	require.Greater(t, joinedVolume.Value.Base()-joinedVolume.Bound.Base(),
		blankVolume.Value.Base()+blankVolume.Bound.Base())
	var fits int
	for _, face := range joined.Faces() {
		if _, ok := face.Surface().(decad.NURBSSurface); ok {
			fits++
		}
	}
	require.GreaterOrEqual(t, fits, 4)
}
