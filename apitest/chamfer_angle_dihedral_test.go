package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/chamfer-angle-design.md §5's and
// agent-docs/fillet-chord-design.md §5's public-surface tests for edges that are
// not right angled: the 120 degree edges of a hexagonal prism and the 45 degree
// edge of a right-isosceles triangular prism, 10 mm tall.

const hexRadius = 10.0

// hexPrism is a hexagonal prism of circumradius 10 about the Z axis, 10 mm tall.
func hexPrism(t *testing.T) (*decad.Document, *decad.Body) {
	t.Helper()
	pts := make([][2]float64, 6)
	for i := range pts {
		a := math.Pi / 3 * float64(i)
		pts[i] = [2]float64{hexRadius * math.Cos(a), hexRadius * math.Sin(a)}
	}
	return polygonPrism(t, pts)
}

// hexEdge is the vertical edge of the hexagonal prism at (10, 0), between the faces
// whose outward normals are at ±30 degrees about the +X direction.
func hexEdge() *decad.EdgeQuery {
	return decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.EndpointAt(r3.NewVec(hexRadius, 0, 0))).Exactly(1)
}

// hexReference is the face of the prism under the edge at (10, 0) whose outward
// normal points down and to the right of +X.
func hexReference() decad.FaceSelector {
	return decad.Faces(decad.Facing(r3.NewVec(math.Cos(math.Pi/6), -math.Sin(math.Pi/6), 0)))
}

// hexArea is the area of the hexagon, 3√3/2 of the squared circumradius.
const hexArea = 3 * 1.7320508075688772 / 2 * hexRadius * hexRadius

func TestChamferAngleOnAHexagonEdge(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name  string
		angle float64
	}{{"30 degrees", 30}, {"45 degrees", 45}, {"50 degrees", 50}} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc, prism := hexPrism(t)
			const d = 2.0
			out, err := decad.ChamferAngle(t.Context(), prism, hexEdge(), units.Millimeters(d), hexReference(), units.Degrees(tc.angle))
			require.NoError(t, err)
			require.Equal(t, []*decad.Body{out}, doc.Bodies())
			// The faces meet at 120 degrees, so the bevel meets the other face
			// d·sin(a)/sin(120 + a) from the edge. It cuts a triangle with legs d and that
			// along the two faces and 120 degrees between them: 10 mm tall.
			alpha := tc.angle * math.Pi / 180
			phi := 2 * math.Pi / 3
			other := d * math.Sin(alpha) / math.Sin(phi+alpha)
			removed := 0.5 * d * other * math.Sin(phi) * 10
			decadtest.MeasuresVolume(t, out, units.CubicMillimeters(hexArea*10-removed), decadtest.WithinRel(units.Scalar(1e-9)))
			// The triangle the bevel cuts has the stated angle at the reference face,
			// found from the legs the volume implies, independently of the formula.
			impliedOther := 2 * removed / 10 / (d * math.Sin(phi))
			implied := math.Atan2(impliedOther*math.Sin(phi), d-impliedOther*math.Cos(phi))
			require.InDelta(t, alpha, implied, 1e-9)
		})
	}
}

func TestChamferAngleOnAnAcuteEdge(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	prism := rightTrianglePrism(t, doc)
	edge := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.EndpointAt(r3.NewVec(20, 0, 0))).Exactly(1)
	const d = 2.0
	out, err := decad.ChamferAngle(t.Context(), prism, edge, units.Millimeters(d),
		decad.Faces(decad.Facing(r3.NewVec(0, -1, 0))), units.Degrees(30))
	require.NoError(t, err)
	// The faces meet at 45 degrees at (20, 0): the bottom face runs along -x from the
	// edge and the hypotenuse up and to the left. The bevel is 30 degrees to the bottom.
	phi := math.Pi / 4
	other := d * math.Sin(math.Pi/6) / math.Sin(phi+math.Pi/6)
	removed := 0.5 * d * other * math.Sin(phi) * 10
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(200*10-removed), decadtest.WithinRel(units.Scalar(1e-9)))
}

func TestChamferAngleRefusesWhatHasNoSecondDistance(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	prism := rightTrianglePrism(t, doc)
	edge := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.EndpointAt(r3.NewVec(20, 0, 0))).Exactly(1)
	// At 45 degrees a bevel 140 degrees from the face never meets the other face.
	_, err := decad.ChamferAngle(t.Context(), prism, edge, units.Millimeters(2),
		decad.Faces(decad.Facing(r3.NewVec(0, -1, 0))), units.Degrees(140))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	// All three vertical edges meet at 45, 45 and 90 degrees, so they take different distances.
	all := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)))
	_, err = decad.ChamferAngle(t.Context(), prism, all, units.Millimeters(2),
		decad.Faces(decad.Facing(r3.NewVec(0, -1, 0))), units.Degrees(30))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, []*decad.Body{prism}, doc.Bodies(), "a refused chamfer leaves the prism live and alone")
}

func TestChamferAngleBevelsAConcaveEdge(t *testing.T) {
	t.Parallel()
	// A notched shape whose reflex vertex (10, 5) is a concave edge, and whose walls all face
	// different ways, so a reference face is one face. Its area is 20·20 less the notch's 100.
	pts := [][2]float64{{0, 0}, {20, 0}, {20, 20}, {10, 5}, {0, 20}}
	area := 0.0
	for i := range pts {
		j := (i + 1) % len(pts)
		area += pts[i][0]*pts[j][1] - pts[j][0]*pts[i][1]
	}
	area = math.Abs(area) / 2
	// The notch's two walls face (-15, 10) and (15, 10), so they turn by acos(-125/325) and the
	// open wedge between them is the rest of a half turn.
	phi := math.Pi - math.Acos(-125.0/325.0)
	ref := decad.Faces(decad.Facing(r3.NewVec(-15, 10, 0)))
	for _, deg := range []float64{30, 60} {
		doc, prism := polygonPrism(t, pts)
		edge := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.EndpointAt(r3.NewVec(10, 5, 0))).Exactly(1)
		out, err := decad.ChamferAngle(t.Context(), prism, edge, units.Millimeters(2), ref, units.Degrees(deg))
		require.NoError(t, err)
		require.Equal(t, []*decad.Body{out}, doc.Bodies())
		theta := deg * math.Pi / 180
		other := 2 * math.Sin(theta) / math.Sin(phi+theta)
		// The bevel fills a triangle with sides 2 and other about the open wedge's angle.
		decadtest.MeasuresVolume(t, out, units.CubicMillimeters((area+0.5*2*other*math.Sin(phi))*10),
			decadtest.WithinRel(units.Scalar(1e-9)))
	}
}

func TestFilletChordOnAHexagonEdge(t *testing.T) {
	t.Parallel()
	doc, prism := hexPrism(t)
	const chord = 2.0
	out, err := decad.FilletChord(t.Context(), prism, hexEdge(), units.Millimeters(chord))
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies())
	// The faces meet at 120 degrees, so the arc turns 60 and its radius is
	// chord / (2 cos 60) = chord. The corner a fillet of radius R cuts off has
	// area R²(cot(φ/2) − (π−φ)/2).
	radius := chord / (2 * math.Cos(math.Pi/3))
	removed := radius * radius * (1/math.Tan(math.Pi/3) - (math.Pi-2*math.Pi/3)/2) * 10
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(hexArea*10-removed), decadtest.WithinRel(units.Scalar(1e-6)))
}

func TestFilletChordOnEverySideEdgeOfTheHexagon(t *testing.T) {
	t.Parallel()
	_, prism := hexPrism(t)
	// The six vertical edges all meet at 120 degrees, so they give one radius.
	all := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)))
	const chord = 2.0
	out, err := decad.FilletChord(t.Context(), prism, all, units.Millimeters(chord))
	require.NoError(t, err)
	removed := chord * chord * (1/math.Tan(math.Pi/3) - (math.Pi-2*math.Pi/3)/2) * 10
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(hexArea*10-6*removed), decadtest.WithinRel(units.Scalar(1e-6)))
}

func TestFilletChordRoundsAConcaveEdge(t *testing.T) {
	t.Parallel()
	doc, lshape := polygonPrism(t, [][2]float64{{0, 0}, {20, 0}, {20, 10}, {10, 10}, {10, 20}, {0, 20}})
	reflex := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.EndpointAt(r3.NewVec(10, 10, 0))).Exactly(1)
	// A right-angled corner's arc turns 90 degrees, so a chord of 2√2 is a radius of 2. The
	// round fills (1 − π/4)·r² of the open corner.
	out, err := decad.FilletChord(t.Context(), lshape, reflex, units.Millimeters(2*math.Sqrt2))
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies())
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters((300+(1-math.Pi/4)*4)*10),
		decadtest.WithinRel(units.Scalar(1e-9)))
}

func TestFilletChordRefusesMixedAngles(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	prism := rightTrianglePrism(t, doc)
	all := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)))
	_, err := decad.FilletChord(t.Context(), prism, all, units.Millimeters(2))
	require.ErrorIs(t, err, decad.ErrUnsupported, "edges at 45, 45 and 90 degrees need different radii")
	require.Equal(t, []*decad.Body{prism}, doc.Bodies())
}

// rightTrianglePrism is the 20 x 20 right-isosceles triangle with its right angle at
// the origin, 10 mm tall.
func rightTrianglePrism(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	s := decadtest.NewSketch(t)
	a, b, c := s.CreatePoint(0, 0), s.CreatePoint(20, 0), s.CreatePoint(0, 20)
	s.Fix(a)
	s.Fix(b)
	s.Fix(c)
	s.CreateLine(a, b)
	s.CreateLine(b, c)
	s.CreateLine(c, a)
	return decadtest.NewPrism(t, doc, s, decadtest.SolveRegion(t, s), units.Millimeters(10))
}
