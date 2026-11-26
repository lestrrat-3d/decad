package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// splitSlabCaps splits one point-loft tooth twice and returns the second
// slab's start cap and the first slab's end cap, two caps of different slabs.
func splitSlabCaps(t *testing.T, doc *decad.Document) (*decad.Face, *decad.Face) {
	t.Helper()
	kept := splitSlabs(t, doc)
	return pointSlabCap(t, kept[1], decad.CapStart(kept[1])), pointSlabCap(t, kept[0], decad.CapEnd(kept[0]))
}

// slabLoftSections returns the sections a LoftFaces through both slabs of
// splitSlabs takes: the least slab's toe cap, then each heel cap in source
// order.
func slabLoftSections(t *testing.T, kept []*decad.Body) []*decad.Face {
	t.Helper()
	return []*decad.Face{
		pointSlabCap(t, kept[1], decad.CapStart(kept[1])),
		pointSlabCap(t, kept[1], decad.CapEnd(kept[1])),
		pointSlabCap(t, kept[0], decad.CapEnd(kept[0])),
	}
}

// splitSlabs splits one point-loft tooth twice and returns the two slabs in
// split order, so a loft through them holds the sum of their volumes.
func splitSlabs(t *testing.T, doc *decad.Document) []*decad.Body {
	t.Helper()
	inner := pointLoftRadialTab(t, doc)
	span := pointLoftSliceSpan()
	var kept []*decad.Body
	for _, k := range []float64{1, 2} {
		sheet := pointLoftSliceSheet(t, doc, span*k/6, 30)
		parts, err := decad.Split(inner, sheet).Do(t.Context())
		require.NoError(t, err)
		require.Len(t, parts, 2)
		inner = parts[0]
		kept = append(kept, parts[1])
	}
	return kept
}

// coneTangentLoft is a slab of a thin pointed solid and a cone whose tangent
// plane at the apex is the plane the slab's source lies in, the inputs
// LoftFacesToCone needs: the loft runs from the toe cap to the heel cap and
// ends tangent to the cone along its +Y ruling.
type coneTangentLoft struct {
	toe, heel *decad.Face
	cone      *decad.Body
	radial    r3.Vec
}

// newConeTangentLoft builds the fixture in doc: a pyramid of apex (0, 0, 0)
// cut twice at x = 5 and x = 4.5, and the cone of the triangle (-4, 0),
// (-4, 16), (8, 0) turned about world X.
func newConeTangentLoft(t *testing.T, doc *decad.Document) coneTangentLoft {
	t.Helper()
	w := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.NewVec(8, 0, 0), r3.NewVec(0.6, -0.8, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	plane, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(-3.025, -0.025, -2.975, 0.025)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	spike, err := decad.LoftFromPoint(doc, r3.Vec{}, s, s.Profiles()[0]).Do(t.Context())
	require.NoError(t, err)
	sheet := func(at float64) *decad.Body {
		ss, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		a, b := ss.CreatePoint(at, -20), ss.CreatePoint(at, 20)
		ss.Fix(a)
		ss.Fix(b)
		ss.CreateLine(a, b)
		_, err = ss.Solve(t.Context())
		require.NoError(t, err)
		body, err := decad.Extrude(doc, ss, ss.Chains()[0], decad.TwoSided{
			One: decad.DistanceSide{D: units.Millimeters(20)},
			Two: decad.DistanceSide{D: units.Millimeters(20)}}).Do(t.Context())
		require.NoError(t, err)
		return body
	}
	first, err := decad.Split(spike, sheet(5)).Do(t.Context())
	require.NoError(t, err)
	second, err := decad.Split(first[0], sheet(4.5)).Do(t.Context())
	require.NoError(t, err)
	slab := second[1]
	capOf := func(role decad.FeatureRef) *decad.Face {
		faces, err := decad.Faces(decad.FaceCreatedBy(role)).SelectFaces(slab)
		require.NoError(t, err)
		require.Len(t, faces, 1)
		return faces[0]
	}
	cs, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	pts := []*sketch.Point{cs.CreatePoint(-4, 0), cs.CreatePoint(-4, 16), cs.CreatePoint(8, 0)}
	for _, p := range pts {
		cs.Fix(p)
	}
	for i := range pts {
		cs.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	_, err = cs.Solve(t.Context())
	require.NoError(t, err)
	cone, err := decad.Revolve(doc, cs, cs.Profiles()[0],
		decad.SketchLine{End: decad.Point2{U: 1}}, decad.FullRevolution{}).Do(t.Context())
	require.NoError(t, err)
	return coneTangentLoft{toe: capOf(decad.CapStart(slab)), heel: capOf(decad.CapEnd(slab)),
		cone: cone, radial: r3.NewVec(0, 1, 0)}
}

func TestLoftFacesRefusesUnauthenticatedSections(t *testing.T) {
	doc := decad.New()
	_, end := splitSlabCaps(t, doc)
	otherStart, _ := splitSlabCaps(t, doc)
	foreignStart, _ := splitSlabCaps(t, decad.New())
	tooth := pointLoftRadialTab(t, doc)

	tests := []struct {
		name     string
		sections []*decad.Face
		err      error
		msg      string
	}{
		{"nil section", []*decad.Face{nil, end}, decad.ErrDegenerate, "is not a body face"},
		{"face of an unsplit point loft", []*decad.Face{tooth.Faces()[0], end}, decad.ErrUnsupported,
			"no authenticated Split cap"},
		{"sections of different sources", []*decad.Face{otherStart, end}, decad.ErrUnsupported,
			"different point-loft sources"},
		{"section of another document", []*decad.Face{foreignStart, end}, decad.ErrForeignBody, ""},
	}
	for _, tc := range tests {
		before := doc.Bodies()
		_, err := decad.LoftFaces(tc.sections).Do(t.Context())
		require.ErrorIs(t, err, tc.err, tc.name)
		if tc.msg != "" {
			require.ErrorContains(t, err, tc.msg, tc.name)
		}
		require.Equal(t, before, doc.Bodies(), tc.name)
	}
}
