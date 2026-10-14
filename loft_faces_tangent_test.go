package decad

import (
	"context"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// tangentLoftFixture is the rectangle source of
// TestLoftFacesThreeCapConeContinuationKeepsSource: a point loft whose source
// plane is the tangent plane of the cone with apex 8 and slope 3/4 along the
// ruling in the +Y direction.
type tangentLoftFixture struct {
	doc        *Document
	world      *sketch.World
	s          *sketch.Sketch
	near, far  *Body
	toe, heel  *Face
	farCapFace *Face
}

func newTangentLoftFixture(t *testing.T) tangentLoftFixture {
	return newTangentLoftFixtureTurned(t, 0, 1)
}

// newTangentLoftFixtureTurned is the fixture turned about world X by the
// angle with the given sine and cosine, so its tangent ruling points along
// (0, cos, sin).
func newTangentLoftFixtureTurned(t *testing.T, sin, cos float64) tangentLoftFixture {
	t.Helper()
	doc := New()
	world := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.NewVec(8, 0, 0), r3.NewVec(0.6, -0.8*cos, -0.8*sin), r3.NewVec(0, -sin, cos))
	require.NoError(t, err)
	plane, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(-3.025, -0.025, -2.975, 0.025)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	point, err := doc.LoftFromPoint(t.Context(), r3.Vec{}, s, s.Profiles()[0])
	require.NoError(t, err)
	sheet := func(x float64) *Body {
		ss, e := world.CreateSketch(world.XY())
		require.NoError(t, e)
		a, b := ss.CreatePoint(x, -20), ss.CreatePoint(x, 20)
		ss.Fix(a)
		ss.Fix(b)
		ss.CreateLine(a, b)
		_, e = ss.Solve(t.Context())
		require.NoError(t, e)
		body, e := Extrude(t.Context(), doc, ss, ss.Chains()[0], TwoSided{
			One: DistanceSide{D: units.Millimeters(20)}, Two: DistanceSide{D: units.Millimeters(20)}})
		require.NoError(t, e)
		return body
	}
	first, err := Split(t.Context(), point, sheet(5))
	require.NoError(t, err)
	second, err := Split(t.Context(), first[0], sheet(4.5))
	require.NoError(t, err)
	f := tangentLoftFixture{doc: doc, world: world, s: s, near: second[1], far: first[1]}
	pick := func(body *Body, role provenance.Ref) *Face {
		fs, e := Faces(FaceCreatedBy(featureRefOf(role))).SelectFaces(body)
		require.NoError(t, e)
		require.Len(t, fs, 1)
		return fs[0]
	}
	f.toe = pick(f.near, refOf(CapStart(f.near)))
	f.heel = pick(f.near, refOf(CapEnd(f.near)))
	f.farCapFace = pick(f.far, refOf(CapEnd(f.far)))
	return f
}

// cone revolves the triangle (-4,0), (-4,16), (8,0) about world X: apex 8,
// slope exactly 3/4.
func (f tangentLoftFixture) cone(t *testing.T, doc *Document) *Body {
	t.Helper()
	const far, radius, apex = -4.0, 16.0, 8.0
	cs, err := f.world.CreateSketch(f.world.XY())
	require.NoError(t, err)
	points := []*sketch.Point{cs.CreatePoint(far, 0), cs.CreatePoint(far, radius), cs.CreatePoint(apex, 0)}
	for _, p := range points {
		cs.Fix(p)
	}
	for i := range points {
		cs.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = cs.Solve(t.Context())
	require.NoError(t, err)
	body, err := Revolve(t.Context(), doc, cs, cs.Profiles()[0], SketchLine{
		Start: planePoint{}, End: planePoint{U: 1}}, FullRevolution{})
	require.NoError(t, err)
	return body
}

func TestLoftFacesToConeTangentBuildsExactEnd(t *testing.T) {
	f := newTangentLoftFixture(t)
	cone := f.cone(t, f.doc)
	limit, ok := exactPointCone(cone)
	require.True(t, ok)
	require.Zero(t, limit.slope.Cmp(big.NewRat(3, 4)))

	body, err := LoftFacesToConeTangent(t.Context(), ConeTangentEnd{Cone: cone, Radial: r3.NewVec(0, 1, 0)},
		f.toe, f.heel)
	require.NoError(t, err)

	record := body.n.Payload.(facetedPayload).loftFacesCap
	require.NotNil(t, record)
	require.Len(t, record.maps, 3)
	require.Len(t, record.bounds, 3)
	require.NotNil(t, record.tangent)
	require.True(t, record.tangent.MatchesCone(limit.apex, limit.slope))
	require.True(t, loftface.MapEqual(record.final, record.tangent.Map()),
		"the recorded final map is the constructed map")
	q := loftface.ConePullback(record.final, limit.apex, limit.slope)
	for i := range 5 {
		require.Zero(t, q[i].Sign(), "pullback coefficient %d", i)
	}
	require.Zero(t, q[5].Cmp(big.NewRat(9, 16)), "pullback v^2 coefficient is the squared slope")

	// The held end ring is the rounded constructed map at each station.
	end := body.n.Payload.(facetedPayload)
	n := len(record.source.sourceStation)
	for j, station := range record.source.sourceStation {
		u, v := lfStationUV(station)
		want := loftface.RoundPoint(record.final.At(u, v))
		require.Equal(t, want, end.verts[2*n+j])
	}

	// The end group is one cap-end face of the new producer with a plane tag
	// whose normal is perpendicular to the ruling and the tangent direction.
	endFaces, err := Faces(FaceCreatedBy(CapEnd(body))).SelectFaces(body)
	require.NoError(t, err)
	require.Len(t, endFaces, 1)
	require.Len(t, endFaces[0].Origins(), 1)
	plane, ok := endFaces[0].n.Surface.(surfacegeom.Plane)
	require.True(t, ok)
	normal := plane.Frame.U().Cross(plane.Frame.V())
	require.InDelta(t, 1, math.Abs(normal.Dot(r3.NewVec(0.8, 0.6, 0))), 1e-12)
	require.InDelta(t, 8, plane.Frame.Origin().X, 0)

	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), WithVerification(VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())

	// The tangent plane is the source plane, so the volume matches the plain
	// loft through the far slab's own end cap, within both proven bounds.
	plain, err := f.doc.LoftFaces(t.Context(), f.toe, f.heel, f.farCapFace)
	require.NoError(t, err)
	tv, err := body.Volume()
	require.NoError(t, err)
	pv, err := plain.Volume()
	require.NoError(t, err)
	require.Positive(t, tv.Value.Base()-tv.Bound.Base())
	require.LessOrEqual(t, tv.Value.Base(), pv.Value.Base()+tv.Bound.Base()+pv.Bound.Base())
	require.LessOrEqual(t, math.Abs(tv.Value.Base()-pv.Value.Base()), tv.Bound.Base()+pv.Bound.Base())
}

func TestLoftFacesToConeTangentRefuses(t *testing.T) {
	f := newTangentLoftFixture(t)
	cone := f.cone(t, f.doc)
	good := ConeTangentEnd{Cone: cone, Radial: r3.NewVec(0, 1, 0)}
	faces := []*Face{f.toe, f.heel}

	t.Run("not a cone", func(t *testing.T) {
		_, err := LoftFacesToConeTangent(t.Context(), ConeTangentEnd{Cone: f.near, Radial: good.Radial}, faces...)
		require.ErrorIs(t, err, ErrUnsupported)
	})
	t.Run("placed cone", func(t *testing.T) {
		move, err := r3.Translation(r3.NewVec(1, 0, 0))
		require.NoError(t, err)
		other := f.cone(t, f.doc)
		placed, err := Transform(t.Context(), other, move)
		require.NoError(t, err)
		_, err = LoftFacesToConeTangent(t.Context(), ConeTangentEnd{Cone: placed, Radial: good.Radial}, faces...)
		require.ErrorIs(t, err, ErrUnsupported)
		_, err = LoftFacesToConeTangent(t.Context(), ConeTangentEnd{Cone: other, Radial: good.Radial}, faces...)
		require.ErrorIs(t, err, ErrRetiredBody, "the transformed original is retired")
	})
	t.Run("radial", func(t *testing.T) {
		for name, radial := range map[string]r3.Vec{
			"x nonzero": r3.NewVec(1e-9, 1, 0), "zero": {}, "nan": r3.NewVec(0, math.NaN(), 0),
			"infinite": r3.NewVec(0, math.Inf(1), 0),
		} {
			_, err := LoftFacesToConeTangent(t.Context(), ConeTangentEnd{Cone: cone, Radial: radial}, faces...)
			require.ErrorIs(t, err, ErrDegenerate, name)
		}
	})
	t.Run("other document", func(t *testing.T) {
		foreign := f.cone(t, New())
		_, err := LoftFacesToConeTangent(t.Context(), ConeTangentEnd{Cone: foreign, Radial: good.Radial}, faces...)
		require.ErrorIs(t, err, ErrForeignBody)
	})
	t.Run("section count", func(t *testing.T) {
		_, err := LoftFacesToConeTangent(t.Context(), good)
		require.ErrorIs(t, err, ErrDegenerate)
		many := make([]*Face, maxLoftFacesSections)
		for i := range many {
			many[i] = f.heel
		}
		_, err = LoftFacesToConeTangent(t.Context(), good, many...)
		require.ErrorIs(t, err, ErrDegenerate)
		_, err = LoftFacesToConeTangent(t.Context(), good, f.toe, nil)
		require.ErrorIs(t, err, ErrDegenerate)
	})
	t.Run("nil", func(t *testing.T) {
		_, err := LoftFacesToConeTangent(nil, good, faces...) //nolint:staticcheck // a nil context is the case under test.
		require.ErrorIs(t, err, ErrDegenerate)
		_, err = LoftFacesToConeTangent(t.Context(), ConeTangentEnd{Radial: good.Radial}, faces...)
		require.ErrorIs(t, err, ErrDegenerate)
	})
	t.Run("cone without a document", func(t *testing.T) {
		_, err := LoftFacesToConeTangent(t.Context(), ConeTangentEnd{Cone: bornBody(&Body{}), Radial: good.Radial}, faces...)
		require.ErrorIs(t, err, ErrDegenerate)
	})
	t.Run("stale source", func(t *testing.T) {
		stale := newTangentLoftFixture(t)
		stale.s.CreatePoint(1, 1)
		_, err := LoftFacesToConeTangent(t.Context(),
			ConeTangentEnd{Cone: stale.cone(t, stale.doc), Radial: good.Radial}, stale.toe, stale.heel)
		require.ErrorIs(t, err, ErrStaleProfile)
	})
	require.Len(t, f.doc.Bodies(), 5, "refusals leave the document unchanged")
}

func TestLoftFacesToConeTangentRefusalKeepsProvenance(t *testing.T) {
	f := newTangentLoftFixture(t)
	cone := f.cone(t, f.doc)
	next := f.doc.nextProducerID()
	_, err := LoftFacesToConeTangent(t.Context(),
		ConeTangentEnd{Cone: cone, Radial: r3.NewVec(1e-9, 1, 0)}, f.toe, f.heel)
	require.ErrorIs(t, err, ErrDegenerate)
	_, err = LoftFacesToConeTangent(t.Context(),
		ConeTangentEnd{Cone: f.near, Radial: r3.NewVec(0, 1, 0)}, f.toe, f.heel)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Equal(t, next, f.doc.nextProducerID(), "refused calls consume no provenance number")
	out, err := LoftFacesToConeTangent(t.Context(), ConeTangentEnd{Cone: cone, Radial: r3.NewVec(0, 1, 0)}, f.toe, f.heel)
	require.NoError(t, err)
	require.NotNil(t, out)
	require.Equal(t, next+1, f.doc.nextProducerID(), "the produced feature takes the unconsumed number")
}

// countdownContext reports cancellation after a fixed number of Err calls, so
// a cancel lands inside evaluation rather than at entry.
type countdownContext struct {
	context.Context //nolint:containedctx // a test double must wrap a real context.
	left            int
}

func (c *countdownContext) Err() error {
	if c.left <= 0 {
		return context.Canceled
	}
	c.left--
	return nil
}

func TestLoftFacesToConeTangentCancelledDuringEvaluation(t *testing.T) {
	radial := r3.NewVec(0, 1, 0)
	// Count how many Err checks a successful evaluation makes, on a twin document.
	g := newTangentLoftFixture(t)
	counter := &countdownContext{Context: t.Context(), left: 1 << 30}
	_, err := LoftFacesToConeTangent(counter, ConeTangentEnd{Cone: g.cone(t, g.doc), Radial: radial}, g.toe, g.heel)
	require.NoError(t, err)
	checks := (1 << 30) - counter.left
	require.Greater(t, checks, 2, "evaluation polls the context after entry")

	// A cancel at each of these polls lands inside assembly or proof; the last
	// is the final pre-commit check.
	for name, left := range map[string]int{
		"first poll after entry": 1,
		"middle poll":            checks / 2,
		"second to last poll":    checks - 2,
		"final poll":             checks - 1,
	} {
		t.Run(name, func(t *testing.T) {
			f := newTangentLoftFixture(t)
			end := ConeTangentEnd{Cone: f.cone(t, f.doc), Radial: radial}
			before, next := len(f.doc.Bodies()), f.doc.nextProducerID()
			_, err := LoftFacesToConeTangent(&countdownContext{Context: t.Context(), left: left}, end, f.toe, f.heel)
			require.ErrorIs(t, err, context.Canceled)
			require.Len(t, f.doc.Bodies(), before)
			require.Equal(t, next, f.doc.nextProducerID())
		})
	}
}

func TestLoftFacesToConeTangentCancelled(t *testing.T) {
	f := newTangentLoftFixture(t)
	cone := f.cone(t, f.doc)
	before := len(f.doc.Bodies())
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err := LoftFacesToConeTangent(ctx, ConeTangentEnd{Cone: cone, Radial: r3.NewVec(0, 1, 0)}, f.toe, f.heel)
	require.ErrorIs(t, err, context.Canceled)
	require.Len(t, f.doc.Bodies(), before)
}
