package decad

import (
	"context"
	"errors"
	"maps"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/loftfacestrim"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolvecontact"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
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
	point, err := LoftFromPoint(t.Context(), doc, r3.Vec{}, s, s.Profiles()[0])
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
	limit, ok := revolveplan.PointConeSource(&cone.n)
	require.True(t, ok)
	require.Zero(t, limit.Slope.Cmp(big.NewRat(3, 4)))

	body, err := LoftFacesToConeTangent(t.Context(), ConeTangentEnd{Cone: cone, Radial: r3.NewVec(0, 1, 0)},
		f.toe, f.heel)
	require.NoError(t, err)

	record := body.n.Payload.(facetedPayload).LoftFacesCap
	require.NotNil(t, record)
	require.Len(t, record.Maps, 3)
	require.Len(t, record.Bounds, 3)
	require.NotNil(t, record.Tangent)
	require.True(t, record.Tangent.MatchesCone(limit.Apex, limit.Slope))
	require.True(t, loftface.MapEqual(record.Final, record.Tangent.Map()),
		"the recorded final map is the constructed map")
	q := loftface.ConePullback(record.Final, limit.Apex, limit.Slope)
	for i := range 5 {
		require.Zero(t, q[i].Sign(), "pullback coefficient %d", i)
	}
	require.Zero(t, q[5].Cmp(big.NewRat(9, 16)), "pullback v^2 coefficient is the squared slope")

	// The held end ring is the rounded constructed map at each station.
	end := body.n.Payload.(facetedPayload)
	n := len(record.Source.SourceStation)
	for j, station := range record.Source.SourceStation {
		u, v := lfF(station.U), lfF(station.V)
		want := loftface.RoundPoint(record.Final.At(u, v))
		require.Equal(t, want, end.Verts[2*n+j])
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
	plain, err := LoftFaces(t.Context(), f.doc, f.toe, f.heel, f.farCapFace)
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

// coneAt revolves the triangle (far,0), (far,radius), (apex,0) about world X.
func (f tangentLoftFixture) coneAt(t *testing.T, far, radius, apex float64) *Body {
	t.Helper()
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
	body, err := Revolve(t.Context(), f.doc, cs, cs.Profiles()[0], SketchLine{
		Start: planePoint{}, End: planePoint{U: 1}}, FullRevolution{})
	require.NoError(t, err)
	return body
}

// tangentTrimChain lofts through the toe and heel caps to the cone-tangent
// end, then cuts by the toe cone (apex 7.5). The heel Intersect by the cone
// the end touches (apex 8, slope 3/4) is left to the caller.
type tangentTrimChain struct {
	f                 tangentLoftFixture
	loft, cut         *Body
	toeTool, heelTool *Body
	loftRecord        *loftFacesCapRecord
}

// The tangent ruling points along (0, cos, sin). Its angle is 53 degrees, so
// it stays clear of the sample rulings of the held heel cone and no held cone
// edge lies on the cap plane.
const tangentSin, tangentCos = 0.8, 0.6

func buildTangentTrimChain(t *testing.T) tangentTrimChain {
	t.Helper()
	f := newTangentLoftFixtureTurned(t, tangentSin, tangentCos)
	c := tangentTrimChain{f: f}
	c.heelTool = f.cone(t, f.doc)
	c.toeTool = f.coneAt(t, -4, 8.625, 7.5)
	var err error
	c.loft, err = LoftFacesToConeTangent(t.Context(), ConeTangentEnd{Cone: c.heelTool, Radial: r3.NewVec(0, tangentCos, tangentSin)},
		f.toe, f.heel)
	require.NoError(t, err)
	c.loftRecord = c.loft.n.Payload.(facetedPayload).LoftFacesCap
	c.cut, err = Cut(t.Context(), c.loft, c.toeTool)
	require.NoError(t, err)
	require.NotNil(t, c.cut.n.Payload.(facetedPayload).LoftFacesToe)
	return c
}

// tangentGateInputs returns the pieces of the real chain that G1 reads: the
// recorded final and preceding cap maps, the source box and the heel cone.
func tangentGateInputs(t *testing.T, c tangentTrimChain) (final, previous loftFaceMap, box [4]*big.Rat, heel *pointConeLimit) {
	t.Helper()
	record := c.cut.n.Payload.(facetedPayload).LoftFacesToe.Cap
	var err error
	box, err = revolveloftfaces.LfSourceBox(record.Source)
	require.NoError(t, err)
	heel, ok := revolveplan.PointConeSource(&c.heelTool.n)
	require.True(t, ok)
	return record.Final, record.Maps[len(record.Maps)-2], box, heel
}

func TestLoftFacesTangentHeelIntersectKeepsTrimRecord(t *testing.T) {
	c := buildTangentTrimChain(t)

	// G1 holds on the real maps. Its side is checked against the plane
	// functional N.(P - end point), which is phi without a stored constant.
	final, previous, box, heel := tangentGateInputs(t, c)
	side, ok := loftface.ConeOnLoftSide(final, previous, box, heel.Apex, heel.Slope, heel.Far)
	require.True(t, ok, "G1 admits the real chain")
	normal := loftface.VCross(final.QU, final.QV)
	origin := final.At(box[0], box[2])
	phi := func(p loftface.RatVec) int { return loftface.VDot(normal, loftface.VSub(p, origin)).Sign() }
	for _, corner := range loftface.BoxCorners(box) {
		require.Equal(t, side, phi(previous.At(corner[0], corner[1])), "the preceding cap is on the loft side")
	}
	mid := new(big.Rat).Quo(new(big.Rat).Add(heel.Apex, heel.Far), big.NewRat(2, 1))
	require.Equal(t, side, phi(loftface.RatVec{mid, new(big.Rat), new(big.Rat)}), "the cone is on the loft side")

	// G2 holds on the real maps, and the extended proxy is what the ordinary
	// gate accepts. The unextended held tooth, with no pair exempted, is
	// refused at the first chord scale whose tessellation the held cut accepts:
	// the end cap touches the cone, which the proximity gate cannot decide.
	require.True(t, loftface.CapsOnSide(final, c.loftRecord.Maps[:len(c.loftRecord.Maps)-1], box, side),
		"G2 admits the real chain")
	decided := false
	for scale := 4.0; scale <= 32 && !decided; scale *= 2 {
		_, err := evaluateBooleanMeshes(t.Context(), meshbool.OpIntersect, c.cut, c.heelTool,
			pairMeshes{}, scale)
		var expected *meshbool.BooleanExpectedError
		require.Error(t, err, "the unextended tooth is never admitted without an exemption")
		require.True(t, errors.As(err, &expected))
		if expected.Kind == meshbool.BooleanExpectedStaging {
			continue
		}
		decided = true
		require.ErrorIs(t, err, ErrUnsupported)
		require.Equal(t, meshbool.BooleanExpectedContact, expected.Kind, "the ordinary near-miss gate refuses")
	}
	require.True(t, decided, "the held cut meshes within the ladder")

	// Gate E5 passes on the meshes the extended Boolean used, and no result
	// vertex lies beyond the slab.
	plane, ok := loftface.NewTangentPlane(final, side)
	require.True(t, ok)
	toe := c.cut.n.Payload.(facetedPayload).LoftFacesToe
	var meshes *loftfacestrim.TangentHeelMeshes
	var eval booleanEvaluation
	for scale := 4.0; scale <= 32; scale *= 2 {
		var err error
		meshes = &loftfacestrim.TangentHeelMeshes{Base: pairMeshes{}, Target: bodyNode(c.cut), Toe: toe, Plane: plane}
		eval, err = evaluateBooleanMeshes(t.Context(), meshbool.OpIntersect, c.cut, c.heelTool,
			meshes, scale)
		if err == nil {
			break
		}
		var expected *meshbool.BooleanExpectedError
		require.True(t, errors.As(err, &expected))
		require.Equal(t, meshbool.BooleanExpectedStaging, expected.Kind, "only staging retries: %v", err)
	}
	require.NotNil(t, meshes.ConeTris, "the extended Boolean succeeded within the ladder")
	require.NoError(t, loftface.ExtensionClearsCone(meshes.Slab, meshes.ConeVerts, meshes.ConeTris, plane,
		proofbound.NewWorkBudget(t.Context())), "E5 passes on the recorded meshes")
	require.NoError(t, meshes.ClearsResult(t.Context(), eval.payload.Verts))
	depth, ok := loftface.ConeDepth(meshes.ConeVerts, plane)
	require.True(t, ok)
	floor := depth
	if floor.Cmp(meshes.Slab.Eps) < 0 {
		floor = meshes.Slab.Eps
	}
	require.False(t, loftface.ResultBeyondSlab(eval.payload.Verts, plane, floor),
		"no result vertex lies beyond the slab")

	// The result's volume allowance is the composition of the UNEXTENDED
	// operand's allowance, the cone's and the final stitch's swept-volume
	// charge. The stitch is rebuilt from the meshes the Boolean read.
	held, _, err := tessellateBooleanOperand(t.Context(), c.cut, meshes.PairTol)
	require.NoError(t, err)
	faceID := map[*topo.Face]int{}
	for i, f := range append(c.cut.Faces(), c.heelTool.Faces()...) {
		faceID[faceNode(f)] = i
	}
	srcA, err := meshbool.SourceIDs(t.Context(), meshes.Proxy.Source, faceID)
	require.NoError(t, err)
	srcB, err := meshbool.SourceIDs(t.Context(), meshes.Cone.Source, faceID)
	require.NoError(t, err)
	bmA, err := booleaneval.PrepBoolMesh(t.Context(), meshes.Proxy, srcA)
	require.NoError(t, err)
	bmB, err := booleaneval.PrepBoolMesh(t.Context(), meshes.Cone, srcB)
	require.NoError(t, err)
	kept, _, err := meshbool.MeshBoolean(t.Context(), meshbool.OpIntersect, bmA, bmB, meshbool.NewContactMemo(bmA, bmB))
	require.NoError(t, err)
	stitched, err := meshbool.StitchFacetsContext(t.Context(), kept)
	require.NoError(t, err)
	require.Positive(t, held.n.VolSymDiff)
	require.Positive(t, meshes.Cone.VolSymDiff)
	roundVol := proofbound.SweptVolumeAllow(stitched.Round, stitched.PreArea)
	require.Equal(t, proofbound.AbsSumUpper(held.n.VolSymDiff, meshes.Cone.VolSymDiff, roundVol),
		eval.payload.VolSymDiff, "the result volume allowance composes the unextended operand's")

	trim, err := Intersect(t.Context(), c.cut, c.heelTool)
	require.NoError(t, err)
	require.Equal(t, eval.payload.VolSymDiff, trim.n.Payload.(facetedPayload).VolSymDiff,
		"the published allowance is the composed one")
	trimRecord := trim.n.Payload.(facetedPayload).LoftFacesTrim
	require.NotNil(t, trimRecord, "the tangent heel Intersect keeps the trim record")
	require.NotNil(t, trimRecord.Tangent)
	require.Same(t, c.loftRecord.Tangent, trimRecord.Tangent, "the trim record copies the cap record's tangent end")
	require.True(t, trimRecord.Tangent.MatchesCone(heel.Apex, heel.Slope))
	require.Same(t, c.loftRecord.Source, trimRecord.Source)
	require.Len(t, trimRecord.Maps, 3)

	// The end cap does not survive; the heel cone is one face.
	require.NotNil(t, revolvecontact.UniqueResultSourceFace(bodyNode(trim), trimRecord.HeelCapGroup))
	require.NotNil(t, revolvecontact.UniqueResultSourceFace(bodyNode(trim), trimRecord.ToeCapGroup))
	_, err = Faces(FaceCreatedBy(CapEnd(c.loft))).SelectFaces(trim)
	require.ErrorIs(t, err, ErrNoMatch, "no result face descends from the planar end cap")

	loftVolume, err := c.loft.Volume()
	require.NoError(t, err)
	cutVolume, err := c.cut.Volume()
	require.NoError(t, err)
	trimVolume, err := trim.Volume()
	require.NoError(t, err)
	require.Less(t, cutVolume.Value.Base(), loftVolume.Value.Base())
	require.Less(t, trimVolume.Value.Base(), cutVolume.Value.Base())
	mesh, err := trim.Tessellate(t.Context(), units.Millimeters(0.1), WithVerification(VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

// tangentCapFixture builds the real toe-cut tooth and returns its held mesh,
// toe record, tangent plane and the plane's loft side.
func tangentCapFixture(t *testing.T) (*Mesh, *loftFacesToeRecord, loftface.TangentPlane, int) {
	t.Helper()
	c := buildTangentTrimChain(t)
	final, previous, box, heel := tangentGateInputs(t, c)
	side, ok := loftface.ConeOnLoftSide(final, previous, box, heel.Apex, heel.Slope, heel.Far)
	require.True(t, ok)
	plane, ok := loftface.NewTangentPlane(final, side)
	require.True(t, ok)
	toe := c.cut.n.Payload.(facetedPayload).LoftFacesToe
	var held *Mesh
	for scale := 4.0; scale <= 32 && held == nil; scale *= 2 {
		tolMM, _, err := booloperand.PairChord(t.Context(), &c.cut.n, &c.heelTool.n)
		require.NoError(t, err)
		mesh, _, err := tessellateBooleanOperand(t.Context(), c.cut, tolMM*scale)
		if err == nil {
			held = mesh
		}
	}
	require.NotNil(t, held, "the held cut meshes within the ladder")
	return held, toe, plane, side
}

// TestTangentCapExtensionOnRealChain extends the real toe-cut tooth and checks
// the proxy against the held mesh: the cap triangles move off the plane to the
// far side, every ring vertex is a station vertex, every extension wall covers
// its chord's certified bound, and the proxy carries the unextended operand's
// volume and area allowances.
func TestTangentCapExtensionOnRealChain(t *testing.T) {
	held, toe, plane, side := tangentCapFixture(t)
	// Hand the adapter face bounds far above any facet bound. The proxy's face
	// bounds are recomputed from its final facets, never maxed with the old ones.
	oversized := bornMesh(&Mesh{n: held.n})
	oversized.n.FaceBound = maps.Clone(held.n.FaceBound)
	for f := range oversized.n.FaceBound {
		oversized.n.FaceBound[f] = 1e6
	}
	proxyNode, slab, err := revolveloftfaces.ExtendTangentCap(t.Context(), &oversized.n, toe, plane)
	require.NoError(t, err)
	proxy := bornMesh(&Mesh{n: *proxyNode})
	require.NotNil(t, slab.Eps)
	changed := map[*Face]bool{faceHandle(toe.CapFace): true}
	for _, f := range proxy.n.Source[len(proxy.n.Source)-2*len(toe.Cap.Source.SourceStation):] {
		changed[faceHandle(f)] = true
	}
	for f := range changed {
		want := 0.0
		for i, source := range proxy.n.Source {
			if source != faceNode(f) {
				continue
			}
			for _, v := range proxy.n.Triangles[i] {
				want = max(want, proxy.n.VertexBound[v])
			}
		}
		require.Equal(t, want, proxy.n.FaceBound[faceNode(f)], "a changed face bound is the recomputed facet maximum")
		require.Less(t, want, 1e6)
	}
	for f := range proxy.n.FaceBound {
		if !changed[faceHandle(f)] {
			require.Equal(t, 1e6, proxy.n.FaceBound[f], "an unchanged face keeps its bound")
		}
	}

	capFacets := 0
	for i, f := range proxy.n.Source {
		if f != toe.CapFace {
			continue
		}
		capFacets++
		for _, v := range proxy.n.Triangles[i] {
			// Side*phi <= 0 at every extension vertex: it is past the plane.
			phi := plane.Phi(loftface.Vec(proxy.n.Vertices[v]))
			require.Negative(t, phi.Sign()*side, "the extension cap is beyond the plane")
		}
	}
	require.Positive(t, capFacets)
	require.Greater(t, len(proxy.n.Triangles), len(held.n.Triangles))
	require.Equal(t, held.n.VolSymDiff, proxy.n.VolSymDiff, "the proxy carries the unextended volume allowance")
	require.Equal(t, held.n.AreaSlack, proxy.n.AreaSlack, "the proxy carries the unextended area allowance")
	require.GreaterOrEqual(t, proxy.n.Bound, held.n.Bound)
	require.Len(t, proxy.n.VertexBound, len(proxy.n.Vertices))
	require.Len(t, held.n.Vertices, len(held.n.VertexBound), "the held mesh is not modified")

	// Every ring vertex maps to its station, and each extension wall facet bound
	// is at least the chord bound that the final cap's proof certifies.
	link, err := revolveloftfaces.TangentToeStationLink(held.n.Vertices, toe)
	require.NoError(t, err)
	record := toe.Cap
	last := len(record.Maps) - 1
	n := len(record.Source.SourceStation)
	capBounds := record.Bounds[last]
	walls := proxy.n.Triangles[len(proxy.n.Triangles)-2*n:]
	seen := make([]bool, n)
	for i := 0; i < len(walls); i += 2 {
		a, b := walls[i][0], walls[i][1]
		require.Less(t, a, len(held.n.Vertices))
		require.Less(t, b, len(held.n.Vertices))
		sa, sb := link.Station[a], link.Station[b]
		require.GreaterOrEqual(t, min(sa, sb), 0, "a ring vertex is a station vertex")
		require.Equal(t, held.n.Vertices[a], toe.Base.Verts[last*n+sa], "ring vertex equals its station vertex")
		j := min(sa, sb)
		if max(sa, sb)-min(sa, sb) != 1 {
			j = max(sa, sb)
		}
		seen[j] = true
		chord := proofbound.AbsSumUpper(capBounds.SourceDeparture[j], capBounds.ProjectiveDeparture[j],
			capBounds.StationRound)
		for k := range 2 {
			corner := 0.0
			for _, v := range walls[i+k] {
				corner = max(corner, proxy.n.VertexBound[v])
			}
			require.GreaterOrEqual(t, corner, chord, "extension wall facet covers chord %d", j)
		}
	}
	for j, ok := range seen {
		require.True(t, ok, "station chord %d has an extension wall", j)
	}

	// A ring with two neighbouring stations swapped has an edge that is no chord.
	require.GreaterOrEqual(t, n, 4)
	swapped := *toe
	swapped.Base.Verts = append([]r3.Vec(nil), toe.Base.Verts...)
	ring := swapped.Base.Verts[last*n : (last+1)*n]
	ring[0], ring[1] = ring[1], ring[0]
	_, _, err = revolveloftfaces.ExtendTangentCap(t.Context(), &held.n, &swapped, plane)
	require.ErrorIs(t, err, ErrUnsupported)
}

// movePreviousCap moves the cap before the tangent end along the end plane's
// normal by factor times the normal, and installs the result in a copy of the
// cut's cap record.
func movePreviousCap(t *testing.T, cut *Body, factor int64) {
	t.Helper()
	p := cut.n.Payload.(facetedPayload)
	toe := *p.LoftFacesToe
	record := *toe.Cap
	record.Maps = append([]loftFaceMap(nil), record.Maps...)
	previous := len(record.Maps) - 2
	normal := loftface.VCross(record.Final.QU, record.Final.QV)
	moved := record.Final
	moved.Constant = loftface.VAdd(moved.Constant, loftface.VScale(normal, big.NewRat(factor, 8)))
	record.Maps[previous] = moved
	toe.Cap = &record
	p.LoftFacesToe = &toe
	cut.n.Payload = p
}

func TestLoftFacesTangentHeelRefusesWrongSide(t *testing.T) {
	// The tangent end touches the cone, but the preceding cap lies on the
	// other side of the end plane from the cone. G1 refuses before any mesh.
	for name, test := range map[string]struct {
		factor  int64
		refused bool
	}{"cone side": {factor: 1}, "far side": {factor: -1, refused: true}} {
		t.Run(name, func(t *testing.T) {
			c := buildTangentTrimChain(t)
			movePreviousCap(t, c.cut, test.factor)
			trim, err := Intersect(t.Context(), c.cut, c.heelTool)
			if !test.refused {
				require.NoError(t, err)
				require.NotNil(t, trim.n.Payload.(facetedPayload).LoftFacesTrim)
				return
			}
			require.ErrorIs(t, err, ErrUnsupported)
			require.ErrorContains(t, err, "the heel cone is not provably on the loft side", "G1 itself refuses, not a later gate")
			require.Nil(t, trim)
			require.Len(t, c.f.doc.Bodies(), 5, "the refusal leaves the document unchanged")
		})
	}
}

// A tangent heel result whose toe or heel tool face is not a single result
// patch is refused before the body is built, never returned without a trim
// record. Naming the removed end cap as the toe cap names a source group the
// result has no facet of.
func TestLoftFacesTangentHeelRefusesMissingToolFace(t *testing.T) {
	c := buildTangentTrimChain(t)
	p := c.cut.n.Payload.(facetedPayload)
	toe := *p.LoftFacesToe
	toe.ToeCapFace = toe.CapFace
	p.LoftFacesToe = &toe
	c.cut.n.Payload = p

	trim, err := Intersect(t.Context(), c.cut, c.heelTool)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Nil(t, trim)
	require.Len(t, c.f.doc.Bodies(), 5, "the refusal leaves the document unchanged")
}

func TestLoftFacesTangentHeelRefusesOtherCone(t *testing.T) {
	// A cone one float step off the tangent apex does not match the record,
	// and the strict form needs a cap that misses its own apex.
	c := buildTangentTrimChain(t)
	other := c.f.coneAt(t, -4, 16, math.Nextafter(8, 9))
	limit, ok := revolveplan.PointConeSource(&other.n)
	require.True(t, ok)
	require.False(t, c.loftRecord.Tangent.MatchesCone(limit.Apex, limit.Slope))
	box, err := revolveloftfaces.LfSourceBox(c.loftRecord.Source)
	require.NoError(t, err)
	require.False(t, facetedrecord.CapOutsideHeelCone(c.loftRecord.Final, box, limit))
	plane, err := facetedrecord.CapTangentToHeelCone(c.loftRecord, box, limit)
	require.NoError(t, err)
	require.Nil(t, plane, "the record gate fails")

	trim, err := Intersect(t.Context(), c.cut, other)
	if err == nil {
		require.Nil(t, trim.n.Payload.(facetedPayload).LoftFacesTrim, "no trim record is issued")
		return
	}
	require.ErrorIs(t, err, ErrUnsupported)
}

// The extension refuses a mesh that is not verified or that has no end cap
// facet, charges the held coordinate bound into the proxy, and refuses a bound
// it cannot state.
func TestTangentCapExtensionGatesAndCharges(t *testing.T) {
	held, toe, plane, side := tangentCapFixture(t)
	extend := func(edit func(*Mesh), toeRecord *loftFacesToeRecord) (*Mesh, error) {
		mesh := bornMesh(&Mesh{n: held.n})
		mesh.n.FaceBound = maps.Clone(held.n.FaceBound)
		if edit != nil {
			edit(mesh)
		}
		proxy, _, err := revolveloftfaces.ExtendTangentCap(t.Context(), &mesh.n, toeRecord, plane)
		if err != nil {
			return nil, err
		}
		return bornMesh(&Mesh{n: *proxy}), nil
	}

	for name, edit := range map[string]func(*Mesh){
		"boundary not verified": func(m *Mesh) { m.n.BoundaryOK = false },
		"volume not verified":   func(m *Mesh) { m.n.SymDiffOK = false },
	} {
		_, err := extend(edit, toe)
		require.ErrorIs(t, err, ErrUnsupported, name)
		require.ErrorContains(t, err, "mesh is not verified", name)
	}

	elsewhere := *toe
	elsewhere.CapFace = new(topo.Face)
	_, err := extend(nil, &elsewhere)
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorContains(t, err, "mesh has no end cap facet")

	proxy, err := extend(nil, toe)
	require.NoError(t, err)
	faceMax := 0.0
	for _, bound := range proxy.n.FaceBound {
		faceMax = max(faceMax, bound)
	}
	require.Equal(t, faceMax, proxy.n.Bound, "the global bound is the largest face bound")
	require.Greater(t, proxy.n.Bound, 0.0)

	loose, err := extend(func(m *Mesh) { m.n.Bound = 1e9 }, toe)
	require.NoError(t, err)
	require.Equal(t, faceMax, loose.n.Bound, "the held global bound does not carry over")

	charged, err := extend(func(m *Mesh) { m.n.CoordBound = 7 }, toe)
	require.NoError(t, err)
	require.Greater(t, charged.n.CoordBound, 7.0, "the held coordinate bound is charged and rounded up")
	require.Less(t, charged.n.CoordBound, 7.5)
	require.Greater(t, charged.n.CoordBound, proxy.n.CoordBound)

	_, err = extend(func(m *Mesh) { m.n.CoordBound = math.Inf(1) }, toe)
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorContains(t, err, "unbounded bound")
	_, err = extend(func(m *Mesh) {
		for f := range m.n.FaceBound {
			m.n.FaceBound[f] = math.Inf(1)
		}
	}, toe)
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorContains(t, err, "unbounded bound")

	// The prism is one millimetre long along the plane normal.
	norm := 0.0
	var unit [3]float64
	for i, c := range plane.N {
		f, _ := c.Float64()
		unit[i] = f
		norm += f * f
	}
	norm = math.Sqrt(norm)
	far := 0.0
	for _, v := range proxy.n.Vertices[len(held.n.Vertices):] {
		far = max(far, math.Abs((v.X*unit[0]+v.Y*unit[1]+v.Z*unit[2])/norm-planeOffset(plane, norm)))
	}
	require.InDelta(t, 1.0, far, 1e-9, "side %d", side)
}

func planeOffset(plane loftface.TangentPlane, norm float64) float64 {
	d, _ := plane.D.Float64()
	return d / norm
}
