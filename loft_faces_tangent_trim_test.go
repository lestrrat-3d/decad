package decad

import (
	"errors"
	"maps"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

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
	heel, ok := exactPointCone(c.heelTool)
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
		_, err := evaluateBooleanMeshesWithConeCap(t.Context(), meshbool.OpIntersect, c.cut, c.heelTool,
			pairMeshes{}, scale, nil, nil)
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
	var meshes *tangentHeelMeshes
	var eval booleanEvaluation
	for scale := 4.0; scale <= 32; scale *= 2 {
		var err error
		meshes = &tangentHeelMeshes{target: c.cut, toe: toe, plane: plane}
		eval, err = evaluateBooleanMeshesWithConeCap(t.Context(), meshbool.OpIntersect, c.cut, c.heelTool,
			meshes, scale, nil, nil)
		if err == nil {
			break
		}
		var expected *meshbool.BooleanExpectedError
		require.True(t, errors.As(err, &expected))
		require.Equal(t, meshbool.BooleanExpectedStaging, expected.Kind, "only staging retries: %v", err)
	}
	require.NotNil(t, meshes.coneTris, "the extended Boolean succeeded within the ladder")
	require.NoError(t, loftface.ExtensionClearsCone(meshes.slab, meshes.coneVerts, meshes.coneTris, plane,
		proofbound.NewWorkBudget(t.Context())), "E5 passes on the recorded meshes")
	require.NoError(t, meshes.clearsResult(t.Context(), eval.payload.Verts))
	depth, ok := loftface.ConeDepth(meshes.coneVerts, plane)
	require.True(t, ok)
	floor := depth
	if floor.Cmp(meshes.slab.Eps) < 0 {
		floor = meshes.slab.Eps
	}
	require.False(t, loftface.ResultBeyondSlab(eval.payload.Verts, plane, floor),
		"no result vertex lies beyond the slab")

	// The result's volume allowance is the composition of the UNEXTENDED
	// operand's allowance, the cone's and the final stitch's swept-volume
	// charge. The stitch is rebuilt from the meshes the Boolean read.
	held, _, err := tessellateBooleanOperand(t.Context(), c.cut, meshes.pairTol)
	require.NoError(t, err)
	faceID := map[*topo.Face]int{}
	for i, f := range append(c.cut.Faces(), c.heelTool.Faces()...) {
		faceID[faceNode(f)] = i
	}
	srcA, err := meshbool.SourceIDs(t.Context(), meshes.proxy.n.Source, faceID)
	require.NoError(t, err)
	srcB, err := meshbool.SourceIDs(t.Context(), meshes.cone.n.Source, faceID)
	require.NoError(t, err)
	bmA, err := prepBoolMeshContext(t.Context(), meshes.proxy, srcA)
	require.NoError(t, err)
	bmB, err := prepBoolMeshContext(t.Context(), meshes.cone, srcB)
	require.NoError(t, err)
	kept, _, err := meshbool.MeshBoolean(t.Context(), meshbool.OpIntersect, bmA, bmB, meshbool.NewContactMemo(bmA, bmB))
	require.NoError(t, err)
	stitched, err := meshbool.StitchFacetsContext(t.Context(), kept)
	require.NoError(t, err)
	require.Positive(t, held.n.VolSymDiff)
	require.Positive(t, meshes.cone.n.VolSymDiff)
	roundVol := proofbound.SweptVolumeAllow(stitched.Round, stitched.PreArea)
	require.Equal(t, proofbound.AbsSumUpper(held.n.VolSymDiff, meshes.cone.n.VolSymDiff, roundVol),
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
	require.NotNil(t, uniqueResultSourceFace(trim, trimRecord.HeelCapGroup))
	require.NotNil(t, uniqueResultSourceFace(trim, trimRecord.ToeCapGroup))
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

// TestTangentCapExtensionOnRealChain extends the real toe-cut tooth and checks
// the proxy against the held mesh: the cap triangles move off the plane to the
// far side, every ring vertex is a station vertex, every extension wall covers
// its chord's certified bound, and the proxy carries the unextended operand's
// volume and area allowances.
func TestTangentCapExtensionOnRealChain(t *testing.T) {
	c := buildTangentTrimChain(t)
	final, previous, box, heel := tangentGateInputs(t, c)
	side, ok := loftface.ConeOnLoftSide(final, previous, box, heel.Apex, heel.Slope, heel.Far)
	require.True(t, ok)
	plane, ok := loftface.NewTangentPlane(final, side)
	require.True(t, ok)
	toe := c.cut.n.Payload.(facetedPayload).LoftFacesToe
	var held *Mesh
	for scale := 4.0; scale <= 32 && held == nil; scale *= 2 {
		tolMM, _, err := pairChordTolerance(t.Context(), c.cut, c.heelTool)
		require.NoError(t, err)
		mesh, _, err := tessellateBooleanOperand(t.Context(), c.cut, tolMM*scale)
		if err == nil {
			held = mesh
		}
	}
	require.NotNil(t, held, "the held cut meshes within the ladder")
	// Hand the adapter face bounds far above any facet bound. The proxy's face
	// bounds are recomputed from its final facets, never maxed with the old ones.
	oversized := bornMesh(&Mesh{n: held.n})
	oversized.n.FaceBound = maps.Clone(held.n.FaceBound)
	for f := range oversized.n.FaceBound {
		oversized.n.FaceBound[f] = 1e6
	}
	proxy, slab, err := extendTangentCap(t.Context(), oversized, toe, plane)
	require.NoError(t, err)
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
	_, _, err = extendTangentCap(t.Context(), held, &swapped, plane)
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
	limit, ok := exactPointCone(other)
	require.True(t, ok)
	require.False(t, c.loftRecord.Tangent.MatchesCone(limit.Apex, limit.Slope))
	box, err := revolveloftfaces.LfSourceBox(c.loftRecord.Source)
	require.NoError(t, err)
	require.False(t, loftCapOutsideHeelCone(c.loftRecord.Final, box, limit))
	plane, err := loftCapTangentToHeelCone(c.loftRecord, box, limit)
	require.NoError(t, err)
	require.Nil(t, plane, "the record gate fails")

	trim, err := Intersect(t.Context(), c.cut, other)
	if err == nil {
		require.Nil(t, trim.n.Payload.(facetedPayload).LoftFacesTrim, "no trim record is issued")
		return
	}
	require.ErrorIs(t, err, ErrUnsupported)
}
