package loftmesh

import (
	"context"
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// randomCornerPoint draws an exact point. Kind 0 is a float image, kind 1 a
// rational with a positive odd denominator, kind 2 an extreme point: a huge
// coordinate (the float box refuses), a tiny one, or a negative zero.
func randomCornerPoint(rng *rand.Rand, kind int) proofarith.Xpt {
	signed := func() float64 {
		v := math.Ldexp(0.5+rng.Float64()/2, rng.IntN(121)-60)
		if rng.IntN(2) == 0 {
			v = -v
		}
		return v
	}
	switch kind {
	case 0:
		return proofarith.XptOf(r3.Vec{X: signed(), Y: signed(), Z: signed()})
	case 1:
		coord := func() *big.Int {
			n := big.NewInt(int64(rng.Uint64() >> 4))
			if rng.IntN(2) == 0 {
				n.Neg(n)
			}
			return n
		}
		w := new(big.Int).SetUint64(rng.Uint64()>>24 | 1)
		return proofarith.Xpt{X: coord(), Y: coord(), Z: coord(), W: w}
	}
	huge := new(big.Int).Lsh(big.NewInt(1), 1100)
	tinyW := new(big.Int).Add(huge, big.NewInt(1))
	switch rng.IntN(4) {
	case 0:
		return proofarith.Xpt{X: huge, Y: big.NewInt(3), Z: big.NewInt(-5), W: big.NewInt(1)}
	case 1:
		return proofarith.Xpt{X: big.NewInt(1), Y: big.NewInt(-1), Z: big.NewInt(0), W: tinyW}
	case 2:
		return proofarith.XptOf(r3.Vec{X: math.Copysign(0, -1), Y: signed(), Z: math.Copysign(0, -1)})
	}
	return proofarith.Xpt{X: new(big.Int).Neg(huge), Y: big.NewInt(1), Z: big.NewInt(1), W: big.NewInt(1)}
}

func requireBoxesBitEqual(t *testing.T, want, got [2]r3.Vec) {
	t.Helper()
	for side := range 2 {
		w := [3]float64{want[side].X, want[side].Y, want[side].Z}
		g := [3]float64{got[side].X, got[side].Y, got[side].Z}
		for axis := range 3 {
			require.Equal(t, math.Float64bits(w[axis]), math.Float64bits(g[axis]),
				"side %d axis %d: want %v got %v", side, axis, w[axis], g[axis])
		}
	}
}

func TestChamferCornersBoxMatchesExactTriBox(t *testing.T) {
	rng := rand.New(rand.NewPCG(103, 107))
	perms := [6][3]int{{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}}
	refused, accepted := 0, 0
	check := func(tri [3]proofarith.Xpt) {
		for _, perm := range perms {
			p := [3]proofarith.Xpt{tri[perm[0]], tri[perm[1]], tri[perm[2]]}
			want, wantOK := ChamferExactTriBox(p)
			a, b, c := ChamferCornerOf(p[0]), ChamferCornerOf(p[1]), ChamferCornerOf(p[2])
			got, gotOK := ChamferCornersBox(a, b, c)
			require.Equal(t, wantOK, gotOK)
			if !wantOK {
				refused++
				continue
			}
			accepted++
			requireBoxesBitEqual(t, want, got)
		}
		for _, p := range tri {
			c := ChamferCornerOf(p)
			v := p.Vec()
			require.Equal(t, math.Float64bits(v.X), math.Float64bits(c.Float.X))
			require.Equal(t, math.Float64bits(v.Y), math.Float64bits(c.Float.Y))
			require.Equal(t, math.Float64bits(v.Z), math.Float64bits(c.Float.Z))
		}
	}
	for range 1500 {
		kind := rng.IntN(2)
		check([3]proofarith.Xpt{randomCornerPoint(rng, kind), randomCornerPoint(rng, kind),
			randomCornerPoint(rng, kind)})
	}
	// Mixed kinds, shared and repeated corners, degenerate triangles.
	for range 300 {
		a, b := randomCornerPoint(rng, rng.IntN(3)), randomCornerPoint(rng, rng.IntN(3))
		check([3]proofarith.Xpt{a, b, a})
		check([3]proofarith.Xpt{a, a, a})
		check([3]proofarith.Xpt{a, b, randomCornerPoint(rng, 2)})
	}
	require.Positive(t, refused)
	require.Positive(t, accepted)
}

// Every corner box has a distinct lower and upper end only when the exact
// coordinate is not a float, so the rational points must exercise both
// directions of rounding.
func TestChamferCornerOfRoundsOutward(t *testing.T) {
	rng := rand.New(rand.NewPCG(109, 113))
	strict := 0
	for range 500 {
		p := randomCornerPoint(rng, 1)
		c := ChamferCornerOf(p)
		for axis, coord := range [3]*big.Rat{
			meshbool.RatCoordOf(p, 0), meshbool.RatCoordOf(p, 1), meshbool.RatCoordOf(p, 2),
		} {
			lo := [3]float64{c.Box[0].X, c.Box[0].Y, c.Box[0].Z}[axis]
			hi := [3]float64{c.Box[1].X, c.Box[1].Y, c.Box[1].Z}[axis]
			require.LessOrEqual(t, new(big.Rat).SetFloat64(lo).Cmp(coord), 0)
			require.GreaterOrEqual(t, new(big.Rat).SetFloat64(hi).Cmp(coord), 0)
			if lo < hi {
				strict++
			}
		}
	}
	require.Positive(t, strict)
}

// The face table must name, for each of the seven faces, the corners the
// face is built from, in order.
func TestChamferPrismFaceCornersMatchFaces(t *testing.T) {
	var pts [6]proofarith.Xpt
	for i := range pts {
		pts[i] = proofarith.XptOf(r3.Vec{X: float64(i + 1), Y: float64(10 * (i + 1)), Z: float64(100 * (i + 1))})
	}
	base := [3]proofarith.Xpt{pts[0], pts[1], pts[2]}
	q := [3]proofarith.Xpt{pts[3], pts[4], pts[5]}
	var faces [7][3]proofarith.Xpt
	faces[0] = q
	for i := range 3 {
		j := (i + 1) % 3
		faces[1+2*i] = [3]proofarith.Xpt{base[i], base[j], q[j]}
		faces[2+2*i] = [3]proofarith.Xpt{base[i], q[j], q[i]}
	}
	for i, face := range faces {
		for k, p := range face {
			want := pts[chamferPrismFaceCorners[i][k]]
			require.Zero(t, meshbool.RatCoordOf(p, 0).Cmp(meshbool.RatCoordOf(want, 0)), "face %d corner %d", i, k)
			require.Zero(t, meshbool.RatCoordOf(p, 1).Cmp(meshbool.RatCoordOf(want, 1)), "face %d corner %d", i, k)
			require.Zero(t, meshbool.RatCoordOf(p, 2).Cmp(meshbool.RatCoordOf(want, 2)), "face %d corner %d", i, k)
		}
	}
}

// The seven face boxes of random prisms, built from rounded corners through the
// face table, equal the per-face ChamferExactTriBox boxes bit for bit.
func TestChamferPrismFaceBoxesMatchPerFace(t *testing.T) {
	rng := rand.New(rand.NewPCG(127, 131))
	refused := 0
	for range 400 {
		var pts [6]proofarith.Xpt
		pool := [3]proofarith.Xpt{
			randomCornerPoint(rng, rng.IntN(3)), randomCornerPoint(rng, rng.IntN(2)),
			randomCornerPoint(rng, rng.IntN(2)),
		}
		for i := range pts {
			if rng.IntN(3) == 0 {
				pts[i] = pool[rng.IntN(len(pool))] // shared and repeated corners
				continue
			}
			pts[i] = randomCornerPoint(rng, rng.IntN(2))
		}
		var corners [6]ChamferCorner
		for i := range pts {
			corners[i] = ChamferCornerOf(pts[i])
		}
		base := [3]proofarith.Xpt{pts[0], pts[1], pts[2]}
		q := [3]proofarith.Xpt{pts[3], pts[4], pts[5]}
		var faces [7][3]proofarith.Xpt
		faces[0] = q
		for i := range 3 {
			j := (i + 1) % 3
			faces[1+2*i] = [3]proofarith.Xpt{base[i], base[j], q[j]}
			faces[2+2*i] = [3]proofarith.Xpt{base[i], q[j], q[i]}
		}
		for i, face := range faces {
			want, wantOK := ChamferExactTriBox(face)
			fc := chamferPrismFaceCorners[i]
			got, gotOK := ChamferCornersBox(corners[fc[0]], corners[fc[1]], corners[fc[2]])
			require.Equal(t, wantOK, gotOK)
			if !wantOK {
				refused++
				continue
			}
			requireBoxesBitEqual(t, want, got)
		}
	}
	require.Positive(t, refused)
}

// legacyChamferMiddlePrism is ChamferMiddlePrism before each corner was
// rounded once: boxes from ChamferExactTriBox and float faces from Vec.
func legacyChamferMiddlePrism(ctx context.Context, triCount int,
	old *LoftAuditData, oldBoxes [][2]r3.Vec, oldGrid *ChamferBoxGrid,
	oldParity *meshbool.ParityMesh,
	oldIndices []int, base, q [3]proofarith.Xpt, shift proofarith.Xpt) error {
	if len(oldBoxes) != triCount || oldGrid == nil ||
		meshbool.RatCoordOf(shift, 2).Sign() != 0 {
		return decaderr.ErrUnsupported
	}
	var faces [7][3]proofarith.Xpt
	faces[0] = q
	for i := range 3 {
		j := (i + 1) % 3
		faces[1+2*i] = [3]proofarith.Xpt{base[i], base[j], q[j]}
		faces[2+2*i] = [3]proofarith.Xpt{base[i], q[j], q[i]}
	}
	baseNormal := proofarith.Xcross(proofarith.Xsub(base[1], base[0]),
		proofarith.Xsub(base[2], base[0]))
	if proofarith.XdotSign(baseNormal, baseNormal) == 0 ||
		proofarith.XdotSign(baseNormal, shift) >= 0 {
		return decaderr.ErrUnsupported
	}
	for i, face := range faces {
		if i%2 == 0 && ctx.Err() != nil {
			return ctx.Err()
		}
		normal := proofarith.Xcross(proofarith.Xsub(face[1], face[0]),
			proofarith.Xsub(face[2], face[0]))
		if proofarith.XdotSign(normal, normal) == 0 {
			return decaderr.ErrUnsupported
		}
		box, ok := ChamferExactTriBox(face)
		if !ok {
			return decaderr.ErrUnsupported
		}
		floatFace := [3]r3.Vec{face[0].Vec(), face[1].Vec(), face[2].Vec()}
		plane := NewLoftExactPlane(face[0], normal)
		candidates, err := oldGrid.candidates(box)
		if err != nil {
			return err
		}
		for _, j := range candidates {
			if !meshbool.BoxesOverlap(box, oldBoxes[j]) {
				continue
			}
			signsA := TrianglePlaneSigns(old.Planes[j], face)
			signsB := TrianglePlaneSigns(plane, old.Xtris[j])
			contact, err := meshbool.TriTriContactSpanCore(floatFace, old.Corners[j],
				face, old.Xtris[j], normal, old.Norms[j], nil, nil,
				&signsA, &signsB, false, r3.Vec{}, r3.Vec{}, false)
			if err != nil {
				return err
			}
			if contact.Kind == meshbool.ContactNone {
				continue
			}
			if contact.Kind == meshbool.ContactRegion ||
				proofarith.OrientSignExact(base[0], base[1], base[2], contact.P0) != 0 ||
				!meshbool.PointOnTri(contact.P0, base, baseNormal) {
				return decaderr.ErrUnsupported
			}
			if contact.Kind == meshbool.ContactSegment &&
				(proofarith.OrientSignExact(base[0], base[1], base[2], contact.P1) != 0 ||
					!meshbool.PointOnTri(contact.P1, base, baseNormal)) {
				return decaderr.ErrUnsupported
			}
		}
	}
	witness := meshbool.XCentroid(q[0], q[1], q[2])
	inside, on, err := meshbool.MeshParityPreparedContext(ctx, witness, oldParity, oldIndices)
	if err != nil || !inside || on {
		return decaderr.ErrUnsupported
	}
	return nil
}

func TestChamferMiddlePrismCornersMatchLegacy(t *testing.T) {
	verts := []r3.Vec{
		{X: 0, Y: 0, Z: 0}, {X: 1, Y: 0, Z: 0}, {X: 1, Y: 1, Z: 0}, {X: 0, Y: 1, Z: 0},
		{X: 0, Y: 0, Z: 1}, {X: 1, Y: 0, Z: 1}, {X: 1, Y: 1, Z: 1}, {X: 0, Y: 1, Z: 1},
	}
	tris := [][3]int{
		{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4},
		{1, 2, 6}, {1, 6, 5}, {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7},
	}
	indices := make([]int, len(tris))
	for i := range indices {
		indices[i] = i
	}
	old := NewLoftAuditData(verts, tris)
	boxes := make([][2]r3.Vec, len(tris))
	for i, tri := range tris {
		boxes[i] = meshbool.TriBox(verts, tri)
	}
	parity := meshbool.NewParityMesh(verts, tris)
	pt := func(x, y, z float64) proofarith.Xpt { return proofarith.XptOf(r3.Vec{X: x, Y: y, Z: z}) }

	type shot struct {
		base [3]proofarith.Xpt
		d    float64
	}
	lower := [3]proofarith.Xpt{pt(0.25, 0, 0.25), pt(0.75, 0, 0.25), pt(0.5, 0, 0.75)}
	edge := [3]proofarith.Xpt{pt(0.25, 0, 0.5), pt(0.75, 0, 0.5), pt(0.5, 0, 1)}
	// A base corner pokes through the right wall, so side faces cross it away
	// from the base: only their boxes find the wall's triangles.
	poke := [3]proofarith.Xpt{pt(0.5, 0, 0.25), pt(1.05, 0, 0.25), pt(0.5, 0, 0.75)}
	// A base corner pokes through the top wall.
	pokeTop := [3]proofarith.Xpt{pt(0.25, 0, 0.5), pt(0.75, 0, 0.5), pt(0.5, 0, 1.05)}
	var shots []shot
	for _, d := range []float64{0.1, 0.5} {
		shots = append(shots, shot{poke, d}, shot{pokeTop, d})
	}
	for _, d := range []float64{0.1, 0.5, 0.99, 1, 1.5} {
		shots = append(shots, shot{lower, d})
	}
	for _, d := range []float64{0.1, 0.5} {
		shots = append(shots, shot{edge, d})
	}
	passed, failed := 0, 0
	for _, s := range shots {
		shiftPt := proofarith.XptOf(r3.Vec{Y: s.d})
		var q [3]proofarith.Xpt
		var corners [6]ChamferCorner
		for i := range q {
			q[i] = chamferTranslate(s.base[i], shiftPt)
			corners[i] = ChamferCornerOf(s.base[i])
			corners[3+i] = ChamferCornerOf(q[i])
		}
		run := func(newForm bool) error {
			grid, err := ChamferBoxGridOf(boxes, 0, 1)
			require.NoError(t, err)
			if newForm {
				return ChamferMiddlePrism(t.Context(), len(tris), old, boxes, grid, parity,
					s.base, q, shiftPt, corners)
			}
			return legacyChamferMiddlePrism(t.Context(), len(tris), old, boxes, grid, parity, indices,
				s.base, q, shiftPt)
		}
		legacyErr, newErr := run(false), run(true)
		require.Equal(t, legacyErr == nil, newErr == nil, "distance %v", s.d)
		if legacyErr == nil {
			passed++
			continue
		}
		failed++
	}
	require.Positive(t, passed)
	require.Positive(t, failed)
}
